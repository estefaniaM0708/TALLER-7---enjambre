"""Gemelo digital del enjambre: PyBullet + panel web + registros.

No calcula rutas: reproduce lo que hacen las 3 ESP32-S3.
  - Se anuncia al carro 1 (punto de acceso) con HOLA cada segundo y recibe por UDP 5005 la
    telemetría de las 3 placas (TEL, RUT, FER, CAR, OBS, OLA).
  - Mueve 3 carritos de tracción diferencial (carrito.urdf) en PyBullet a 240 Hz, cada uno
    detrás de la posición que reporta su ESP32.
  - Pinta en el piso la feromona de la ESP32 elegida y muestra todo en http://localhost:8000
  - Envía los comandos del panel (CMD) al carro 1, que los reparte a las otras dos.
  - Guarda registros/sesion_..._telemetria.csv y ..._eventos.csv

Uso:  python gemelo.py [--ap 192.168.4.1] [--gui]
"""

import argparse
import csv
import io
import json
import math
import multiprocessing as mp
import os
import queue
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import laberinto as L

AQUI = os.path.dirname(os.path.abspath(__file__))
PUERTO_NODOS = 4210
PUERTO_PC = 5005
NOMBRES = {1: "Carro 1", 2: "Carro 2", 3: "Carro 3"}
COLORES = {1: (0.86, 0.20, 0.20), 2: (0.20, 0.45, 0.92), 3: (0.15, 0.70, 0.30)}
ESTADOS = {0: "sin ruta", 1: "andando", 2: "en extremo"}
TEL_CAMPOS = ["id", "seq", "t_ms", "iter", "mejor", "iter_pasos", "iter_mejora", "compartir", "estado",
              "sentido", "idx", "ruta_n", "viaje", "celda", "rx1", "rx2", "rx3", "perd1", "perd2", "perd3",
              "malas", "tx", "crc", "periodo", "vel", "mejoras_ext", "bloqueadas", "reinicios", "desvios"]


# =============================================================================== utilidades
def checksum(cuerpo):
    cs = 0
    for ch in cuerpo:
        cs ^= ord(ch)
    return cs


def trama(cuerpo):
    return ("$%s*%02X" % (cuerpo, checksum(cuerpo))).encode()


def partir(texto):
    """Valida '$...*CS' y devuelve la lista de campos, o None."""
    texto = texto.strip()
    if len(texto) < 5 or texto[0] != "$" or texto[-3] != "*":
        return None
    cuerpo = texto[1:-3]
    try:
        if checksum(cuerpo) != int(texto[-2:], 16):
            return None
    except ValueError:
        return None
    return cuerpo.split(",")


def de_hex(h):
    try:
        return list(bytes.fromhex(h))
    except ValueError:
        return None


def ruta_optima():
    """Pasos de la ruta más corta (búsqueda en anchura), para marcar cuándo llega cada ESP32."""
    dist = {L.INICIO: 0}
    cola = [L.INICIO]
    for u in cola:
        f, c = L.CELDAS[u]
        for df, dc in ((0, 1), (1, 0), (0, -1), (-1, 0)):
            v = L.ID.get((f + df, c + dc))
            if v is not None and v not in dist:
                dist[v] = dist[u] + 1
                cola.append(v)
    return dist[L.META]


OPTIMO = ruta_optima()


# =============================================================================== registros
class Registro:
    def __init__(self, carpeta):
        os.makedirs(carpeta, exist_ok=True)
        base = os.path.join(carpeta, time.strftime("sesion_%Y%m%d_%H%M%S"))
        self.ft = open(base + "_telemetria.csv", "w", newline="")
        self.fe = open(base + "_eventos.csv", "w", newline="")
        self.t = csv.writer(self.ft)
        self.e = csv.writer(self.fe)
        self.t.writerow(["t_pc"] + TEL_CAMPOS + ["x_virtual", "y_virtual", "dist_esp_m"])
        self.e.writerow(["t_pc", "nodo", "evento", "detalle"])
        self.lock = threading.Lock()
        self.t0 = time.time()
        self.base = base

    def tel(self, campos, x, y, dist):
        with self.lock:
            self.t.writerow(["%.3f" % (time.time() - self.t0)] + campos + ["%.3f" % x, "%.3f" % y, "%.3f" % dist])

    def evento(self, nodo, evento, detalle=""):
        with self.lock:
            self.e.writerow(["%.3f" % (time.time() - self.t0), nodo, evento, detalle])
            self.fe.flush()

    def vaciar(self):
        with self.lock:
            self.ft.flush()
            self.fe.flush()


# =============================================================================== estado de cada ESP32
class Nodo:
    def __init__(self, i):
        self.id = i
        self.tel = None             # último TEL como diccionario
        self.ultimo = 0.0           # hora del último paquete
        self.ip = ""
        self.mejor = []             # mejor ruta (celdas)
        self.feromona = []          # 0-255 por arista
        self.car = {"viaje": 0, "sentido": 0, "desvio": 0, "ruta": []}
        self.obst = set()
        self.historial = []         # (t, iteración, mejor)
        self.llego_optimo = None    # iteración en que tuvo la ruta óptima
        self.latencias = []
        self.seq_tel = 0
        self.huecos_tel = 0

    def en_linea(self):
        return time.time() - self.ultimo < 3


class Enlace:
    """UDP con las ESP32: recibe en 5005 y envía HOLA y CMD al carro 1."""

    def __init__(self, ap_ip, registro):
        self.ap = (ap_ip, PUERTO_NODOS)
        self.reg = registro
        self.nodos = {i: Nodo(i) for i in (1, 2, 3)}
        self.lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", PUERTO_PC))
        self.sock.settimeout(0.5)
        self.hola = {}              # número de HOLA -> hora de envío
        self.n_hola = 0
        self.n_cmd = int(time.time()) % 30000
        self.malas = 0
        self.paquetes = 0
        self.posiciones = {}        # posición del carrito virtual (la llena la física)
        threading.Thread(target=self._recibir, daemon=True).start()
        threading.Thread(target=self._anunciar, daemon=True).start()

    def _mandar(self, cuerpo):
        try:
            self.sock.sendto(trama(cuerpo), self.ap)
        except OSError:
            pass                    # sin red todavía

    def _anunciar(self):
        while True:
            self.n_hola += 1
            self.hola[self.n_hola] = time.time()
            self.hola.pop(self.n_hola - 30, None)
            self._mandar("HOLA,%d" % self.n_hola)
            time.sleep(1)

    def comando(self, accion, valor=0):
        """Se envía 2 veces; el carro 1 lo repite otras 2 a las demás. Cada ESP32 lo aplica una vez."""
        self.n_cmd = (self.n_cmd + 1) % 60000
        if accion == "RESET":
            valor = self.n_cmd               # número de "época": las rutas de antes del reinicio se ignoran
        cuerpo = "CMD,%d,%s,%d" % (self.n_cmd, accion, int(valor))
        self._mandar(cuerpo)
        threading.Timer(0.25, self._mandar, args=(cuerpo,)).start()
        self.reg.evento(0, "CMD", "%s %d" % (accion, int(valor)))

    def _recibir(self):
        while True:
            try:
                datos, (ip, _) = self.sock.recvfrom(2048)
            except (socket.timeout, OSError):
                continue
            c = partir(datos.decode(errors="replace"))
            if not c or len(c) < 2:
                self.malas += 1
                continue
            try:
                i = int(c[1])
            except ValueError:
                continue
            if i not in self.nodos:
                continue
            with self.lock:
                self.paquetes += 1
                self._procesar(self.nodos[i], c, ip)

    def _procesar(self, n, c, ip):
        ahora = time.time()
        n.ultimo, n.ip = ahora, ip
        tipo = c[0]
        if tipo == "TEL" and len(c) >= len(TEL_CAMPOS) + 1:
            campos = c[1:len(TEL_CAMPOS) + 1]
            t = dict(zip(TEL_CAMPOS, campos))
            for k in t:
                if k != "crc":
                    t[k] = int(t[k])
            if n.tel and t["iter"] < n.tel["iter"] - 1:
                self.reg.evento(n.id, "REINICIO", "iteración %d -> %d" % (n.tel["iter"], t["iter"]))
                n.historial, n.llego_optimo = [], None
            if n.tel and t["mejor"] != n.tel["mejor"] and t["mejor"] > 0:
                self.reg.evento(n.id, "MEJORA", "%d pasos en la iteración %d" % (t["mejor"], t["iter"]))
            if t["mejor"] == OPTIMO and n.llego_optimo is None:
                n.llego_optimo = t["iter_mejora"]
                self.reg.evento(n.id, "OPTIMA", "iteración %d" % t["iter_mejora"])
            if t["mejor"] != OPTIMO:
                n.llego_optimo = None
            if n.seq_tel and t["seq"] > n.seq_tel + 1:
                n.huecos_tel += t["seq"] - n.seq_tel - 1
            n.seq_tel = t["seq"]
            if not n.historial or n.historial[-1][1] != t["iter"]:
                n.historial.append((round(ahora, 2), t["iter"], t["mejor"]))
                del n.historial[:-400]
            n.tel = t
            x, y, d = self.posiciones.get(n.id, (0, 0, 0))
            self.reg.tel(campos, x, y, d)
        elif tipo == "RUT" and len(c) >= 7:
            r = de_hex(c[6])
            if r is not None:
                n.mejor = r
        elif tipo == "FER" and len(c) >= 5:
            f = de_hex(c[4])
            if f is not None and len(f) == len(L.ARISTAS):
                n.feromona = f
        elif tipo == "CAR" and len(c) >= 7:
            r = de_hex(c[6])
            if r is not None:
                nuevo = {"viaje": int(c[3]), "sentido": int(c[4]), "desvio": int(c[5]), "ruta": r}
                if nuevo["desvio"] and not n.car.get("desvio"):
                    self.reg.evento(n.id, "OBSTACULO", "media vuelta en la celda %d" % (r[0] if r else -1))
                n.car = nuevo
        elif tipo == "OBS" and len(c) >= 4:
            r = de_hex(c[3])
            if r is not None:
                n.obst = set(r)
        elif tipo == "OLA" and len(c) >= 3:
            enviado = self.hola.get(int(c[2]))
            if enviado:
                ms = (ahora - enviado) * 1000
                n.latencias.append(round(ms, 1))
                del n.latencias[:-60]
                self.reg.evento(n.id, "LATENCIA", "%.1f ms" % ms)


# =============================================================================== física
class Carrito:
    R, B = 0.03, 0.124      # radio de rueda y distancia entre ruedas (m)
    Z_CM = 0.04             # altura del centro de masa del chasis sobre el piso (carrito.urdf)

    def __init__(self, p, i, x, y):
        self.p, self.id = p, i
        self.cuerpo = p.loadURDF(os.path.join(AQUI, "carrito.urdf"), [x, y, 0.002],
                                 p.getQuaternionFromEuler([0, 0, 0]))
        nombres = {p.getJointInfo(self.cuerpo, j)[1].decode(): j for j in range(p.getNumJoints(self.cuerpo))}
        self.izq, self.der = nombres["motor_izq"], nombres["motor_der"]
        for apoyo in ("fija_apoyo_atras", "fija_apoyo_adelante"):
            p.changeDynamics(self.cuerpo, nombres[apoyo], lateralFriction=0.0, rollingFriction=0.0,
                             spinningFriction=0.0)
        for rueda in (self.izq, self.der):
            p.changeDynamics(self.cuerpo, rueda, lateralFriction=1.2)
        # Bullet "duerme" los cuerpos quietos unos 2 s y luego no responden a los motores:
        # el carrito espera en los extremos, así que no se le deja dormir.
        p.changeDynamics(self.cuerpo, -1, activationState=p.ACTIVATION_STATE_DISABLE_SLEEPING)
        r, g, b = COLORES[i]
        p.changeVisualShape(self.cuerpo, nombres["fija_baliza"], rgbaColor=[r, g, b, 1])
        self.ruta, self.viaje, self.desvio, self.k = [], -1, 0, 0
        self.mejor_dist, self.t_avance = 1e9, time.time()
        self.resinc = 0

    def pose(self):
        (x, y, _), q = self.p.getBasePositionAndOrientation(self.cuerpo)
        return x, y, self.p.getEulerFromQuaternion(q)[2]

    def ubicar(self, celda, siguiente=None):
        x, y = L.posicion(celda)
        yaw = 0.0
        if siguiente is not None:
            x2, y2 = L.posicion(siguiente)
            yaw = math.atan2(y2 - y, x2 - x)
        # PyBullet ubica la base por su centro de masa, que en carrito.urdf está 4 cm arriba del
        # origen del chasis: con z = Z_CM las ruedas quedan apoyadas en el piso, no enterradas.
        self.p.resetBasePositionAndOrientation(self.cuerpo, [x, y, self.Z_CM + 0.002],
                                               self.p.getQuaternionFromEuler([0, 0, yaw]))
        self.p.resetBaseVelocity(self.cuerpo, [0, 0, 0], [0, 0, 0])
        for j in (self.izq, self.der):
            self.p.resetJointState(self.cuerpo, j, 0, 0)

    def ruedas(self, v, w):
        wl = (v - w * self.B / 2) / self.R
        wr = (v + w * self.B / 2) / self.R
        for j, vel in ((self.izq, wl), (self.der, wr)):
            self.p.setJointMotorControl2(self.cuerpo, j, self.p.VELOCITY_CONTROL, targetVelocity=vel, force=1.5)

    def controlar(self, nodo, reg):
        """Lleva el carrito a la celda donde va su ESP32."""
        car, tel = nodo.car, nodo.tel
        if not tel or not car["ruta"]:
            self.ruedas(0, 0)
            return 0.0
        if car["viaje"] != self.viaje or car["desvio"] != self.desvio or car["ruta"] != self.ruta:
            self.ruta, self.viaje, self.desvio = car["ruta"], car["viaje"], car["desvio"]
            self.k = 0
            self.mejor_dist, self.t_avance = 1e9, time.time()
        if tel["viaje"] != self.viaje or tel["ruta_n"] != len(self.ruta):
            self.ruedas(0, 0)                       # la telemetría aún es del tramo anterior
            self.t_avance = time.time()
            return 0.0
        idx = min(max(tel["idx"], 0), len(self.ruta) - 1)
        x, y, yaw = self.pose()
        ex, ey = L.posicion(self.ruta[idx])
        dist_esp = math.hypot(ex - x, ey - y)

        # Resincronización: muy atrasado o atascado
        if idx - self.k > 10 or time.time() - self.t_avance > 6:
            self.k = idx
            self.ubicar(self.ruta[idx], self.ruta[idx + 1] if idx + 1 < len(self.ruta) else None)
            self.mejor_dist, self.t_avance = 1e9, time.time()
            self.resinc += 1
            reg.evento(self.id, "RESINC", "celda %d" % idx)
            return dist_esp

        j = min(self.k + 1, idx) if self.k < idx else self.k
        tx, ty = L.posicion(self.ruta[j])
        d = math.hypot(tx - x, ty - y)
        # Celda intermedia: se da por alcanzada un poco antes del centro (gira sin detenerse)
        radio = 0.04 if j == idx else 0.09
        if j > self.k and d < radio:
            self.k = j
            self.mejor_dist, self.t_avance = 1e9, time.time()
            j = min(self.k + 1, idx) if self.k < idx else self.k
            tx, ty = L.posicion(self.ruta[j])
            d = math.hypot(tx - x, ty - y)
        if d < self.mejor_dist - 0.005:
            self.mejor_dist, self.t_avance = d, time.time()
        if j == self.k and d < 0.06:                # alcanzó a su ESP32: espera
            self.t_avance = time.time()
            self.ruedas(0, 0)
            return dist_esp
        error = (math.atan2(ty - y, tx - x) - yaw + math.pi) % (2 * math.pi) - math.pi
        v_nom = L.CELDA / max(tel["vel"], 50) * 1000
        atraso = idx - self.k
        v = v_nom * min(2.5, 1.0 + 0.5 * max(0, atraso - 1))   # si se atrasa, acelera
        if j == idx and d < 0.15:
            v = min(v, 0.3 + 3 * d)                 # frena al llegar a donde va la ESP32
        if abs(error) > 0.6:                        # esquina o media vuelta: gira en el sitio
            self.ruedas(0, math.copysign(8.0, error))
        else:                                       # avanza corrigiendo el rumbo
            self.ruedas(v * max(0.2, math.cos(error)), 8.0 * error)
        return dist_esp


def construir_escena(p, con_colision):
    """Piso, muros y baldosas. Devuelve {celda: id de la baldosa}."""
    p.setGravity(0, 0, -9.81)
    ancho, alto = L.COLUMNAS * L.CELDA, L.FILAS * L.CELDA
    cx, cy = (L.COLUMNAS - 1) * L.CELDA / 2, (L.FILAS - 1) * L.CELDA / 2
    piso_c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[ancho / 2 + 1, alto / 2 + 1, 0.01]) if con_colision else -1
    piso_v = p.createVisualShape(p.GEOM_BOX, halfExtents=[ancho / 2 + 1, alto / 2 + 1, 0.01], rgbaColor=[0.85, 0.85, 0.83, 1])
    p.createMultiBody(0, piso_c, piso_v, [cx, cy, -0.01])
    h = 0.08
    muro_c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[L.CELDA / 2, L.CELDA / 2, h / 2]) if con_colision else -1
    muro_v = p.createVisualShape(p.GEOM_BOX, halfExtents=[L.CELDA / 2, L.CELDA / 2, h / 2], rgbaColor=[0.33, 0.36, 0.42, 1])
    for f in range(L.FILAS):
        for c in range(L.COLUMNAS):
            if L.es_pared(f, c):
                p.createMultiBody(0, muro_c, muro_v, [c * L.CELDA, (L.FILAS - 1 - f) * L.CELDA, h / 2])
    baldosas = {}
    if not con_colision:
        for i in range(len(L.CELDAS)):
            x, y = L.posicion(i)
            color = [0.2, 0.75, 0.3, 1] if i == L.INICIO else [0.85, 0.25, 0.25, 1] if i == L.META else [0.95, 0.95, 0.93, 1]
            v = p.createVisualShape(p.GEOM_BOX, halfExtents=[L.CELDA / 2 - 0.006, L.CELDA / 2 - 0.006, 0.002], rgbaColor=color)
            baldosas[i] = p.createMultiBody(0, -1, v, [x, y, 0.002])
    return baldosas


# =============================================================================== proceso de imagen
def proceso_render(entrada, salida, ancho, alto):
    """PyBullet propio solo para dibujar: así el renderizado por software no frena la física."""
    import pybullet as p
    from PIL import Image
    p.connect(p.DIRECT)
    baldosas = construir_escena(p, con_colision=False)
    carros = {}
    for i in (1, 2, 3):
        cid = p.loadURDF(os.path.join(AQUI, "carrito.urdf"), [0, -5, 0], useFixedBase=True)
        for j in range(p.getNumJoints(cid)):
            if p.getJointInfo(cid, j)[1].decode() == "fija_baliza":
                p.changeVisualShape(cid, j, rgbaColor=list(COLORES[i]) + [1])
        carros[i] = cid
    cajas = {}
    caja_v = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.09, 0.09, 0.09], rgbaColor=[1.0, 0.55, 0.05, 1])
    colores_actuales = {}
    cx, cy = (L.COLUMNAS - 1) * L.CELDA / 2, (L.FILAS - 1) * L.CELDA / 2
    vista = p.computeViewMatrixFromYawPitchRoll([cx, cy - 0.15, 0], 5.0, 0, -66, 0, 2)
    proy = p.computeProjectionMatrixFOV(50, ancho / alto, 0.1, 20)
    while True:
        datos = entrada.get()
        while True:                                 # quedarse con el más reciente
            try:
                datos = entrada.get_nowait()
            except queue.Empty:
                break
        for i, (x, y, yaw) in datos["carros"].items():
            p.resetBasePositionAndOrientation(carros[i], [x, y, Carrito.Z_CM + 0.002], p.getQuaternionFromEuler([0, 0, yaw]))
        for celda, color in datos["baldosas"].items():
            if colores_actuales.get(celda) != color:
                p.changeVisualShape(baldosas[celda], -1, rgbaColor=list(color) + [1])
                colores_actuales[celda] = color
        obst = set(datos["obst"])
        for celda in obst - set(cajas):
            x, y = L.posicion(celda)
            cajas[celda] = p.createMultiBody(0, -1, caja_v, [x, y, 0.09])
        for celda in set(cajas) - obst:
            p.removeBody(cajas.pop(celda))
        _, _, rgba, _, _ = p.getCameraImage(ancho, alto, vista, proy, renderer=p.ER_TINY_RENDERER,
                                            shadow=0, lightDirection=[0.5, -0.6, 1])
        img = Image.frombytes("RGBA", (ancho, alto), bytes(bytearray(rgba)) if not isinstance(rgba, bytes) else rgba)
        b = io.BytesIO()
        img.convert("RGB").save(b, format="JPEG", quality=80)
        try:
            salida.put_nowait(b.getvalue())
        except queue.Full:
            pass


def colores_piso(nodo):
    """Color de cada baldosa según la feromona de la ESP32 elegida."""
    base = (0.95, 0.95, 0.93)
    r, g, b = COLORES[nodo.id]
    intensidad = [0.0] * len(L.CELDAS)
    if nodo.feromona:
        for k, (a, bb) in enumerate(L.ARISTAS):
            v = nodo.feromona[k] / 255.0
            intensidad[a] = max(intensidad[a], v)
            intensidad[bb] = max(intensidad[bb], v)
    colores = {}
    for i, v in enumerate(intensidad):
        if i in (L.INICIO, L.META):
            continue
        v = round(v * 8) / 8                         # pocos niveles: menos cambios que dibujar
        colores[i] = tuple(round(base[k] + (c - base[k]) * v * 0.85, 3) for k, c in enumerate((r, g, b)))
    return colores


# =============================================================================== panel web
def servidor(enlace, video, elegido, puerto):
    with open(os.path.join(AQUI, "web", "index.html"), encoding="utf-8") as f:
        pagina = f.read().encode()
    mapa = json.dumps({"filas": L.FILAS, "columnas": L.COLUMNAS, "mapa": L.MAPA, "celdas": L.CELDAS,
                       "aristas": L.ARISTAS, "inicio": L.INICIO, "meta": L.META, "crc": "%08X" % L.CRC,
                       "optimo": OPTIMO, "nombres": NOMBRES,
                       "colores": {i: "rgb(%d,%d,%d)" % tuple(int(255 * x) for x in c) for i, c in COLORES.items()}}).encode()

    def estado():
        with enlace.lock:
            nodos = {}
            for i, n in enlace.nodos.items():
                lat = sorted(n.latencias)
                nodos[i] = {"en_linea": n.en_linea(), "ip": n.ip, "tel": n.tel, "mejor": n.mejor,
                            "feromona": n.feromona, "car": n.car, "obst": sorted(n.obst),
                            "historial": n.historial[-200:], "llego_optimo": n.llego_optimo,
                            "latencia": lat[len(lat) // 2] if lat else None,
                            "laberinto_ok": n.tel is None or n.tel["crc"].upper() == "%08X" % L.CRC,
                            "virtual": enlace.posiciones.get(i)}
            return {"nodos": nodos, "paquetes": enlace.paquetes, "malas": enlace.malas, "elegido": elegido[0]}

    class Panel(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _responder(self, cuerpo, tipo):
            self.send_response(200)
            self.send_header("Content-Type", tipo)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(cuerpo)

        def do_GET(self):
            ruta = self.path.split("?")[0]
            if ruta == "/":
                self._responder(pagina, "text/html; charset=utf-8")
            elif ruta == "/mapa":
                self._responder(mapa, "application/json")
            elif ruta == "/estado":
                self._responder(json.dumps(estado()).encode(), "application/json")
            elif ruta == "/foto.jpg":
                self._responder(video[0] or b"", "image/jpeg")
            elif ruta == "/video":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=cuadro")
                self.end_headers()
                ultimo = None
                try:
                    while True:
                        img = video[0]
                        if img is not None and img is not ultimo:
                            ultimo = img
                            self.wfile.write(b"--cuadro\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(img))
                            self.wfile.write(img + b"\r\n")
                        time.sleep(0.05)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_error(404)

        def do_POST(self):
            largo = int(self.headers.get("Content-Length", 0))
            try:
                d = json.loads(self.rfile.read(largo) or b"{}")
            except ValueError:
                return self.send_error(400)
            if self.path == "/cmd":
                accion = str(d.get("accion", "")).upper()
                if accion in ("RESET", "RSTN", "COMP", "PER", "VEL", "BLQ", "DBQ", "LIBRE"):
                    enlace.comando(accion, int(d.get("valor", 0)))
            elif self.path == "/ver":
                elegido[0] = int(d.get("nodo", 1))
            self._responder(b"{}", "application/json")

    srv = ThreadingHTTPServer(("0.0.0.0", puerto), Panel)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()


# =============================================================================== programa
def main():
    ap = argparse.ArgumentParser(description="Gemelo digital del enjambre ACO")
    ap.add_argument("--ap", default=os.environ.get("AP_IP", "192.168.4.1"), help="IP del carro 1 (punto de acceso)")
    ap.add_argument("--puerto", type=int, default=int(os.environ.get("PUERTO_WEB", 8000)))
    ap.add_argument("--fps", type=float, default=float(os.environ.get("FPS", 8)))
    ap.add_argument("--registros", default=os.environ.get("REGISTROS", os.path.join(AQUI, "..", "registros")))
    ap.add_argument("--gui", action="store_true", help="abrir también la ventana de PyBullet (mundo del carro 1)")
    a = ap.parse_args()

    import pybullet as p
    reg = Registro(a.registros)
    enlace = Enlace(a.ap, reg)
    video, elegido = [None], [1]
    servidor(enlace, video, elegido, a.puerto)

    entrada, salida = mp.Queue(maxsize=2), mp.Queue(maxsize=2)
    render = mp.Process(target=proceso_render, args=(entrada, salida, 800, 600), daemon=True)
    render.start()

    # Cada carrito tiene su propio mundo físico (mismo laberinto): así no chocan entre ellos,
    # igual que las 3 ESP32, que emulan cada una su carrito por separado.
    from pybullet_utils.bullet_client import BulletClient
    mundos, carros = {}, {}
    xa, ya = L.posicion(L.INICIO)
    for i in (1, 2, 3):
        mundos[i] = BulletClient(connection_mode=p.GUI if (a.gui and i == 1) else p.DIRECT)
        mundos[i].setTimeStep(1 / 240)
        construir_escena(mundos[i], con_colision=True)
        carros[i] = Carrito(mundos[i], i, xa, ya)
        carros[i].ubicar(L.INICIO)

    print("Gemelo digital listo. Carro 1 (punto de acceso): %s:%d" % (a.ap, PUERTO_NODOS))
    print("Laberinto %dx%d, CRC %08X, ruta óptima %d pasos" % (L.FILAS, L.COLUMNAS, L.CRC, OPTIMO))
    print("Panel: http://localhost:%d   Registros: %s_*.csv" % (a.puerto, reg.base))

    dt = 1 / 240
    reloj = time.perf_counter()
    t_img = t_vaciar = 0.0
    while True:
        # Física a 240 Hz en tiempo real (hasta 24 pasos seguidos para alcanzar el reloj)
        pasos = 0
        while reloj <= time.perf_counter() and pasos < 24:
            with enlace.lock:
                for i, c in carros.items():
                    dist = c.controlar(enlace.nodos[i], reg)
                    x, y, _ = c.pose()
                    enlace.posiciones[i] = (round(x, 3), round(y, 3), round(dist, 3))
            for m in mundos.values():
                m.stepSimulation()
            reloj += dt
            pasos += 1
        if time.perf_counter() - reloj > 0.2:
            reloj = time.perf_counter()             # no acumular retraso
        ahora = time.perf_counter()
        if ahora - t_img >= 1 / a.fps:
            t_img = ahora
            with enlace.lock:
                n = enlace.nodos[elegido[0]]
                obst = set().union(*(m.obst for m in enlace.nodos.values()))
                datos = {"carros": {i: c.pose() for i, c in carros.items()}, "baldosas": colores_piso(n),
                         "obst": sorted(obst)}
            try:
                entrada.put_nowait(datos)
            except queue.Full:
                pass
        try:
            video[0] = salida.get_nowait()
        except queue.Empty:
            pass
        if ahora - t_vaciar >= 2:
            t_vaciar = ahora
            reg.vaciar()
        time.sleep(max(0.0, reloj - time.perf_counter()))


if __name__ == "__main__":
    main()
