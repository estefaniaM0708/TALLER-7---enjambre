"""gemelo_digital.py  -  Gemelo digital del enjambre en PyBullet + visor web.

MODO REAL (por defecto): se une a la red de las ESP32-S3 (placa 1 en modo AP),
recibe el estado de cada placa por UDP y mueve los 3 carritos en PyBullet.
Desde el visor web se envian las ordenes a las placas: iniciar, reiniciar,
obstaculo y velocidad de cada carro.

MODO SIMULACION (--simular o MODO=sim): no necesita placas; corre aqui mismo
los 3 "cerebros" con los mismos archivos de la ESP32 (enjambre.py, aco.py,
laberinto.py).

Uso:
    python pc/gemelo_digital.py                    # real, central en 192.168.4.1
    python pc/gemelo_digital.py --simular          # sin placas
    python pc/gemelo_digital.py --simular --gui    # ademas, ventana 3D de PyBullet
Visor: http://localhost:8080
"""

import argparse
import json
import math
import os
import socket
import struct
import sys
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import pybullet as p

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.environ.get("ESP32_DIR", os.path.join(AQUI, "..", "esp32")))
import laberinto as L          # noqa: E402  (el mismo mapa de las ESP32)
from enjambre import Carro     # noqa: E402  (el mismo cerebro, para simular)

PUERTO_UDP = 4210
COLORES = {1: (0.85, 0.15, 0.15, 1), 2: (0.15, 0.35, 0.90, 1), 3: (0.10, 0.65, 0.25, 1)}
IDS = (1, 2, 3)


# =====================================================================
#  Fuentes de datos: placas reales o simulacion
# =====================================================================
class FuenteReal:
    """Escucha a la central (placa 1) y le envia las ordenes del visor."""

    modo = "REAL"

    def __init__(self, central):
        self.central = (central, PUERTO_UDP)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 0))
        self.sock.settimeout(0.5)
        self.estados = {}
        self.mensajes = 0
        self.ultimo = 0.0
        self.lock = threading.Lock()
        threading.Thread(target=self._escuchar, daemon=True).start()
        threading.Thread(target=self._saludar, daemon=True).start()

    def _saludar(self):
        while True:              # se registra en la central cada 2 s
            self._mandar({"t": "hola"})
            time.sleep(2)

    def _mandar(self, msg):
        try:
            self.sock.sendto(json.dumps(msg).encode(), self.central)
        except OSError:
            pass

    def _escuchar(self):
        while True:
            try:
                datos, _ = self.sock.recvfrom(4096)
                msg = json.loads(datos)
            except (socket.timeout, OSError, ValueError):
                continue
            with self.lock:
                self.mensajes += 1
                self.ultimo = time.time()
                if msg.get("t") == "est":
                    self.estados[msg["id"]] = msg

    def conectado(self):
        return time.time() - self.ultimo < 3

    def orden(self, msg):
        self._mandar(msg)
        if msg.get("c") == "reset":
            with self.lock:
                self.estados = {}


class FuenteSimulada:
    """Los 3 cerebros corriendo en el PC, con una red simulada."""

    modo = "SIMULACION"

    def __init__(self):
        self.lock = threading.Lock()
        self.cola = []
        self.carros = {i: Carro(i, self._de(i)) for i in IDS}
        self.estados = {}
        self.mensajes = 0
        threading.Thread(target=self._bucle, daemon=True).start()

    def _de(self, i):
        def enviar(msg):
            self.cola.append(msg)
        return enviar

    def _bucle(self):
        ahora, t_est = 0.0, 0.0
        dt = 0.02
        while True:
            with self.lock:
                ahora += dt
                while self.cola:                 # entrega los mensajes a todos
                    msg = self.cola.pop(0)
                    self.mensajes += 1
                    for c in self.carros.values():
                        c.recibir(msg, ahora)
                for c in self.carros.values():
                    c.paso(dt, ahora)
                    c.eventos.clear()
                if ahora - t_est >= 0.25:
                    t_est = ahora
                    for c in self.carros.values():
                        e = c.estado()
                        self.cola.append(e)
                        self.estados[c.id] = json.loads(json.dumps(e))
            time.sleep(dt)

    def conectado(self):
        return True

    def orden(self, msg):
        with self.lock:
            self.cola.append(msg)


# =====================================================================
#  Escena de PyBullet
# =====================================================================
def caja(medio, centro, color, yaw=0.0):
    v = p.createVisualShape(p.GEOM_BOX, halfExtents=medio, rgbaColor=color)
    return p.createMultiBody(0, -1, v, basePosition=centro,
                             baseOrientation=p.getQuaternionFromEuler((0, 0, yaw)))


class Escena:
    ALTO_MURO = 0.06

    def __init__(self, gui=False):
        p.connect(p.GUI if gui else p.DIRECT)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        P = L.PASO
        cols = max(c for c, _ in L.NODOS) + 1
        filas = max(f for _, f in L.NODOS) + 1
        ancho, largo = cols * P, filas * P
        self.centro = ((cols - 1) * P / 2, (filas - 1) * P / 2)

        # Piso
        caja((ancho / 2 + 0.5, largo / 2 + 0.6, 0.005),
             (self.centro[0], self.centro[1] + 0.1, -0.005), (0.93, 0.93, 0.90, 1))

        # Cinta de cada tramo (cambia de color con la feromona)
        self.cinta = []
        for a, b in L.TRAMOS:
            (xa, ya), (xb, yb) = L.POS[a], L.POS[b]
            horiz = abs(yb - ya) < 1e-6
            medio = (P / 2 + 0.012, 0.012, 0.002) if horiz else (0.012, P / 2 + 0.012, 0.002)
            self.cinta.append(caja(medio, ((xa + xb) / 2, (ya + yb) / 2, 0.002),
                                   (0.2, 0.2, 0.2, 1)))

        # Muros: entre nodos vecinos sin tramo y en el borde
        gris = (0.55, 0.58, 0.65, 1)
        h = self.ALTO_MURO / 2
        nodo = {L.NODOS[n]: n for n in range(len(L.NODOS))}
        for (c, f), n in nodo.items():
            for dc, df in ((1, 0), (0, 1)):
                m = nodo.get((c + dc, f + df))
                if m is not None and L.tramo(n, m) is None:
                    x, y = (c + dc / 2) * P, (f + df / 2) * P
                    medio = (0.01, P / 2, h) if dc else (P / 2, 0.01, h)
                    caja(medio, (x, y, h), gris)
        ini, met = L.NODOS[L.INICIO], L.NODOS[L.META]
        for c in range(cols):
            if not (ini[1] == 0 and ini[0] == c):            # puerta de entrada (A)
                caja((P / 2, 0.01, h), (c * P, -P / 2, h), gris)
            if not (met[1] == filas - 1 and met[0] == c):    # puerta de salida (META)
                caja((P / 2, 0.01, h), (c * P, (filas - 0.5) * P, h), gris)
        for f in range(filas):
            for c in (-0.5, cols - 0.5):
                caja((0.01, P / 2, h), (c * P, f * P, h), gris)

        # Inicio (A), meta (bandera a cuadros) y zonas de salida/llegada
        xa, ya = L.POS[L.INICIO]
        xm, ym = L.POS[L.META]
        caja((0.06, 0.06, 0.003), (xa, ya, 0.004), (0.2, 0.75, 0.3, 1))
        caja((0.06, 0.06, 0.003), (xm, ym, 0.004), (0.9, 0.2, 0.2, 1))
        caja((0.07, 0.3, 0.002), (xa, ya - 0.3, 0.003), (0.75, 0.9, 0.75, 1))
        caja((0.07, 0.3, 0.002), (xm, ym + 0.3, 0.003), (0.95, 0.8, 0.8, 1))
        for dx in (-0.09, 0.09):
            caja((0.006, 0.006, 0.12), (xm + dx, ym + 0.2, 0.12), (0.8, 0.1, 0.1, 1))
        for i in range(6):
            for j in range(2):
                color = (0.05, 0.05, 0.05, 1) if (i + j) % 2 else (1, 1, 1, 1)
                caja((0.015, 0.004, 0.015),
                     (xm - 0.075 + i * 0.03, ym + 0.2, 0.2 + j * 0.03), color)

        # Carritos
        self.carros = {i: self._carro(COLORES[i]) for i in IDS}
        self.obstaculos = {}
        self.colores_cinta = [None] * len(L.TRAMOS)

    @staticmethod
    def _carro(color):
        """Carrito: chasis + cabina + 4 ruedas (eslabones fijos al chasis)."""
        chasis = p.createVisualShape(p.GEOM_BOX, halfExtents=(0.06, 0.04, 0.015),
                                     rgbaColor=color)
        cabina = p.createVisualShape(p.GEOM_BOX, halfExtents=(0.025, 0.03, 0.012),
                                     rgbaColor=(0.75, 0.85, 0.95, 1))
        rueda = p.createVisualShape(p.GEOM_CYLINDER, radius=0.018, length=0.012,
                                    rgbaColor=(0.08, 0.08, 0.08, 1))
        partes = [(cabina, (-0.005, 0, 0.027), (0, 0, 0, 1))]
        giro = p.getQuaternionFromEuler((math.pi / 2, 0, 0))
        for dx in (0.038, -0.038):
            for dy in (0.047, -0.047):
                partes.append((rueda, (dx, dy, -0.012), giro))
        n = len(partes)
        return p.createMultiBody(
            0, -1, chasis, basePosition=(0, -5, 0.03),
            linkMasses=[0] * n, linkCollisionShapeIndices=[-1] * n,
            linkVisualShapeIndices=[v for v, _, _ in partes],
            linkPositions=[q for _, q, _ in partes],
            linkOrientations=[o for _, _, o in partes],
            linkInertialFramePositions=[(0, 0, 0)] * n,
            linkInertialFrameOrientations=[(0, 0, 0, 1)] * n,
            linkParentIndices=[0] * n, linkJointTypes=[p.JOINT_FIXED] * n,
            linkJointAxis=[(0, 0, 1)] * n)

    @staticmethod
    def pose(e):
        """Posicion (x, y) y rumbo (rad) del carro segun el estado de su ESP."""
        xa, ya = L.POS[L.INICIO]
        if e["f"] == "META":
            xm, ym = L.POS[L.META]
            return (xm, ym + 0.12 + 0.16 * (3 - e.get("p", 1)), math.pi / 2)
        if e["f"] != "RUTA" or e.get("c", 0) > 0:
            return (xa, ya - e.get("c", 0), math.pi / 2)
        (x1, y1), (x2, y2) = L.POS[e["u"]], L.POS[e["o"]]
        x = e.get("x", 0)
        return (x1 + (x2 - x1) * x, y1 + (y2 - y1) * x, math.radians(e.get("h", 90)))

    def actualizar(self, estados):
        estados = {int(k): v for k, v in estados.items()}
        tau, ruta, bloq = None, set(), set()
        for i in sorted(estados):
            e = estados[i]
            bloq.update(e.get("blq", []))
            if tau is None and e.get("tau"):
                tau = e["tau"]
            if e.get("best") and not ruta:
                b = e["best"]
                ruta = {L.tramo(b[k], b[k + 1]) for k in range(len(b) - 1)}
        for i, cuerpo in self.carros.items():
            e = estados.get(i)
            if e is None:
                p.resetBasePositionAndOrientation(cuerpo, (0, -5, 0), (0, 0, 0, 1))
                continue
            x, y, yaw = self.pose(e)
            p.resetBasePositionAndOrientation(cuerpo, (x, y, 0.03),
                                              p.getQuaternionFromEuler((0, 0, yaw)))
        # Color de la cinta: verde = mejor ruta, naranja = mucha feromona
        tmax = max(tau) if tau else 1
        for t in range(len(L.TRAMOS)):
            if t in bloq:
                color = (0.9, 0.1, 0.1, 1)
            elif t in ruta:
                color = (0.1, 0.8, 0.3, 1)
            elif tau:
                k = min(1.0, tau[t] / max(tmax, 1e-6))
                color = (0.2 + 0.8 * k, 0.2 + 0.4 * k, 0.2 - 0.1 * k, 1)
            else:
                color = (0.2, 0.2, 0.2, 1)
            color = tuple(round(c, 2) for c in color)
            if color != self.colores_cinta[t]:
                p.changeVisualShape(self.cinta[t], -1, rgbaColor=color)
                self.colores_cinta[t] = color
        # Cajas de obstaculo
        for t in bloq - set(self.obstaculos):
            a, b = L.TRAMOS[t]
            (xa, ya), (xb, yb) = L.POS[a], L.POS[b]
            self.obstaculos[t] = caja((0.035, 0.035, 0.035),
                                      ((xa + xb) / 2, (ya + yb) / 2, 0.035),
                                      (1.0, 0.55, 0.0, 1))
        for t in set(self.obstaculos) - bloq:
            p.removeBody(self.obstaculos.pop(t))

    def foto(self, ancho=640, alto=480):
        vista = p.computeViewMatrixFromYawPitchRoll(
            (self.centro[0], self.centro[1] - 0.12, 0), 2.6, 0, -64, 0, 2)
        proy = p.computeProjectionMatrixFOV(50, ancho / alto, 0.05, 10)
        _, _, rgba, _, _ = p.getCameraImage(ancho, alto, vista, proy,
                                            renderer=p.ER_TINY_RENDERER,
                                            shadow=0, lightDirection=(0.4, -0.5, 1))
        return png(rgba, ancho, alto)


def png(rgba, ancho, alto):
    """Codifica una imagen RGBA en PNG (sin librerias de imagenes)."""
    if isinstance(rgba, (bytes, bytearray)):
        datos = np.frombuffer(rgba, dtype=np.uint8)
    else:
        datos = np.asarray(rgba, dtype=np.uint8)
    datos = datos.reshape(alto, ancho * 4)
    filas = b"".join(b"\x00" + datos[f].tobytes() for f in range(alto))

    def bloque(tipo, datos):
        return (struct.pack(">I", len(datos)) + tipo + datos +
                struct.pack(">I", zlib.crc32(tipo + datos) & 0xFFFFFFFF))
    return (b"\x89PNG\r\n\x1a\n" +
            bloque(b"IHDR", struct.pack(">IIBBBBB", ancho, alto, 8, 6, 0, 0, 0)) +
            bloque(b"IDAT", zlib.compress(filas, 3)) + bloque(b"IEND", b""))


# =====================================================================
#  Visor web
# =====================================================================
PAGINA = r"""<!doctype html><html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Gemelo digital - Enjambre ACO</title>
<style>
body{margin:0;font-family:system-ui,sans-serif;background:#f4f5f7;color:#1d2330}
header{padding:12px 16px;background:#1d2330;color:#fff;display:flex;gap:12px;align-items:center;flex-wrap:wrap}
header b{font-size:18px}#modo{padding:3px 10px;border-radius:12px;font-size:13px}
main{display:flex;gap:16px;padding:16px;flex-wrap:wrap}
.panel{background:#fff;border-radius:10px;padding:12px;box-shadow:0 1px 3px #0002}
img,canvas{max-width:100%;display:block;border-radius:6px}
button{padding:8px 14px;border:0;border-radius:6px;font-size:14px;cursor:pointer;margin:2px}
.ini{background:#1f9d55;color:#fff}.rst{background:#555;color:#fff}.obs{background:#e67e22;color:#fff}
table{border-collapse:collapse;width:100%;font-size:14px}td,th{padding:6px;border-bottom:1px solid #eee;text-align:left}
.dot{display:inline-block;width:12px;height:12px;border-radius:50%;margin-right:6px}
small{color:#667}
</style></head><body>
<header><b>Enjambre de 3 carritos con ACO</b><span id="modo">...</span><span id="info"></span>
<span style="flex:1"></span>
<button class="ini" onclick="orden({c:'inicio'})">Iniciar</button>
<button class="rst" onclick="orden({c:'reset'})">Reiniciar</button></header>
<main>
<div class="panel"><div><b>PyBullet</b> <small>vista 3D</small></div><img id="foto" width="640" height="480"></div>
<div class="panel" style="flex:1;min-width:300px">
<div><b>Mapa de feromonas</b> <small>naranja = mas feromona, verde = mejor ruta, rojo = bloqueado</small></div>
<canvas id="mapa" width="420" height="340"></canvas>
<table id="tabla"><tr><th>Carro</th><th>Fase</th><th>Iter.</th><th>Ruta</th><th>Velocidad</th><th></th></tr></table>
<small>Velocidad y Obstaculo se envian por Wi-Fi a la ESP32-S3 de cada carro.</small>
</div></main>
<script>
const COL={1:'#d92626',2:'#2659e6',3:'#1aa640'};let mapa=null;
async function orden(o){o.t='cmd';await fetch('/cmd',{method:'POST',body:JSON.stringify(o)})}
function fila(i){return `<tr id="c${i}"><td><span class="dot" style="background:${COL[i]}"></span>${i}</td>
<td class="f">-</td><td class="it">-</td><td class="r">-</td>
<td><input type="range" min="0.3" max="2" step="0.1" value="1" oninput="this.nextElementSibling.textContent=this.value+'x'"
 onchange="orden({c:'vel',id:${i},k:parseFloat(this.value)})"><span>1x</span></td>
<td><button class="obs" onclick="orden({c:'obst',id:${i}})">Obstaculo</button></td></tr>`}
document.getElementById('tabla').insertAdjacentHTML('beforeend',[1,2,3].map(fila).join(''));
function dibujar(est){const c=document.getElementById('mapa'),g=c.getContext('2d');
 g.clearRect(0,0,c.width,c.height);if(!mapa)return;
 const s=60,ox=40,oy=c.height-50,X=n=>ox+mapa.pos[n][0]/mapa.paso*s,Y=n=>oy-mapa.pos[n][1]/mapa.paso*s;
 let tau=null,best=[],blq=new Set();for(const i of [1,2,3]){const e=est[i];if(!e)continue;
  (e.blq||[]).forEach(b=>blq.add(b));if(!tau&&e.tau)tau=e.tau;if(!best.length&&e.best)best=e.best}
 const tm=tau?Math.max(...tau):1,enRuta=new Set();
 for(let k=0;k+1<best.length;k++)enRuta.add(mapa.tramo[best[k]+'-'+best[k+1]]);
 mapa.tramos.forEach(([a,b],t)=>{let k=tau?tau[t]/tm:0;g.lineWidth=3+10*k;
  g.strokeStyle=blq.has(t)?'#e02020':`rgba(240,${150-60*k|0},20,${0.15+0.85*k})`;
  g.beginPath();g.moveTo(X(a),Y(a));g.lineTo(X(b),Y(b));g.stroke();
  if(enRuta.has(t)&&!blq.has(t)){g.lineWidth=3;g.strokeStyle='#18b04a';g.stroke()}});
 mapa.pos.forEach((q,n)=>{g.fillStyle=n==mapa.inicio?'#1aa640':n==mapa.meta?'#d92626':'#333';
  g.beginPath();g.arc(X(n),Y(n),n==mapa.inicio||n==mapa.meta?8:3,0,7);g.fill()});
 g.fillStyle='#000';g.font='bold 13px sans-serif';g.fillText('A',X(mapa.inicio)-18,Y(mapa.inicio)+5);
 g.fillText('META',X(mapa.meta)+10,Y(mapa.meta)+5);
 for(const i of [1,2,3]){const e=est[i];if(!e||e.f!='RUTA'||e.c>0)continue;
  const x=X(e.u)+(X(e.o)-X(e.u))*e.x,y=Y(e.u)+(Y(e.o)-Y(e.u))*e.x;
  g.fillStyle=COL[i];g.beginPath();g.arc(x,y,9,0,7);g.fill();g.fillStyle='#fff';g.fillText(i,x-4,y+5)}}
async function ciclo(){try{const r=await fetch('/estado'),d=await r.json();
 const m=document.getElementById('modo');m.textContent=d.modo+(d.modo=='REAL'?(d.conectado?' (conectado)':': sin datos de la central'):'');
 m.style.background=d.conectado?'#1f9d55':'#c0392b';
 document.getElementById('info').textContent=d.mensajes+' mensajes';
 for(const i of [1,2,3]){const e=d.carros[i],tr=document.getElementById('c'+i);
  tr.querySelector('.f').textContent=e?e.f+(e.w?' (esperando)':'')+(e.f=='META'?' #'+e.p:''):'sin datos';
  tr.querySelector('.it').textContent=e?e.it:'-';
  tr.querySelector('.r').textContent=e&&e.best?(e.best.length-1)+' tramos':'-';
  const s=tr.querySelector('input');if(e&&document.activeElement!==s){s.value=e.k;s.nextElementSibling.textContent=e.k+'x'}}
 dibujar(d.carros)}catch(x){}setTimeout(ciclo,250)}
setInterval(()=>{document.getElementById('foto').src='/foto.png?'+Date.now()},250);
fetch('/mapa').then(r=>r.json()).then(m=>{mapa=m;ciclo()});
</script></body></html>"""


def servidor(fuente, foto, puerto):
    tramo = {}
    for t, (a, b) in enumerate(L.TRAMOS):
        tramo["%d-%d" % (a, b)] = t
        tramo["%d-%d" % (b, a)] = t
    mapa = json.dumps({"pos": L.POS, "tramos": L.TRAMOS, "tramo": tramo, "paso": L.PASO,
                       "inicio": L.INICIO, "meta": L.META}).encode()

    class Manejador(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _enviar(self, datos, tipo):
            self.send_response(200)
            self.send_header("Content-Type", tipo)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(datos)

        def do_GET(self):
            ruta = self.path.split("?")[0]
            if ruta == "/":
                self._enviar(PAGINA.encode(), "text/html; charset=utf-8")
            elif ruta == "/mapa":
                self._enviar(mapa, "application/json")
            elif ruta == "/foto.png":
                self._enviar(foto["png"], "image/png")
            elif ruta == "/estado":
                with fuente.lock:
                    d = {"modo": fuente.modo, "conectado": fuente.conectado(),
                         "mensajes": fuente.mensajes, "carros": dict(fuente.estados)}
                self._enviar(json.dumps(d).encode(), "application/json")
            else:
                self.send_error(404)

        def do_POST(self):
            if self.path != "/cmd":
                return self.send_error(404)
            largo = int(self.headers.get("Content-Length", 0))
            try:
                msg = json.loads(self.rfile.read(largo))
            except ValueError:
                return self.send_error(400)
            msg["t"] = "cmd"
            fuente.orden(msg)
            self._enviar(b"{}", "application/json")

    srv = ThreadingHTTPServer(("0.0.0.0", puerto), Manejador)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def main():
    ap = argparse.ArgumentParser(description="Gemelo digital del enjambre ACO")
    ap.add_argument("--simular", action="store_true", help="sin placas")
    ap.add_argument("--central", default=os.environ.get("CENTRAL", "192.168.4.1"),
                    help="IP de la placa 1 (modo AP)")
    ap.add_argument("--puerto", type=int, default=int(os.environ.get("PUERTO_WEB", 8080)))
    ap.add_argument("--gui", action="store_true", help="abrir ventana de PyBullet")
    a = ap.parse_args()
    simular = a.simular or os.environ.get("MODO", "").lower() == "sim"

    fuente = FuenteSimulada() if simular else FuenteReal(a.central)
    escena = Escena(gui=a.gui)
    foto = {"png": png(b"\xff" * 4, 1, 1)}
    servidor(fuente, foto, a.puerto)
    print("Gemelo digital en modo", fuente.modo)
    if not simular:
        print("Central (placa 1):", a.central, "puerto UDP", PUERTO_UDP)
    print("Abre http://localhost:%d" % a.puerto)

    while True:
        with fuente.lock:
            estados = json.loads(json.dumps(fuente.estados))
        escena.actualizar(estados)
        foto["png"] = escena.foto()
        time.sleep(0.1)


if __name__ == "__main__":
    main()
