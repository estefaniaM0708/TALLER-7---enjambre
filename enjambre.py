# enjambre.py  -  "Cerebro" de un carrito del enjambre.
# Corre igual en cada ESP32-S3 (MicroPython) y en la simulacion del PC.
#
# El carrito NO existe fisicamente: esta clase decide la fase, la ruta,
# cuando gira y cuanto avanza. Envia su estado por Wi-Fi y PyBullet lo dibuja.
#
# Fases:
#   ESPERA -inicio-> ACO -converge-> LISTO -todos listos-> COLA -su turno-> RUTA -> META
#   (BLOQUEADO si los obstaculos cortan todos los caminos a la meta)

import laberinto as L
from aco import ACO

VELOCIDAD = 0.15      # m/s del carro virtual con velocidad 1.0x
GIRO = 180.0          # grados por segundo al girar
T_ITERACION = 0.25    # segundos entre iteraciones del ACO (para verlo en el visor)
CARRIL = 0.25         # separacion entre carros en el carril de salida (m)
AUSENTE = 3.0         # s sin mensajes para considerar que una placa no esta

DESPUES = ("LISTO", "COLA", "RUTA", "META", "BLOQUEADO")


def rumbo(a, b):
    """Rumbo en grados para ir del nodo a al nodo b (0 = este, 90 = norte)."""
    (xa, ya), (xb, yb) = L.NODOS[a], L.NODOS[b]
    if xb > xa:
        return 0
    if yb > ya:
        return 90
    if xb < xa:
        return 180
    return 270


class Carro:
    def __init__(self, ident, enviar):
        self.id = ident
        self.enviar = enviar        # funcion que manda un dict por la red
        self.aco = ACO()
        self.k = 1.0                # factor de velocidad (0.3 a 2.0)
        self.otros = {}             # id -> ultimo estado recibido
        self.visto = {}             # id -> instante del ultimo mensaje
        self.eventos = []           # avisos para el programa principal
        self.reiniciar()

    # ======================================================== estado
    def reiniciar(self):
        self.aco.reiniciar()
        self.fase = "ESPERA"
        self.nodo = L.INICIO        # ultimo nodo alcanzado
        self.destino = L.INICIO     # nodo hacia el que avanza
        self.x = 0.0                # avance dentro del tramo (0 a 1)
        self.h = 90.0               # rumbo actual en grados
        self.h_obj = 90.0           # rumbo al que esta girando
        self.carril = (self.id - 1) * CARRIL
        self.ruta = None            # ruta acordada (lista de nodos)
        self.i = 0                  # posicion en la ruta
        self.recorridos = 0         # nodos recorridos
        self.puesto = 0             # orden de llegada a la meta
        self.t_aco = 0.0
        self.t_listo = 0.0
        self.replanificar_al_llegar = False
        self.esperando = False
        self.otros = {}

    def presentes(self, ahora):
        ids = [i for i, t in self.visto.items() if ahora - t < AUSENTE]
        if self.id not in ids:
            ids.append(self.id)
        return sorted(ids)

    def estado(self):
        e = {"t": "est", "id": self.id, "f": self.fase, "u": self.nodo,
             "o": self.destino, "x": round(self.x, 3), "h": round(self.h, 1),
             "k": round(self.k, 2), "c": round(self.carril, 3),
             "p": self.puesto, "n": self.recorridos, "it": self.aco.iteraciones,
             "w": 1 if self.esperando else 0,
             "tau": [round(t, 2) for t in self.aco.tau],
             "blq": sorted(self.aco.bloqueados)}
        camino = self.ruta if self.ruta else self.aco.mejor
        if camino:
            e["best"] = camino
        return e

    # ======================================================== mensajes
    def recibir(self, m, ahora):
        t = m.get("t")
        quien = m.get("id")
        if t != "cmd" and quien is not None and quien != self.id:
            self.visto[quien] = ahora

        if t == "cmd":
            c = m.get("c")
            if c == "inicio":
                self.iniciar(ahora)
            elif c == "reset":
                self.reiniciar()
                self.eventos.append("reset")
            elif c == "obst" and quien == self.id:
                self.obstaculo()
            elif c == "vel" and quien == self.id:
                self.k = min(2.0, max(0.3, float(m.get("k", 1.0))))

        elif t == "fer" and quien != self.id:
            if self.fase == "ESPERA":          # otra placa ya empezo
                self.iniciar(ahora)
            if self.fase == "ACO":
                self.aco.sumar(m.get("dep", []))
                self.eventos.append("feromona")

        elif t == "est" and quien != self.id:
            self.otros[quien] = m
            if self.fase == "ESPERA" and m.get("f") == "ACO":
                self.iniciar(ahora)

        elif t == "blq" and quien != self.id:
            e = L.tramo(m["a"], m["b"])
            if e is not None and e not in self.aco.bloqueados:
                self.aco.bloquear(e)
                self._tras_bloqueo(e)

    def iniciar(self, ahora):
        if self.fase == "ESPERA":
            self.fase = "ACO"
            self.t_aco = ahora
            self.eventos.append("inicio")

    # ======================================================== obstaculos
    def obstaculo(self):
        """Simula que el carro encontro algo en el tramo por el que va."""
        if self.fase != "RUTA":
            return
        if self.destino == self.nodo:          # esta parado en un nodo
            if self.i + 1 >= len(self.ruta):
                return
            a, b = self.nodo, self.ruta[self.i + 1]
        else:
            a, b = self.nodo, self.destino
        e = L.tramo(a, b)
        self.aco.bloquear(e)
        self.enviar({"t": "blq", "id": self.id, "a": a, "b": b})
        self.eventos.append("obstaculo")
        self._tras_bloqueo(e)

    def _tras_bloqueo(self, e):
        if self.fase in ("LISTO", "COLA") and self.ruta and \
                e in ACO.tramos_de(self.ruta):
            self.ruta = self.aco.replanificar(L.INICIO)
            self.i = 0
            if self.ruta is None:
                self.fase = "BLOQUEADO"
        elif self.fase == "LISTO" and self.aco.mejor is None:
            self.aco.replanificar(L.INICIO)
        elif self.fase == "RUTA":
            actual = L.tramo(self.nodo, self.destino) if self.destino != self.nodo else None
            if actual == e:                       # va por el tramo bloqueado
                self.nodo, self.destino = self.destino, self.nodo
                self.x = 1.0 - self.x
                self.h_obj = (self.h + 180.0) % 360.0
                self.replanificar_al_llegar = True
            elif e in ACO.tramos_de(self.ruta[self.i:]):
                self.replanificar_al_llegar = True
                if self.destino == self.nodo:
                    self._replanificar(self.nodo)

    def _replanificar(self, desde):
        self.replanificar_al_llegar = False
        nueva = self.aco.replanificar(desde)
        if nueva is None:
            self.fase = "BLOQUEADO"
            self.ruta = None
            self.eventos.append("sin_camino")
            return
        self.ruta = nueva
        self.i = 0
        self.destino = self.nodo

    # ======================================================== tiempo
    def paso(self, dt, ahora):
        """Llamar seguido (cada 20-50 ms)."""
        if self.fase == "ACO":
            if ahora - self.t_aco >= T_ITERACION:
                self.t_aco = ahora
                dep = self.aco.iteracion()
                self.enviar({"t": "fer", "id": self.id, "dep": dep})
                if self.aco.convergio():
                    self.fase = "LISTO"
                    self.t_listo = ahora
                    self.eventos.append("listo")

        elif self.fase == "LISTO":
            self._acordar(ahora)

        elif self.fase == "COLA":
            if self._mi_turno(ahora):
                self.fase = "RUTA"
                self.eventos.append("salida")

        elif self.fase == "RUTA":
            self._mover(dt)

    def _acordar(self, ahora):
        """Cuando todas las placas presentes terminaron, adoptan la ruta mas corta."""
        rutas = {self.id: self.aco.mejor}
        for i in self.presentes(ahora):
            if i == self.id:
                continue
            o = self.otros.get(i)
            if not o or o.get("f") not in DESPUES:
                return
            rutas[i] = o.get("best")
        candidatas = [(len(r), i, r) for i, r in rutas.items() if r]
        if not candidatas:
            self.fase = "BLOQUEADO"
            return
        candidatas.sort()
        self.ruta = list(candidatas[0][2])
        self.i = 0
        self.fase = "COLA"

    def _mi_turno(self, ahora):
        anteriores = [i for i in self.presentes(ahora) if i < self.id]
        if not anteriores:
            return True
        o = self.otros.get(max(anteriores))
        if not o:
            return False
        if o.get("f") in ("META", "BLOQUEADO"):
            return True
        return o.get("f") == "RUTA" and o.get("n", 0) >= 2

    def _ocupado(self, n):
        """Hay un carro de numero menor en el nodo n o yendo hacia el?"""
        for i, o in self.otros.items():
            if i < self.id and o.get("f") == "RUTA" and \
                    (o.get("u") == n or o.get("o") == n):
                return True
        return False

    def _mover(self, dt):
        v = VELOCIDAD * self.k
        # 1) Del carril de salida hasta A
        if self.carril > 0:
            self.carril = max(0.0, self.carril - v * dt)
            return
        # 2) Girando
        if abs(self.h - self.h_obj) > 0.5:
            dif = (self.h_obj - self.h + 540.0) % 360.0 - 180.0
            g = GIRO * self.k * dt
            self.h = self.h_obj if abs(dif) <= g else (self.h + (g if dif > 0 else -g)) % 360.0
            return
        # 3) Parado en un nodo: elegir el siguiente tramo
        if self.destino == self.nodo:
            if self.nodo == L.META:
                self._llegar()
                return
            sig = self.ruta[self.i + 1]
            self.esperando = self._ocupado(sig)
            if self.esperando:
                return
            self.destino = sig
            self.x = 0.0
            self.h_obj = float(rumbo(self.nodo, sig))
            return
        # 4) Avanzando por el tramo
        self.x += v * dt / L.PASO
        if self.x >= 1.0:
            self.x = 0.0
            self.nodo = self.destino
            self.recorridos += 1
            self.eventos.append("nodo")
            if self.replanificar_al_llegar:
                self._replanificar(self.nodo)
            else:
                self.i += 1

    def _llegar(self):
        self.puesto = 1 + sum(1 for o in self.otros.values() if o.get("f") == "META")
        self.fase = "META"
        self.eventos.append("meta")
