# aco.py  -  Optimizacion por colonia de hormigas (ACO) sobre el laberinto
# Corre igual en MicroPython (ESP32-S3) y en Python del PC.

import random
import laberinto as L


def azar():
    """Numero al azar entre 0 y 1 (funciona en MicroPython y en Python)."""
    return random.getrandbits(24) / 16777216


class ACO:
    def __init__(self, hormigas=4, alfa=1.0, beta=2.0, rho=0.10, q=1.0,
                 tau_min=0.05, tau_max=10.0):
        self.hormigas = hormigas
        self.alfa = alfa          # peso de la feromona
        self.beta = beta          # peso de la heuristica (cercania a la meta)
        self.rho = rho            # evaporacion por iteracion
        self.q = q                # feromona que deja cada hormiga
        self.tau_min = tau_min
        self.tau_max = tau_max
        self.bloqueados = set()
        self.reiniciar()

    def reiniciar(self):
        self.tau = [1.0] * len(L.TRAMOS)
        self.bloqueados = set()
        self.dist = L.distancias(L.META)
        self.mejor = None         # mejor camino (lista de nodos)
        self.iteraciones = 0
        self.sin_cambio = 0

    # ---------------- mapa ----------------
    def bloquear(self, e):
        """Borra el tramo e del mapa (obstaculo)."""
        self.bloqueados.add(e)
        self.tau[e] = 0.0
        self.dist = L.distancias(L.META, self.bloqueados)
        if self.mejor and e in self.tramos_de(self.mejor):
            self.mejor = None

    @staticmethod
    def tramos_de(camino):
        return [L.tramo(camino[i], camino[i + 1]) for i in range(len(camino) - 1)]

    def _eta(self, n):
        d = self.dist[n]
        return 1.0 / (1.0 + d) if d >= 0 else 0.01

    # ---------------- una hormiga ----------------
    def hormiga(self, origen):
        """Camina desde origen hasta la META. Retrocede en callejones."""
        camino = [origen]
        visitados = {origen}
        pasos = 0
        while camino[-1] != L.META and pasos < 300:
            pasos += 1
            n = camino[-1]
            opciones = []
            total = 0.0
            for v, e in L.VECINOS[n]:
                if v in visitados or e in self.bloqueados:
                    continue
                w = (self.tau[e] ** self.alfa) * (self._eta(v) ** self.beta)
                opciones.append((v, w))
                total += w
            if not opciones:            # callejon sin salida: retrocede
                camino.pop()
                if not camino:
                    return None
                continue
            r = azar() * total
            elegido = opciones[-1][0]
            for v, w in opciones:
                r -= w
                if r <= 0:
                    elegido = v
                    break
            camino.append(elegido)
            visitados.add(elegido)
        return camino if camino[-1] == L.META else None

    # ---------------- una iteracion ----------------
    def iteracion(self, origen=None):
        """Evapora, lanza las hormigas y deposita. Devuelve el deposito
        [[tramo, cantidad], ...] para compartirlo con el enjambre."""
        if origen is None:
            origen = L.INICIO
        self.iteraciones += 1
        for e in range(len(self.tau)):
            if e not in self.bloqueados:
                self.tau[e] = max(self.tau_min, self.tau[e] * (1 - self.rho))
        deposito = {}
        cambio = False
        for _ in range(self.hormigas):
            c = self.hormiga(origen)
            if c is None:
                continue
            if self.mejor is None or len(c) < len(self.mejor):
                self.mejor = c
                cambio = True
            cantidad = self.q / (len(c) - 1)
            for e in self.tramos_de(c):
                deposito[e] = deposito.get(e, 0.0) + cantidad
        if self.mejor:                  # refuerzo extra al mejor camino
            extra = self.q / (len(self.mejor) - 1)
            for e in self.tramos_de(self.mejor):
                deposito[e] = deposito.get(e, 0.0) + extra
        self.sin_cambio = 0 if cambio else self.sin_cambio + 1
        dep = [[e, round(c, 4)] for e, c in deposito.items()]
        self.sumar(dep)
        return dep

    def sumar(self, dep):
        """Suma un deposito de feromona (propio o de otra ESP)."""
        for e, c in dep:
            if e not in self.bloqueados:
                self.tau[e] = min(self.tau_max, self.tau[e] + c)

    def convergio(self, minimo=20, estable=12, maximo=60):
        if self.mejor is None:
            return self.iteraciones >= maximo
        return (self.iteraciones >= minimo and self.sin_cambio >= estable) or \
            self.iteraciones >= maximo

    def replanificar(self, origen, iteraciones=25):
        """Nueva ruta desde 'origen' usando la feromona que ya se aprendio."""
        if self.dist[origen] < 0:
            return None
        self.mejor = None
        for _ in range(iteraciones):
            self.iteracion(origen)
        return self.mejor
