# laberinto.py  -  Mapa del laberinto/almacen (igual en las 3 ESP32-S3 y en el PC)
#
#   o = nodo (cruce)   A = inicio   M = meta
#   - y | = tramo por donde se puede pasar     espacio = pared
#   Los nodos estan separados 30 cm.

MAPA = (
    "o-o-o o-o-M",
    "|   | |   |",
    "o-o o-o o-o",
    "| | |     |",
    "o o-o-o-o o",
    "|   |   | |",
    "o-o-o o-o-o",
    "|     | |  ",
    "A-o-o-o o-o",
)

PASO = 0.30          # metros entre nodos

NODOS = []           # NODOS[n] = (columna, fila)   fila 0 = abajo
POS = []             # POS[n] = (x, y) en metros
TRAMOS = []          # TRAMOS[e] = (a, b) con a < b
VECINOS = []         # VECINOS[n] = [(vecino, tramo), ...]
INICIO = 0
META = 0

_FILAS = len(MAPA)


def _leer():
    global INICIO, META
    indice = {}
    for f in range(0, _FILAS, 2):
        linea = MAPA[f]
        for c in range(0, len(linea), 2):
            ch = linea[c]
            if ch in "oAM":
                n = len(NODOS)
                col, fila = c // 2, (_FILAS - 1 - f) // 2
                indice[(c, f)] = n
                NODOS.append((col, fila))
                POS.append((col * PASO, fila * PASO))
                VECINOS.append([])
                if ch == "A":
                    INICIO = n
                elif ch == "M":
                    META = n

    def unir(p, q):
        a, b = indice[p], indice[q]
        if a > b:
            a, b = b, a
        e = len(TRAMOS)
        TRAMOS.append((a, b))
        VECINOS[a].append((b, e))
        VECINOS[b].append((a, e))

    for (c, f) in list(indice.keys()):
        linea = MAPA[f]
        if c + 1 < len(linea) and linea[c + 1] == "-":
            unir((c, f), (c + 2, f))
        if f + 1 < _FILAS and c < len(MAPA[f + 1]) and MAPA[f + 1][c] == "|":
            unir((c, f), (c, f + 2))


_leer()


def tramo(a, b):
    """Numero de tramo entre los nodos a y b (o None si no hay)."""
    for v, e in VECINOS[a]:
        if v == b:
            return e
    return None


def distancias(destino, bloqueados=()):
    """Distancia en tramos de cada nodo al destino (BFS). -1 = sin camino."""
    d = [-1] * len(NODOS)
    d[destino] = 0
    cola = [destino]
    i = 0
    while i < len(cola):
        n = cola[i]
        i += 1
        for v, e in VECINOS[n]:
            if e not in bloqueados and d[v] < 0:
                d[v] = d[n] + 1
                cola.append(v)
    return d
