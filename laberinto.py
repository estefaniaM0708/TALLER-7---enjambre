"""Laberinto del almacén, leído de comun/laberinto.txt.

Numera las celdas libres igual que el firmware (aco.cpp): en orden de filas, de 0 a N-1.
Los pasillos (aristas) se numeran como en Colonia::exportarFeromona: para cada celda, primero
el vecino del este y luego el del sur.
"""

import os
import zlib

CELDA = 0.25          # metros por celda

_aqui = os.path.dirname(os.path.abspath(__file__))
_rutas = [os.path.join(_aqui, "..", "comun", "laberinto.txt"),
          os.path.join(_aqui, "comun", "laberinto.txt")]
ARCHIVO = next((r for r in _rutas if os.path.exists(r)), _rutas[0])

with open(ARCHIVO, encoding="utf-8") as _f:
    MAPA = [l.rstrip("\n") for l in _f if l.strip()]

FILAS, COLUMNAS = len(MAPA), len(MAPA[0])
CRC = zlib.crc32("".join(MAPA).encode()) & 0xFFFFFFFF

CELDAS = []           # CELDAS[i] = (fila, columna)
ID = {}               # (fila, columna) -> i
for _f, _fila in enumerate(MAPA):
    for _c, _ch in enumerate(_fila):
        if _ch != "#":
            ID[(_f, _c)] = len(CELDAS)
            CELDAS.append((_f, _c))
            if _ch == "A":
                INICIO = ID[(_f, _c)]
            elif _ch == "M":
                META = ID[(_f, _c)]

ARISTAS = []          # ARISTAS[k] = (celda_a, celda_b)
for _i, (_f, _c) in enumerate(CELDAS):
    for _df, _dc in ((0, 1), (1, 0)):
        _v = ID.get((_f + _df, _c + _dc))
        if _v is not None:
            ARISTAS.append((_i, _v))


def posicion(celda):
    """Centro de la celda en metros. x hacia la derecha (columnas), y hacia arriba (filas)."""
    f, c = CELDAS[celda]
    return (c * CELDA, (FILAS - 1 - f) * CELDA)


def es_pared(f, c):
    return MAPA[f][c] == "#"
