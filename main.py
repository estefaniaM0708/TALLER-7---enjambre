# main.py  -  Programa principal de cada ESP32-S3 (MicroPython)
#
# La placa no lleva nada conectado: solo el cable USB.
#   - Placa 1: crea la red Wi-Fi en modo AP y reenvia los mensajes.
#   - Las 3:   corren el algoritmo de hormigas, comparten feromonas y
#              conducen su carrito virtual en PyBullet.
# Las ordenes (iniciar, reiniciar, obstaculo, velocidad) llegan por Wi-Fi
# desde el visor web del gemelo digital. El boton BOOT de la placa tambien
# sirve para iniciar.

import time
import gc
from machine import Pin
import config
from id_carro import ID
from red import Red
from enjambre import Carro

print("Carro", ID, "- iniciando")
red = Red(ID)
carro = Carro(ID, red.enviar)
boot = Pin(0, Pin.IN, Pin.PULL_UP)

ultimo = time.ticks_ms()
t_estado = 0.0
ahora = 0.0
t_gc = 0.0
boot_antes = 1
fase_antes = ""
print("Carro", ID, "listo. Esperando INICIO desde el visor o el boton BOOT")

while True:
    t = time.ticks_ms()
    dt = time.ticks_diff(t, ultimo) / 1000
    ultimo = t
    ahora += dt

    # 1) Mensajes que llegaron por Wi-Fi
    for msg in red.recibir():
        carro.recibir(msg, ahora)

    # 2) Boton BOOT (el de la placa): iniciar el enjambre
    b = boot.value()
    if b == 0 and boot_antes == 1:
        orden = {"t": "cmd", "c": "inicio"}
        red.enviar(orden)
        carro.recibir(orden, ahora)
    boot_antes = b

    # 3) Algoritmo de hormigas y movimiento del carro virtual
    carro.paso(dt, ahora)

    # 4) Estado al gemelo digital y a las otras placas
    if ahora - t_estado >= config.T_ESTADO:
        t_estado = ahora
        red.enviar(carro.estado())

    # 5) Mensajes en la terminal (mpremote repl)
    if carro.fase != fase_antes:
        print("Fase:", carro.fase, "| ruta:", carro.ruta or carro.aco.mejor)
        fase_antes = carro.fase
    for ev in carro.eventos:
        if ev != "feromona":
            print("Evento:", ev)
    carro.eventos.clear()

    if not red.ok():
        red.conectar()
    if ahora - t_gc >= 1.0:
        t_gc = ahora
        gc.collect()
    time.sleep_ms(20)
