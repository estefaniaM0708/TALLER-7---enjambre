# red.py  -  Wi-Fi y mensajes UDP entre las ESP32-S3 y el PC
#
# Placa 1 (CENTRAL): crea la red en MODO AP y reenvia cada mensaje que le
#                    llega a todos los demas (placas 2 y 3 y el gemelo del PC).
# Placas 2 y 3:      se conectan a esa red como estaciones y le envian todo
#                    a la central.

import json
import socket
import time
import network
import config


class Red:
    def __init__(self, ident):
        self.id = ident
        self.central = ident == 1
        self.clientes = {}            # direccion -> instante del ultimo mensaje
        if self.central:
            self.wlan = network.WLAN(network.AP_IF)
            self.wlan.active(True)
            self.wlan.config(essid=config.AP_SSID, password=config.AP_CLAVE,
                             authmode=network.AUTH_WPA_WPA2_PSK)
            while not self.wlan.active():
                time.sleep_ms(100)
            print("Modo AP: red", config.AP_SSID, "IP", self.wlan.ifconfig()[0])
        else:
            self.wlan = network.WLAN(network.STA_IF)
            self.wlan.active(True)
            self.conectar()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(socket.getaddrinfo("0.0.0.0", config.PUERTO)[0][-1])
        self.sock.setblocking(False)
        self.destino_central = socket.getaddrinfo(config.IP_CENTRAL, config.PUERTO)[0][-1]

    def conectar(self):
        if self.wlan.isconnected():
            return
        print("Buscando la red", config.AP_SSID, "...")
        self.wlan.connect(config.AP_SSID, config.AP_CLAVE)
        while not self.wlan.isconnected():
            time.sleep_ms(250)
        print("Conectada. IP", self.wlan.ifconfig()[0])

    def ok(self):
        return self.central or self.wlan.isconnected()

    def _mandar(self, datos, destino):
        try:
            self.sock.sendto(datos, destino)
        except OSError:
            pass

    def enviar(self, msg, excepto=None):
        """Envia un dict a todo el enjambre (y al gemelo digital)."""
        datos = json.dumps(msg).encode()
        if self.central:
            ahora = time.ticks_ms()
            for dest in list(self.clientes):
                if time.ticks_diff(ahora, self.clientes[dest]) > 10000:
                    del self.clientes[dest]          # cliente que ya no esta
                elif dest != excepto:
                    self._mandar(datos, dest)
        else:
            self._mandar(datos, self.destino_central)

    def recibir(self):
        """Devuelve la lista de mensajes que llegaron (sin bloquear)."""
        mensajes = []
        while True:
            try:
                datos, origen = self.sock.recvfrom(1500)
            except OSError:
                break
            try:
                msg = json.loads(datos)
            except ValueError:
                continue
            if self.central:
                self.clientes[origen] = time.ticks_ms()
                if msg.get("t") != "hola":
                    self.enviar(msg, excepto=origen)   # reenvio al resto
            if msg.get("t") != "hola":
                mensajes.append(msg)
        return mensajes
