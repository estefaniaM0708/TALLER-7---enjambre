# config.py  -  Configuracion de la red (igual en las 3 ESP32-S3)

AP_SSID = "ENJAMBRE_ACO"     # red Wi-Fi que crea la placa 1 en modo AP
AP_CLAVE = "hormigas123"     # minimo 8 caracteres
IP_CENTRAL = "192.168.4.1"   # IP de la placa 1 (la que crea la red)
PUERTO = 4210                # puerto UDP de todos los mensajes

N_CARROS = 3                 # carros del enjambre
T_ESTADO = 0.25              # cada cuanto envia su estado cada placa (s)
