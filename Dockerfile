# Gemelo digital (PyBullet + panel web) en Docker
# Se construye desde la raíz del repositorio:  docker compose build
FROM python:3.11-slim

ENV PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    REGISTROS=/app/registros

WORKDIR /app
COPY gemelo/requirements.txt .
RUN pip install -r requirements.txt

COPY comun/ comun/
COPY gemelo/ .

# 5005/udp: telemetría de las 3 ESP32   8000/tcp: panel web
EXPOSE 5005/udp 8000/tcp
CMD ["python", "gemelo.py"]
