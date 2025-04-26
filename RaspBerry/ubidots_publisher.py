import sqlite3
import json
import time
import paho.mqtt.client as mqtt
from datetime import datetime

# Configuración de MQTT para Ubidots
MQTT_BROKER = "industrial.api.ubidots.com"
MQTT_PORT = 1883
UBIDOTS_TOPIC = "/v1.6/devices/fire_detector"
UBIDOTS_CONTROL_TOPIC = "/v1.6/devices/fire_detector/alarm_control/lv"  # Suscripción al último valor
UBIDOTS_TOKEN = "BBUS-PykenUXEwddQVrPFGjL0arYsHK4fYO"  # Reemplazar con tu token de Ubidots

# Configuración de MQTT para el ESP32
ESP32_BROKER = "localhost"  # Broker local en el Raspberry Pi
ESP32_TOPIC_CONTROL = "fire_detection/control"

DB_NAME = "fire_detection.sqlite3"

# Variable para rastrear el último timestamp publicado
last_published_timestamp = 0

# Callback de conexión a Ubidots
def on_connect(client, userdata, flags, rc):
    print(f"Conectado a Ubidots con código: {rc}")
    client.subscribe(UBIDOTS_CONTROL_TOPIC)
    print(f"Suscrito a {UBIDOTS_CONTROL_TOPIC} en Ubidots")

# Publicar a Ubidots
def publish_to_ubidots(client, data):
    payload = {
        "temperature": {"value": data["temp"]},
        "humidity": {"value": data["humi"]},
        "gas_level": {"value": data["co"]},
        "status": {"value": 1 if data["status"] == "ALERTA INCENDIO!" else 0}
    }
    result = client.publish(UBIDOTS_TOPIC, json.dumps(payload))
    if result.rc == mqtt.MQTT_ERR_SUCCESS:
        print(f"Datos publicados a Ubidots: {payload}")
    else:
        print("Error al publicar a Ubidots")

# Leer datos de la base de datos
def read_and_publish_data():
    global last_published_timestamp
    try:
        conn = sqlite3.connect(DB_NAME)
        c = conn.cursor()
        c.execute("SELECT * FROM sensor_data WHERE timestamp > ? ORDER BY timestamp DESC LIMIT 1", (last_published_timestamp,))
        row = c.fetchone()
        conn.close()
        
        if row:
            data = {
                "timestamp": row[0],  # timestamp en milisegundos
                "temp": row[1],
                "humi": row[2],
                "co": row[3],
                "status": row[4]
            }
            last_published_timestamp = data["timestamp"]
            return data
        return None
    except sqlite3.Error as e:
        print(f"Error al leer de la base de datos: {e}")
        return None

# Callback para manejar mensajes de Ubidots
def on_message(client, userdata, msg):
    print(f"Mensaje recibido de Ubidots: {msg.payload.decode()}")
    try:
        value = int(float(msg.payload.decode()))  # Convertir el mensaje a entero (0 o 1)
        send_to_esp32(value)
    except ValueError:
        print("Error: Valor recibido no es un entero válido.")

# Enviar valor al ESP32
def send_to_esp32(value):
    esp32_client = mqtt.Client()
    try:
        esp32_client.connect(ESP32_BROKER, MQTT_PORT, 60)
        result = esp32_client.publish(ESP32_TOPIC_CONTROL, str(value))
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            print(f"Valor enviado al ESP32: {value}")
        else:
            print("Error al enviar valor al ESP32")
        esp32_client.disconnect()
    except Exception as e:
        print(f"Error al conectar al broker local: {e}")

# Configurar cliente MQTT para Ubidots
client = mqtt.Client()
client.username_pw_set(UBIDOTS_TOKEN)
client.on_connect = on_connect
client.on_message = on_message

# Conectar al broker de Ubidots
try:
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
except Exception as e:
    print(f"Error al conectar a Ubidots: {e}")
    exit(1)

# Iniciar el bucle de MQTT en segundo plano
client.loop_start()

# Publicar datos periódicamente
while True:
    data = read_and_publish_data()
    if data:
        publish_to_ubidots(client, data)
    time.sleep(10)  # Publicar cada 10 segundos