import sqlite3
import json
import time
from datetime import datetime
import paho.mqtt.client as mqtt

# Configuración de MQTT para Ubidots
MQTT_BROKER = "industrial.api.ubidots.com"
MQTT_PORT = 1883
UBIDOTS_TOPIC = "/v1.6/devices/detector_fuego"
UBIDOTS_CONTROL_TOPIC = "/v1.6/devices/detector_fuego/control_alarm/lv"  # Suscripción al último valor
UBIDOTS_TOKEN = "BBUS-PykenUXEwddQVrPFGjL0arYsHK4fYO"  # Reemplazar con tu token de Ubidots

# Configuración de MQTT para el ESP32
ESP32_BROKER = "localhost"  # Broker local en el Raspberry Pi
ESP32_TOPIC_CONTROL = "fire_detection/control"

DB_NAME = "fire_detection.sqlite3"

# Publicar a Ubidots
def publish_to_ubidots(client, data):
    payload = {
        "temperature": {"value": data["temp"]},
        "humidity": {"value": data["humi"]},
        "gas_level": {"value": data["co"]},
        "status": {"value": data["status"]}
    }
    client.publish(UBIDOTS_TOPIC, json.dumps(payload))

# Leer datos de la base de datos
def read_and_publish_data():
    try:
        conn = sqlite3.connect(DB_NAME)
        c = conn.cursor()
        c.execute("SELECT * FROM sensor_data ORDER BY timestamp DESC LIMIT 1")
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
            return data
        return None
    except sqlite3.Error as e:
        print(f"Error al leer de la base de datos: {e}")
        return None

# Callback para manejar mensajes de Ubidots
def on_message(client, userdata, msg):
    print(f"Mensaje recibido de Ubidots: {msg.payload.decode()}")
    try:
        # Convertir el mensaje recibido a un número flotante
        value = float(msg.payload.decode())
        # Convertir el valor flotante a entero (0 o 1)
        value = int(value)
        send_to_esp32(value)
    except ValueError as e:
        print(f"Error al procesar el mensaje: {e}")

# Enviar valor al ESP32
def send_to_esp32(value):
    esp32_client = mqtt.Client()
    esp32_client.connect(ESP32_BROKER, MQTT_PORT, 60)
    esp32_client.publish(ESP32_TOPIC_CONTROL, str(value))
    esp32_client.disconnect()
    print(f"Valor enviado al ESP32: {value}")

# Configurar cliente MQTT para Ubidots
client = mqtt.Client()
client.username_pw_set(UBIDOTS_TOKEN)
client.on_message = on_message
client.connect(MQTT_BROKER, MQTT_PORT, 60)

# Suscribirse al control de alarma en Ubidots
client.subscribe(UBIDOTS_CONTROL_TOPIC)
print(f"Suscrito a {UBIDOTS_CONTROL_TOPIC} en Ubidots")

# Mantener el cliente activo
client.loop_forever()

# Publicar datos periódicamente
while True:
    data = read_and_publish_data()
    if data:
        publish_to_ubidots(client, data)
        print("Datos enviados")
    time.sleep(10)  # Publicar cada 10 segundos
    client.loop()  # Procesar mensajes pendientes

