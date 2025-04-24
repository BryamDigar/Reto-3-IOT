import paho.mqtt.client as mqtt
import sqlite3
import json
from datetime import datetime

# Configuración de MQTT para el broker local
MQTT_BROKER = "localhost"  # Broker local en el Raspberry Pi
MQTT_PORT = 1883
MQTT_TOPIC_DATA = "fire_detection/data"
MQTT_TOPIC_ALERT = "fire_detection/alert"

# Configuración de SQLite
DB_NAME = "fire_detection.sqlite3"

# Inicializar base de datos
def init_db():
    conn = sqlite3.connect(DB_NAME)
    c = conn.cursor()
    c.execute('''CREATE TABLE IF NOT EXISTS sensor_data
                 (timestamp INTEGER, temp REAL, humi REAL, co INTEGER, status TEXT)''')  # Ajustar formato
    c.execute('''CREATE TABLE IF NOT EXISTS alerts
                 (timestamp INTEGER, message TEXT)''')  # Ajustar formato
    conn.commit()
    conn.close()

# Callback de conexión
def on_connect(client, userdata, flags, rc):
    print(f"Conectado al broker MQTT con código: {rc}")
    client.subscribe(MQTT_TOPIC_DATA)
    client.subscribe(MQTT_TOPIC_ALERT)

# Callback de mensaje
def on_message(client, userdata, msg):
    payload = msg.payload.decode()
    topic = msg.topic
    
    if topic == MQTT_TOPIC_DATA:
        data = json.loads(payload)
        store_data(data)
    elif topic == MQTT_TOPIC_ALERT:
        store_alert(payload)

# Almacenar datos en SQLite
def store_data(data):
    conn = sqlite3.connect(DB_NAME)
    c = conn.cursor()
    c.execute("INSERT INTO sensor_data (timestamp, temp, humi, co, status) VALUES (?, ?, ?, ?, ?)",
              (data["timestamp"], data["temp"], data["humi"], data["co"], data["status"]))  # Guardar con el mismo formato
    conn.commit()
    conn.close()

# Almacenar alertas en SQLite
def store_alert(message):
    conn = sqlite3.connect(DB_NAME)
    c = conn.cursor()
    timestamp = int(datetime.now().timestamp() * 1000)  # Guardar timestamp en milisegundos
    c.execute("INSERT INTO alerts (timestamp, message) VALUES (?, ?)", (timestamp, message))
    conn.commit()
    conn.close()

# Configurar cliente MQTT
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

# Conectar al broker local
client.connect(MQTT_BROKER, MQTT_PORT, 60)

# Inicializar base de datos
init_db()

# Mantener el cliente MQTT activo
client.loop_forever()
