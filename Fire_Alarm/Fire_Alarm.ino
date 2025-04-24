#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MQUnifiedsensor.h>
#include <WiFi.h>
#include "WebServer.h"
#include "WebPage.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Definición de pines y constantes
#define DHTPIN 4
#define DHTTYPE DHT11
#define LED_PIN 27
#define BUZZER_PIN 26
#define FLAME_PIN 25
#define Board "ESP-32"
#define Pin 34  // Usamos un pin ADC adecuado para el ESP32

#define Type "MQ-2"
#define Voltage_Resolution 3.3
#define ADC_Bit_Resolution 12 
#define RatioMQ2CleanAir 9.83

#define RED_PIN 5
#define GREEN_PIN 18
#define BLUE_PIN 19

#define TEMP_LOW 8.4
#define TEMP_HIGH 13
#define HUMI_LOW 75
#define HUMI_HIGH 80
#define FIRE_THRESHOLD LOW

#define I2C_ADDR 0x27
#define LCD_COLUMNS 16
#define LCD_LINES 2

#define HISTORY_SIZE 60 // Guardar 60 lecturas (5 minutos con lecturas cada 5 segundos)

// Inicialización del servidor web
WebServer server(HTTP_PORT);

const char* mqtt_server = "192.168.178.242"; // Dirección IP del Raspberry Pi con Mosquitto
const int mqtt_port = 1883;
const char* mqtt_client_id = "ESP32_Fire_Detector";
const char* topic_data = "fire_detection/data";
const char* topic_alert = "fire_detection/alert";
const char* topic_control = "fire_detection/control";

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLUMNS, LCD_LINES);
MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

// Variables para almacenar lecturas de sensores
const int numReadings = 10;
float tempReadings[numReadings] = {0};
float humiReadings[numReadings] = {0};
int coReadings[numReadings] = {0};
int index_gas = 0;

QueueHandle_t sensorDataQueue;
QueueHandle_t mqttQueue; // Cola para controlar el procesamiento MQTT

volatile bool alarmTriggered = false;
volatile bool dataReady = false;

// Variables para histórico de datos
float tempHistory[HISTORY_SIZE] = {0};
float humiHistory[HISTORY_SIZE] = {0};
int coHistory[HISTORY_SIZE] = {0};
String statusHistory[HISTORY_SIZE] = {""};
unsigned long timestampHistory[HISTORY_SIZE] = {0};
int historyIndex = 0;
unsigned long lastHistoryUpdate = 0;
const unsigned long historyInterval = 5000; // Actualizar cada 5 segundos

// Variables para notificaciones y alarmas
volatile bool alarmActive = false;
String notifications[5] = {"", "", "", "", ""};
int notificationCount = 0;

struct SensorData {
  float temp;
  float humi;
  int co;
};

WiFiClient espClient;
PubSubClient client(espClient);

SensorData sensorData;

byte Alert0[8] = {0b00001, 0b00010, 0b00101, 0b00101, 0b01000, 0b10001, 0b10000, 0b01111};
byte Alert1[8] = {0b00000, 0b10000, 0b01000, 0b01000, 0b00100, 0b00010, 0b00010, 0b11100};

void IRAM_ATTR triggerAlarm() {
    alarmTriggered = true;
    publishAlert("Alerta Activada");
}

// Publicar datos vía MQTT
void publishData(SensorData data, String status) {
  StaticJsonDocument<200> doc;
  doc["temp"] = data.temp;
  doc["humi"] = data.humi;
  doc["co"] = data.co;
  doc["status"] = status;
  doc["timestamp"] = millis();
  
  char buffer[256];
  serializeJson(doc, buffer);
  if (client.publish(topic_data, buffer)) {
    Serial.println("Datos publicados a MQTT: " + String(buffer));
  } else {
    Serial.println("Error al publicar datos a MQTT");
  }
}

// Publicar alerta vía MQTT
void publishAlert(String message) {
  if (client.publish(topic_alert, message.c_str())) {
    Serial.println("Alerta publicada a MQTT: " + message);
  } else {
    Serial.println("Error al publicar alerta a MQTT");
  }
}

// Callback de MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println("Mensaje recibido en topic " + String(topic) + ": " + message);

  if (String(topic) == topic_control) {
    int controlValue = message.toInt();
    if (xQueueSend(mqttQueue, &controlValue, portMAX_DELAY) != pdPASS) {
      Serial.println("Error: No se pudo enviar el mensaje a la cola MQTT.");
    }
  }
}

// Reconectar al broker MQTT
void reconnect() {
  while (!client.connected()) {
    Serial.print("Conectando al broker MQTT...");
    if (client.connect(mqtt_client_id)) {
      Serial.println("Conectado al broker Mosquitto");
      client.subscribe(topic_control); // Suscribirse al topic de control
      Serial.println("Suscrito al topic: " + String(topic_control));
    } else {
      Serial.println("Fallo, reintentando en 5 segundos");
      vTaskDelay(5000 / portTICK_PERIOD_MS); // Esperar antes de reintentar
    }
  }
}

// Tarea para leer los sensores periódicamente
void readSensorsTask(void *pvParameters) {
  while (true) {
    sensorData.temp = dht.readTemperature();
    sensorData.humi = dht.readHumidity();
    MQ2.update();
    sensorData.co = analogRead(Pin);
    sensorData.co = map(sensorData.co, 0, 4095, 0, 100);

    xQueueSend(sensorDataQueue, &sensorData, portMAX_DELAY);
    dataReady = true;
    vTaskDelay(500 / portTICK_PERIOD_MS); // Leer sensores cada 500ms
  }
}

// Tarea para procesar mensajes MQTT
void mqttTask(void *pvParameters) {
  int controlValue;
  while (true) {
    if (xQueueReceive(mqttQueue, &controlValue, portMAX_DELAY) == pdTRUE) {
      if (controlValue == 0) {
        alarmTriggered = false;
        alarmActive = false;
        digitalWrite(LED_PIN, LOW);
        noTone(BUZZER_PIN);
        addNotification("Alarma desactivada remotamente");
        Serial.println("Alarma desactivada remotamente");
      } else if (controlValue == 1) {
        alarmTriggered = true;
        alarmActive = true;
        tone(BUZZER_PIN, 1000);
        digitalWrite(LED_PIN, HIGH);
        addNotification("Alarma activada remotamente");
        Serial.println("Alarma activada remotamente");
      }
    }
  }
}

// Manejar la solicitud raíz del servidor web
void handleRoot() {
  server.send_P(200, "text/html", MAIN_page);
}

// Manejar las solicitudes de datos del servidor web
void handleData() {
  String status = "Monitoreo OK";
  if (sensorData.temp > TEMP_HIGH) status = "Temp alta";
  else if (sensorData.temp < TEMP_LOW) status = "Temp baja";
  else if (sensorData.humi < HUMI_LOW) status = "Humedad Baja";
  else if (alarmActive) status = "ALERTA INCENDIO!";

  String json = "{";
  json += "\"temp\":" + String(sensorData.temp) + ",";
  json += "\"humi\":" + String(sensorData.humi) + ",";
  json += "\"co\":" + String(sensorData.co) + ",";
  json += "\"status\":\"" + status + "\",";
  json += "\"alarmActive\":" + String(alarmActive ? "true" : "false") + ",";
  json += "\"lcd\":\"" + status + "\\nTemp: " + String(sensorData.temp) + " 'C\\nHumi: " + String(sensorData.humi) + " %\\nMQ2 = " + String(sensorData.co) + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleHistory() {
  String json = "[";
  int count = 0;
  for (int i = 0; i < HISTORY_SIZE; i++) {
    int idx = (historyIndex - i - 1 + HISTORY_SIZE) % HISTORY_SIZE;
    if (timestampHistory[idx] == 0) continue;
    if (count > 0) json += ",";
    json += "{";
    json += "\"timestamp\":" + String(timestampHistory[idx]) + ",";
    json += "\"temp\":" + String(tempHistory[idx]) + ",";
    json += "\"humi\":" + String(humiHistory[idx]) + ",";
    json += "\"co\":" + String(coHistory[idx]) + ",";
    json += "\"status\":\"" + statusHistory[idx] + "\"";
    json += "}";
    count++;
  }
  json += "]";
  server.send(200, "application/json", json);
}

void handleNotifications() {
  String json = "[";
  for (int i = 0; i < notificationCount; i++) {
    if (i > 0) json += ",";
    json += "\"" + notifications[i] + "\"";
  }
  json += "]";
  server.send(200, "application/json", json);
}

// Manejar la solicitud para desactivar la alarma
void handleDisableAlarm() {
  alarmTriggered = false;
  alarmActive = false;
  digitalWrite(LED_PIN, LOW);
  noTone(BUZZER_PIN);
  addNotification("Alarma desactivada manualmente");
  publishAlert("Alarma desactivada manualmente");
  server.send(200, "text/plain", "Alarma desactivada");
}

// Agregar una notificación al historial
void addNotification(String message) {
  for (int i = 4; i > 0; i--) notifications[i] = notifications[i-1];
  unsigned long currentTime = millis();
  String timeString = String(currentTime / 60000) + "m " + String((currentTime / 1000) % 60) + "s";
  notifications[0] = "[" + timeString + "] " + message;
  if (notificationCount < 5) notificationCount++;
}

// Agregar datos al historial
void addToHistory(float temp, float humi, int co, String status) {
  tempHistory[historyIndex] = temp;
  humiHistory[historyIndex] = humi;
  coHistory[historyIndex] = co;
  statusHistory[historyIndex] = status;
  timestampHistory[historyIndex] = millis();
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;
}

void setRGB(int red, int green, int blue) {
  analogWrite(RED_PIN, 255 - red);
  analogWrite(GREEN_PIN, 255 - green);
  analogWrite(BLUE_PIN, 255 - blue);
}

// Bucle principal del programa
void myLoopTask( void *pvParameters) {
  SensorData receivedData;
  while(true){
    server.handleClient();

    if (xQueueReceive(sensorDataQueue, &receivedData, portMAX_DELAY) == pdTRUE) {
      dataReady = false;

      // Mostrar datos en el LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Monitoreo OK");
      lcd.setCursor(0, 1);
      lcd.print("Temp: ");
      lcd.print(sensorData.temp);
      lcd.print("'C");
      delay(1000);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Monitoreo OK");
      lcd.setCursor(0, 1);
      lcd.print("Humi: ");
      lcd.print(sensorData.humi);
      lcd.print(" %");
      delay(1000);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Monitoreo OK");
      lcd.setCursor(0, 1);
      lcd.print("MQ2 = ");
      lcd.print(sensorData.co);
      delay(1000);
      lcd.clear();
      Serial.println("Temp: " + String(sensorData.temp) + " Humi: " + String(sensorData.humi) + " CO: " + String(sensorData.co));

      // Actualizar lecturas de sensores
      tempReadings[index_gas] = sensorData.temp;
      humiReadings[index_gas] = sensorData.humi;
      coReadings[index_gas] = sensorData.co;
      index_gas = (index_gas + 1) % numReadings;

      float tempDiff = tempReadings[index_gas] - tempReadings[(index_gas + 1) % numReadings];
      float humiDiff = humiReadings[index_gas] - humiReadings[(index_gas + 1) % numReadings];
      int coDiff = coReadings[index_gas] - coReadings[(index_gas + 1) % numReadings];

      // Detectar condiciones de alarma
      bool fireDetected = digitalRead(FLAME_PIN) == FIRE_THRESHOLD;
      bool highTemp = sensorData.temp > TEMP_HIGH;
      bool lowTemp = sensorData.temp < TEMP_LOW;
      bool lowHumi = sensorData.humi < HUMI_LOW;
      bool highCO = sensorData.co > 30;
      bool rapidChange = (tempDiff > 2) || (humiDiff < -5) || (coDiff > 5);

      String status = "Monitoreo OK";
      if (highTemp) {
        status = "Temp alta";
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Monitoreo OK");
        lcd.setCursor(0, 1);
        lcd.print("Temp alta");
        lcd.setCursor(14,1);
        lcd.write(byte(0));
        lcd.setCursor(15,1);
        lcd.write(byte(1));
        delay(1000);
        setRGB(255, 0, 0);
        lcd.clear();
      } else if (lowTemp) {
        status = "Temp baja";
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Monitoreo OK");
        lcd.setCursor(0, 1);
        lcd.print("Temp baja");
        lcd.setCursor(14,1);
        lcd.write(byte(0));
        lcd.setCursor(15,1);
        lcd.write(byte(1));
        delay(1000);
        setRGB(255, 0, 0);
        lcd.clear();  
      } else if (lowHumi) {
        status = "Humedad Baja";
        digitalWrite(LED_PIN, HIGH);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Monitoreo OK");
        lcd.setCursor(0, 1);
        lcd.print("Humedad Baja");
        lcd.setCursor(14,1);
        lcd.write(byte(0));
        lcd.setCursor(15,1);
        lcd.write(byte(1));
        delay(1000);
      } else {
        setRGB(0, 255, 0);
        digitalWrite(LED_PIN, LOW);
      }

      // Activar alarma si se detectan condiciones de incendio
      if (fireDetected || (highTemp && lowHumi) || (rapidChange && highCO)) {
        triggerAlarm();
      }

      // Manejar la activación de la alarma
      if (alarmTriggered && !alarmActive) {
        alarmActive = true;
        addNotification("¡ALARMA DE INCENDIO ACTIVADA!");
        tone(BUZZER_PIN, 1000);
        digitalWrite(LED_PIN, HIGH);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Alerta incendio!!");
        lcd.setCursor(0, 1);
        delay(1000);
      }

      if (alarmActive) {
        status = "ALERTA INCENDIO!";
      }

      // Actualizar el historial de datos
      if (millis() - lastHistoryUpdate >= historyInterval) {
        addToHistory(sensorData.temp, sensorData.humi, sensorData.co, status);
        lastHistoryUpdate = millis();
      }

      publishData(receivedData, status);
    }
    vTaskDelay(100/ portTICK_PERIOD_MS);
  }
}

// Configuración inicial del sistema
void setup() {
  Serial.begin(115200);
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FLAME_PIN, INPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  sensorDataQueue = xQueueCreate(5, sizeof(SensorData));
  if (sensorDataQueue == NULL) {
    Serial.println("Error al crear la cola de datos de sensores.");
  }

  mqttQueue = xQueueCreate(5, sizeof(int)); // Crear la cola para mensajes MQTT
  if (mqttQueue == NULL) {
    Serial.println("Error al crear la cola MQTT.");
  }

  MQ2.setRegressionMethod(1);
  MQ2.setA(36974); 
  MQ2.setB(-3.109);
  MQ2.init(); 

  lcd.init();
  lcd.backlight();
  lcd.setCursor(4, 0);
  lcd.print("----*----");
  lcd.setCursor(2, 1);
  lcd.print("Alarm System");
  delay(1000);
  lcd.clear();

  Serial.print("Calibrating MQ2...");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++) {
    MQ2.update();
    calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
    Serial.print(".");
  }
  MQ2.setR0(calcR0 / 10);
  Serial.println("\nCalibration complete. R0 = " + String(calcR0 / 10));

  attachInterrupt(digitalPinToInterrupt(FLAME_PIN), triggerAlarm, FALLING);
  lcd.createChar(0, Alert0);
  lcd.createChar(1, Alert1);

  // Conexión a WiFi
  Serial.println("Conectando a WiFi...");
  lcd.setCursor(0, 0);
  lcd.print("Conectando a");
  lcd.setCursor(0, 1);
  lcd.print("WiFi...");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    WiFi.begin(SSID, PASSWORD);
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConectado a WiFi! IP: " + WiFi.localIP().toString());
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Conectado");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());
    delay(2000);
  } else {
    Serial.println("\nFallo al conectar a WiFi.");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Fallo WiFi");
    while (true) delay(1000);
  }

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Configuración de rutas del servidor web
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/history", handleHistory);
  server.on("/notifications", handleNotifications);
  server.on("/disableAlarm", handleDisableAlarm);
  server.begin();
  Serial.println("Servidor web iniciado.");

  // Crear tarea para leer sensores
  xTaskCreate(readSensorsTask, "ReadSensorsTask", 2048, NULL, 1, NULL);

  // Crear tarea para procesar mensajes MQTT
  xTaskCreate(mqttTask, "MQTTTask", 4096, NULL, 1, NULL);
}

void loop() {
  client.loop(); // Procesar mensajes MQTT
}