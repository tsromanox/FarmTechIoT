#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ArduinoJson.h> // Para formatar dados em JSON

#define DHTPIN 4       // Pino de dados do DHT22
#define DHTTYPE DHT22  // Tipo do sensor DHT

#define PHOSPHORUS_PIN 16
#define POTASSIUM_PIN 17
#define LDR_PIN 34       // Pino ADC para o LDR (pH)
#define RELAY_PIN 5
#define LED_PIN 2        // LED de status da bomba

// Configurações Wi-Fi (Wokwi usa 'Wokwi-GUEST' por padrão sem senha)
const char* ssid = "Wokwi-GUEST";
const char* password = ""; // Sem senha para Wokwi-GUEST

// Configurações MQTT
const char* mqtt_server = "test.mosquitto.org"; // Ou broker.hivemq.com
const int mqtt_port = 1883;
const char* topic_sensores_esp32 = "farmtech/esp32/sensores";
const char* topic_comandos_python = "farmtech/python/comando_bomba";
const char* mqtt_client_id = "esp32-farmtech-client-UNIQUE"; // Use um ID único

WiFiClient espClient;
PubSubClient client(espClient);

// Variáveis para controle da bomba por tempo
unsigned long pumpStartTime = 0;
unsigned long pumpDuration = 0;
bool pumpRunning = false;

DHT dht(DHTPIN, DHTTYPE);

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println("Endereco IP: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensagem recebida no topico: ");
  Serial.println(topic);
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0'; // Null-terminate the string
  Serial.print("Mensagem: ");
  Serial.println(message);

  if (strcmp(topic, topic_comandos_python) == 0) {
    StaticJsonDocument<256> doc; // Ajuste o tamanho conforme necessário
    DeserializationError error = deserializeJson(doc, message);

    if (error) {
      Serial.print(F("deserializeJson() falhou: "));
      Serial.println(error.f_str());
      return;
    }

    const char* acao = doc["acao"]; // "LIGAR" ou "DESLIGAR"
    
    if (strcmp(acao, "LIGAR") == 0) {
      pumpDuration = doc["duracao_ms"] | 0; // Pega duracao_ms, default 0 se não existir
      if (pumpDuration > 0) {
        Serial.print("Ligando bomba por ");
        Serial.print(pumpDuration / 1000);
        Serial.println(" segundos.");
        digitalWrite(RELAY_PIN, HIGH);
        digitalWrite(LED_PIN, HIGH);
        pumpStartTime = millis();
        pumpRunning = true;
      } else {
        Serial.println("Duracao invalida para ligar a bomba. Mantendo desligada.");
        digitalWrite(RELAY_PIN, LOW);
        digitalWrite(LED_PIN, LOW);
        pumpRunning = false;
      }
    } else if (strcmp(acao, "DESLIGAR") == 0) {
      Serial.println("Desligando bomba por comando MQTT.");
      digitalWrite(RELAY_PIN, LOW);
      digitalWrite(LED_PIN, LOW);
      pumpRunning = false;
      pumpDuration = 0; // Reseta a duração
    }
  }
}

void reconnect_mqtt() {
  while (!client.connected()) {
    Serial.print("Tentando conexao MQTT...");
    if (client.connect(mqtt_client_id)) { // ID do cliente MQTT
      Serial.println("conectado");
      client.subscribe(topic_comandos_python);
      Serial.print("Inscrito no topico: ");
      Serial.println(topic_comandos_python);
    } else {
      Serial.print("falhou, rc=");
      Serial.print(client.state());
      Serial.println(" tentando novamente em 5 segundos");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  dht.begin();

  pinMode(PHOSPHORUS_PIN, INPUT_PULLUP); // Ou INPUT_PULLDOWN se usar resistor interno
  pinMode(POTASSIUM_PIN, INPUT_PULLUP);  // Ou INPUT_PULLDOWN
  // LDR_PIN é ADC, não precisa de pinMode para input analógico
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, LOW); // Bomba desligada inicialmente
  digitalWrite(LED_PIN, LOW);   // LED desligado inicialmente
  Serial.println("Sistema de Irrigacao Inteligente FarmTech Solutions Inicializado");

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  reconnect_mqtt(); // Conecta ao MQTT pela primeira vez
}

unsigned long lastMsgTime = 0;
const long msgInterval = 10000; // Publicar dados a cada 10 segundos

void loop() {
  if (!client.connected()) {
    reconnect_mqtt();
  }
  client.loop(); // Mantém a conexão MQTT e processa mensagens recebidas

  // Lógica para controlar a bomba por tempo
  if (pumpRunning) {
    if (millis() - pumpStartTime >= pumpDuration) {
      Serial.println("Tempo de irrigacao concluido. Desligando bomba.");
      digitalWrite(RELAY_PIN, LOW);
      digitalWrite(LED_PIN, LOW);
      pumpRunning = false;
      pumpDuration = 0; // Reseta
    }
  }

  unsigned long currentTime = millis();
  if (currentTime - lastMsgTime > msgInterval) {
    lastMsgTime = currentTime;

    float humidity = dht.readHumidity();
    bool phosphorus_present = digitalRead(PHOSPHORUS_PIN) == HIGH;
    bool potassium_present = digitalRead(POTASSIUM_PIN) == HIGH;
    int ldr_value = analogRead(LDR_PIN);
    float ph_value = map(ldr_value, 0, 4095, 40, 90) / 10.0; // Exemplo

    if (isnan(humidity)) {
      Serial.println("Falha ao ler do sensor DHT!");
      // Poderia enviar um status de erro via MQTT também
    } else {
      // Criar payload JSON
      StaticJsonDocument<256> jsonDoc; // Ajuste o tamanho conforme necessário
      jsonDoc["umidade"] = humidity;
      jsonDoc["ph"] = ph_value;
      jsonDoc["fosforo"] = phosphorus_present;
      jsonDoc["potassio"] = potassium_present;
      // Adicione um timestamp se desejar, ex: jsonDoc["timestamp_esp32"] = millis();

      char jsonBuffer[256];
      serializeJson(jsonDoc, jsonBuffer);

      // Publicar dados
      if (client.publish(topic_sensores_esp32, jsonBuffer)) {
        Serial.print("Dados publicados em ");
        Serial.print(topic_sensores_esp32);
        Serial.print(": ");
        Serial.println(jsonBuffer);
      } else {
        Serial.println("Falha ao publicar dados via MQTT.");
      }
    }
  }
}