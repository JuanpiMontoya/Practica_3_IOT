//Librerias necesarias y PIN del sensor y actuador
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#define Sensor_PIN  13
#define Solenoid_Valve_PIN  15
#define Irrigation_Area 20 // En metros cuadrados (m2)

// MQTT Variables (Variables del sensor)

const char * WIFI_SSID = "FAMILIA MONTOY"; // Red Wifi a utilizar
const char * WIFI_PASS = "N12JP16SR"; // Contraseña red Wifi

const char * MQTT_BROKER_HOST = "a2mnkqceizplps-ats.iot.us-east-1.amazonaws.com"; // Servidor MQTT de AWS IoT.
const int MQTT_BROKER_PORT = 8883; // Puerto del servidor MQTT.

const char * MQTT_CLIENT_ID = "ESP-32"; // Identificador único del ESP-32 para el MQTT

const char * UPDATE_TOPIC = "$aws/things/thing/shadow/name/Shadow_RiegoInteligente/update"; // Topico del shadow - Update              
const char * UPDATE_DELTA_TOPIC = "$aws/things/thing/shadow/name/Shadow_RiegoInteligente/update/delta";  // Topico del shadow - Update Delta

// Certificado raíz de AWS
const char AMAZON_ROOT_CA1[] PROGMEM = R"EOF( 
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

//Certificado del servidor MQTT.
const char CERTIFICATE[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIUIc3rCvyXegtaOCvJbd83tUx4pMQwDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTI0MDQxNTIzNDQz
N1oXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALq1uThfNuc5Si4FfoMb
OcIGxnlXS2BX/tnYSe8QZFBwm6MSHzhpBm6tUZAzLflHavCgi31QNSekQCvXEVPn
d64G4rhTrIXO4Hfi+FmD8u53kKY/0IHb3bQjHN7kHHKkd1DK3kAYD3GC/OUTyvkq
9f7QVxiWzEHCc0qOmwu4QEMdkky17862AV5zb5n8UnXWX3WIH2nNhg2t0jE+XOK0
icdem6p8NrCcIpJObE1xZcATnQ6MOEM8Vw6EQw/qa/lC1abxnPDtY49gwdxThovM
Yl9g8S/Yo4JXgf0P6tztoU9BoJnTSEtO3ZLYw2OrdwNntJ/ZlFsM4vLwEiJAq/dj
pFcCAwEAAaNgMF4wHwYDVR0jBBgwFoAU0ddw5sP5LZ7TvjwaCVK419bMCmUwHQYD
VR0OBBYEFKFuc4wLkZbpdPTkZMBKRO1D1updMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQBrYDlZTioXrDWw10F9aoG1powO
wRQH+kru/ctixMPcxJazuflFM7rjemQEkwQcTCDJDVgva6cTH2EfaEW3yNmDds9P
7d+6vVKwWek0YAGpwyElHR7m3npvmoq1WvCA2UP4FVq3EBFX21mR2zKiLBEq5r1Y
1AUzo/RrzY+GHeJSNMtQ31val00CUyLV7I9v9LLww4Tn0P7xEIc1FI03hy2Yv9C1
nIPhDWacVWTCQ/Vi6EXJXsqwyny4LQq8FXfN6HCEScIEANLWWp+qmIQp7PDONzct
S3m7vOvmjr1NsbOT2Uu+fa2PONBQfSG5JxacG7i+TJ/ssY6uVMC4i02X3z02
-----END CERTIFICATE-----
)KEY";

// Clave privada para el servidor MQTT.
const char PRIVATE_KEY[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEogIBAAKCAQEAurW5OF825zlKLgV+gxs5wgbGeVdLYFf+2dhJ7xBkUHCboxIf
OGkGbq1RkDMt+Udq8KCLfVA1J6RAK9cRU+d3rgbiuFOshc7gd+L4WYPy7neQpj/Q
gdvdtCMc3uQccqR3UMreQBgPcYL85RPK+Sr1/tBXGJbMQcJzSo6bC7hAQx2STLXv
zrYBXnNvmfxSddZfdYgfac2GDa3SMT5c4rSJx16bqnw2sJwikk5sTXFlwBOdDow4
QzxXDoRDD+pr+ULVpvGc8O1jj2DB3FOGi8xiX2DxL9ijgleB/Q/q3O2hT0GgmdNI
S07dktjDY6t3A2e0n9mUWwzi8vASIkCr92OkVwIDAQABAoIBAHoBT6RHi6/ynseN
7YqsmsRv+vfFlErDlZQyorVmJ7bsrrcTm3UYJAzkkrzAxNT01yugLSZY6DMcQJal
Nfx275y3gSVkAAN8GPAeT5zm/TLYzJvmbVC+JJSWsUHxG6nKUPFpb2vAjS7momzq
zyKc8RxudctqltSNxRq+LxCseMmhhmMxfpHqGRqT8hl/nRjRJ5uGwPoNk9j2ODbm
Q/ebGCXXHXSG/rTDN3dXflhyNBJWPbz6j+nKALKryVirYQ++3J/J/qriOxx+ZyhC
CL7f3EH5bFiwY92dKrX5ouyTtL1FDKdUOpflaDtYOThEoLe4/FthzzP+rLVBtHBj
BQkjSfECgYEA4S/iSwszrvvThvOqOwJIARQRtJBMU0CMvRDICNN6P9NkmGyGovtD
7VX+X5ebW0kWTaj6RdUKK1Zq6OoXubZRww8skhopvViWXU4nSpi+ZqIDfV8pdVYl
QGfwK3gEBk5OZxDW7IRzk609X5on/Ixdh+KnMEpVQEjB/NU03GhaIUkCgYEA1EID
86aH1VA2cbBAZMow7UhGnymj1aI//J32r1FOXSXwCK8ZRm25jKJIVAOZJdJxNq1b
HPXklneKrGrvKIgnP1OyVDw7eYHwzWb+N4hJhYkMkhSHMolTFBQaS+VAonbT8wM1
e6TOlD0PSlGX8MaeQ0LiPJ/1YEaKD0lnJLAZOJ8CgYApC/tpxm7zVH/oN34uJ2pM
mAXqLetbuYZt6CMuSK4d+EB78DxSdQ6asnQdpJry9s2T9ls4Y27pG2V9cz5D4dHh
hmCPeSlmlEFEXLyYwmLifmG4dqIt+zfrF4cVxVimdZug+dbTYglXP9rSFF15kXRz
EoN7L/F+OETz+0EZdAQJ0QKBgGYsPPC9OeeRzBNfzy91MKYXw378MVQ13O1CMKvl
zH4ENA5KEzWWvdmXiI+Ah+jv6vVx70j6iIVxl7w1YVg2agYOyOnJcKulw/9r03Vx
0kpIVVfAL/BAsFAa+SmNelZ6Mr5ozgbtp08uMT0KF8ScJzSjbnnWWex1KCIBqHe9
mPZrAoGABweMDxR1XYR7RWE3MFjLVkEwpElPf5LIr2LU89/zEXeNtxQYol5N8YoP
UvEvAy/JGb9wRl4ivsROz+gFDfyzqclimPz9/LGEQiJ/BrCHzMLKnIfTQFfWFT84
PLEOanlo7NgHrI0hDI4amoiOxaqDWQgMEcte34pL/oKmzpXHwzs=
-----END RSA PRIVATE KEY-----
)KEY";

StaticJsonDocument<JSON_OBJECT_SIZE(64)> inputDoc; // Almacenamos y procesamos los datos de entrada como JSON.
StaticJsonDocument<JSON_OBJECT_SIZE(4)> outputDoc; // Almacenamos y procesamos los datos de salida como JSON.
char outputBuffer[128]; // Almacenamos los JSON antes de ser enviados.

// Flow Sensor Variables (Variables del sensor)

unsigned long currentMillis; // Milisegundos desde que se inició el programa
unsigned long previousMillis; // Milisegundos del bucle anterior
byte pulse1Sec; // número de pulsos del sensor que ocurrieron en el último segundo
float flowRate; // El valor de litros / minuto
unsigned int flowMilliLitres ; // Cantidad de líquido (ml)
unsigned long totalMilliLitres ; // Cantidad de líquido total (ml)
float MAX_WATER_LIMIT = (Irrigation_Area * 0.04) * 1000000; // Medida en ml //0.04 Medida de profundidad estandar para el cesped

volatile byte pulseCount; // Variable global para contar los pulsos

// Clase FlowSensor
class FlowSensor {
  private:
    const float calibrationFactor; 
  public:
    // Constructor de FlowSensor
    FlowSensor(float calibrationFactor) : calibrationFactor(calibrationFactor) {}

    // Método get del factor de calibración
    float getCalibrationFactor() const {
      return calibrationFactor;
    }
};

// Creamos una instancia FlowSensor con el factor de calibración respectivo
FlowSensor flowSensor(3);

// Función para contar los pulsos del sensor de flujo
void IRAM_ATTR pulseCounter() {
  pulseCount++; // Incrementamos el contador de pulsos
}

// Calculo de la la tasa de flujo
void calculateFlowRate(unsigned long currentMillis) {
  // Verifica si ha pasado 1.5 segundos desde la última vez que se calculó la tasa de flujo
  if (currentMillis - previousMillis > 1500) {
    byte pulse1Sec = pulseCount;
    pulseCount = 0;

    // Calculo de la tasa de flujo (L/min)
    flowRate = ((1000.0 / (currentMillis - previousMillis)) * pulse1Sec) / flowSensor.getCalibrationFactor();
    previousMillis = currentMillis;
  }
}

// Calculo del flujo total
void calculateTotalFlow() {
  flowMilliLitres = (flowRate / 60) * 1000; // Conversion de la tasa de flujo (a ml/seg)
  totalMilliLitres += flowMilliLitres; // Incrementa el total de mililitros
}

// Clase SolenoidValve
class SolenoidValve {
  private:
    const int pin; // Pin  de la electroválvula
    bool isOpen; // Estado de la electroválvula (abierta o cerrada)
  public:
    // Constructor de SolenoidValve 
    SolenoidValve(int pin) : pin(pin), isOpen(true) {
        pinMode(pin, OUTPUT); 
    }

    // Método para abrir o cerrar la válvula
    void open_or_close(bool valve_state) {
        digitalWrite(pin, valve_state); // Cambiamos estado del pin
        isOpen = valve_state; // Actualizamos el estado
    }

    // Método para verificar si la válvula está abierta
    bool isOpened(){
        return isOpen; 
    }

    void change_Opened_state(bool valve_state){
        isOpen = valve_state; 
    }
};

// Creamos una instancia de SolenoidValve
SolenoidValve solenoidValve(Solenoid_Valve_PIN);

// Función de callback para procesar los mensajes recibidos del servidor MQTT
void callback(const char * topic, byte * payload, unsigned int length) {
  DeserializationError err = deserializeJson(inputDoc, payload); // Deserializa el mensaje JSON
  if (!err) {
    if (String(topic) == UPDATE_DELTA_TOPIC) { 
      bool valve_state = (inputDoc["state"]["valve_open"].as<int8_t>() != 0); // Obtenemos el estado de la válvula
      solenoidValve.open_or_close(valve_state); // Abrimos o cerramos la válvula
      solenoidValve.change_Opened_state(valve_state);
    }
  }
}

// Clase MQTTHandler
class MQTTHandler {
  private:
    WiFiClientSecure wifiClient; // Cliente WiFi seguro
    PubSubClient mqttClient; // Cliente MQTT
  public:
    // Constructor de MQTTHandler
    MQTTHandler() : mqttClient(wifiClient) {}

    // Conectarse al WiFi
    void connectWiFi() {
      Serial.print("Connecting to " + String(WIFI_SSID)); 
      WiFi.begin(WIFI_SSID, WIFI_PASS); // Iniciamos la conexión WiFi
      while (WiFi.status() != WL_CONNECTED) { // Esperamos la conexión WiFi
        delay(200); 
        Serial.print("."); 
      }
      Serial.println(" Connected to WiFi!"); // Mensaje de conexión WiFi exitosa
    }

    // Conectarse al MQTT Broker
    void connectMQTT() {
      wifiClient.setCACert(AMAZON_ROOT_CA1); // Añadimos el certificado raíz del servidor MQTT
      wifiClient.setCertificate(CERTIFICATE); // Añadimos el certificado 
      wifiClient.setPrivateKey(PRIVATE_KEY); //Añadimos la clave privada 

      mqttClient.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT); // Ingresamos el servidor MQTT y el puerto
      mqttClient.setCallback(callback); // Configuramos el callback para manejar los mensajes MQTT recibidos

      Serial.print("Connecting to " + String(MQTT_BROKER_HOST)); 
      if (mqttClient.connect(MQTT_CLIENT_ID)) { 
        Serial.println(" Connected to MQTT!"); // Mensaje de conexión MQTT exitosa

        delay(100); 
        mqttClient.subscribe(UPDATE_DELTA_TOPIC); // Suscripcion al topico de actualización delta
        Serial.println("Subscribed to " + String(UPDATE_DELTA_TOPIC)); 
        delay(100); 
      }
    }

    // Loop para mantener la conexión MQTT
    void loop() {
      if (mqttClient.connected()) { // Verificamos conexion
        mqttClient.loop(); // Ejecutamos loop del cliente MQTT
      } else {
        Serial.println("MQTT broker not connected!"); 
        delay(2000); 
      }
    }

    // Método para publicar un mensaje MQTT en un topic específico
    void publishMessage(const char* topic, const char* message) {
      mqttClient.publish(topic, message); // Publica el mensaje en el topic especificado
    }
};

// Creamos una instancia de MQTTHandler
MQTTHandler mqttHandler;


void reportWaterSensor() {
  outputDoc["state"]["reported"]["valve_open"] = solenoidValve.isOpened();
  outputDoc["state"]["reported"]["water_used"] = totalMilliLitres;
  //outputDoc["state"]["reported"]["max_water_limit"] = MAX_WATER_LIMIT;
  outputDoc["state"]["reported"]["max_water_limit"] = 100;
  serializeJson(outputDoc, outputBuffer);
  mqttHandler.publishMessage(UPDATE_TOPIC, outputBuffer);
}

void setup() {
  Serial.begin(115200);

  pinMode(Sensor_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Sensor_PIN), pulseCounter, FALLING);
  
  mqttHandler.connectWiFi();
  mqttHandler.connectMQTT();

  // Inicializar todas las variables en 0
  currentMillis = 0;
  previousMillis = 0;
  pulse1Sec = 0;
  flowRate = 0;
  flowMilliLitres = 0;
  totalMilliLitres = 0; 

  solenoidValve.open_or_close(1);
}

void calculateFlowAndPublishData(unsigned long currentMillis) {
  calculateFlowRate(currentMillis);
  calculateTotalFlow();
  reportWaterSensor();
}

void loop() {
  currentMillis = millis();
  
  if (currentMillis - previousMillis >= 1500) {
    mqttHandler.loop();
    calculateFlowAndPublishData(currentMillis);
  }
}
