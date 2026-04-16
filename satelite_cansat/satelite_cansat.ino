#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <RadioLib.h>
#include <heltec_unofficial.h>

// ---------------- CONFIGURACIÓN ----------------

#define PIN_SDA 41
#define PIN_SCL 42
#define BAND 868.5
#define SEALEVELPRESSURE_HPA 1013.25

#define ALTURA_DESPEGUE 300
#define ALTURA_ATERRIZAJE 100
#define ALTURA_CAMARA 600

#define PIN_BUZZER 45
#define PIN_CAMARA 46

Adafruit_BME280 bme;
TwoWire I2CBME = TwoWire(1);

// ---------------- VARIABLES ----------------

// datos de baliza 1
String id1 = "";
String hora1 = "";
String sat1 = "";
String lat1 = "";
String lon1 = "";
String alt1 = "";
String vel1 = "";

// datos de baliza 2
String id2 = "";
String hora2 = "";
String sat2 = "";
String lat2 = "";
String lon2 = "";
String alt2 = "";
String vel2 = "";

// estado del sistema
int estado = 0;

// banderas
bool haDespegado = false;
bool haAterrizado = false;
bool camaraActivada = false;

// envío
unsigned long tiempoAnteriorEnvio = 0;
const unsigned long intervaloEnvio = 1000;

// buzzer aterrizaje
unsigned long tiempoAnteriorPitido = 0;
const unsigned long intervaloPitido = 300;
bool estadoBuzzer = false;

// altura
float alturaInicial = 0.0;
float alturaRelativa = 0.0;
//
void imprimirEstado() {
  if (estado == 0) {
    Serial.println("reposo");
  } else if (estado == 1) {
    Serial.println("despegue");
  } else if (estado == 2) {
    Serial.println("camara");
  } else if (estado == 3) {
    Serial.println("aterrizaje");
  } else if (estado == 4) {
    Serial.println("error");
  }
}
// ---------------- SETUP ----------------

void setup() {
  Serial.begin(115200);
  heltec_setup();

  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_CAMARA, OUTPUT);

  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_CAMARA, LOW);

  I2CBME.begin(PIN_SDA, PIN_SCL);

  if (!bme.begin(0x76, &I2CBME)) {
    estado = 4;
  }

  int state = radio.begin(BAND);
  if (state != RADIOLIB_ERR_NONE) {
    estado = 4;
  }

  radio.setSpreadingFactor(8);
  radio.setBandwidth(125.0);
  radio.setCodingRate(5);
  radio.setSyncWord(0x12);
  radio.setCRC(true);

  alturaInicial = bme.readAltitude(SEALEVELPRESSURE_HPA);

  if (isnan(alturaInicial)) {
    alturaInicial = 0.0;
    estado = 4;
  }
}

// ---------------- LOOP ----------------

void loop() {
  heltec_loop();

  // -------- 1. SENSOR --------

  float t = bme.readTemperature();
  float p = bme.readPressure() / 100.0;
  float h = bme.readHumidity();
  float a = bme.readAltitude(SEALEVELPRESSURE_HPA);

  if (isnan(t) || isnan(p) || isnan(h) || isnan(a)) {
    estado = 4;
  }

  alturaRelativa = a - alturaInicial;

  // -------- 2. DESPEGUE --------

  if (alturaRelativa > ALTURA_DESPEGUE && !haDespegado) {
    haDespegado = true;
    estado = 1;
  }

  // -------- 3. CÁMARA --------

  if (alturaRelativa > ALTURA_CAMARA && !camaraActivada) {
    camaraActivada = true;
    estado = 2;
    digitalWrite(PIN_CAMARA, HIGH);
  }

  // -------- 4. ATERRIZAJE --------

  if (alturaRelativa < ALTURA_ATERRIZAJE && haDespegado && !haAterrizado) {
    haAterrizado = true;
    estado = 3;
  }

  // -------- 5. PITIDO CONTINUO --------

  if (haAterrizado) {
    unsigned long ahora = millis();

    if (ahora - tiempoAnteriorPitido >= intervaloPitido) {
      tiempoAnteriorPitido = ahora;

      estadoBuzzer = !estadoBuzzer;
      digitalWrite(PIN_BUZZER, estadoBuzzer);
    }
  }

  // -------- 6. RECEPCIÓN --------

  String recibido = "";
  int state = radio.receive(recibido, 5000);

  
  if (state == RADIOLIB_ERR_NONE) {
    String campos[19];
    for (int i = 0; i < 19; i++) campos[i] = ""; //Vacía campos
    int inicio = 0;
    int n = 0;
    for (int i = 0; i < recibido.length(); i++) { //Separa el mensaje en campos separados por comas
      if (recibido.charAt(i) == ',') {
        if (n < 19) {
          campos[n] = recibido.substring(inicio, i);
          campos[n].trim();
          inicio = i + 1;
          n++;
        }
      }
    }
    if (n < 19 && inicio < recibido.length()) { //Extrae el último campo si lo hubiera
      campos[n] = recibido.substring(inicio);
      campos[n].trim();
    }
    //Rellena las variables globales con los campos
    if (campos[5] != "") id1 = campos[5];
    if (campos[6] != "") hora1 = campos[6];
    if (campos[7] != "") sat1 = campos[7];
    if (campos[8] != "") lat1 = campos[8];
    if (campos[9] != "") lon1 = campos[9];
    if (campos[10] != "") alt1 = campos[10];
    if (campos[11] != "") vel1 = campos[11];

    if (campos[12] != "") id2 = campos[12];
    if (campos[13] != "") hora2 = campos[13];
    if (campos[14] != "") sat2 = campos[14];
    if (campos[15] != "") lat2 = campos[15];
    if (campos[16] != "") lon2 = campos[16];
    if (campos[17] != "") alt2 = campos[17];
    if (campos[18] != "") vel2 = campos[18];
  }
  else if (state != RADIOLIB_ERR_RX_TIMEOUT) {
    estado = 4;
  }


/*
  if (state == RADIOLIB_ERR_NONE) {
    Serial.print("Recibido: ");
    Serial.println(recibido);
  } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
    Serial.println("Timeout RX");
  } else {
    Serial.print("Error RX: ");
    Serial.println(state);
    estado = 4;
  }
  */
  // -------- 7. ENVÍO --------

  unsigned long ahoraEnvio = millis();

  if (ahoraEnvio - tiempoAnteriorEnvio >= intervaloEnvio) {

    String mensaje = "";

    mensaje += String(estado) + ",";
    mensaje += String(t, 1) + ",";
    mensaje += String(p, 0) + ",";
    mensaje += String(h, 0) + ",";
    mensaje += String(a, 0) + ",";

    mensaje += id1 + ",";
    mensaje += hora1 + ",";
    mensaje += sat1 + ",";
    mensaje += lat1 + ",";
    mensaje += lon1 + ",";
    mensaje += alt1 + ",";
    mensaje += vel1 + ",";

    mensaje += id2 + ",";
    mensaje += hora2 + ",";
    mensaje += sat2 + ",";
    mensaje += lat2 + ",";
    mensaje += lon2 + ",";
    mensaje += alt2 + ",";
    mensaje += vel2 + ",";

    int estadoEnvio = radio.transmit(mensaje);

    tiempoAnteriorEnvio = ahoraEnvio;

    if (estadoEnvio != RADIOLIB_ERR_NONE) {
      estado = 4;
    }
    //Serial.println(mensaje);
  }
}