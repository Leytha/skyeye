#include <RadioLib.h>
#include <heltec_unofficial.h>

// ---------------- CONFIG ----------------

#define BAND 868.0

// -------- VARIABLES GLOBALES --------

// SkyEye
String estadoSky = "";
String temp = "";
String pres = "";
String hum = "";
String alt = "";

// Baliza 1
String id1 = "";
String hora1 = "";
String sat1 = "";
String lat1 = "";
String lon1 = "";
String alt1 = "";
String vel1 = "";

// Baliza 2
String id2 = "";
String hora2 = "";
String sat2 = "";
String lat2 = "";
String lon2 = "";
String alt2 = "";
String vel2 = "";

// ---------------- SETUP ----------------

void setup() {
  Serial.begin(115200);
  heltec_setup();

  int state = radio.begin(BAND);

  if (state != RADIOLIB_ERR_NONE) {
    Serial.println("Error LoRa");
    while (1);
  }

  // mismos parámetros que SkyEye
  radio.setSpreadingFactor(8);
  radio.setBandwidth(125.0);
  radio.setCodingRate(5);
  radio.setSyncWord(0x12);

  Serial.println("Estacion lista");
}

// ---------------- LOOP ----------------

void loop() {

  String recibido = "";
  int state = radio.receive(recibido);

  if (state == RADIOLIB_ERR_NONE) {

    String campos[19];

    for (int i = 0; i < 19; i++) {
      campos[i] = "";
    }

    int inicio = 0;
    int n = 0;

    for (int i = 0; i < recibido.length(); i++) {
      if (recibido.charAt(i) == ',') {
        if (n < 19) {
          campos[n] = recibido.substring(inicio, i);
          campos[n].trim();
          inicio = i + 1;
          n++;
        }
      }
    }

    if (n < 19 && inicio < recibido.length()) {
      campos[n] = recibido.substring(inicio);
      campos[n].trim();
    }

    // SkyEye
    if (campos[0] != "") estadoSky = campos[0];
    if (campos[1] != "") temp = campos[1];
    if (campos[2] != "") pres = campos[2];
    if (campos[3] != "") hum = campos[3];
    if (campos[4] != "") alt = campos[4];

    // Baliza 1
    if (campos[5] != "") id1 = campos[5];
    if (campos[6] != "") hora1 = campos[6];
    if (campos[7] != "") sat1 = campos[7];
    if (campos[8] != "") lat1 = campos[8];
    if (campos[9] != "") lon1 = campos[9];
    if (campos[10] != "") alt1 = campos[10];
    if (campos[11] != "") vel1 = campos[11];

    // Baliza 2
    if (campos[12] != "") id2 = campos[12];
    if (campos[13] != "") hora2 = campos[13];
    if (campos[14] != "") sat2 = campos[14];
    if (campos[15] != "") lat2 = campos[15];
    if (campos[16] != "") lon2 = campos[16];
    if (campos[17] != "") alt2 = campos[17];
    if (campos[18] != "") vel2 = campos[18];

    // enviar bloque completo a Processing
    Serial.print(estadoSky); Serial.print(",");
    Serial.print(temp);      Serial.print(",");
    Serial.print(pres);      Serial.print(",");
    Serial.print(hum);       Serial.print(",");
    Serial.print(alt);       Serial.print(",");

    Serial.print(id1);       Serial.print(",");
    Serial.print(hora1);     Serial.print(",");
    Serial.print(sat1);      Serial.print(",");
    Serial.print(lat1);      Serial.print(",");
    Serial.print(lon1);      Serial.print(",");
    Serial.print(alt1);      Serial.print(",");
    Serial.print(vel1);      Serial.print(",");

    Serial.print(id2);       Serial.print(",");
    Serial.print(hora2);     Serial.print(",");
    Serial.print(sat2);      Serial.print(",");
    Serial.print(lat2);      Serial.print(",");
    Serial.print(lon2);      Serial.print(",");
    Serial.print(alt2);      Serial.print(",");
    Serial.println(vel2);
  }
}