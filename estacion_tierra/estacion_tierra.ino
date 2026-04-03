#include <RadioLib.h>
#include <heltec_unofficial.h>

// ---------------- CONFIGURACIÓN ----------------
#define BAND 868.0

// ---------------- VARIABLES GLOBALES ----------------

// Datos de SkyEye
String idSky = "";
String temp = "";
String pres = "";
String hum = "";
String altBme = "";

// Datos de baliza 1
String id1 = "";
String hora1 = "";
String sat1 = "";
String lat1 = "";
String lon1 = "";
String alt1 = "";
String vel1 = "";

// Datos de baliza 2
String id2 = "";
String hora2 = "";
String sat2 = "";
String lat2 = "";
String lon2 = "";
String alt2 = "";
String vel2 = "";
// ---------------- FUNCIONES ----------------
// Separa el mensaje recibido en 19 campos
void separarCampos(String mensaje, String campos[]) {
  int inicio = 0;
  int n = 0;
  for (int i = 0; i < mensaje.length(); i++) {
    if (mensaje.charAt(i) == ',') {
      campos[n] = mensaje.substring(inicio, i);
      campos[n].trim();
      inicio = i + 1;
      n++;
      if (n >= 19) {
        break;
      }
    }
  }
  while (n < 19) {
    campos[n] = "";
    n++;
  }
}
// Actualiza las variables con los datos que hayan llegado
void guardarDatos(String campos[]) {
  // SkyEye
  if (campos[0] != "") idSky = campos[0];
  if (campos[1] != "") temp = campos[1];
  if (campos[2] != "") pres = campos[2];
  if (campos[3] != "") hum = campos[3];
  if (campos[4] != "") altBme = campos[4];
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
}
// Envía por serie una línea completa para que Processing la lea
void enviarASerie() {
  Serial.print(idSky);   Serial.print(",");
  Serial.print(temp);    Serial.print(",");
  Serial.print(pres);    Serial.print(",");
  Serial.print(hum);     Serial.print(",");
  Serial.print(altBme);  Serial.print(",");

  Serial.print(id1);     Serial.print(",");
  Serial.print(hora1);   Serial.print(",");
  Serial.print(sat1);    Serial.print(",");
  Serial.print(lat1);    Serial.print(",");
  Serial.print(lon1);    Serial.print(",");
  Serial.print(alt1);    Serial.print(",");
  Serial.print(vel1);    Serial.print(",");

  Serial.print(id2);     Serial.print(",");
  Serial.print(hora2);   Serial.print(",");
  Serial.print(sat2);    Serial.print(",");
  Serial.print(lat2);    Serial.print(",");
  Serial.print(lon2);    Serial.print(",");
  Serial.print(alt2);    Serial.print(",");
  Serial.println(vel2);
}
// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  Serial.println("Comunicacion serie establecida");
  heltec_setup();
  int state = radio.begin(BAND);
  Serial.print("[LoRa] Inicializando... ");
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Radio iniciada");
  } else {
    Serial.print("Error al iniciar radio. Codigo: ");
    Serial.println(state);
  }
}
// ---------------- LOOP ----------------
void loop() {
  String mensajeRecibido = "";
  int state = radio.receive(mensajeRecibido);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Paquete recibido:");
    Serial.println(mensajeRecibido);
    String campos[19];
    separarCampos(mensajeRecibido, campos);
    guardarDatos(campos);

    Serial.println("Datos completos enviados a Processing:");
    enviarASerie();
  }
  else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
    // No hacemos nada si no llega nada
  }
  else {
    Serial.print("Error al recibir. Codigo: ");
    Serial.println(state);
  }
}