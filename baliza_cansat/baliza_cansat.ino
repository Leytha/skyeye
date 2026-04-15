#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// ---------------- CONFIGURACIÓN GENERAL ----------------

#define ID_BALIZA 1   // 1 o 2

// ---------------- PINES ----------------

// LoRa (SX1276) en NodeMCU
#define NSS_LORA  D8
#define RST_LORA  D0
#define DIO0_LORA D2

// GPS (NEO-6M)
#define RX_GPS D1
#define TX_GPS D3

// Frecuencia LoRa: 868.5 MHz
#define BAND 868.5E6

// Tiempo de escucha del GPS (ms)
#define T_GPS 1000

// ---------------- OBJETOS ----------------

TinyGPSPlus gps;
SoftwareSerial gps_serial(RX_GPS, TX_GPS);

bool loraInicializada = false;

// ---------------- FUNCIONES ----------------

void configurarLora() {
  LoRa.setPins(NSS_LORA, RST_LORA, DIO0_LORA);

  if (!LoRa.begin(BAND)) {
    Serial.println("Error al iniciar LoRa.");
    loraInicializada = false;
    return;
  }

  LoRa.setSpreadingFactor(11);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x12);

  loraInicializada = true;
  Serial.println("LoRa configurado.");
}

bool obtenerDatosGPS() {
  unsigned long start = millis();
  bool nuevoDato = false;

  while (millis() - start < T_GPS) {
    while (gps_serial.available()) {
      char c = gps_serial.read();
      if (gps.encode(c)) {
        if (gps.location.isValid()) {
          nuevoDato = true;
        }
      }
    }
  }

  return nuevoDato;
}

String construirMensajeDesdeCampos(String campos[], int numCampos) {
  String mensaje = "";

  for (int i = 0; i < numCampos; i++) {
    mensaje += campos[i];
    if (i < numCampos - 1) {
      mensaje += ",";
    }
  }

  return mensaje;
}

bool enviarPorRadio() {
  if (!loraInicializada) {
    Serial.println("Error: LoRa no está inicializado.");
    return false;
  }

  char hora[9];
  sprintf(hora, "%02d:%02d:%02d",
          gps.time.hour(),
          gps.time.minute(),
          gps.time.second());

  String sats = String(gps.satellites.value());
  String lat  = String(gps.location.lat(), 4);
  String lon  = String(gps.location.lng(), 4);
  String alt  = String(gps.altitude.meters(), 1);
  String vel  = String(gps.speed.kmph(), 1);

  String campos[19];
  for (int i = 0; i < 19; i++) {
    campos[i] = "";
  }

  int inicioBloque = -1;

  if (ID_BALIZA == 1) {
    inicioBloque = 5;
  } else if (ID_BALIZA == 2) {
    inicioBloque = 12;
  } else {
    Serial.println("Error: ID_BALIZA debe ser 1 o 2.");
    return false;
  }

  campos[inicioBloque + 0] = String(ID_BALIZA);
  campos[inicioBloque + 1] = String(hora);
  campos[inicioBloque + 2] = sats;
  campos[inicioBloque + 3] = lat;
  campos[inicioBloque + 4] = lon;
  campos[inicioBloque + 5] = alt;
  campos[inicioBloque + 6] = vel;

  String mensaje = construirMensajeDesdeCampos(campos, 19);

  Serial.println("Enviando mensaje:");
  Serial.println(mensaje);

  int okBegin = LoRa.beginPacket();
  if (okBegin == 0) {
    Serial.println("Error: no se pudo iniciar el paquete LoRa.");
    return false;
  }

  LoRa.print(mensaje);

  int okEnd = LoRa.endPacket();
  if (okEnd != 1) {
    Serial.println("Error: el paquete no se envió correctamente.");
    return false;
  }

  Serial.println("OK");
  return true;
}

// ---------------- SETUP ----------------

void setup() {
  Serial.begin(115200);
  gps_serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // LED apagado

  configurarLora();

  Serial.println("Sistema listo. Esperando señal de satélites...");
}

// ---------------- LOOP ----------------

void loop() {
  if (obtenerDatosGPS()) {
    digitalWrite(LED_BUILTIN, LOW);  // LED encendido = fix

    if (!enviarPorRadio()) {
      Serial.println("Fallo en el envío.");
    }

  } else {
    digitalWrite(LED_BUILTIN, HIGH); // LED apagado = sin fix
    Serial.println("Buscando Fix GPS...");
  }

  delay(500);
}