#include "Arduino.h"
#include <SPI.h>
#include <LoRa.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// --- CONFIGURACIÓN GENERAL ---

// Cambiar solo este valor:
// 1 -> baliza 1
// 2 -> baliza 2
#define ID_BALIZA 1

// --- Pines de Conexión ---

// LoRa (módulo SX1276)
#define NSS_LORA 10
#define RST_LORA 8
#define DIO0_LORA 2
#define PIN_MOSI 11
#define PIN_MISO 12
#define PIN_SCK 13

// GPS (NEO-6M)
#define RX_GPS 4
#define TX_GPS 3

// Frecuencia LoRa
#define BAND 868E6

// Tiempo de escucha del GPS (ms)
#define T_GPS 1000

// --- Objetos y Configuración ---
TinyGPSPlus gps;

// Puerto serie para GPS
SoftwareSerial gps_serial(RX_GPS, TX_GPS);


// --- FUNCIONES DE APOYO ---

// Configura el módulo LoRa
void configurarLora() {
  LoRa.setPins(NSS_LORA, RST_LORA, DIO0_LORA);

  if (!LoRa.begin(BAND)) {
    Serial.println(F("Error al iniciar LoRa."));
  } 
  else {
    // Parámetros LoRa compatibles con la Heltec
    LoRa.setSpreadingFactor(11);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.setSyncWord(0x12);
    Serial.println(F("LoRa configurado."));
  }
}


// Escucha el GPS y procesa las tramas NMEA
// Devuelve true si se ha obtenido una posición válida
bool obtenerDatosGPS() {
  gps_serial.listen();

  unsigned long start = millis();
  bool nuevoDato = false;

  while (millis() - start < T_GPS) {
    while (gps_serial.available()) {
      if (gps.encode(gps_serial.read())) {
        if (gps.location.isValid()) {
          nuevoDato = true;
        }
      }
    }
  }
  return nuevoDato;
}


// Une los 19 campos con comas para formar el mensaje final
String construirMensajeDesdeCampos(String campos[], int numCampos) {
  String mensaje = "";

  for (int i = 0; i < numCampos; i++) {
    mensaje += campos[i];
    mensaje += ",";
  }

  return mensaje;
}


// Construye el mensaje universal y lo envía por radio
void enviarPorRadio() {

  // --- 1. Formateo de la hora ---
  char hora[9];
  sprintf(hora, "%02d:%02d:%02d",
          gps.time.hour(),
          gps.time.minute(),
          gps.time.second());

  // --- 2. Conversión de datos GPS a texto ---
  String sats = String(gps.satellites.value());
  String lat = String(gps.location.lat(), 4);
  String lon = String(gps.location.lng(), 4);
  String alt = String(gps.altitude.meters(), 1);
  String vel = String(gps.speed.kmph(), 1);

  // --- 3. Inicializamos los 19 campos vacíos ---
  String campos[19];

  for (int i = 0; i < 19; i++) {
    campos[i] = "";
  }

  // --- 4. Calculamos dónde empieza el bloque de la baliza ---
  // Baliza 1 -> empieza en campo 6  -> índice 5
  // Baliza 2 -> empieza en campo 13 -> índice 12
  int inicioBloque = -1;

  if (ID_BALIZA == 1) {
    inicioBloque = 5;
  } else if (ID_BALIZA == 2) {
    inicioBloque = 12;
  } else {
    Serial.println(F("Error: ID_BALIZA debe ser 1 o 2."));
    return;
  }

  // --- 5. Rellenamos solo el bloque correspondiente a esta baliza ---
  // Formato compacto:
  // id,hora,sats,lat,lon,alt,vel
  campos[inicioBloque + 0] = String(ID_BALIZA);
  campos[inicioBloque + 1] = String(hora);
  campos[inicioBloque + 2] = sats;
  campos[inicioBloque + 3] = lat;
  campos[inicioBloque + 4] = lon;
  campos[inicioBloque + 5] = alt;
  campos[inicioBloque + 6] = vel;

  // --- 6. Construimos el mensaje final ---
  String mensaje = construirMensajeDesdeCampos(campos, 19);

  // --- 7. Envío por radio ---
  Serial.println(F("Enviando mensaje:"));
  Serial.println(mensaje);

  LoRa.beginPacket();
  LoRa.print(mensaje);
  LoRa.endPacket();

  Serial.println(F("OK!"));

  delay(100);
}


// --- BLOQUES PRINCIPALES ---

void setup() {
  Serial.begin(115200);
  gps_serial.begin(9600);

  configurarLora();

  Serial.println(F("Sistema listo. Esperando señal de satélites..."));
}

void loop() {

  // Solo se envía si el GPS tiene posición válida
  if (obtenerDatosGPS()) {
    enviarPorRadio();
  } else {
    Serial.println(F("Buscando Fix GPS..."));
  }

  /*
  gps_serial.listen();

  while (gps_serial.available()) {
    char c = gps_serial.read();
    Serial.write(c);
  }
  */
}