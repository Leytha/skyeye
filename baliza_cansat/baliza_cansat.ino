#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
// CONFIGURACIÓN
#define ID_BALIZA 1
#define BAND 868500000
#define TIEMPO_LECTURA_GPS 3000
// PINES LORA
#define NSS_LORA D8
#define RST_LORA D0
#define DIO0_LORA D2
// PINES GPS
#define RX_GPS D1
#define TX_GPS D3
// OBJETOS
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RX_GPS, TX_GPS);
// VARIABLES DE ESTADO
bool loraInicializada = false;
bool gpsConectado = false;
bool gpsConFix = false;
// FUNCIÓN PARA CREAR EL MENSAJE
String crearMensaje() {
  String campos[19];
  for (int i = 0; i < 19; i++) {
    campos[i] = "";
  }
  int inicioBloque;
  if (ID_BALIZA == 1) {
    inicioBloque = 5;
  }
  else {
    inicioBloque = 12;
  }
  String hora;
  if (gps.time.isValid()) {
    char textoHora[9];
    sprintf(textoHora, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
    hora = String(textoHora);
  }
  else {
    hora = "SIN_HORA";
  }
  String satelites;
  if (gps.satellites.isValid()) {
    satelites = String(gps.satellites.value());
  }
  else {
    satelites = "0";
  }
  String latitud;
  String longitud;
  String altitud;
  String velocidad;
  if (gpsConFix) {
    latitud = String(gps.location.lat(), 4);
    longitud = String(gps.location.lng(), 4);
    altitud = String(gps.altitude.meters(), 1);
    velocidad = String(gps.speed.kmph(), 1);
  }
  else {
    latitud = "SIN_FIX";
    longitud = "SIN_FIX";
    altitud = "SIN_FIX";
    velocidad = "SIN_FIX";
  }
  campos[inicioBloque + 0] = String(ID_BALIZA);
  campos[inicioBloque + 1] = hora;
  campos[inicioBloque + 2] = satelites;
  campos[inicioBloque + 3] = latitud;
  campos[inicioBloque + 4] = longitud;
  campos[inicioBloque + 5] = altitud;
  campos[inicioBloque + 6] = velocidad;
  String mensaje = "";
  for (int i = 0; i < 19; i++) {
    mensaje += campos[i];
    if (i < 18) {
      mensaje += ",";
    }
  }
  return mensaje;
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);
  delay(1000);
  LoRa.setPins(NSS_LORA, RST_LORA, DIO0_LORA);
  if (!LoRa.begin(BAND)) {
    Serial.println("ERROR: LoRa no se inicializó.");
    loraInicializada = false;
    return;
  }
  else {
    LoRa.setSpreadingFactor(11);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.setSyncWord(0x12);
    LoRa.enableCrc();
    loraInicializada = true;
    Serial.println("LoRa configurado correctamente.");
  }
}

void loop() {
  if (!loraInicializada) {
    Serial.println("LoRa sigue sin inicializarse.");
    delay(TIEMPO_LECTURA_GPS);
    return;
  }
  gpsConectado = false;
  gpsConFix = false;
  unsigned long inicio = millis();
  while (millis() - inicio < TIEMPO_LECTURA_GPS) {
    while (gpsSerial.available()) {
      char c = gpsSerial.read();
      gpsConectado = true;
      gps.encode(c);
    }
  }
  if (gps.location.isValid()) {
    gpsConFix = true;
  }
  if (!gpsConectado) {
    Serial.println("GPS sin conexión.");
  }
  else {
    String mensaje = crearMensaje();
    Serial.println("Mensaje:");
    Serial.println(mensaje);
    if (LoRa.beginPacket() == 0) {
      Serial.println("ERROR: beginPacket() falló.");
    }
    else {
      LoRa.print(mensaje);
      if (LoRa.endPacket() == 1) {
        Serial.println("Enviado OK");
      }
      else {
        Serial.println("ERROR: endPacket() falló.");
      }
    }
  }
  Serial.println("--------------------");
}