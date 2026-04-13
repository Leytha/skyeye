#include "Arduino.h"
#include "LoRa_E32.h"
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// --- CONFIGURACIÓN GENERAL ---

// Cambiar solo este valor:
// 1 -> baliza 1
// 2 -> baliza 2
#define ID_BALIZA 1

// --- Pines de Conexión ---

// LoRa (módulo E32)
#define RX_LORA 4
#define TX_LORA 5
#define AUX 6
#define M0 7
#define M1 8

// GPS (NEO-6M)
#define RX_GPS 4
#define TX_GPS 3

#define LED_GPS 13
// --- Objetos y Configuración ---
TinyGPSPlus gps;

// Puerto serie para GPS
SoftwareSerial gps_serial(RX_GPS, TX_GPS);

// Puerto serie para LoRa
SoftwareSerial lora_serial(RX_LORA, TX_LORA);

// Instancia del módulo LoRa E32
LoRa_E32 e32(&lora_serial, AUX, M0, M1);

// Canal de radio
#define RADIO_CHAN 0x06

// Tiempo de escucha del GPS (ms)
#define T_GPS 1000


// --- FUNCIONES DE APOYO ---

// Configura el módulo LoRa
void configurarLora() {
  e32.begin();

  ResponseStructContainer rsc = e32.getConfiguration();
  Configuration configuration = *(Configuration*)rsc.data;

  configuration.CHAN = RADIO_CHAN;

  // Forzamos parámetros compatibles con Heltec
  configuration.SPED.airDataRate = AIR_DATA_RATE_010_24;  // 2.4 kbps
  configuration.SPED.uartBaudRate = UART_BPS_9600;
  configuration.SPED.uartParity = MODE_00_8N1;

  configuration.OPTION.transmissionPower = POWER_20;
  configuration.OPTION.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;

  e32.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
  rsc.close();

  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);

  Serial.println(F("LoRa configurado."));
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
  // Se reducen etiquetas y decimales para no superar los 58 bytes del E32
  String sats = String(gps.satellites.value());
  String lat = String(gps.location.lat(), 4);
  String lon = String(gps.location.lng(), 4);
  String alt = String(gps.altitude.meters(), 1);
  String vel = String(gps.speed.kmph(), 1);

  // --- 3. Inicializamos los 19 campos vacíos ---
  // campo 1  -> campos[0]
  // campo 19 -> campos[18]
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
  campos[inicioBloque + 0] = String(ID_BALIZA);  // ID de baliza
  campos[inicioBloque + 1] = String(hora);       // HH:MM:SS
  campos[inicioBloque + 2] = sats;               // Satélites
  campos[inicioBloque + 3] = lat;                // Latitud
  campos[inicioBloque + 4] = lon;                // Longitud
  campos[inicioBloque + 5] = alt;                // Altura
  campos[inicioBloque + 6] = vel;                // Velocidad

  // --- 6. Construimos el mensaje final ---
  String mensaje = construirMensajeDesdeCampos(campos, 19);

  // --- 7. Envío por radio ---
  Serial.println(F("Enviando mensaje:"));
  Serial.println(mensaje);

  lora_serial.listen();
  delay(50);

  ResponseStatus rs = e32.sendMessage(mensaje);

  if (rs.code != 1) {
    Serial.println(rs.getResponseDescription());
  } else {
    Serial.println(F("OK!"));
  }

  delay(100);
}


// --- BLOQUES PRINCIPALES ---

void setup() {
  Serial.begin(9600);
  gps_serial.begin(9600);
  pinMode(LED_GPS, OUTPUT);

  configurarLora();

  Serial.println(F("Sistema listo. Esperando señal de satélites..."));
}

void loop() {

if (obtenerDatosGPS()) {
  digitalWrite(LED_GPS, HIGH);
  enviarPorRadio();
} else {
  digitalWrite(LED_GPS, LOW); 
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