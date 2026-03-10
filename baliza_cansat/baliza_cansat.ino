#include "Arduino.h"
#include "LoRa_E32.h"
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// --- Pines de Conexión ---
#define RX_LORA 2
#define TX_LORA 7
#define AUX 8
#define M0 5
#define M1 6

#define RX_GPS 4
#define TX_GPS 3

// --- Objetos y Configuración ---
TinyGPSPlus gps;
SoftwareSerial gps_serial(RX_GPS, TX_GPS);   // Puerto serie para el GPS
SoftwareSerial lora_serial(RX_LORA, TX_LORA); // Puerto serie para el LoRa
LoRa_E32 e32(&lora_serial, AUX, M0, M1);      // Instancia del módulo LoRa

#define RADIO_CHAN 0x08  // Canal de frecuencia
#define T_GPS 1000       // Tiempo de escucha del GPS (ms)

// --- FUNCIONES DE APOYO ---

// Configura los parámetros internos del módulo LoRa
void configurarLora() {
  e32.begin();
  ResponseStructContainer rsc = e32.getConfiguration();
  Configuration configuration = *(Configuration*)rsc.data;
  configuration.CHAN = RADIO_CHAN;
  // Guarda la configuración permanentemente
  e32.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
  rsc.close();
  
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  digitalWrite(M0, LOW); // Modo Normal
  digitalWrite(M1, LOW);
  Serial.println(F("LoRa configurado."));
}

// Escucha el puerto del GPS y procesa las sentencias NMEA
bool obtenerDatosGPS() {
  gps_serial.listen(); // Activa la escucha en los pines 4 y 3
  unsigned long start = millis();
  bool nuevoDato = false;

  while (millis() - start < T_GPS) {
    while (gps_serial.available()) {
      if (gps.encode(gps_serial.read())) {
        // Solo es válido si tiene Fix (posición real)
        if (gps.location.isValid()) {
          nuevoDato = true;
        }
      }
    }
  }
  return nuevoDato;
}

// Crea la cadena de texto y la envía por el aire
void enviarPorRadio() {
  // Extraemos tiempo y fecha
  // Usamos sprintf para asegurar que si la hora es "9", se vea como "09"
  char reloj[7]; 
  sprintf(reloj, "%02d%02d%02d", gps.time.hour(), gps.time.minute(), gps.time.second());

  // Construimos el mensaje de telemetría extendido
  // T: Hora | D: Fecha | L: Lat,Lon | A: Alt | V: Vel | S: Sat
  String mensaje = "T:" + String(reloj) + 
                   "|D:" + String(gps.date.day()) + "/" + String(gps.date.month()) +
                   "|L:" + String(gps.location.lat(), 5) + "," + String(gps.location.lng(), 5) + 
                   "|A:" + String(gps.altitude.meters(), 0) + 
                   "|V:" + String(gps.speed.kmph(), 1) + 
                   "|S:" + String(gps.satellites.value());

  Serial.print(F("Enviando Pack con hora -> "));
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
  Serial.begin(115200);   // Monitor Serie del PC
  gps_serial.begin(9600); // Velocidad estándar GPS
  configurarLora();
  Serial.println(F("Sistema listo. Esperando señal de satélites..."));
}

void loop() {
  // Si hay posición válida, se dispara el envío por radio
  if (obtenerDatosGPS()) {
    enviarPorRadio();
  } else {
    // Si no hay Fix, avisamos por el monitor cada segundo
    Serial.println(F("Buscando Fix GPS..."));
  }
}