#include <RadioLib.h>
#include <heltec_unofficial.h>
#include <SPI.h>
#include <LoRa.h>
// --- CONFIGURACIÓN DE PINES I2C ---
#define WIFI_LoRa_32
#define BAND 868E6
#define t_ciclo 1000


unsigned long tiempoAnterior;


void setup() {
  Serial.begin(115200);
  //while (!Serial) ;
  Serial.println("Comunicación serie establecida");
  // En el setup:
  heltec_setup();
  int state = radio.begin(868.0);
  Serial.print(F("[LoRa] Inicializando... "));
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("Radio iniciada"));
  } else {
    Serial.print(F("Radio falló, código: "));
    Serial.println(state);
    //while (true);
  }

  tiempoAnterior = millis();

  
}
void loop() {

  // delay(250);

  // while (millis() - tiempoAnterior <= t_ciclo) {
  //   delay(1);
  // }
  // tiempoAnterior = millis();

  String str;
  // receive() es una función bloqueante: espera hasta recibir algo o timeout
  int state = radio.receive(str);

  if (state == RADIOLIB_ERR_NONE) {
    // Se recibió paquete
    Serial.println(F("¡Recibido!"));
    Serial.print(F("[LoRa] Datos:\t"));
    Serial.println(str);
    
    // Mostrar RSSI (fuerza de señal)
    Serial.print(F("[LoRa] RSSI:\t"));
    Serial.print(radio.getRSSI());
    Serial.println(F(" dBm"));
  }
   else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
    // Se acabó el tiempo de espera
    Serial.println(F("Tiempo de espera agotado."));
  } else {
    // Otro error
    Serial.print(F("Fallo al recibir, código: "));
    Serial.println(state);
  }
}





  
}