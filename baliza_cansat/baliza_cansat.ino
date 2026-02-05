#include "Arduino.h"
#include "LoRa_E32.h"
#include <SoftwareSerial.h>

// Configuración de pines para Arduino Uno
SoftwareSerial mySerial(2, 3);    // RX (al TX del E32), TX (al RX del E32)
LoRa_E32 e32(&mySerial, 4, 5, 6); // AUX, M0, M1

//Configuración de canal - Ejemplo: Para obtener 870 MHz -> 870 - 862 = 8. Ponemos canal 8.
#define RADIO_CHAN 0x08    

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Configurando emisor LoRa E32...");

  // Inicializa el módulo
  e32.begin();
  
  // Cambiar la configuración actual del módulo
  ResponseStructContainer rsc = e32.getConfiguration();
  Configuration configuration = *(Configuration*)rsc.data;
  configuration.CHAN = RADIO_CHAN; 
  e32.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);   // WRITE_CFG_PWR_DWN_SAVE: Se guarda aunque quites la batería.
  rsc.close();
  Serial.println("Configuración de frecuencia actualizada.");

  // M0 y M1 en LOW para "Modo Normal" (Transmisión transparente)
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);

  Serial.println("Listo para enviar.");
}

void loop() {
  // El mensaje que quieres mandar
  String mensaje = "Hola desde Arduino!";

  Serial.print("Enviando: ");
  Serial.println(mensaje);

  // Comando para enviar el string
  ResponseStatus rs = e32.sendMessage(mensaje);
  
  // Verificamos si salió bien
  if (rs.code != 1) {
    Serial.println("Error al enviar: " + rs.getResponseDescription());
  } else {
    Serial.println("¡Enviado con éxito!");
  }

  delay(2000); // Espera 2 segundos para el siguiente envío
}