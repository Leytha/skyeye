//#include "heltec.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <RadioLib.h>
#include <heltec_unofficial.h>

// --- CONFIGURACIÓN DE PINES I2C ---
#define PIN_SDA 19  // Cambia este número por tu pin de datos (ej. 21 en ESP32, 4 en ESP8266)
#define PIN_SCL 20  // Cambia este número por tu pin de reloj (ej. 22 en ESP32, 5 en ESP8266)

#define BAND 868E6
#define SEALEVELPRESSURE_HPA (1013.25)
#define t_ciclo 1000

// Definición de pines para Heltec V3
// NSS: 8, DIO1: 14, NRST: 12, BUSY: 13
//SX1262 radio = new Module(8, 14, 12, 13);

Adafruit_BME280 bme;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("Comunicación serie establecida");

  // Inicializamos el bus I2C con tus pines definidos arriba
  // Si usas un Arduino Uno/Nano, ignora estos parámetros (usa A4/A5)
  Wire.begin(PIN_SDA, PIN_SCL);

  // Iniciamos el sensor. Dirección común: 0x76 o 0x77
  unsigned status = bme.begin(0x76);

  if (!status) {
    Serial.println("No se encuentra el sensor BME280. ¡Revisa conexiones y dirección!");
    while (1) delay(10);


    // Inicializar el chip LoRa
    //Encender radio
    //pinMode(18, OUTPUT);
    //digitalWrite(18, LOW);  // En la V3, LOW activa la energía
    //delay(100);             // Dar tiempo a que el chip despierte
    Serial.print(F("[LoRa] Inicializando... "));
    // Frecuencia: 433.0, 868.0 o 915.0 MHz (según tu región)
    // Ancho de banda: 125.0, Factor de esparcimiento: 9, Tasa de código: 7
    int state = radio.begin(868.0);

    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("¡Éxito!"));
    } else {
      Serial.print(F("Falló, código: "));
      Serial.println(state);
      while (true)
        ;
    }
  }


  Serial.println("-- Sensor BME280 configurado correctamente --");
}

void loop() {
  Serial.print("Temperatura = ");
  Serial.print(bme.readTemperature());
  Serial.println(" °C");

  Serial.print("Presión = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Altitud aprox. = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humedad = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");
  Serial.println("-----------------------");

  //Radio
  Serial.println(F("[LoRa] Enviando paquete... "));

  //Enviar mensaje
  int state = radio.transmit("Hola desde Heltec V3!");

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("¡Enviado correctamente!"));
  } else {
    Serial.print(F("Error al enviar, código: "));
    Serial.println(state);
  }
  /*Enviar mensaje por radio
    // 2. Formatear mensaje (formato compacto para radio)
    String mensaje = "T:" + String(t, 1) + " H:" + String(h, 0) + " P:" + String(p, 0);

    // 3. Enviar por LoRa
    LoRa.beginPacket();
    LoRa.print(mensaje);
    LoRa.endPacket();
    */


  delay(t_ciclo);  //Cambiar por  if (millis() - tiempoAnterior >= intervalo) {
                   // Guarda el tiempo actual como el nuevo tiempo de inicio
                   //tiempoAnterior = tiempoActual;
                   //...
                   //}
                   //delay(1)
}
