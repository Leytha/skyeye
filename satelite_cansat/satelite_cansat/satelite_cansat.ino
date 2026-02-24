#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <RadioLib.h>
#include <heltec_unofficial.h>
// --- CONFIGURACIÓN DE PINES I2C ---
#define PIN_SDA 19
#define PIN_SCL 20
#define WIFI_LoRa_32
#define BAND 868E6
#define SEALEVELPRESSURE_HPA (1013.25)
#define t_ciclo 1000
unsigned long tiempoAnterior;
Adafruit_BME280 bme;
TwoWire I2CBME = TwoWire(1);
void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("Comunicación serie establecida");
  // En el setup:
  heltec_setup();

  // Inicializas tu SEGUNDO bus I2C en los pines 19 y 20
  I2CBME.begin(PIN_SDA, PIN_SCL, 100000);

  // Le dices al BME que use I2CBME en lugar del Wire normal
  if (!bme.begin(0x76, &I2CBME)) {
    Serial.println("No se encuentra el BME280");
    while (1)
      ;
  }

  Serial.println("-- Sensor BME280 configurado correctamente --");


  int state = radio.begin(868.0);
  Serial.print(F("[LoRa] Inicializando... "));
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("Radio iniciada"));
  } else {
    Serial.print(F("Radio falló, código: "));
    Serial.println(state);
    while (true)
      ;
  }
  tiempoAnterior = millis();
}
void loop() {
  float t = bme.readTemperature();
  Serial.print("Temperatura = ");
  Serial.print(t);
  Serial.println(" °C");
  float p = bme.readPressure() / 100.0F;
  Serial.print("Presión = ");
  Serial.print(p);
  Serial.println(" hPa");
  float a = bme.readAltitude(SEALEVELPRESSURE_HPA);
  Serial.print("Altitud aprox. = ");
  Serial.print(a);
  Serial.println(" m");
  float h = bme.readHumidity();
  Serial.print("Humedad = ");
  Serial.print(h);
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
  delay(250);
  //Enviar mensaje por radio
  //2. Formatear mensaje (formato compacto para radio)
   String mensaje = "T:" + String(t, 1) + " H:" + String(h, 0) + " P:" + String(p, 0) + " A:" String(a, 0);
  // 3. Enviar por LoRa
  // LoRa.beginPacket();
  //LoRa.print(mensaje);
  //LoRa.endPacket();
  //while (millis() - tiempoAnterior <= t_ciclo) {
  // delay(1);
}
//tiempoAnterior = millis();
