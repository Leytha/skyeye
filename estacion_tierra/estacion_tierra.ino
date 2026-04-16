#include <RadioLib.h>
#include <heltec_unofficial.h>

// ---------------- CONFIGURACIÓN ----------------
// En RadioLib la frecuencia va en MHz (no en Hz)
#define BAND 868.5

// ---------------- VARIABLES GLOBALES ----------------
// Guardan el último valor válido recibido (si un campo viene vacío, no se borra)
String estadoSky = "", temp = "", pres = "", hum = "", alt = "";
String id1 = "", hora1 = "", sat1 = "", lat1 = "", lon1 = "", alt1 = "", vel1 = "";
String id2 = "", hora2 = "", sat2 = "", lat2 = "", lon2 = "", alt2 = "", vel2 = "";

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  heltec_setup();

  // Inicializamos el módulo LoRa
  int state = radio.begin(BAND);

  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("Error LoRa: ");
    Serial.println(state);
    while (1) {
    }
  } else {
    // Estos parámetros tienen que ser iguales que en la baliza
    radio.setSpreadingFactor(8);
    radio.setBandwidth(125.0);
    radio.setCodingRate(5);
    radio.setSyncWord(0x12);
    radio.setCRC(true);

    Serial.println("Estacion lista");
  }
}

// ---------------- LOOP ----------------
void loop() {
  String recibido = "";

  // Intentamos recibir un paquete
  int state = radio.receive(recibido);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Paquete recibido:");
    Serial.println(recibido);

    // El mensaje tiene 19 campos separados por comas
    String campos[19];
    for (int i = 0; i < 19; i++) {
      campos[i] = "";
    }

    // Separamos el mensaje en campos
    int inicio = 0;
    int n = 0;
    for (int i = 0; i < recibido.length(); i++) {
      if (recibido.charAt(i) == ',') {
        if (n < 19) {
          campos[n] = recibido.substring(inicio, i);
          campos[n].trim();
          inicio = i + 1;
          n++;
        }
      }
    }
    // Último campo (no termina en coma)
    if (n < 19 && inicio <= recibido.length()) {
      campos[n] = recibido.substring(inicio);
      campos[n].trim();
    }

    // ---------- SKYEYE ----------
    if (campos[0] != "") { estadoSky = campos[0]; }
    if (campos[1] != "") { temp = campos[1]; }
    if (campos[2] != "") { pres = campos[2]; }
    if (campos[3] != "") { hum = campos[3]; }
    if (campos[4] != "") { alt = campos[4]; }

    // ---------- BALIZA 1 ----------
    if (campos[5] != "") { id1 = campos[5]; }
    if (campos[6] != "") { hora1 = campos[6]; }
    if (campos[7] != "") { sat1 = campos[7]; }
    if (campos[8] != "") { lat1 = campos[8]; }
    if (campos[9] != "") { lon1 = campos[9]; }
    if (campos[10] != "") { alt1 = campos[10]; }
    if (campos[11] != "") { vel1 = campos[11]; }

    // ---------- BALIZA 2 ----------
    if (campos[12] != "") { id2 = campos[12]; }
    if (campos[13] != "") { hora2 = campos[13]; }
    if (campos[14] != "") { sat2 = campos[14]; }
    if (campos[15] != "") { lat2 = campos[15]; }
    if (campos[16] != "") { lon2 = campos[16]; }
    if (campos[17] != "") { alt2 = campos[17]; }
    if (campos[18] != "") { vel2 = campos[18]; }

    // Enviamos todo como una línea CSV (para Processing o similar)
    Serial.print(estadoSky);
    Serial.print(",");
    Serial.print(temp);
    Serial.print(",");
    Serial.print(pres);
    Serial.print(",");
    Serial.print(hum);
    Serial.print(",");
    Serial.print(alt);
    Serial.print(",");
    Serial.print(id1);
    Serial.print(",");
    Serial.print(hora1);
    Serial.print(",");
    Serial.print(sat1);
    Serial.print(",");
    Serial.print(lat1);
    Serial.print(",");
    Serial.print(lon1);
    Serial.print(",");
    Serial.print(alt1);
    Serial.print(",");
    Serial.print(vel1);
    Serial.print(",");
    Serial.print(id2);
    Serial.print(",");
    Serial.print(hora2);
    Serial.print(",");
    Serial.print(sat2);
    Serial.print(",");
    Serial.print(lat2);
    Serial.print(",");
    Serial.print(lon2);
    Serial.print(",");
    Serial.print(alt2);
    Serial.print(",");
    Serial.println(vel2);

    // Esto sirve para ver si la señal llega bien o mal
    Serial.print("RSSI: ");
    Serial.println(radio.getRSSI());
    Serial.print("SNR: ");
    Serial.println(radio.getSNR());

    Serial.println("--------------------");
  } else {
    // Si no entra aquí arriba, es que no ha recibido nada o hay error
    Serial.print("No recibido. Codigo: ");
    Serial.println(state);
  }
}