#include "FS.h"
#include "SD_MMC.h"
#include "esp32cam.h"

// Ya no necesitamos el btnPin para el disparador automático
const auto RES = esp32cam::Resolution::find(1600, 1200);

void takePicAndSave() {
  static int cnt = 0;

  auto frame = esp32cam::capture();
  if (frame == nullptr) {
    Serial.println("Capture failed!");
    return;
  }
  
  String path = "/img" + String(cnt++) + ".jpg";
  File file = SD_MMC.open(path.c_str(), FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file!");
    return; // Añadido return para evitar intentar escribir si falló el archivo
  }
  frame->writeTo(file);
  Serial.printf("Wrote: %s\n", path.c_str());
  file.close();
}

void enableFlash(bool enable) {
  digitalWrite(GPIO_NUM_4, enable ? HIGH : LOW);
}

void initCamera() {
    using namespace esp32cam;
    Config cfg;
    cfg.setPins(pins::AiThinker);
    cfg.setResolution(RES);
    cfg.setJpeg(80);

    bool ok = Camera.begin(cfg);
    Serial.println(ok ? "CAMERA OK" : "CAMERA FAIL");
}

void initSDCard() {
  // Nota: El segundo parámetro 'true' usa el modo 1-wire, común en ESP32-CAM
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("SD Card Mount Failed!");
  } else if (SD_MMC.cardType() == CARD_NONE) {
    Serial.println("No SD card inserted!");
  } else {
    Serial.println("SD card ready.");
  }
}

void setup() {
  Serial.begin(115200);
  initCamera();
  initSDCard();
  
  pinMode(GPIO_NUM_4, OUTPUT);
  enableFlash(false);  
  
  Serial.println("Iniciando secuencia automática cada 2 segundos...");
  delay(2000); // Espera inicial de cortesía
}

void loop() {
  // Lógica automática
  
  takePicAndSave();    // Captura y guarda
  
  
  // Espera 2000 milisegundos (2 segundos) antes de la siguiente foto
  delay(2000); 
}
