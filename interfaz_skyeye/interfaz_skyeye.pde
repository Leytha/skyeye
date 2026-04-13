import processing.serial.*;

Serial puerto;

// ---------------- DATOS SERIE ----------------

String linea = "";
String[] datos = new String[19];

// ---------------- HISTÓRICOS ----------------

int maxPuntos = 120;

float[] historialTemp = new float[maxPuntos];
float[] historialPres = new float[maxPuntos];
float[] historialHum  = new float[maxPuntos];
float[] historialAlt  = new float[maxPuntos];

// ---------------- ÚLTIMOS DATOS GPS ----------------

String[] ultimosBaliza1 = new String[6];
String[] ultimosBaliza2 = new String[6];

// ---------------- COLORES ----------------

color blanco = color(255, 255, 255);
color anil = color(63, 81, 181);
color anilClaro = color(236, 240, 255);
color grisTexto = color(45, 45, 45);
color grisSuave = color(220, 220, 220);

// ---------------- SETUP ----------------

void setup() {
  size(1280, 720);

  println(Serial.list());   // revisar puertos
  String puertoNombre = Serial.list()[5];   // cambiar si hace falta
  puerto = new Serial(this, puertoNombre, 115200);
  puerto.bufferUntil('\n');

  textFont(createFont("Arial", 14));

  for (int i = 0; i < maxPuntos; i++) {
    historialTemp[i] = 0;
    historialPres[i] = 0;
    historialHum[i] = 0;
    historialAlt[i] = 0;
  }

  for (int i = 0; i < 19; i++) {
    datos[i] = "";
  }

  inicializarBaliza(ultimosBaliza1);
  inicializarBaliza(ultimosBaliza2);
}

// ---------------- DRAW ----------------

void draw() {
  background(blanco);

  // banda superior
  noStroke();
  fill(anil);
  rect(0, 0, width, 70);

  fill(255);
  textAlign(CENTER, CENTER);
  textSize(26);
  text("SKYEYE: Estación de tierra", width/2, 35);

  // izquierda: balizas
  dibujarTarjetaBaliza(20, 90, 280, 250, "Baliza 1", ultimosBaliza1);
  dibujarTarjetaBaliza(20, 360, 280, 250, "Baliza 2", ultimosBaliza2);

  // derecha: 4 gráficas visibles
  dibujarGrafica(320, 90, 930, 120, "Temperatura (°C)", historialTemp);
  dibujarGrafica(320, 230, 930, 120, "Presión (hPa)", historialPres);
  dibujarGrafica(320, 370, 930, 120, "Humedad (%)", historialHum);
  dibujarGrafica(320, 510, 930, 120, "Altura (m)", historialAlt);
}

// ---------------- TARJETAS BALIZAS ----------------

void dibujarTarjetaBaliza(int x, int y, int w, int h, String titulo, String[] datosBaliza) {
  noStroke();
  fill(anilClaro);
  rect(x, y, w, h, 18);

  fill(anil);
  textAlign(LEFT, TOP);
  textSize(20);
  text(titulo, x + 16, y + 14);

  fill(grisTexto);
  textSize(14);

  int y0 = y + 52;
  int salto = 30;

  text("Hora: " + valorSeguro(datosBaliza[0]),      x + 16, y0);
  text("Satélites: " + valorSeguro(datosBaliza[1]), x + 16, y0 + salto);
  text("Latitud: " + valorSeguro(datosBaliza[2]),   x + 16, y0 + 2 * salto);
  text("Longitud: " + valorSeguro(datosBaliza[3]),  x + 16, y0 + 3 * salto);
  text("Altitud: " + valorSeguro(datosBaliza[4]),   x + 16, y0 + 4 * salto);
  text("Velocidad: " + valorSeguro(datosBaliza[5]), x + 16, y0 + 5 * salto);
}

// ---------------- GRÁFICAS ----------------

void dibujarGrafica(int x, int y, int w, int h, String titulo, float[] historial) {
  noStroke();
  fill(250);
  rect(x, y, w, h, 16);

  fill(anil);
  textAlign(LEFT, TOP);
  textSize(17);
  text(titulo, x + 12, y + 10);

  // zona útil
  int margenIzq = 55;
  int margenDer = 18;
  int margenSup = 32;
  int margenInf = 18;

  int gx = x + margenIzq;
  int gy = y + margenSup;
  int gw = w - margenIzq - margenDer;
  int gh = h - margenSup - margenInf;

  // marco
  stroke(grisSuave);
  noFill();
  rect(gx, gy, gw, gh);

  // min y max
  float minVal = historial[0];
  float maxVal = historial[0];

  for (int i = 1; i < historial.length; i++) {
    if (historial[i] < minVal) minVal = historial[i];
    if (historial[i] > maxVal) maxVal = historial[i];
  }

  // evitar rango cero
  if (abs(maxVal - minVal) < 0.001) {
    maxVal = minVal + 1;
  }

  // etiquetas eje Y
  fill(grisTexto);
  textSize(11);
  textAlign(RIGHT, CENTER);
  text(nf(maxVal, 0, 1), gx - 6, gy + 4);
  text(nf(minVal, 0, 1), gx - 6, gy + gh - 4);

  float medio = (maxVal + minVal) / 2.0;
  float yMedio = map(medio, minVal, maxVal, gy + gh, gy);

  stroke(grisSuave);
  line(gx, yMedio, gx + gw, yMedio);

  fill(grisTexto);
  text(nf(medio, 0, 1), gx - 6, yMedio);

  // línea de datos
  stroke(anil);
  noFill();
  beginShape();
  for (int i = 0; i < historial.length; i++) {
    float px = map(i, 0, historial.length - 1, gx, gx + gw);
    float py = map(historial[i], minVal, maxVal, gy + gh, gy);
    vertex(px, py);
  }
  endShape();

  // valor actual
  fill(anil);
  textAlign(LEFT, CENTER);
  textSize(12);
  text("Actual: " + nf(historial[historial.length - 1], 0, 1), gx + gw - 95, y + 16);
}

// ---------------- RECEPCIÓN SERIE ----------------

void serialEvent(Serial puerto) {
  linea = puerto.readStringUntil('\n');

  if (linea != null) {
    linea = trim(linea);
    println("RECIBIDO -> " + linea);

    String[] partes = split(linea, ',');
    println("CAMPOS -> " + partes.length);

    if (partes.length >= 19) {
      for (int i = 0; i < 19; i++) {
        datos[i] = trim(partes[i]);
      }

      // gráficas
      actualizarHistorial(historialTemp, parseSeguro(datos[1]));
      actualizarHistorial(historialPres, parseSeguro(datos[2]));
      actualizarHistorial(historialHum,  parseSeguro(datos[3]));
      actualizarHistorial(historialAlt,  parseSeguro(datos[4]));

      // baliza 1
      actualizarBaliza(ultimosBaliza1, datos[6], datos[7], datos[8], datos[9], datos[10], datos[11]);

      // baliza 2
      actualizarBaliza(ultimosBaliza2, datos[13], datos[14], datos[15], datos[16], datos[17], datos[18]);
    } else {
      println("Línea ignorada. Campos recibidos: " + partes.length);
    }
  }
}

// ---------------- FUNCIONES AUXILIARES ----------------

void actualizarHistorial(float[] historial, float nuevoValor) {
  for (int i = 0; i < historial.length - 1; i++) {
    historial[i] = historial[i + 1];
  }
  historial[historial.length - 1] = nuevoValor;
}

float parseSeguro(String s) {
  if (s == null || s.equals("")) {
    return 0;
  }

  try {
    return float(s);
  } 
  catch(Exception e) {
    return 0;
  }
}

String valorSeguro(String s) {
  if (s == null || s.equals("")) {
    return "-";
  }
  return s;
}

void inicializarBaliza(String[] baliza) {
  for (int i = 0; i < 6; i++) {
    baliza[i] = "-";
  }
}

void actualizarBaliza(String[] baliza, String hora, String sat, String lat, String lon, String alt, String vel) {
  if (hora != null && !hora.equals("")) baliza[0] = hora;
  if (sat  != null && !sat.equals(""))  baliza[1] = sat;
  if (lat  != null && !lat.equals(""))  baliza[2] = lat;
  if (lon  != null && !lon.equals(""))  baliza[3] = lon;
  if (alt  != null && !alt.equals(""))  baliza[4] = alt;
  if (vel  != null && !vel.equals(""))  baliza[5] = vel;
}
