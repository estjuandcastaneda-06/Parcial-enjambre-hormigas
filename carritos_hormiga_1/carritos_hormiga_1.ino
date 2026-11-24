/*
 * CARRITOS HORMIGA 1 — versión SIMPLE: encontrar SU vaso y quedarse quieto.
 * Sensor de color GRANDE: TCS3200 (el de 4 LEDs).
 * ---------------------------------------------------------------------------
 * Qué hace:
 *   1. Anda en línea recta dentro del espacio (1,5 m x 1,5 m) y cambia de rumbo cada tanto.
 *   2. Si va a salirse del espacio, gira hacia el centro.
 *   3. Si el VL53L0X ve algo DENTRO del espacio, se acerca despacio y frena a 4 cm.
 *   4. Lee el color:  - si es SU vaso  -> se queda quieto, LED verde titilando.
 *                     - si no lo es    -> retrocede, gira y sigue buscando.
 *
 * Calibración del color (MUY fácil, y se guarda sola en el ESP32 aunque lo apagues):
 *   'o' con SU vaso frente al sensor de color (a ~2 cm).
 *   '1'..'4' con los OTROS vasos (o cualquier cosa que lo confunda, p. ej. otro azul).
 *   'k' para ver en vivo si reconoce bien (dice SÍ / NO).
 *
 * Conexiones: L298N (25,26,27 / 14,32,4) · VL53L0X y MPU6050 en SDA 21, SCL 22 ·
 *   TCS3200: VCC 3V3, GND, S0 3V3, S1 GND, S2 -> 18, S3 -> 19, OUT -> 23 · LED WS2812 -> 5 ·
 *   divisor de batería -> 33.
 * Librería: "Adafruit_VL53L0X". Placa: "ESP32 Dev Module".
 *
 * Comandos por el monitor serie (115200):
 *   x  arrancar / parar                 r  "estoy en la esquina de inicio" (reinicia la posición)
 *   o  guardar el color de MI vaso      1..4  guardar colores que NO son mi vaso
 *   0  borrar los colores 1..4          k  ver el color en vivo 10 s (SÍ / NO)
 *   t  leer todos los sensores          f  ver el sensor de distancia 5 s
 *   i  ver qué sensores I2C responden   m  probar cada motor
 *   a  avanzar 25 cm                    + / -  se pasó / se quedó corto con 'a'
 *   g  girar 90° a la derecha           d  ver posición y estado
 *   p  ver la calibración guardada      B  borrar TODA la calibración
 *   l  probar el LED
 */
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>          // guarda la calibración en la memoria del ESP32
#include <Adafruit_VL53L0X.h>

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// ============================================================================
// 1. EL ESPACIO
// ============================================================================
static const float ARENA_MM        = 1500.0f;   // lado del espacio (6 cuadros x 25 cm)
static const float MARGEN_BORDE_MM = 180.0f;    // el centro del carro no se acerca más que esto al borde
// Dónde arranca: centro del cuadro de la esquina de arriba a la izquierda, mirando a la
// derecha (a lo largo del borde de arriba). x hacia la derecha, y hacia abajo, en mm.
static const float INICIO_X_MM = 125.0f, INICIO_Y_MM = 125.0f;
static const float INICIO_RUMBO = 0.0f;         // 0 = derecha, 90 = abajo, 180 = izquierda, -90 = arriba
static const float TRAMO_MIN_MM = 300.0f, TRAMO_MAX_MM = 900.0f;   // cuánto anda derecho antes de cambiar de rumbo
static const uint16_t LARGO_FRENTE_MM = 90;     // del centro del carro al sensor de distancia

// ============================================================================
// 2. PINES
// ============================================================================
static const uint8_t PIN_VEL_I = 25, PIN_DIR_I1 = 26, PIN_DIR_I2 = 27;   // motor izquierdo: ENA, IN1, IN2
static const uint8_t PIN_VEL_D = 14, PIN_DIR_D1 = 32, PIN_DIR_D2 = 4;    // motor derecho:  ENB, IN3, IN4
static const uint8_t PIN_TCS_S2 = 18, PIN_TCS_S3 = 19, PIN_TCS_OUT = 23; // TCS3200 (S0 a 3V3, S1 a GND)
static const uint8_t PIN_SDA = 21, PIN_SCL = 22;                         // VL53L0X + MPU6050
static const uint8_t PIN_LED = 5, PIN_VBAT = 33;

// ============================================================================
// 3. MOVIMIENTO
// ============================================================================
static const int   PWM_CRUCERO  = 230;          // fuerza de los motores al buscar (0..255)
static const int   PWM_LENTO    = 190;          // fuerza al acercarse a un objeto
static const int   PWM_GIRO_MIN = 185, PWM_GIRO_MAX = 240;
static const float KP_GIRO  = 3.0f;             // qué tan fuerte gira según lo que le falta
static const float KP_RECTO = 4.0f;             // qué tan fuerte se endereza al ir derecho
static const int   RAMPA_PWM = 30;              // arranque suave: cambio máximo de PWM cada 20 ms
static const float TOL_GIRO_GRADOS = 4.0f;      // un giro termina a menos de 4° del rumbo pedido
static const uint32_t T_GIRO_MAX_MS = 4000;     // si un giro tarda más, se da por terminado
static const uint32_t PWM_FREQ = 1000;
static const bool   AUTO_SIGNO_GYRO = true;     // al prender averigua cómo quedó montado el giroscopio
static const int8_t SIGNO_GYRO_DEFECTO = -1;
// Velocidades estimadas (sin encoder). Se ajustan con 'a' y '+'/'-' y se GUARDAN solas.
static float MM_S_CRUCERO = 150.0f;
static float MM_S_LENTO   = 70.0f;
static float VBAT_VEL     = 0.0f;               // batería con la que se ajustaron (para compensar)

// ============================================================================
// 4. SENSORES
// ============================================================================
static const uint16_t DIST_VER_MM      = 300;   // algo más cerca que esto = va a revisarlo
static const uint16_t DIST_LEER_MM     = 40;    // frena a esta distancia y lee el color
static const float    MAX_ACERCA_MM    = 350.0f;
static const float    RETROCESO_MM     = 150.0f; // si no era su vaso, retrocede esto
static const uint32_t TOF_SIN_DATOS_MS = 600;   // sin datos del VL53L0X en este tiempo = se detiene
// Color: "dif" = qué tan distinto es lo leído del color guardado (0 = idéntico).
// El límite de cada color se calcula solo al guardarlo, entre estos dos valores:
static const float UMBRAL_MIN = 0.025f, UMBRAL_MAX = 0.08f;
static const float PESO_BRILLO = 0.06f;         // cuánto cuenta lo claro/oscuro
static const float VBAT_MIN = 6.4f;
static const float DIVISOR_VBAT = (100.0f + 47.0f) / 47.0f;
static const float VBAT_CAL = 1.36f;            // corrección del ADC (multímetro / lo que mide el ESP32)

// ============================================================================
// 5. ESTADO
// ============================================================================
enum Estado : uint8_t { E_INICIO, E_BUSCA, E_GIRA, E_ACERCA, E_LEE, E_RETROCEDE,
                        E_OBJETIVO, E_PARADO, E_BATERIA, E_SENSOR };
static const char* NOMBRE_ESTADO[] = {"inicio", "busca", "gira", "acerca", "lee", "retrocede",
                                      "OBJETIVO", "parado", "bateria", "sensor"};
enum Mov : uint8_t { MOV_NADA, MOV_GIRO, MOV_RECTO };

static Estado estado = E_INICIO;
static uint32_t tEstado = 0;                    // cuándo entró al estado actual
static float posX = INICIO_X_MM, posY = INICIO_Y_MM;   // dónde cree que está (mm)

static Mov mov = MOV_NADA;
static float rumbo = 0, rumboObjetivo = 0, gzBias = 0;
static int8_t signoGyro = SIGNO_GYRO_DEFECTO;
static bool mpuOk = false, signoDetectado = false;
static float distObjetivo = 0, distHecha = 0;
static bool movLento = false;
static uint32_t tGiroOk = 0, tGiroIni = 0, tQuieto = 0;
static int pwmI = 0, pwmD = 0;

static Adafruit_VL53L0X lox;
static bool tofOk = false;
static uint16_t tofMm = 8190, tofCrudo = 8190, tofAnterior = 8190;
static uint8_t  tofEstado = 255;
static uint32_t tTof = 0;

static float vbatActual = 0;
static uint8_t vbatBajas = 0;
static bool colorOk = false;

// Un color guardado: {proporción rojo, verde, azul, ln(brillo)}, su límite y si es válido.
struct ColorRef { float v[4]; float umbral; bool ok; };
static ColorRef miVaso;
static ColorRef otros[4];
static Preferences prefs;

// ============================================================================
// 6. UTILIDADES
// ============================================================================
static inline float envolver180(float a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static float azar() { return (esp_random() >> 8) / 16777216.0f; }          // 0..1
static float azarEntre(float a, float b) { return a + (b - a) * azar(); }
static float signoAzar() { return azar() < 0.5f ? -1.0f : 1.0f; }
static void cambiar(Estado e) { estado = e; tEstado = millis(); }

static void ponerLED(uint8_t r, uint8_t g, uint8_t b) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  rgbLedWrite(PIN_LED, r, g, b);
#else
  neopixelWrite(PIN_LED, r, g, b);
#endif
}
static float leerVBat() {                       // promedio de 8 lecturas del divisor
  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(PIN_VBAT);
  return mv / 8.0f / 1000.0f * DIVISOR_VBAT * VBAT_CAL;
}
static float factorBateria() {                  // con menos batería anda más lento
  if (VBAT_VEL < 3.0f || vbatActual < 3.0f) return 1.0f;
  return clampf(vbatActual / VBAT_VEL, 0.75f, 1.25f);
}

// ---- guardar / cargar la calibración en la memoria del ESP32 ----
static void guardarCalibracion() {
  prefs.begin("hormiga", false);
  prefs.putBytes("mivaso", &miVaso, sizeof(miVaso));
  prefs.putBytes("otros", otros, sizeof(otros));
  prefs.putFloat("mms", MM_S_CRUCERO);
  prefs.putFloat("mmsl", MM_S_LENTO);
  prefs.putFloat("vbv", VBAT_VEL);
  prefs.end();
}
static void cargarCalibracion() {
  memset(&miVaso, 0, sizeof(miVaso)); memset(otros, 0, sizeof(otros));
  if (!prefs.begin("hormiga", true)) return;     // primera vez: no hay nada guardado
  if (prefs.getBytesLength("mivaso") == sizeof(miVaso)) prefs.getBytes("mivaso", &miVaso, sizeof(miVaso));
  if (prefs.getBytesLength("otros") == sizeof(otros))   prefs.getBytes("otros", otros, sizeof(otros));
  MM_S_CRUCERO = prefs.getFloat("mms", MM_S_CRUCERO);
  MM_S_LENTO   = prefs.getFloat("mmsl", MM_S_LENTO);
  VBAT_VEL     = prefs.getFloat("vbv", VBAT_VEL);
  prefs.end();
}

// ============================================================================
// 7. MOTORES Y GIROSCOPIO
// ============================================================================
static const uint8_t PWM_BITS = 8, CH_A = 4, CH_B = 5;
static void pwmIniciar(uint8_t pin, uint8_t ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(pin, PWM_FREQ, PWM_BITS, ch);
#else
  ledcSetup(ch, PWM_FREQ, PWM_BITS); ledcAttachPin(pin, ch);
#endif
}
static void pwmEscribir(uint8_t pin, uint8_t ch, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch; ledcWrite(pin, duty);
#else
  (void)pin; ledcWrite(ch, duty);
#endif
}
// duty en [-255, 255]: positivo = adelante.
static void motorIzqDirecto(int duty) {
  duty = constrain(duty, -255, 255);
  digitalWrite(PIN_DIR_I1, duty > 0); digitalWrite(PIN_DIR_I2, duty < 0);
  pwmEscribir(PIN_VEL_I, CH_A, abs(duty));
}
static void motorDerDirecto(int duty) {
  duty = constrain(duty, -255, 255);
  digitalWrite(PIN_DIR_D1, duty > 0); digitalWrite(PIN_DIR_D2, duty < 0);
  pwmEscribir(PIN_VEL_D, CH_B, abs(duty));
}
static int rampa(int actual, int objetivo) {     // sube de a poco; para frenar corta de una
  if (objetivo == 0) return 0;
  return actual + constrain(objetivo - actual, -RAMPA_PWM, RAMPA_PWM);
}
static void motores(int objI, int objD) {
  pwmI = rampa(pwmI, objI); pwmD = rampa(pwmD, objD);
  motorIzqDirecto(pwmI); motorDerDirecto(pwmD);
}
static void apagarMotores() { pwmI = pwmD = 0; motorIzqDirecto(0); motorDerDirecto(0); }

static const uint8_t MPU_ADDR = 0x68;
static float leerGyroZ() {                       // velocidad de giro en °/s
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x47); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return 0;
  int16_t raw = (Wire.read() << 8) | Wire.read();
  return raw / 131.0f;
}
static void calibrarGyro() {                     // quieto: mide el error del giroscopio
  float s = 0;
  for (int i = 0; i < 300; i++) { s += leerGyroZ(); delay(3); }
  gzBias = s / 300;
}
// Gira un poco a la derecha y mira hacia dónde lo mide el giroscopio (por si quedó al revés).
static void detectarSignoGyro() {
  float giro = 0; uint32_t t0 = millis(), tPrev = micros();
  motorIzqDirecto(PWM_GIRO_MAX); motorDerDirecto(-PWM_GIRO_MAX);
  while (millis() - t0 < 550) {
    if (millis() - t0 >= 300) { motorIzqDirecto(0); motorDerDirecto(0); }
    uint32_t ahora = micros(); float dt = (ahora - tPrev) / 1e6f; tPrev = ahora;
    giro += (leerGyroZ() - gzBias) * dt;
    delay(5);
  }
  motorIzqDirecto(-PWM_GIRO_MAX); motorDerDirecto(PWM_GIRO_MAX); delay(300);   // devolverse
  apagarMotores(); delay(300);
  if (fabsf(giro) >= 8.0f) {
    signoGyro = giro > 0 ? 1 : -1; signoDetectado = true;
    Serial.printf("Giroscopio: giró %.0f°, sentido = %d (listo)\n", fabsf(giro), signoGyro);
  } else {
    Serial.printf("Giroscopio: casi no giró (%.1f°). ¿Switch de las pilas apagado? Lo repito al escribir 'x'.\n", giro);
  }
}

// ---- órdenes de movimiento (las ejecuta lazoControl) ----
static void parar() { mov = MOV_NADA; }
static void girarA(float r) { rumboObjetivo = envolver180(r); mov = MOV_GIRO; tGiroOk = 0; tGiroIni = millis(); }
static void moverMM(float mm, bool lento) { distObjetivo = mm; distHecha = 0; movLento = lento; mov = MOV_RECTO; }

// Cada 20 ms: rumbo (giroscopio), posición estimada y PWM de cada motor.
static void lazoControl(float dt) {
  float gz = leerGyroZ();
  rumbo = envolver180(rumbo + signoGyro * (gz - gzBias) * dt);     // + = horario visto desde arriba
  if (mov == MOV_NADA && pwmI == 0 && pwmD == 0) {                 // quieto: corrige la deriva
    if (!tQuieto) tQuieto = millis();
    if (millis() - tQuieto > 400 && fabsf(gz - gzBias) < 1.5f) gzBias += 0.02f * (gz - gzBias);
  } else tQuieto = 0;
  if (estado == E_OBJETIVO || estado == E_BATERIA) mov = MOV_NADA;
  int uI = 0, uD = 0;
  if (mov == MOV_GIRO) {
    float err = envolver180(rumboObjetivo - rumbo);
    if (fabsf(err) < TOL_GIRO_GRADOS) {
      if (!tGiroOk) tGiroOk = millis();
      else if (millis() - tGiroOk > 150) mov = MOV_NADA;            // giro terminado
    } else {
      tGiroOk = 0;
      float p = clampf(KP_GIRO * fabsf(err), (float)PWM_GIRO_MIN, (float)PWM_GIRO_MAX);
      int s = err > 0 ? 1 : -1;
      uI = (int)(s * p); uD = (int)(-s * p);                        // gira en el sitio
    }
    if (mov == MOV_GIRO && millis() - tGiroIni > T_GIRO_MAX_MS) { mov = MOV_NADA; uI = uD = 0; }
  } else if (mov == MOV_RECTO) {
    if (fabsf(distHecha) >= fabsf(distObjetivo)) mov = MOV_NADA;   // ya recorrió lo pedido
    else {
      float signo = distObjetivo >= 0 ? 1.0f : -1.0f;
      float base = movLento ? PWM_LENTO : PWM_CRUCERO;
      float corr = clampf(KP_RECTO * envolver180(rumboObjetivo - rumbo), -60.0f, 60.0f);
      uI = (int)(signo * base + corr); uD = (int)(signo * base - corr);
      // Sin encoder: distancia = velocidad × tiempo, y con el rumbo se suma a la posición.
      float d = signo * (movLento ? MM_S_LENTO : MM_S_CRUCERO) * factorBateria() * dt;
      distHecha += d;
      posX += d * cosf(rumbo * DEG_TO_RAD);
      posY += d * sinf(rumbo * DEG_TO_RAD);
    }
  }
  motores(uI, uD);
}

// ============================================================================
// 8. SENSOR DE DISTANCIA (VL53L0X)
// ============================================================================
static bool tofSano() { return tofOk && millis() - tTof < TOF_SIN_DATOS_MS; }
static uint16_t frenteMM() { return (tofSano() && tofMm < 2000) ? tofMm : 9999; }
static bool actualizarTof() {                    // lee si hay dato nuevo (no bloquea)
  if (!tofOk || !lox.isRangeComplete()) return false;
  uint16_t r = lox.readRange();
  tofEstado = lox.readRangeStatus();
  // Estado 4 = no hay nada. Los estados 1-3 ("dudosa") SÍ se usan: cerca de un vaso casi
  // todas las lecturas salen así.
  tofCrudo = (tofEstado == 4 || r == 0xFFFF || r >= 2000) ? 8190 : max<uint16_t>(r, 20);
  tofMm = min(tofCrudo, tofAnterior);            // la menor de las 2 últimas: si lo vio, frena
  tofAnterior = tofCrudo;
  tTof = millis();
  return true;
}
static bool iniciarTof() {
  for (int i = 0; i < 3; i++) {
    if (lox.begin()) { lox.startRangeContinuous(50); tofOk = true; tTof = millis(); return true; }
    delay(100);
  }
  tofOk = false;
  return false;
}

// ============================================================================
// 9. SENSOR DE COLOR (TCS3200) Y RECONOCIMIENTO
// ============================================================================
// Frecuencia de un canal (S2/S3 eligen el filtro). 0 = no llega señal.
static float leerCanal(bool s2, bool s3) {
  digitalWrite(PIN_TCS_S2, s2); digitalWrite(PIN_TCS_S3, s3);
  delayMicroseconds(400);
  unsigned long suma = 0; int n = 0;
  for (int i = 0; i < 6; i++) { unsigned long t = pulseIn(PIN_TCS_OUT, LOW, 20000); if (t) { suma += t; n++; } }
  if (!n) return 0;
  return 1.0e6f / (2.0f * (float)suma / n);
}
// Una lectura = {rojo/(r+g+b), verde/(r+g+b), azul/(r+g+b), ln(r+g+b)}.
// Las proporciones dicen el TONO; el ln de la suma dice qué tan CLARO es.
static bool leerVec(float v[4]) {
  float r = leerCanal(LOW, LOW), b = leerCanal(LOW, HIGH), g = leerCanal(HIGH, HIGH);
  if (r <= 0 || g <= 0 || b <= 0) return false;
  float s = r + g + b;
  v[0] = r / s; v[1] = g / s; v[2] = b / s; v[3] = logf(s);
  return true;
}
static bool leerVecProm(float v[4], int n) {      // promedio de n lecturas
  float acc[4] = {0, 0, 0, 0}, x[4]; int k = 0;
  for (int i = 0; i < n; i++) if (leerVec(x)) { for (int j = 0; j < 4; j++) acc[j] += x[j]; k++; }
  if (!k) return false;
  for (int j = 0; j < 4; j++) v[j] = acc[j] / k;
  return true;
}
// Diferencia entre dos colores (0 = iguales).
static float difColor(const float a[4], const float b[4]) {
  float d = 0;
  for (int j = 0; j < 3; j++) { float e = a[j] - b[j]; d += e * e; }
  float e = PESO_BRILLO * (a[3] - b[3]); d += e * e;
  return sqrtf(d);
}
// Guarda un color: 30 lecturas, su promedio y cuánto varían (de ahí sale su límite).
static bool calibrarColor(ColorRef& c, const char* nombre) {
  const int N = 30; float m[N][4]; int k = 0;
  for (int i = 0; i < N; i++) if (leerVec(m[k])) k++;
  if (k < 10) { Serial.println(F("el sensor de color casi no responde: revisa S2 18, S3 19, OUT 23, S0 3V3, S1 GND")); return false; }
  for (int j = 0; j < 4; j++) { c.v[j] = 0; for (int i = 0; i < k; i++) c.v[j] += m[i][j]; c.v[j] /= k; }
  float var = 0;
  for (int i = 0; i < k; i++) { float d = difColor(m[i], c.v); var += d * d; }
  float ruido = sqrtf(var / k);
  c.umbral = clampf(3.0f * ruido + 0.015f, UMBRAL_MIN, UMBRAL_MAX);
  c.ok = true;
  guardarCalibracion();
  Serial.printf("%s guardado: r %.3f g %.3f b %.3f | ruido %.3f -> límite %.3f\n",
                nombre, c.v[0], c.v[1], c.v[2], ruido, c.umbral);
  return true;
}
// ¿Lo leído es MI vaso? Debe parecerse a mi vaso (dentro de su límite) y parecerse MÁS a mi
// vaso que a cualquiera de los colores guardados como "no es mi vaso".
static bool esMiVaso(const float v[4], float& dMio, float& dOtro, int& kOtro) {
  dMio = miVaso.ok ? difColor(v, miVaso.v) : 9.0f;
  dOtro = 9.0f; kOtro = -1;
  for (int k = 0; k < 4; k++) if (otros[k].ok) {
    float d = difColor(v, otros[k].v);
    if (d < dOtro) { dOtro = d; kOtro = k; }
  }
  return miVaso.ok && dMio <= miVaso.umbral && dMio < dOtro;
}
static void imprimirLectura(const float v[4]) {
  float dMio, dOtro; int kOtro;
  bool si = esMiVaso(v, dMio, dOtro, kOtro);
  Serial.printf("  color r %.3f g %.3f b %.3f | dif con MI vaso %.3f (límite %.3f)", v[0], v[1], v[2], dMio, miVaso.umbral);
  if (kOtro >= 0) Serial.printf(" | con el color %d: %.3f", kOtro + 1, dOtro);
  Serial.println(si ? "  -> SÍ, ES MI VASO" : "  -> no");
}

// ============================================================================
// 10. BÚSQUEDA (máquina de estados, cada 20 ms)
// ============================================================================
static float acercaUltMM = 9999, acercaHechaEnLect = 0;
static uint32_t acercaTLect = 0;

static bool dentroArena(float x, float y, float margen) {
  return x >= margen && y >= margen && x <= ARENA_MM - margen && y <= ARENA_MM - margen;
}
// ¿Está cerca del borde Y mirando hacia afuera?
static bool saliendo() {
  if (dentroArena(posX, posY, MARGEN_BORDE_MM)) return false;
  float c = ARENA_MM / 2;
  float hx = cosf(rumbo * DEG_TO_RAD), hy = sinf(rumbo * DEG_TO_RAD);
  return hx * (posX - c) + hy * (posY - c) > 0;
}
static float rumboAlCentro() {
  float c = ARENA_MM / 2;
  return atan2f(c - posY, c - posX) * RAD_TO_DEG;
}
static void empezarTramo() {                      // andar derecho un trecho al azar
  moverMM(azarEntre(TRAMO_MIN_MM, TRAMO_MAX_MM), false);
  cambiar(E_BUSCA);
}
static void girarHacia(float r) { girarA(r); cambiar(E_GIRA); }

static void entrarAcerca(float r) {
  parar();
  Serial.printf("veo algo a %.0f mm: me acerco despacio\n", r);
  acercaUltMM = r; acercaHechaEnLect = 0; acercaTLect = tTof;
  moverMM(clampf(r - DIST_LEER_MM + 20.0f, 0.0f, MAX_ACERCA_MM), true);   // nunca más allá del objeto
  cambiar(E_ACERCA);
}

static void decidir() {
  uint32_t enEstado = millis() - tEstado;
  bool activo = estado == E_BUSCA || estado == E_GIRA || estado == E_ACERCA || estado == E_LEE || estado == E_RETROCEDE;
  if (activo && !tofSano()) {                     // sin sensor de distancia no se mueve
    parar(); apagarMotores();
    Serial.println(F("el VL53L0X no manda datos: me detengo (revisa VIN, GND, SDA 21, SCL 22)"));
    cambiar(E_SENSOR);
    return;
  }
  switch (estado) {
    case E_INICIO:
      if (enEstado > 3000) {
        if (!miVaso.ok) { Serial.println(F("Aún no sé cuál es mi vaso: ponlo frente al sensor de color y escribe 'o'.")); cambiar(E_PARADO); }
        else { Serial.println(F("¡A buscar!")); rumboObjetivo = rumbo; empezarTramo(); }
      }
      break;

    case E_BUSCA: {
      float r = frenteMM();
      if (r < DIST_VER_MM) {
        // ¿Ese objeto está DENTRO del espacio? Si está afuera (pared, pies, mesa), lo ignora.
        float d = r + LARGO_FRENTE_MM;
        float ox = posX + d * cosf(rumbo * DEG_TO_RAD), oy = posY + d * sinf(rumbo * DEG_TO_RAD);
        if (dentroArena(ox, oy, 0)) { entrarAcerca(r); break; }
        Serial.println(F("lo que veo está fuera del espacio: giro"));
        parar(); girarHacia(rumboAlCentro() + azarEntre(-40, 40)); break;
      }
      if (saliendo()) {
        parar();
        Serial.printf("borde (estoy en x %.0f, y %.0f): giro hacia el centro\n", posX, posY);
        girarHacia(rumboAlCentro() + azarEntre(-40, 40));
        break;
      }
      if (mov == MOV_NADA) girarHacia(rumbo + signoAzar() * azarEntre(45, 135));   // fin del tramo
      break;
    }

    case E_GIRA:
      if (mov == MOV_NADA) empezarTramo();
      break;

    case E_ACERCA: {                              // despacio hasta quedar a DIST_LEER_MM
      if (tTof != acercaTLect && frenteMM() < 2000) { acercaUltMM = frenteMM(); acercaHechaEnLect = distHecha; acercaTLect = tTof; }
      float falta = acercaUltMM - (distHecha - acercaHechaEnLect);   // si lo pierde de vista, lo calcula
      if (falta <= DIST_LEER_MM || mov == MOV_NADA) {
        parar(); apagarMotores();
        Serial.printf("frené frente al objeto (sensor %u mm): leo el color\n", frenteMM());
        cambiar(E_LEE);
      }
      break;
    }

    case E_LEE: {
      if (enEstado < 300) break;                  // quieto antes de leer
      apagarMotores();
      int votos = 0;
      for (int i = 0; i < 3; i++) {               // 3 lecturas; tienen que decir SÍ al menos 2
        float v[4];
        if (!leerVecProm(v, 5)) { Serial.println(F("  el sensor de color no da señal")); continue; }
        imprimirLectura(v);
        float a, b; int k;
        if (esMiVaso(v, a, b, k)) votos++;
      }
      if (votos >= 2) {
        cambiar(E_OBJETIVO);
        Serial.println(F("¡ENCONTRÉ MI VASO! Me quedo quieto. (escribe 'x' para seguir buscando)"));
      } else {
        Serial.println(F("no es mi vaso: retrocedo y sigo buscando"));
        moverMM(-RETROCESO_MM, true);
        cambiar(E_RETROCEDE);
      }
      break;
    }

    case E_RETROCEDE:
      if (mov == MOV_NADA) girarHacia(rumbo + signoAzar() * azarEntre(100, 160));
      break;

    case E_OBJETIVO: case E_BATERIA: case E_PARADO:
      parar();
      break;

    case E_SENSOR:
      parar();
      if (enEstado > 2000) {
        if (iniciarTof()) { Serial.println(F("VL53L0X otra vez OK")); rumboObjetivo = rumbo; empezarTramo(); }
        else cambiar(E_SENSOR);
      }
      break;
  }
}

static void actualizarLED() {
  switch (estado) {
    case E_INICIO:    ponerLED(30, 30, 0); break;                              // amarillo
    case E_BUSCA: case E_GIRA: ponerLED(0, 20, 50); break;                     // celeste: buscando
    case E_ACERCA: case E_LEE: ponerLED(40, 0, 40); break;                     // morado: revisando
    case E_RETROCEDE: ponerLED(50, 15, 0); break;                              // naranja: no era
    case E_OBJETIVO:  ponerLED(0, (millis() / 250) % 2 ? 80 : 10, 0); break;   // verde titilando: ¡lo encontró!
    case E_PARADO:    ponerLED(20, 20, 20); break;                             // blanco tenue
    case E_BATERIA:   ponerLED((millis() / 300) % 2 ? 60 : 0, 0, 0); break;    // rojo titilando
    case E_SENSOR:    ponerLED(0, 0, (millis() / 300) % 2 ? 60 : 0); break;    // azul titilando
  }
}

// ============================================================================
// 11. COMANDOS POR SERIAL
// ============================================================================
// Mueve con el lazo de control durante 'ms' (para las pruebas 'a' y 'g').
static void ejecutarMovimiento(uint32_t msMax) {
  uint32_t t0 = millis(), tPrev = millis();
  while (mov != MOV_NADA && millis() - t0 < msMax) {
    uint32_t ahora = millis();
    if (ahora - tPrev >= 20) { lazoControl((ahora - tPrev) / 1000.0f); tPrev = ahora; }
    delay(2);
  }
  parar(); apagarMotores();
}
static void escanearI2C() {
  Serial.print(F("I2C en 21/22 (deben salir 0x29 = distancia y 0x68 = giroscopio):"));
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) { Wire.beginTransmission(a); if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", a); n++; } }
  Serial.println(n ? "" : " NADA (revisa cables)");
}
static void probarTof() {
  if (!tofOk) { Serial.println(F("el VL53L0X no respondió al prender: revisa VIN, GND, SDA 21, SCL 22")); return; }
  Serial.println(F("5 s: acerca y aleja un vaso del sensor"));
  uint32_t t0 = millis(), tImp = 0; int n = 0;
  while (millis() - t0 < 5000) {
    if (actualizarTof()) n++;
    if (millis() - tImp >= 250) {
      tImp = millis();
      if (tofCrudo >= 2000) Serial.println(F("  nada enfrente"));
      else Serial.printf("  %u mm\n", tofCrudo);
    }
    delay(5);
  }
  Serial.printf("%d lecturas (deberían ser ~100)\n", n);
}
static void imprimirCalibracion() {
  Serial.printf("MI vaso: %s", miVaso.ok ? "" : "SIN GUARDAR (usa 'o')\n");
  if (miVaso.ok) Serial.printf("r %.3f g %.3f b %.3f, límite %.3f\n", miVaso.v[0], miVaso.v[1], miVaso.v[2], miVaso.umbral);
  for (int k = 0; k < 4; k++) if (otros[k].ok) {
    Serial.printf("NO es mi vaso, color %d: r %.3f g %.3f b %.3f | dif con mi vaso %.3f%s\n", k + 1,
                  otros[k].v[0], otros[k].v[1], otros[k].v[2], miVaso.ok ? difColor(otros[k].v, miVaso.v) : 0.0f,
                  (miVaso.ok && difColor(otros[k].v, miVaso.v) < miVaso.umbral) ? "  <- OJO: casi igual a mi vaso" : "");
  }
  Serial.printf("velocidad %.0f mm/s (lento %.0f), batería de referencia %.2f V\n", MM_S_CRUCERO, MM_S_LENTO, VBAT_VEL);
}
static void colorEnVivo() {
  Serial.println(F("10 s: pon distintos objetos frente al sensor de color (a ~2 cm)"));
  uint32_t t0 = millis();
  while (millis() - t0 < 10000) {
    float v[4];
    if (leerVecProm(v, 3)) imprimirLectura(v); else Serial.println(F("  sin señal del sensor de color"));
    delay(250);
  }
}
static void detenerParaProbar() { if (estado != E_PARADO) { parar(); apagarMotores(); cambiar(E_PARADO); } }

static void leerSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'o') { detenerParaProbar(); calibrarColor(miVaso, "color de MI vaso"); }
    else if (c >= '1' && c <= '4') {
      detenerParaProbar();
      char nombre[32]; snprintf(nombre, sizeof(nombre), "color %c (NO es mi vaso)", c);
      calibrarColor(otros[c - '1'], nombre);
    }
    else if (c == '0') { for (int k = 0; k < 4; k++) otros[k].ok = false; guardarCalibracion(); Serial.println(F("borrados los colores 1..4")); }
    else if (c == 'B') {
      miVaso.ok = false; for (int k = 0; k < 4; k++) otros[k].ok = false;
      MM_S_CRUCERO = 150; MM_S_LENTO = 70; VBAT_VEL = 0;
      guardarCalibracion(); Serial.println(F("calibración borrada"));
    }
    else if (c == 'k') { detenerParaProbar(); colorEnVivo(); }
    else if (c == 'p') imprimirCalibracion();
    else if (c == 't') {
      float v[4];
      Serial.printf("distancia %u mm%s | batería %.2f V | rumbo %.0f° | posición x %.0f y %.0f\n",
                    tofMm, tofSano() ? "" : " (SIN DATOS)", leerVBat(), rumbo, posX, posY);
      if (leerVecProm(v, 3)) imprimirLectura(v); else Serial.println(F("  sin señal del sensor de color"));
    }
    else if (c == 'f') { detenerParaProbar(); probarTof(); }
    else if (c == 'i') { detenerParaProbar(); escanearI2C(); }
    else if (c == 'm') {
      detenerParaProbar();
      Serial.println(F("motor IZQUIERDO adelante 1 s...")); motorIzqDirecto(PWM_CRUCERO); delay(1000); motorIzqDirecto(0); delay(300);
      Serial.println(F("motor DERECHO adelante 1 s..."));   motorDerDirecto(PWM_CRUCERO); delay(1000); motorDerDirecto(0);
      Serial.println(F("Las DOS ruedas deben ir hacia ADELANTE, primero la IZQUIERDA."));
    }
    else if (c == 'a') {
      detenerParaProbar(); rumboObjetivo = rumbo; moverMM(250, false);
      Serial.println(F("avanzo 25 cm: mide cuánto avanzó de verdad"));
      ejecutarMovimiento(6000);
    }
    else if (c == '+' || c == '-') {               // '+' = se pasó, '-' = se quedó corto
      float f = (c == '+') ? 1.05f : 0.95f;
      MM_S_CRUCERO *= f; MM_S_LENTO *= f;
      if (leerVBat() > 3.0f) VBAT_VEL = leerVBat();
      guardarCalibracion();
      Serial.printf("velocidad estimada %.0f mm/s (guardada). Prueba otra vez con 'a'.\n", MM_S_CRUCERO);
    }
    else if (c == 'g') {
      detenerParaProbar(); girarA(rumbo + 90);
      Serial.println(F("giro 90° a la derecha"));
      ejecutarMovimiento(T_GIRO_MAX_MS + 500);
    }
    else if (c == 'd') Serial.printf("[%s] x %.0f mm, y %.0f mm, rumbo %.0f°\n", NOMBRE_ESTADO[estado], posX, posY, rumbo);
    else if (c == 'r') {
      detenerParaProbar();
      posX = INICIO_X_MM; posY = INICIO_Y_MM; rumbo = rumboObjetivo = INICIO_RUMBO;
      Serial.println(F("posición reiniciada: el carro debe estar en la esquina de inicio mirando a la derecha"));
    }
    else if (c == 'l') { ponerLED(60, 0, 0); delay(400); ponerLED(0, 60, 0); delay(400); ponerLED(0, 0, 60); delay(400); Serial.println(F("LED: rojo, verde, azul")); }
    else if (c == 'x') {
      if (estado == E_PARADO || estado == E_OBJETIVO || estado == E_BATERIA) {
        if (!miVaso.ok) { Serial.println(F("primero guarda el color de tu vaso con 'o'")); continue; }
        if (AUTO_SIGNO_GYRO && mpuOk && !signoDetectado) { calibrarGyro(); detectarSignoGyro(); }
        Serial.println(F("arranca")); rumboObjetivo = rumbo; empezarTramo();
      } else { parar(); apagarMotores(); cambiar(E_PARADO); Serial.println(F("parado")); }
    }
  }
}

// ============================================================================
// 12. SETUP / LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(PIN_DIR_I1, OUTPUT); pinMode(PIN_DIR_I2, OUTPUT); pinMode(PIN_DIR_D1, OUTPUT); pinMode(PIN_DIR_D2, OUTPUT);
  pwmIniciar(PIN_VEL_I, CH_A); pwmIniciar(PIN_VEL_D, CH_B);
  apagarMotores();
  ponerLED(30, 30, 0);

  pinMode(PIN_TCS_S2, OUTPUT); pinMode(PIN_TCS_S3, OUTPUT); pinMode(PIN_TCS_OUT, INPUT);
  colorOk = leerCanal(LOW, LOW) > 0;
  Serial.println(colorOk ? F("TCS3200 OK") : F("TCS3200 NO responde: revisa VCC, GND, S0 3V3, S1 GND, S2 18, S3 19, OUT 23"));

  Wire.begin(PIN_SDA, PIN_SCL); Wire.setClock(100000);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0);        // despierta el MPU6050
  mpuOk = (Wire.endTransmission() == 0);
  Serial.println(mpuOk ? F("MPU6050 OK") : F("MPU6050 NO responde: sin él no gira bien"));
  Serial.println(iniciarTof() ? F("VL53L0X OK") : F("VL53L0X NO responde: el carro no arranca sin él"));
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);

  cargarCalibracion();
  imprimirCalibracion();

  Serial.println(F("Déjalo QUIETO 1 s (calibro el giroscopio)..."));
  delay(500); calibrarGyro();
  if (AUTO_SIGNO_GYRO && mpuOk) detectarSignoGyro();

  posX = INICIO_X_MM; posY = INICIO_Y_MM;
  rumbo = rumboObjetivo = INICIO_RUMBO;
  vbatActual = leerVBat();
  cambiar(tofOk ? E_INICIO : E_SENSOR);
  Serial.println(F("Listo. Arranco en 3 s (escribe 'x' para parar)."));
}

void loop() {
  static uint32_t tCtrl = 0, tDec = 0, tLed = 0, tBat = 0;
  uint32_t now = millis();
  leerSerial();
  actualizarTof();
  if (now - tCtrl >= 20) { lazoControl(min((now - tCtrl) / 1000.0f, 0.06f)); tCtrl = now; }
  if (now - tDec >= 20) { decidir(); tDec = now; }
  if (now - tLed >= 100) { actualizarLED(); tLed = now; }

  // Batería: solo con los motores quietos (al arrancar, el voltaje cae un momento).
  if (now - tBat >= 1000 && mov == MOV_NADA) {
    vbatActual = leerVBat();
    if (vbatActual > 3.0f) {                       // < 3 V = solo USB: no se protege
      vbatBajas = vbatActual < VBAT_MIN ? vbatBajas + 1 : 0;
      if (vbatBajas >= 3 && estado != E_BATERIA && estado != E_PARADO) {
        parar(); cambiar(E_BATERIA);
        Serial.printf("batería baja (%.2f V): me detengo\n", vbatActual);
      }
    }
    tBat = now;
  }
}
