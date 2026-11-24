/*
 * CARRITO SIMPLE — busca UN vaso objetivo. Sensor de color GRANDE (TCS3200, 4 LEDs).
 * ---------------------------------------------------------------------------
 * Versión sencilla, hecha para que funcione lo básico:
 *
 *  1. RECORRE EL ESPACIO EN FILAS (como cortando pasto), no al azar:
 *       arranca en una esquina, avanza la fila completa, baja un cuadro, se devuelve
 *       por la fila siguiente... Así nunca se sale: cada fila mide lo mismo.
 *
 *         inicio → → → → → ↓
 *         ↓ ← ← ← ← ← ← ← ←
 *         → → → → → → → → ↓      (6 filas de 6 cuadros de 25 cm)
 *
 *  2. Solo se acerca a objetos que estén DENTRO de la fila (no persigue cosas de afuera).
 *  3. Frena a 4 cm, lee el color 5 veces. Si 3 o más dicen que es su vaso: se queda
 *     quieto con el LED VERDE titilando. Si no: se devuelve y sigue por la fila siguiente.
 *  4. Los colores y la velocidad se GUARDAN SOLOS en la memoria del ESP32
 *     (no hay que copiar ni pegar nada en el código).
 *
 * CONEXIONES (las mismas de siempre):
 *   Motores L298N: ENA 25, IN1 26, IN2 27, ENB 14, IN3 32, IN4 4
 *   VL53L0X y MPU6050: SDA 21, SCL 22 (VIN a 3V3)
 *   TCS3200: VCC 3V3, GND, S0 a 3V3, S1 a GND, S2 18, S3 19, OUT 23
 *   LED WS2812: 5
 *
 * PASOS (monitor serie a 115200):
 *   1. 'w' con una hoja blanca a 2 cm del sensor de color.
 *   2. 'o' con el vaso OBJETIVO a 2 cm del sensor de color.
 *   3. (recomendado) '1'..'4' con cada uno de los OTROS vasos: así no los confunde.
 *   4. 'k' para probar: pon y quita vasos; LED verde = lo reconoce, rojo = no.
 *   5. Pon el carro en la esquina, mirando a lo largo del borde, y escribe 'x'.
 *   Escribe 'h' para ver todos los comandos.
 *
 * Librería: "Adafruit_VL53L0X". Placa: "ESP32 Dev Module".
 */
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>          // memoria del ESP32 que no se borra al apagar
#include <Adafruit_VL53L0X.h>

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// ============================================================================
// 1. ESPACIO
// ============================================================================
const int   CUADROS  = 6;                                // cuadros por lado (6 x 6)
const float CELDA_MM = 250.0f;                           // lado de cada cuadro
const float LARGO_FILA_MM = (CUADROS - 1) * CELDA_MM;    // del centro del 1.º al centro del último cuadro
// Al arrancar, ¿las filas siguientes quedan a la DERECHA del carro? (true) o a la izquierda (false)
// Ej.: carro en la esquina de arriba a la izquierda mirando a la derecha -> las filas van hacia abajo = derecha.
const bool  FILAS_A_LA_DERECHA = true;

// ============================================================================
// 2. PINES
// ============================================================================
const uint8_t PIN_VEL_I = 25, PIN_DIR_I1 = 26, PIN_DIR_I2 = 27;   // motor izquierdo
const uint8_t PIN_VEL_D = 14, PIN_DIR_D1 = 32, PIN_DIR_D2 = 4;    // motor derecho
const uint8_t PIN_TCS_S2 = 18, PIN_TCS_S3 = 19, PIN_TCS_OUT = 23; // sensor de color TCS3200
const uint8_t PIN_SDA = 21, PIN_SCL = 22;                         // VL53L0X + MPU6050
const uint8_t PIN_LED = 5;

// ============================================================================
// 3. MOVIMIENTO
// ============================================================================
const int   PWM_RAPIDO = 230, PWM_LENTO = 190;           // fuerza de los motores (0..255)
const int   PWM_GIRO_MIN = 185, PWM_GIRO_MAX = 230;      // fuerza al girar
float       MM_S_RAPIDO = 150.0f;                        // velocidad estimada: se ajusta con 'a' + 'c'/'l'
float       MM_S_LENTO  = 70.0f;                         // (y se guarda sola)
const float KP_RECTO = 4.0f;                             // corrección para ir derecho (por grado torcido)
const float KP_GIRO  = 3.0f;                             // fuerza de giro por grado que falta
const int   AJUSTE_IZQ = 0, AJUSTE_DER = 0;              // si un motor es más rápido, réstale aquí
const uint32_t T_GIRO_90_MS = 600;                       // si NO hay giroscopio: tiempo para girar 90°
const int8_t SIGNO_GYRO_DEFECTO = -1;                    // si la prueba del giroscopio falla
const int   RAMPA_PWM = 30;                              // arranque suave

// ============================================================================
// 4. SENSORES
// ============================================================================
const uint16_t VER_OBJETO_MM = 200;                      // algo más cerca que esto = objeto a revisar
const uint16_t PARAR_MM      = 40;                       // se acerca hasta esta distancia y lee el color
const float    MAX_ACERCA_MM = 220.0f;                   // lo máximo que avanza para acercarse

// ============================================================================
// 5. VARIABLES
// ============================================================================
Preferences mem;                                         // memoria permanente
Adafruit_VL53L0X lox;
bool tofOk = false, mpuOk = false, signoListo = false;
uint16_t tofMm = 8190, tofPrev = 8190;                   // última y penúltima lectura (mm)
uint32_t tTof = 0;                                       // cuándo llegó la última lectura
float rumbo = 0, rumboRef = 0, gzBias = 0;               // grados; + = giro a la derecha
int8_t signoGyro = SIGNO_GYRO_DEFECTO;
uint32_t tRumbo = 0;                                     // micros() de la última actualización
int pwmI = 0, pwmD = 0;

struct Color { float c[4]; bool ok; };                   // c = {rojo, verde, azul, brillo}
Color objetivo = {{0, 0, 0, 0}, false};                  // el vaso que busca
Color otros[4];                                          // vasos que NO son el objetivo
float tolerancia = 0.04f;                                // qué tan parecido debe ser (se calcula con 'o')
float ganancia[3] = {1, 1, 1};                           // balance de blancos

enum Fin : uint8_t { LLEGO, OBJETO, CERCA, USUARIO };    // por qué terminó un movimiento

// ============================================================================
// 6. UTILIDADES
// ============================================================================
float limitar(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void ponerLED(uint8_t r, uint8_t g, uint8_t b) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  rgbLedWrite(PIN_LED, r, g, b);
#else
  neopixelWrite(PIN_LED, r, g, b);
#endif
}

// ¿Escribieron 'x' por el monitor serie? (para detener el carro en cualquier momento)
bool pidioParar() {
  while (Serial.available()) if (Serial.read() == 'x') return true;
  return false;
}

// ============================================================================
// 7. MOTORES
// ============================================================================
void pwmIniciar(uint8_t pin, uint8_t canal) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(pin, 1000, 8, canal);                // 1 kHz, 8 bits
#else
  ledcSetup(canal, 1000, 8); ledcAttachPin(pin, canal);
#endif
}
void pwmEscribir(uint8_t pin, uint8_t canal, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(canal, duty);
#endif
}
// duty de -255 (atrás) a 255 (adelante)
void motorIzq(int duty) {
  if (duty) duty += duty > 0 ? AJUSTE_IZQ : -AJUSTE_IZQ;
  duty = constrain(duty, -255, 255);
  digitalWrite(PIN_DIR_I1, duty > 0); digitalWrite(PIN_DIR_I2, duty < 0);   // sentido
  pwmEscribir(PIN_VEL_I, 4, abs(duty));                                     // fuerza
}
void motorDer(int duty) {
  if (duty) duty += duty > 0 ? AJUSTE_DER : -AJUSTE_DER;
  duty = constrain(duty, -255, 255);
  digitalWrite(PIN_DIR_D1, duty > 0); digitalWrite(PIN_DIR_D2, duty < 0);
  pwmEscribir(PIN_VEL_D, 5, abs(duty));
}
void apagarMotores() { pwmI = pwmD = 0; motorIzq(0); motorDer(0); }
// sube el PWM de a poco (arranque suave)
int rampa(int actual, int objetivo) {
  int d = constrain(objetivo - actual, -RAMPA_PWM, RAMPA_PWM);
  return actual + d;
}

// ============================================================================
// 8. GIROSCOPIO (MPU6050)
// ============================================================================
float leerGyroZ() {                                      // grados por segundo
  Wire.beginTransmission(0x68); Wire.write(0x47); Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x68, (uint8_t)2);
  if (Wire.available() < 2) return 0;
  int16_t raw = (Wire.read() << 8) | Wire.read();
  return raw / 131.0f;
}
// Suma lo que giró desde la última vez (integra la velocidad de giro).
void actualizarRumbo() {
  uint32_t ahora = micros();
  float dt = (ahora - tRumbo) / 1e6f; tRumbo = ahora;
  if (dt > 0.1f) dt = 0.1f;                              // si pasó mucho tiempo, no inventa giro
  if (mpuOk) rumbo += signoGyro * (leerGyroZ() - gzBias) * dt;
}
// Espera 'ms' sin dejar de seguir el giroscopio.
void esperar(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) { actualizarRumbo(); delay(5); }
}
void calibrarGyro() {                                    // con el carro QUIETO: mide el error del sensor
  float s = 0;
  for (int i = 0; i < 300; i++) { s += leerGyroZ(); delay(3); }
  gzBias = s / 300;
  tRumbo = micros();
}
// Gira un poco a la derecha para saber si el MPU6050 mide el giro al derecho o al revés.
void detectarSignoGyro() {
  float giro = 0; uint32_t t0 = millis(), tp = micros();
  motorIzq(PWM_GIRO_MAX); motorDer(-PWM_GIRO_MAX);
  while (millis() - t0 < 550) {
    if (millis() - t0 >= 300) { motorIzq(0); motorDer(0); }
    uint32_t a = micros(); giro += (leerGyroZ() - gzBias) * (a - tp) / 1e6f; tp = a;
    delay(5);
  }
  motorIzq(-PWM_GIRO_MAX); motorDer(PWM_GIRO_MAX); delay(300);   // se devuelve
  apagarMotores(); delay(300);
  if (fabsf(giro) >= 8) {
    signoGyro = giro > 0 ? 1 : -1; signoListo = true;
    Serial.printf("giroscopio listo (giró %.0f°, sentido %d)\n", fabsf(giro), signoGyro);
  } else Serial.println(F("giroscopio: el carro no giró (¿pilas apagadas?). Uso el sentido por defecto."));
  rumbo = 0; tRumbo = micros();
}

// ============================================================================
// 9. SENSOR DE DISTANCIA (VL53L0X)
// ============================================================================
void actualizarTof() {
  if (!tofOk || !lox.isRangeComplete()) return;
  uint16_t r = lox.readRange();
  uint8_t st = lox.readRangeStatus();
  tofPrev = tofMm;
  tofMm = (st == 4 || r == 0xFFFF || r >= 2000) ? 8190 : (r < 20 ? 20 : r);   // 4 = nada enfrente
  tTof = millis();
}
// Distancia al frente: la menor de las 2 últimas (si vio algo en una, cuenta).
uint16_t frente() {
  if (!tofOk || millis() - tTof > 600) return 9999;      // sin datos recientes
  return min(tofMm, tofPrev);
}

// ============================================================================
// 10. SENSOR DE COLOR (TCS3200)
// ============================================================================
// Frecuencia de un canal (más luz de ese color = número más grande).
float leerCanal(bool s2, bool s3) {
  digitalWrite(PIN_TCS_S2, s2); digitalWrite(PIN_TCS_S3, s3);
  delayMicroseconds(400);
  unsigned long suma = 0; int n = 0;
  for (int i = 0; i < 6; i++) { unsigned long t = pulseIn(PIN_TCS_OUT, LOW, 20000); if (t) { suma += t; n++; } }
  return n ? 1.0e6f / (2.0f * suma / n) : 0;
}
bool leerRGB(float& r, float& g, float& b) {
  r = leerCanal(LOW, LOW);                               // filtro rojo
  b = leerCanal(LOW, HIGH);                              // filtro azul
  g = leerCanal(HIGH, HIGH);                             // filtro verde
  return r > 0 && g > 0 && b > 0;
}
// Color = proporción de rojo, verde y azul (+ brillo). Promedia n lecturas.
bool leerColor(float c[4], int n = 3) {
  float R = 0, G = 0, B = 0, r, g, b; int k = 0;
  for (int i = 0; i < n; i++) if (leerRGB(r, g, b)) { R += r; G += g; B += b; k++; }
  if (!k) return false;
  R = R / k * ganancia[0]; G = G / k * ganancia[1]; B = B / k * ganancia[2];   // balance de blancos
  float s = R + G + B;
  c[0] = R / s; c[1] = G / s; c[2] = B / s; c[3] = s;
  return true;
}
// Diferencia de tono entre dos colores (0 = iguales).
float difTono(const float a[4], const float b[4]) {
  float d = 0;
  for (int j = 0; j < 3; j++) d += (a[j] - b[j]) * (a[j] - b[j]);
  return sqrtf(d);
}
// ¿Es el vaso objetivo? Tiene que cumplir las 3 cosas:
//   - tono parecido al guardado (dif <= tolerancia)
//   - brillo parecido (ni mucho más claro ni mucho más oscuro)
//   - más parecido al objetivo que a cualquiera de los OTROS vasos guardados
bool esObjetivo(const float c[4], float& dObj, float& dOtro, float& brillo) {
  dObj = difTono(c, objetivo.c);
  dOtro = 9;
  for (int i = 0; i < 4; i++) if (otros[i].ok) dOtro = min(dOtro, difTono(c, otros[i].c));
  brillo = objetivo.c[3] > 0 ? c[3] / objetivo.c[3] : 1;
  return objetivo.ok && dObj <= tolerancia && brillo > 0.6f && brillo < 1.7f && dObj < 0.7f * dOtro;
}
// Guarda el color que tiene enfrente (30 lecturas). Devuelve qué tanto variaron.
bool medirColor(Color& destino, float& variacion) {
  float suma[4] = {0, 0, 0, 0}, v[30][4]; int k = 0;
  for (int i = 0; i < 30; i++) if (leerColor(v[k], 1)) k++;
  if (k < 10) return false;
  for (int i = 0; i < k; i++) for (int j = 0; j < 4; j++) suma[j] += v[i][j];
  for (int j = 0; j < 4; j++) destino.c[j] = suma[j] / k;
  variacion = 0;
  for (int i = 0; i < k; i++) variacion = max(variacion, difTono(v[i], destino.c));
  destino.ok = true;
  return true;
}

// ============================================================================
// 11. MEMORIA (lo calibrado queda guardado aunque se apague)
// ============================================================================
void guardarTodo() {
  mem.putBytes("obj", &objetivo, sizeof(objetivo));
  mem.putBytes("otros", otros, sizeof(otros));
  mem.putBytes("gan", ganancia, sizeof(ganancia));
  mem.putFloat("tol", tolerancia);
  mem.putFloat("vr", MM_S_RAPIDO);
  mem.putFloat("vl", MM_S_LENTO);
}
void cargarTodo() {
  for (int i = 0; i < 4; i++) otros[i].ok = false;
  if (mem.getBytesLength("obj") == sizeof(objetivo)) mem.getBytes("obj", &objetivo, sizeof(objetivo));
  if (mem.getBytesLength("otros") == sizeof(otros)) mem.getBytes("otros", otros, sizeof(otros));
  if (mem.getBytesLength("gan") == sizeof(ganancia)) mem.getBytes("gan", ganancia, sizeof(ganancia));
  tolerancia  = mem.getFloat("tol", tolerancia);
  MM_S_RAPIDO = mem.getFloat("vr", MM_S_RAPIDO);
  MM_S_LENTO  = mem.getFloat("vl", MM_S_LENTO);
}

// ============================================================================
// 12. MOVIMIENTOS
// ============================================================================
// Avanza 'mm' (negativo = atrás) derecho, con el giroscopio. Devuelve por qué paró
// y en 'hecho' cuánto avanzó.  modo 0: solo avanzar
//                              modo 1: parar si ve un objeto DENTRO de lo que le falta
//                              modo 2: acercarse: parar al quedar a PARAR_MM
Fin avanzar(float mm, bool lento, uint8_t modo, float& hecho) {
  hecho = 0;
  float signo = mm >= 0 ? 1 : -1, meta = fabsf(mm);
  uint32_t tPrev = millis();
  Fin fin = LLEGO;
  while (hecho < meta) {
    actualizarTof();
    if (pidioParar()) { fin = USUARIO; break; }
    uint16_t r = frente();
    // solo cuenta objetos que estén dentro de la fila (no persigue cosas de afuera)
    if (modo == 1 && r < min((float)VER_OBJETO_MM, meta - hecho + 60.0f)) { fin = OBJETO; break; }
    if (modo == 2 && r <= PARAR_MM) { fin = CERCA; break; }
    uint32_t ahora = millis();
    if (ahora - tPrev >= 20) {                           // cada 20 ms
      float dt = min((ahora - tPrev) / 1000.0f, 0.06f); tPrev = ahora;
      actualizarRumbo();
      float base = lento ? PWM_LENTO : PWM_RAPIDO;
      float corr = mpuOk ? limitar(KP_RECTO * (rumboRef - rumbo), -60, 60) : 0;   // enderezar
      pwmI = rampa(pwmI, (int)(signo * base + corr));
      pwmD = rampa(pwmD, (int)(signo * base - corr));
      motorIzq(pwmI); motorDer(pwmD);
      hecho += (lento ? MM_S_LENTO : MM_S_RAPIDO) * dt;  // distancia = velocidad x tiempo
    }
    delay(1);
  }
  apagarMotores();
  esperar(200);                                          // que se detenga del todo
  return fin;
}
// Gira 'grados' (+ derecha, - izquierda) sobre su eje.
void girar(float grados) {
  rumboRef += grados;                                    // el rumbo que DEBE tener (sin acumular errores)
  if (!mpuOk) {                                          // sin giroscopio: por tiempo
    int s = grados > 0 ? 1 : -1;
    motorIzq(s * PWM_GIRO_MAX); motorDer(-s * PWM_GIRO_MAX);
    delay((uint32_t)(T_GIRO_90_MS * fabsf(grados) / 90));
    apagarMotores(); delay(200);
    return;
  }
  uint32_t t0 = millis(), tOk = 0;
  while (millis() - t0 < 4000) {
    actualizarRumbo();
    float err = rumboRef - rumbo;
    if (fabsf(err) < 4) {                                // ya casi: espera a que se asiente
      apagarMotores();
      if (!tOk) tOk = millis(); else if (millis() - tOk > 150) break;
    } else {
      tOk = 0;
      int p = (int)limitar(KP_GIRO * fabsf(err), PWM_GIRO_MIN, PWM_GIRO_MAX);
      int s = err > 0 ? 1 : -1;
      motorIzq(s * p); motorDer(-s * p);
    }
    delay(10);
  }
  apagarMotores(); esperar(200);
}

// ============================================================================
// 13. BUSCAR EL VASO
// ============================================================================
// Se acerca despacio al objeto, lee el color 5 veces. 1 = es el objetivo, 0 = no, 2 = lo paraste.
int acercarYRevisar(float& movido) {
  ponerLED(40, 0, 40);                                   // morado: revisando
  uint16_t r = frente();
  float maxMM = r < 2000 ? limitar(r - PARAR_MM + 15, 0, MAX_ACERCA_MM) : 0;   // nunca más de lo que lo separa
  Serial.printf("veo algo a %u mm: me acerco\n", r);
  if (avanzar(maxMM, true, 2, movido) == USUARIO) return 2;
  int votos = 0;
  for (int i = 0; i < 5; i++) {
    float c[4], dObj, dOtro, brillo;
    bool si = leerColor(c) && esObjetivo(c, dObj, dOtro, brillo);
    Serial.printf("  lectura %d: dif %.3f (tolerancia %.3f), brillo x%.2f -> %s\n",
                  i + 1, dObj, tolerancia, brillo, si ? "ES" : "no");
    votos += si;
  }
  return votos >= 3 ? 1 : 0;
}
// Se queda quieto con el LED verde titilando hasta que escribas 'x'.
void encontrado() {
  apagarMotores();
  Serial.println(F("¡ENCONTRÉ EL VASO! Me quedo quieto. Escribe 'x' para volver al menú."));
  while (!pidioParar()) { ponerLED(0, (millis() / 250) % 2 ? 80 : 5, 0); delay(20); }
}
// Recorre el espacio fila por fila.
void buscar() {
  if (!tofOk)       { Serial.println(F("sin sensor de distancia no puedo buscar (revisa con 'i')")); return; }
  if (!objetivo.ok) { Serial.println(F("primero guarda el color del vaso objetivo con 'o'")); return; }
  if (mpuOk && !signoListo) { calibrarGyro(); detectarSignoGyro(); }
  rumboRef = rumbo;
  float x = 0;                                           // dónde va en la fila (mm desde el borde de inicio)
  int fila = 0, dirFilas = 1;                            // fila actual y hacia dónde van las filas
  bool ida = true;                                       // true = va alejándose del borde de inicio
  Serial.println(F("buscando... (escribe 'x' para parar)"));
  while (true) {
    ponerLED(0, 20, 50);                                 // celeste: buscando
    float falta = ida ? LARGO_FILA_MM - x : x;           // lo que falta para el final de la fila
    float hecho = 0; Fin f = LLEGO;
    if (falta > 5) f = avanzar(falta, false, 1, hecho);
    x += ida ? hecho : -hecho;
    if (f == USUARIO) { Serial.println(F("parado")); return; }
    if (f == OBJETO) {
      float movido; int res = acercarYRevisar(movido);
      if (res == 2) return;
      if (res == 1) { encontrado(); return; }
      Serial.println(F("no es mi vaso: me devuelvo y paso a la fila siguiente"));
      ponerLED(50, 15, 0);                               // naranja
      float h; avanzar(-movido, true, 0, h);
    }
    // --- pasar a la fila siguiente ---
    if (fila + dirFilas < 0 || fila + dirFilas >= CUADROS) {
      dirFilas = -dirFilas;                              // llegó a la última fila: barre de vuelta
      Serial.println(F("recorrí todas las filas: vuelvo a barrer"));
    }
    // lado del giro: depende de si va de ida o de vuelta y hacia dónde van las filas
    int lado = (ida ? 1 : -1) * dirFilas * (FILAS_A_LA_DERECHA ? 1 : -1);
    girar(90 * lado);
    float h = 0; Fin fp = avanzar(CELDA_MM, false, 1, h);   // baja un cuadro
    if (fp == USUARIO) return;
    if (fp == OBJETO) {                                  // algo en el paso entre filas: también lo revisa
      float movido; int res = acercarYRevisar(movido);
      if (res == 2) return;
      if (res == 1) { encontrado(); return; }
      float h2; avanzar(-movido, true, 0, h2);
    }
    fila += dirFilas;
    girar(90 * lado);
    ida = !ida;
    Serial.printf("fila %d (x = %.0f mm)\n", fila + 1, x);
  }
}

// ============================================================================
// 14. MENÚ POR EL MONITOR SERIE
// ============================================================================
void ayuda() {
  Serial.println(F("\n--- COMANDOS ---"));
  Serial.println(F(" w  balance de blancos (hoja blanca a 2 cm del sensor de color)"));
  Serial.println(F(" o  guardar el color del vaso OBJETIVO (ponlo a 2 cm)"));
  Serial.println(F(" 1..4  guardar el color de OTROS vasos (los que NO debe aceptar)"));
  Serial.println(F(" b  borrar los otros vasos guardados"));
  Serial.println(F(" k  probar el reconocimiento 15 s (LED verde = es, rojo = no)"));
  Serial.println(F(" +  más tolerante con el color     -  más exigente"));
  Serial.println(F(" a  avanzar un cuadro (25 cm)  -> si se quedó corto 'c', si se pasó 'l'"));
  Serial.println(F(" g  girar 90° a la derecha     m  probar motores"));
  Serial.println(F(" t  leer sensores   f  ver distancia 5 s   i  ver sensores I2C"));
  Serial.println(F(" p  ver lo guardado    x  BUSCAR / parar"));
}
void imprimirEstado() {
  Serial.printf("objetivo: %s (r %.3f g %.3f b %.3f) | tolerancia %.3f | otros vasos:",
                objetivo.ok ? "guardado" : "NO guardado", objetivo.c[0], objetivo.c[1], objetivo.c[2], tolerancia);
  for (int i = 0; i < 4; i++) if (otros[i].ok) Serial.printf(" %d", i + 1);
  Serial.printf("\nvelocidad %.0f mm/s (lento %.0f) | balance %.2f %.2f %.2f | ToF %s | giroscopio %s\n",
                MM_S_RAPIDO, MM_S_LENTO, ganancia[0], ganancia[1], ganancia[2],
                tofOk ? "OK" : "NO", mpuOk ? "OK" : "NO");
}
void menu(char c) {
  if (c == 'w') {                                        // balance de blancos
    float R = 0, G = 0, B = 0, r, g, b; int k = 0;
    for (int i = 0; i < 10; i++) if (leerRGB(r, g, b)) { R += r; G += g; B += b; k++; }
    if (!k) { Serial.println(F("el sensor de color no responde: revisa S2 18, S3 19, OUT 23, S0 3V3, S1 GND")); return; }
    float m = (R + G + B) / 3;
    ganancia[0] = m / R; ganancia[1] = m / G; ganancia[2] = m / B;
    guardarTodo();
    Serial.println(F("balance de blancos guardado. Ahora guarda los vasos otra vez ('o' y '1'..'4')."));
  } else if (c == 'o') {                                 // vaso objetivo
    float var;
    if (!medirColor(objetivo, var)) { Serial.println(F("el sensor de color no responde")); return; }
    tolerancia = limitar(3.0f * var, 0.02f, 0.06f);      // tolerancia según qué tanto varían las lecturas
    guardarTodo();
    Serial.printf("objetivo guardado: r %.3f g %.3f b %.3f | tolerancia %.3f\n",
                  objetivo.c[0], objetivo.c[1], objetivo.c[2], tolerancia);
  } else if (c >= '1' && c <= '4') {                     // otros vasos
    float var; int i = c - '1';
    if (!medirColor(otros[i], var)) { Serial.println(F("el sensor de color no responde")); return; }
    guardarTodo();
    float d = difTono(otros[i].c, objetivo.c);
    Serial.printf("otro vaso %d guardado (diferencia con el objetivo %.3f)%s\n", i + 1, d,
                  d < 1.5f * tolerancia ? "  OJO: se parece MUCHO al objetivo" : "");
  } else if (c == 'b') {
    for (int i = 0; i < 4; i++) otros[i].ok = false;
    guardarTodo(); Serial.println(F("otros vasos borrados"));
  } else if (c == 'k') {                                 // probar reconocimiento
    Serial.println(F("15 s: pon y quita vasos frente al sensor de color (cualquier tecla para salir)"));
    uint32_t t0 = millis();
    while (millis() - t0 < 15000 && !Serial.available()) {
      float col[4], dObj, dOtro, brillo;
      if (!leerColor(col)) { Serial.println(F("  sin lectura")); continue; }
      bool si = esObjetivo(col, dObj, dOtro, brillo);
      si ? ponerLED(0, 60, 0) : ponerLED(60, 0, 0);
      Serial.printf("  dif %.3f (tol %.3f) | brillo x%.2f | otro vaso más cercano %.3f -> %s\n",
                    dObj, tolerancia, brillo, dOtro < 9 ? dOtro : 0, si ? "ES EL OBJETIVO" : "no");
      delay(200);
    }
    while (Serial.available()) Serial.read();
    ponerLED(20, 20, 20);
  } else if (c == '+' || c == '-') {
    tolerancia *= (c == '+') ? 1.2f : 1 / 1.2f;
    guardarTodo(); Serial.printf("tolerancia %.3f\n", tolerancia);
  } else if (c == 'a') {
    float h; rumboRef = rumbo; avanzar(CELDA_MM, false, 0, h);
    Serial.println(F("¿avanzó 25 cm? si se quedó CORTO escribe 'c', si se PASÓ escribe 'l'"));
  } else if (c == 'c' || c == 'l') {                     // ajustar la velocidad estimada
    float f = (c == 'c') ? 0.95f : 1.05f;                // corto: cree que va más rápido de lo que va
    MM_S_RAPIDO *= f; MM_S_LENTO *= f;
    guardarTodo(); Serial.printf("velocidad estimada %.0f mm/s. Prueba otra vez con 'a'.\n", MM_S_RAPIDO);
  } else if (c == 'g') { girar(90); Serial.println(F("giré 90° a la derecha"));
  } else if (c == 'm') {
    Serial.println(F("rueda IZQUIERDA adelante...")); motorIzq(PWM_RAPIDO); delay(1000); motorIzq(0); delay(300);
    Serial.println(F("rueda DERECHA adelante..."));   motorDer(PWM_RAPIDO); delay(1000); motorDer(0);
    Serial.println(F("las dos deben ir hacia ADELANTE (si no, intercambia los cables de ese motor)"));
  } else if (c == 't') {
    float col[4] = {0, 0, 0, 0}, dObj = 0, dOtro = 0, brillo = 0;
    bool hay = leerColor(col);
    bool si = hay && esObjetivo(col, dObj, dOtro, brillo);
    Serial.printf("distancia %u mm | color r %.3f g %.3f b %.3f | dif con objetivo %.3f -> %s\n",
                  frente(), col[0], col[1], col[2], dObj, hay ? (si ? "ES EL OBJETIVO" : "no") : "sin lectura");
  } else if (c == 'f') {
    uint32_t t0 = millis();
    while (millis() - t0 < 5000) { actualizarTof(); Serial.printf("  %u mm\n", frente()); delay(200); }
  } else if (c == 'i') {
    Serial.print(F("sensores en 21/22:"));
    for (uint8_t a = 1; a < 127; a++) { Wire.beginTransmission(a); if (Wire.endTransmission() == 0) Serial.printf(" 0x%02X", a); }
    Serial.println(F("   (0x29 = distancia, 0x68 = giroscopio)"));
  } else if (c == 'p') imprimirEstado();
  else if (c == 'h') ayuda();
  else if (c == 'x') buscar();
}

// ============================================================================
// 15. SETUP / LOOP
// ============================================================================
void setup() {
  Serial.begin(115200); delay(300);
  pinMode(PIN_DIR_I1, OUTPUT); pinMode(PIN_DIR_I2, OUTPUT);
  pinMode(PIN_DIR_D1, OUTPUT); pinMode(PIN_DIR_D2, OUTPUT);
  pwmIniciar(PIN_VEL_I, 4); pwmIniciar(PIN_VEL_D, 5);
  apagarMotores();
  pinMode(PIN_TCS_S2, OUTPUT); pinMode(PIN_TCS_S3, OUTPUT); pinMode(PIN_TCS_OUT, INPUT);
  ponerLED(30, 30, 0);                                   // amarillo: prendiendo

  Wire.begin(PIN_SDA, PIN_SCL); Wire.setClock(100000);
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0);   // despierta el MPU6050
  mpuOk = Wire.endTransmission() == 0;
  for (int i = 0; i < 3 && !tofOk; i++) { tofOk = lox.begin(); if (!tofOk) delay(100); }
  if (tofOk) lox.startRangeContinuous(50);               // mide cada 50 ms sin bloquear

  mem.begin("carrito", false);
  cargarTodo();

  Serial.printf("\nCARRITO SIMPLE | distancia %s | giroscopio %s | color objetivo %s\n",
                tofOk ? "OK" : "NO RESPONDE", mpuOk ? "OK" : "NO RESPONDE", objetivo.ok ? "guardado" : "FALTA ('o')");
  if (mpuOk) { Serial.println(F("quieto 1 s: calibro el giroscopio...")); calibrarGyro(); detectarSignoGyro(); }
  ayuda();
  Serial.println(F("\nSi no escribes nada, empieza a buscar solo en 10 s."));
}

void loop() {
  static uint32_t t0 = millis();
  static bool autoArranque = true;
  actualizarTof();
  actualizarRumbo();
  ponerLED(20, 20, 20);                                  // blanco tenue: esperando órdenes
  if (Serial.available()) {
    char c = Serial.read();
    if (c > ' ') { autoArranque = false; menu(c); }      // ignora enters y espacios
  }
  // Sin computador: si ya está calibrado, arranca solo a los 10 s.
  if (autoArranque && millis() - t0 > 10000) { autoArranque = false; if (objetivo.ok) buscar(); }
  delay(5);
}
