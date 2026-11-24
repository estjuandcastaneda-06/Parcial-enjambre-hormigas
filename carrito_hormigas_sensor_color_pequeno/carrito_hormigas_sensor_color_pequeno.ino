/*
 * CARRITO HORMIGAS SENSOR COLOR PEQUEÑO — sensor TCS34725 (el morado, I2C). Un solo carro, sin ESP-NOW.
 * ---------------------------------------------------------------------------
 * Explora un espacio de 6 x 6 cuadros buscando el vaso de SU color:
 *   1. Arranca en una esquina (cuadro 0,0) mirando a lo largo de la pared.
 *   2. Avanza de cuadro en cuadro. Como una hormiga, deja "feromona" en cada cuadro que
 *      pisa y prefiere ir a los cuadros con MENOS feromona (los que no ha visitado).
 *      Así recorre todo el espacio sin quedarse dando vueltas en el mismo sitio.
 *   3. Nunca sale del espacio de 6 x 6: conoce sus bordes.
 *   4. Si el VL53L0X ve algo en el cuadro de enfrente, se acerca despacio y lee el color:
 *        - si ES su color objetivo: para los motores y se queda quieto (LED verde);
 *        - si NO es: se devuelve, marca ese cuadro como ocupado y sigue buscando.
 *
 * Por qué antes daba vueltas en círculo: si el MPU6050 queda montado de otra forma
 * (volteado o girado), el giroscopio mide el giro al revés y el carro "corrige" hacia el
 * lado equivocado. Ahora, al prenderlo, el carro hace un giro corto a la derecha y de vuelta
 * para averiguar solo el sentido correcto del giroscopio.
 *
 * Por qué se estrellaba y no seguía los cuadros (corregido en esta versión):
 *   - Solo frenaba si el objeto estaba a menos de 4 cm, y el VL53L0X tan cerca casi siempre
 *     da lectura "inválida", así que nunca frenaba. Ahora mira TODO el cuadro de enfrente
 *     antes de avanzar, vigila mientras avanza y frena a 8 cm.
 *   - Si el VL53L0X deja de responder, el carro se detiene (LED azul titilando) en vez de
 *     andar a ciegas, y lo vuelve a intentar solo.
 *   - Después de cada giro "olvidaba" el error del giro (hasta 4°): el error se sumaba y
 *     el carro se torcía de los cuadros. Ya no.
 *   - El avance de un cuadro depende de MM_S_CRUCERO. Ahora el carro lo mide solo con la
 *     tecla 'c' mirando una pared, y si el espacio tiene paredes las usa para corregirse.
 *
 * Hardware: ESP32 DevKit · L298N + 2 motores sin encoder · VL53L0X · sensor de color · MPU6050
 *           LED WS2812 · divisor de batería en GPIO33.
 * Sensor de color: se elige con SENSOR_COLOR (sección 2).
 * TCS3200 (SENSOR_COLOR 3200, el de 4 LEDs):
 *   VCC -> 3V3, GND -> GND, S0 -> 3V3, S1 -> GND, S2 -> GPIO18, S3 -> GPIO19, OUT -> GPIO23.
 * TCS34725 (SENSOR_COLOR 34725) en su PROPIO bus I2C:
 *   VIN -> 3V3, GND -> GND, SDA -> GPIO18, SCL -> GPIO19. LED, INT y 3V3 sin conectar.
 *   (No va en 21/22: tiene la misma dirección I2C que el VL53L0X.)
 *   Al cambiar de sensor hay que volver a calibrar los colores: 'w', '1'..'5', 'p'.
 * Librería: "Adafruit_VL53L0X". Placa: "ESP32 Dev Module".
 *
 * Comandos por el monitor serie (115200):
 *   x  parar / arrancar        t  leer sensores          m  probar cada motor
 *   f  ver el sensor de distancia 5 s (pon la mano enfrente)
 *   i  ver qué sensores I2C encuentra el ESP32 (diagnóstico de cables)
 *   c  medir la velocidad solo (carro mirando una pared blanca a 40-50 cm)
 *   v  avanzar 2 s (medir MM_S_CRUCERO a mano)           n  avanzar 2 s lento (MM_S_LENTO)
 *   a  avanzar un cuadro       g  girar 90° a la derecha  d  ver el mapa y el estado
 *   +  se pasó del cuadro con 'a' (avanzará menos)       -  se quedó corto (avanzará más)
 *   r  reiniciar el mapa: vuelve a la esquina de inicio
 *   l  probar el LED           w  balance de blancos (hoja blanca enfrente)
 *   1..5  guardar el color del vaso 1..5 (ponlo frente al sensor)
 *   p  imprimir colores, balance de blancos y velocidades para pegarlos en el código
 *
 * EL ESPACIO: el carro no "ve" el piso. Sabe dónde está contando cuadros desde la esquina
 * de inicio, con el tiempo que avanza (no tiene encoders) y el giroscopio. Para que respete
 * el 6x6:  1) arráncalo SIEMPRE en el cuadro (0,0) mirando hacia INICIO_DIR;
 *          2) ajusta el largo del cuadro con 'a' y '+'/'-' hasta que avance justo 25 cm;
 *          3) mejor aún: pon bordes de cartón alrededor y HAY_PAREDES = true (se corrige solo).
 */
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// ============================================================================
// 1. LO QUE MÁS VAS A CAMBIAR
// ============================================================================
// Vaso que busca ESTE carro: 1 = $50, 2 = $100, 3 = $200, 4 = $500, 5 = $1000
// (el mismo número que usaste para guardar su color con las teclas 1..5).
// A cada carro súbele un número distinto.
static const int8_t VASO_OBJETIVO = 1;
static const int8_t COLOR_OBJETIVO = VASO_OBJETIVO - 1;   // índice interno 0..4 (no tocar)

// Espacio: GRID_N x GRID_N cuadros de CELDA_MM. 6 x 250 mm = 1,5 m x 1,5 m.
static const uint8_t  GRID_N   = 6;
static const uint16_t CELDA_MM = 250;

// Dónde y cómo arranca: cuadro (0,0) = esquina de arriba a la izquierda.
// Dirección: 0 = Norte (arriba), 1 = Este (derecha), 2 = Sur (abajo), 3 = Oeste (izquierda).
static const uint8_t INICIO_X = 0, INICIO_Y = 0, INICIO_DIR = 1;

// Distancia del centro del carro al VL53L0X (la mitad del largo del carro, ~17,9 cm / 2).
static const uint16_t LARGO_FRENTE_MM = 90;

// ============================================================================
// 2. PINES (los mismos de siempre)
// ============================================================================
static const uint8_t PIN_VEL_I = 25, PIN_DIR_I1 = 26, PIN_DIR_I2 = 27;   // motor izquierdo: ENA, IN1, IN2
static const uint8_t PIN_VEL_D = 14, PIN_DIR_D1 = 32, PIN_DIR_D2 = 4;    // motor derecho:  ENB, IN3, IN4
// SENSOR DE COLOR: 3200 = TCS3200 (el de 4 LEDs con pin OUT, el que tiene montado el carro)
//                  34725 = TCS34725 / CJMCU-34725 (I2C)
#define SENSOR_COLOR 34725
#if SENSOR_COLOR == 3200
static const uint8_t PIN_TCS_S2 = 18, PIN_TCS_S3 = 19, PIN_TCS_OUT = 23; // S0 a 3V3, S1 a GND
#else
static const uint8_t PIN_COLOR_SDA = 18, PIN_COLOR_SCL = 19;            // TCS34725 (VIN a 3V3; LED, INT y 3V3 sin conectar)
#endif
static const uint8_t PIN_SDA = 21, PIN_SCL = 22;                         // VL53L0X + MPU6050
static const uint8_t PIN_LED = 5, PIN_VBAT = 33;

// ============================================================================
// 3. MOVIMIENTO [CALIBRAR]
// ============================================================================
static const int   PWM_CRUCERO  = 230;                  // 0..255
static const int   PWM_LENTO    = 190;
static float MM_S_CRUCERO = 150.0f;                     // [CALIBRAR] con 'c', o con 'a' + '+'/'-'
static float MM_S_LENTO   = 70.0f;                      // [CALIBRAR] con 'c' (o a mano con 'n')
// Voltaje de la batería cuando se midieron esas velocidades (lo pone 'c' o '+'/'-').
// Con la batería más baja el carro anda más lento: el código lo compensa con esto. 0 = no compensar.
static float VBAT_VEL = 0.0f;
static const int   PWM_GIRO_MIN = 185, PWM_GIRO_MAX = 240;
static const float KP_GIRO  = 3.0f;
static const float KP_RECTO = 4.0f;                     // corrección por grado para ir derecho
static const int   AJUSTE_IZQ = 0, AJUSTE_DER = 0;      // si un motor es más rápido, réstale aquí (p. ej. -15)
static const int   RAMPA_PWM = 30;                      // arranque suave: máximo cambio de PWM cada 20 ms
static const float TOL_GIRO_GRADOS = 4.0f;
static const uint32_t T_GIRO_MAX_MS = 4000;             // si un giro tarda más, se da por terminado
static const uint32_t PWM_FREQ = 1000;
// Sentido del giroscopio. Se averigua solo al prender (AUTO_SIGNO_GYRO). Si la prueba
// falla (ruedas en el aire, sin batería), usa este valor.
static const bool   AUTO_SIGNO_GYRO = true;
static const int8_t SIGNO_GYRO_DEFECTO = -1;

// ============================================================================
// 4. SENSORES [CALIBRAR]
// ============================================================================
// Desde el centro de un cuadro, el cuadro de enfrente va de 35 a 285 mm del sensor
// (CELDA_MM 250, LARGO_FRENTE_MM 90). Por eso el umbral cubre hasta 280 mm.
static const uint16_t UMBRAL_OBJETO_MM  = 280;          // algo más cerca que esto = hay algo en el cuadro de enfrente
static const uint16_t DIST_FRENO_MM     = 80;           // avanzando: si algo queda así de cerca, frena ya
static const uint16_t DIST_REVISAR_MM   = 40;           // se acerca hasta esta distancia, frena y lee el color
static const float    MAX_ACERCA_MM     = 260.0f;       // máximo que avanza al acercarse
// ¿El espacio de 6x6 tiene PAREDES de verdad (tablas, cajas)? Si es solo cinta en el piso,
// déjalo en false: así el carro nunca confunde un vaso con "la pared" y siempre frena a revisar.
static const bool     HAY_PAREDES       = false;
static const float    MARGEN_PARED_MM   = 80.0f;        // (solo con paredes) lectura a ±8 cm de la pared = es la pared
static const uint32_t TOF_SIN_DATOS_MS  = 600;          // si el VL53L0X no manda nada en este tiempo, el carro se detiene
// Reconocer SU vaso exige 3 cosas (así no acepta "cualquier azul"):
//   1. que su color sea el más parecido de los 5,
//   2. que la diferencia ("dif") sea menor que UMBRAL_COLOR,
//   3. que esté claramente más cerca que el 2.º más parecido: dif <= MARGEN_COLOR × dif del 2.º.
static const float    UMBRAL_COLOR      = 0.035f;       // más pequeño = más exigente ('t' muestra "dif")
static const float    MARGEN_COLOR      = 0.6f;         // más pequeño = más exigente
// Además del tono, se compara qué tan CLARO u OSCURO es (azul claro ≠ azul oscuro).
// Necesita el balance de blancos ('w') hecho a la misma distancia a la que frena el carro.
static const float    PESO_BRILLO       = 0.15f;

static const float VBAT_MIN = 6.4f;
static const float DIVISOR_VBAT = (100.0f + 47.0f) / 47.0f;
static const float VBAT_CAL = 1.36f;                    // (multímetro) / (lo que muestra 't')

// Colores de los 5 vasos: {rojo, verde, azul, brillo}. Para recalibrar: 'w' (hoja blanca),
// '1'..'5' (cada vaso), 'p', y pega aquí lo que imprime 'p' (reemplaza CENT, gainRGB y BLANCO_SUMA).
// Estos valores son de ejemplo: HAY QUE CALIBRAR con tus vasos y tu sensor.
static float CENT[5][4] = {
  {0.461f, 0.261f, 0.278f, 0.0f},   // vaso 1 ($50)
  {0.336f, 0.334f, 0.331f, 0.0f},   // vaso 2 ($100)
  {0.276f, 0.338f, 0.386f, 0.0f},   // vaso 3 ($200)
  {0.356f, 0.322f, 0.322f, 0.0f},   // vaso 4 ($500)
  {0.430f, 0.320f, 0.251f, 0.0f},   // vaso 5 ($1000)
};
static float gainRGB[3] = {0.991f, 1.067f, 0.949f};   // corrige el sensor para que el blanco dé R = G = B
static float BLANCO_SUMA = 0.0f;                      // qué tan clara se ve la hoja blanca (0 = no usar brillo)
static const uint16_t VASO_VALOR[5] = {50, 100, 200, 500, 1000};

// Feromona de exploración
static const float FEROMONA_DEPOSITO = 1.0f;            // lo que deja en cada cuadro que pisa
static const float FEROMONA_EVAPORA  = 0.02f;           // fracción que se evapora en cada paso
// Cuánto prefiere seguir DERECHO en vez de girar. Más grande = líneas rectas más largas
// (casi siempre va derecho hasta la pared o hasta un objeto). 1 = gira mucho.
static const float PREFERIR_DERECHO  = 8.0f;

// ============================================================================
// 5. TIPOS Y ESTADO
// ============================================================================
enum Estado : uint8_t { E_INICIO, E_DECIDE, E_GIRA, E_MIRA, E_AVANZA, E_ACERCA, E_REVISA,
                        E_RETROCEDE, E_OBJETIVO, E_PARADO, E_BATERIA, E_SENSOR };
static const char* NOMBRE_ESTADO[] = {"inicio", "decide", "gira", "mira", "avanza", "acerca", "revisa",
                                      "retrocede", "objetivo", "parado", "bateria", "sensor"};
enum Mov : uint8_t { MOV_NADA, MOV_GIRO, MOV_RECTO };

static const int8_t DX[4] = {0, 1, 0, -1};
static const int8_t DY[4] = {-1, 0, 1, 0};
static const float  RUMBO_DIR[4] = {-90.0f, 0.0f, 90.0f, 180.0f};   // 0° = Este, +90° = Sur
static const char   LETRA_DIR[4] = {'N', 'E', 'S', 'O'};

static Estado estado = E_INICIO;
static uint32_t tEstado = 0;
static uint8_t miX = INICIO_X, miY = INICIO_Y, miDir = INICIO_DIR, dirPend = INICIO_DIR;
static float feromona[GRID_N][GRID_N];
static bool  ocupada[GRID_N][GRID_N];
static float avanceDesdeCentro = 0;
static uint16_t pasos = 0, revisados = 0;

static Mov mov = MOV_NADA;
static float rumbo = 0, rumboObjetivo = 0, gzBias = 0;
static int8_t signoGyro = SIGNO_GYRO_DEFECTO;
static bool mpuOk = false;                              // ¿el MPU6050 responde?
static bool signoDetectado = false;                     // ¿ya se averiguó el sentido del giroscopio?
static float distObjetivo = 0, distHecha = 0;
static bool movLento = false;
static uint32_t tGiroOk = 0, tGiroIni = 0;
static int pwmI = 0, pwmD = 0;

static Adafruit_VL53L0X lox;
static bool tofOk = false;
static uint16_t tofMm = 8190;                           // distancia filtrada (la menor de las 2 últimas)
static uint16_t tofCrudo = 8190;
static uint8_t  tofEstado = 255;                        // 0 = buena, 1-3 = dudosa, 4 = nada enfrente
static uint32_t tTof = 0;                               // cuándo llegó la última lectura
static uint16_t tofHist[3] = {8190, 8190, 8190};
static uint8_t  tofIdx = 0;
static uint32_t tQuieto = 0;                            // desde cuándo los motores están quietos
static float vbatActual = 0;
// Cuánto más rápido/lento anda con la batería de ahora que cuando se calibró (1 = igual).
static float factorBateria() {
  if (VBAT_VEL < 3.0f || vbatActual < 3.0f) return 1.0f;   // sin datos: no compensa
  return constrain(vbatActual / VBAT_VEL, 0.75f, 1.25f);
}
static uint8_t vbatBajas = 0;

// ============================================================================
// 6. UTILIDADES
// ============================================================================
static inline float envolver180(float a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static float azar() { return (esp_random() >> 8) / 16777216.0f; }
static void cambiar(Estado e) { estado = e; tEstado = millis(); }
static bool dentro(int x, int y) { return x >= 0 && y >= 0 && x < GRID_N && y < GRID_N; }

static void ponerLED(uint8_t r, uint8_t g, uint8_t b) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  rgbLedWrite(PIN_LED, r, g, b);
#else
  neopixelWrite(PIN_LED, r, g, b);
#endif
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
// duty en [-255, 255]: positivo = adelante. Directo al L298N, sin rampa.
static void motorIzqDirecto(int duty) {
  if (duty != 0) duty += (duty > 0 ? AJUSTE_IZQ : -AJUSTE_IZQ);
  duty = constrain(duty, -255, 255);
  digitalWrite(PIN_DIR_I1, duty > 0); digitalWrite(PIN_DIR_I2, duty < 0);
  pwmEscribir(PIN_VEL_I, CH_A, abs(duty));
}
static void motorDerDirecto(int duty) {
  if (duty != 0) duty += (duty > 0 ? AJUSTE_DER : -AJUSTE_DER);
  duty = constrain(duty, -255, 255);
  digitalWrite(PIN_DIR_D1, duty > 0); digitalWrite(PIN_DIR_D2, duty < 0);
  pwmEscribir(PIN_VEL_D, CH_B, abs(duty));
}
// Con rampa (arranque suave: menos pico de corriente). Para frenar, corta de una.
static int rampa(int actual, int objetivo) {
  if (objetivo == 0) return 0;
  int d = objetivo - actual;
  if (d > RAMPA_PWM) d = RAMPA_PWM;
  if (d < -RAMPA_PWM) d = -RAMPA_PWM;
  return actual + d;
}
static void motores(int objI, int objD) {
  pwmI = rampa(pwmI, objI); pwmD = rampa(pwmD, objD);
  motorIzqDirecto(pwmI); motorDerDirecto(pwmD);
}
static void apagarMotores() { pwmI = pwmD = 0; motorIzqDirecto(0); motorDerDirecto(0); }

static const uint8_t MPU_ADDR = 0x68;
static float leerGyroZ() {                               // °/s, ±250 °/s
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x47); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return 0;
  int16_t raw = (Wire.read() << 8) | Wire.read();
  return raw / 131.0f;
}
static void calibrarGyro() {
  float s = 0; const int N = 300;
  for (int i = 0; i < N; i++) { s += leerGyroZ(); delay(3); }
  gzBias = s / N;
}
// Gira un poco a la derecha (horario) y mira hacia dónde lo mide el giroscopio.
// Así el carro sabe solo cómo quedó montado el MPU6050 (y si los motores están cruzados).
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
    signoGyro = giro > 0 ? 1 : -1;
    signoDetectado = true;
    Serial.printf("Giroscopio: giró %.0f°, sentido = %d (listo)\n", fabsf(giro), signoGyro);
  } else {
    Serial.printf("Giroscopio: casi no giró (%.1f°). Uso el sentido por defecto (%d). "
                  "¿Switch de las pilas apagado? Lo vuelvo a probar cuando escribas 'x'.\n", giro, signoGyro);
  }
}

// ---- órdenes de movimiento (las ejecuta lazoControl) ----
static void parar() { mov = MOV_NADA; }
static void girarA(uint8_t d) { rumboObjetivo = RUMBO_DIR[d]; mov = MOV_GIRO; tGiroOk = 0; tGiroIni = millis(); }
static void moverMM(float mm, bool lento) { distObjetivo = mm; distHecha = 0; movLento = lento; mov = MOV_RECTO; }

// Cada 20 ms: rumbo (giroscopio), distancia estimada por tiempo y PWM de cada motor.
static void lazoControl(float dt) {
  float gz = leerGyroZ();
  rumbo = envolver180(rumbo + signoGyro * (gz - gzBias) * dt);   // + = horario visto desde arriba
  // Con el carro quieto, el giroscopio debería marcar 0: si no, ajusta poco a poco su
  // error (deriva). Así el rumbo no se va torciendo con el tiempo.
  if (mov == MOV_NADA && pwmI == 0 && pwmD == 0) {
    if (!tQuieto) tQuieto = millis();
    if (millis() - tQuieto > 400 && fabsf(gz - gzBias) < 1.5f) gzBias += 0.02f * (gz - gzBias);
  } else tQuieto = 0;
  if (estado == E_BATERIA || estado == E_OBJETIVO) mov = MOV_NADA;
  int uI = 0, uD = 0;
  if (mov == MOV_GIRO) {
    float err = envolver180(rumboObjetivo - rumbo);
    if (fabsf(err) < TOL_GIRO_GRADOS) {
      if (!tGiroOk) tGiroOk = millis();
      else if (millis() - tGiroOk > 150) mov = MOV_NADA;   // el error que quede lo corrige al avanzar
    } else {
      tGiroOk = 0;
      float p = clampf(KP_GIRO * fabsf(err), (float)PWM_GIRO_MIN, (float)PWM_GIRO_MAX);
      int s = err > 0 ? 1 : -1;
      uI = (int)(s * p); uD = (int)(-s * p);              // giro en el sitio
    }
    if (mov == MOV_GIRO && millis() - tGiroIni > T_GIRO_MAX_MS) {
      mov = MOV_NADA; uI = uD = 0;
      Serial.printf("giro incompleto (faltaron %.0f°): sigo igual\n", envolver180(rumboObjetivo - rumbo));
    }
  } else if (mov == MOV_RECTO) {
    float falta = fabsf(distObjetivo) - fabsf(distHecha);
    if (falta <= 0) mov = MOV_NADA;
    else {
      float signo = distObjetivo >= 0 ? 1.0f : -1.0f;
      float base = movLento ? PWM_LENTO : PWM_CRUCERO;
      float corr = clampf(KP_RECTO * envolver180(rumboObjetivo - rumbo), -60.0f, 60.0f);
      uI = (int)(signo * base + corr); uD = (int)(signo * base - corr);
      // Sin encoder: distancia = velocidad × tiempo (corregida por el voltaje de la batería).
      distHecha += signo * (movLento ? MM_S_LENTO : MM_S_CRUCERO) * factorBateria() * dt;
    }
  }
  motores(uI, uD);
}

// ============================================================================
// 8. SENSORES
// ============================================================================
// ¿El VL53L0X está mandando lecturas?
static bool tofSano() { return tofOk && millis() - tTof < TOF_SIN_DATOS_MS; }
static uint16_t frenteMM() { return (tofSano() && tofMm < 2000) ? tofMm : 9999; }

// Convierte una lectura del VL53L0X en mm. Antes solo se aceptaban las lecturas "perfectas"
// (estado 0); muy cerca de un objeto el sensor da estado 1-3 y el carro creía que no había
// nada y se estrellaba. Ahora solo se descarta el estado 4 (= no hay nada enfrente).
static uint16_t tofAmm(uint16_t r, uint8_t st) {
  if (st == 4 || r == 0xFFFF || r >= 2000) return 8190;
  return r < 20 ? 20 : r;
}
// Lee el VL53L0X si tiene un dato nuevo (no bloquea). Devuelve true si llegó uno.
static bool actualizarTof() {
  if (!tofOk || !lox.isRangeComplete()) return false;
  uint16_t r = lox.readRange();
  tofEstado = lox.readRangeStatus();
  tofCrudo = tofAmm(r, tofEstado);
  // Tu sensor pierde muchas lecturas (estado 2): si se usaba la mediana, un vaso visto 1 de
  // cada 3 veces "desaparecía". Ahora vale la MENOR de las 2 últimas: si lo vio, frena.
  uint16_t anterior = tofHist[(tofIdx + 2) % 3];
  tofHist[tofIdx] = tofCrudo; tofIdx = (tofIdx + 1) % 3;
  tofMm = min(tofCrudo, anterior);
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
// Promedio de n lecturas (bloquea; solo para calibrar). -1 si no hay lecturas.
static float leerTofPromedio(int n) {
  float s = 0; int k = 0; uint32_t t0 = millis();
  while (k < n && millis() - t0 < 2000) {
    if (actualizarTof() && tofCrudo < 2000) { s += tofCrudo; k++; }
    delay(5);
  }
  return k ? s / k : -1.0f;
}

// Distancia que debería medir el VL53L0X hasta la PARED del espacio (no hasta un objeto).
// Sin paredes de verdad devuelve "muy lejos": todo lo que vea es un objeto a revisar.
static float paredEsperadaMM() {
  if (!HAY_PAREDES) return 99999.0f;
  int celdas;
  switch (miDir) {
    case 0: celdas = miY; break;
    case 1: celdas = GRID_N - 1 - miX; break;
    case 2: celdas = GRID_N - 1 - miY; break;
    default: celdas = miX; break;
  }
  return celdas * (float)CELDA_MM + CELDA_MM / 2.0f - LARGO_FRENTE_MM;
}

static bool tcsOk = false;                               // ¿el sensor de color responde?

#if SENSOR_COLOR == 3200
// ---- TCS3200 (el de 4 LEDs y pin OUT): mide la frecuencia de la salida OUT ----
static float leerCanal(bool s2, bool s3) {
  digitalWrite(PIN_TCS_S2, s2); digitalWrite(PIN_TCS_S3, s3);
  delayMicroseconds(400);
  unsigned long suma = 0; int n = 0;
  for (int i = 0; i < 6; i++) { unsigned long t = pulseIn(PIN_TCS_OUT, LOW, 20000); if (t) { suma += t; n++; } }
  if (!n) return 0;
  return 1.0e6f / (2.0f * (float)suma / n);
}
static bool iniciarColor() {
  pinMode(PIN_TCS_S2, OUTPUT); pinMode(PIN_TCS_S3, OUTPUT); pinMode(PIN_TCS_OUT, INPUT);
  tcsOk = leerCanal(LOW, LOW) > 0 || leerCanal(HIGH, HIGH) > 0;
  if (tcsOk) Serial.println(F("TCS3200 OK"));
  else Serial.println(F("TCS3200 no responde: revisa VCC, GND, S0 a 3V3, S1 a GND, S2 al 18, S3 al 19 y OUT al 23"));
  return tcsOk;
}
static bool leerRGB(float& r, float& g, float& b) {
  r = leerCanal(LOW, LOW); b = leerCanal(LOW, HIGH); g = leerCanal(HIGH, HIGH);
  return r > 0 && g > 0 && b > 0;
}
#else
// ---- TCS34725 (sensor de color I2C) en su propio bus: SDA 18, SCL 19 ----
// Va aparte porque usa la misma dirección (0x29) que el VL53L0X.
static const uint8_t TCS_ADDR = 0x29;
static uint8_t tcsGanancia = 1;                          // 0 = 1x, 1 = 4x, 2 = 16x
static const uint16_t TCS_MAX = 20480;                   // cuenta máxima con ATIME 0xEB (~50 ms)

static void tcsEscribir(uint8_t reg, uint8_t v) {
  Wire1.beginTransmission(TCS_ADDR); Wire1.write(0x80 | reg); Wire1.write(v); Wire1.endTransmission();
}
static int tcsLeer8(uint8_t reg) {
  Wire1.beginTransmission(TCS_ADDR); Wire1.write(0x80 | reg);
  if (Wire1.endTransmission() != 0) return -1;
  if (Wire1.requestFrom(TCS_ADDR, (uint8_t)1) != 1) return -1;
  return Wire1.read();
}
static bool iniciarColor() {
  Wire1.begin(PIN_COLOR_SDA, PIN_COLOR_SCL, 100000);
  int id = tcsLeer8(0x12);
  if (id < 0) {
    Serial.println(F("TCS34725 no responde: revisa VIN a 3V3, GND, SDA al 18 y SCL al 19 (y que los pines estén SOLDADOS)"));
    tcsOk = false; return false;
  }
  if (id != 0x44 && id != 0x4D)                          // algunos clones dan otro número: igual se usa
    Serial.printf("sensor de color con id 0x%02X (no es el típico 0x44, pruebo igual)\n", id);
  tcsEscribir(0x00, 0x01); delay(3);                     // ENABLE: prender
  tcsEscribir(0x00, 0x03);                               // ENABLE: prender + medir
  tcsEscribir(0x01, 0xEB);                               // ATIME: ~50 ms por lectura
  tcsEscribir(0x0F, tcsGanancia);                        // CONTROL: ganancia
  tcsOk = true;
  Serial.println(F("TCS34725 OK"));
  return true;
}
// Lee los canales crudos. Si se satura (vaso muy cerca y muy iluminado), baja la ganancia;
// si casi no ve luz, la sube.
static bool leerRGB(float& r, float& g, float& b) {
  if (!tcsOk) return false;
  for (int intento = 0; intento < 3; intento++) {
    delay(55);                                           // esperar una lectura nueva
    Wire1.beginTransmission(TCS_ADDR); Wire1.write(0xA0 | 0x14);   // leer C, R, G, B seguidos
    if (Wire1.endTransmission() != 0) return false;
    if (Wire1.requestFrom(TCS_ADDR, (uint8_t)8) != 8) return false;
    uint16_t v[4];
    for (int i = 0; i < 4; i++) { uint8_t lo = Wire1.read(), hi = Wire1.read(); v[i] = lo | (hi << 8); }
    if (v[0] > TCS_MAX * 0.9f && tcsGanancia > 0) { tcsEscribir(0x0F, --tcsGanancia); delay(60); continue; }
    if (v[0] < 300 && tcsGanancia < 2)               { tcsEscribir(0x0F, ++tcsGanancia); delay(60); continue; }
    static const float MULT[3] = {1.0f, 4.0f, 16.0f};   // divide por la ganancia: así el brillo
    r = v[1] / MULT[tcsGanancia];                        // no cambia cuando el sensor la ajusta
    g = v[2] / MULT[tcsGanancia];
    b = v[3] / MULT[tcsGanancia];
    return (r + g + b) > 0;
  }
  return false;
}
#endif
// Un color = {proporción de rojo, de verde, de azul, brillo}.
// Las proporciones (r/(r+g+b)...) casi no cambian con la distancia; el brillo distingue
// un color claro de uno oscuro del mismo tono.
static bool leerColor(float out[4]) {
  float r, g, b;
  if (!leerRGB(r, g, b)) return false;
  r *= gainRGB[0]; g *= gainRGB[1]; b *= gainRGB[2];   // balance de blancos
  float s = r + g + b;
  if (s <= 0) return false;
  out[0] = r / s; out[1] = g / s; out[2] = b / s;
  out[3] = BLANCO_SUMA > 0 ? s / BLANCO_SUMA : 0.0f;  // 1 = tan claro como la hoja blanca
  return true;
}
// Promedio de n lecturas (quita el ruido). false si el sensor no respondió.
static bool leerColorPromedio(float out[4], int n) {
  float acc[4] = {0, 0, 0, 0}, v[4]; int k = 0;
  for (int i = 0; i < n; i++) if (leerColor(v)) { for (int j = 0; j < 4; j++) acc[j] += v[j]; k++; }
  if (!k) return false;
  for (int j = 0; j < 4; j++) out[j] = acc[j] / k;
  return true;
}
// Diferencia entre un color leído y el guardado del vaso k (0 = idénticos).
static float distanciaColor(const float v[4], int k) {
  float d = 0;
  for (int j = 0; j < 3; j++) { float e = v[j] - CENT[k][j]; d += e * e; }
  if (BLANCO_SUMA > 0 && CENT[k][3] > 0) { float e = PESO_BRILLO * (v[3] - CENT[k][3]); d += e * e; }
  return sqrtf(d);
}
// Devuelve el vaso más parecido (0..4), su diferencia (dif) y la del 2.º más parecido (dif2).
static int clasificarColor(float& dif, float* dif2 = nullptr) {
  float v[4];
  dif = 1.0f; if (dif2) *dif2 = 1.0f;
  if (!leerColorPromedio(v, 5)) return -1;
  int mejor = -1; float d1 = 1e9f, d2 = 1e9f;
  for (int k = 0; k < 5; k++) {
    float d = distanciaColor(v, k);
    if (d < d1) { d2 = d1; d1 = d; mejor = k; }
    else if (d < d2) d2 = d;
  }
  dif = d1; if (dif2) *dif2 = d2;
  return mejor;
}
// ¿Lo leído es SU vaso? (las 3 condiciones explicadas en la sección 4)
static bool esMiVaso(int c, float dif, float dif2) {
  return c == COLOR_OBJETIVO && dif <= UMBRAL_COLOR && dif <= MARGEN_COLOR * dif2;
}
// 'w': con una hoja blanca enfrente, calcula las ganancias para que el blanco dé R = G = B
// y guarda qué tan clara se ve (para comparar el brillo de los vasos).
static void balanceBlancos() {
  float r = 0, g = 0, b = 0, sr = 0, sg = 0, sb = 0; int n = 0;
  for (int i = 0; i < 8; i++) if (leerRGB(r, g, b)) { sr += r; sg += g; sb += b; n++; }
  if (!n) { Serial.println(F("el sensor de color no responde: no pude hacer el balance de blancos")); return; }
  r = sr / n; g = sg / n; b = sb / n;
  if (r > 0 && g > 0 && b > 0) {
    float m = (r + g + b) / 3;
    gainRGB[0] = m / r; gainRGB[1] = m / g; gainRGB[2] = m / b;
    BLANCO_SUMA = r + g + b;                             // con las ganancias, el blanco suma 3·m = r+g+b
  }
}
static float leerVBat() {
  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(PIN_VBAT);
  return mv / 8.0f / 1000.0f * DIVISOR_VBAT * VBAT_CAL;
}

// ============================================================================
// 9. EXPLORACIÓN CON FEROMONA
// ============================================================================
// Elige a qué cuadro vecino ir: prefiere los de MENOS feromona (los menos visitados),
// un poco más si es seguir derecho (girar gasta tiempo), y devolverse solo si no hay más.
static int elegirDireccion() {
  float peso[4], total = 0;
  for (uint8_t d = 0; d < 4; d++) {
    peso[d] = 0;
    int nx = miX + DX[d], ny = miY + DY[d];
    if (!dentro(nx, ny) || ocupada[ny][nx]) continue;
    float f = 1.0f + feromona[ny][nx];
    float w = 1.0f / (f * f);
    if (d == miDir) w *= PREFERIR_DERECHO;
    if (d == ((miDir + 2) & 3)) w *= 0.3f;
    peso[d] = w; total += w;
  }
  if (total <= 0) return -1;
  float r = azar() * total;
  for (uint8_t d = 0; d < 4; d++) { if (peso[d] <= 0) continue; r -= peso[d]; if (r <= 0) return d; }
  for (int d = 3; d >= 0; d--) if (peso[d] > 0) return d;
  return -1;
}
static void llegarACelda() {
  for (uint8_t y = 0; y < GRID_N; y++)
    for (uint8_t x = 0; x < GRID_N; x++) feromona[y][x] *= (1.0f - FEROMONA_EVAPORA);
  feromona[miY][miX] += FEROMONA_DEPOSITO;
  pasos++;
  Serial.printf("cuadro (%u,%u) mirando %c | pasos %u\n", miX, miY, LETRA_DIR[miDir], pasos);
}
static void marcarOcupadaEnfrente() {
  int nx = miX + DX[miDir], ny = miY + DY[miDir];
  if (dentro(nx, ny)) ocupada[ny][nx] = true;
}
static void imprimirMapa() {
  Serial.printf("[%s] cuadro (%u,%u) mira %c | rumbo %.1f° | busco el vaso %d ($%u) | pasos %u, revisados %u\n",
                NOMBRE_ESTADO[estado], miX, miY, LETRA_DIR[miDir], rumbo, COLOR_OBJETIVO + 1,
                VASO_VALOR[COLOR_OBJETIVO], pasos, revisados);
  Serial.println(F("mapa (C = carro, # = ocupado, . = sin visitar, 1-9 = feromona):"));
  for (uint8_t y = 0; y < GRID_N; y++) {
    Serial.print("  ");
    for (uint8_t x = 0; x < GRID_N; x++) {
      char c;
      if (x == miX && y == miY) c = 'C';
      else if (ocupada[y][x]) c = '#';
      else if (feromona[y][x] < 0.05f) c = '.';
      else c = '0' + (char)min(9, (int)ceilf(feromona[y][x]));
      Serial.print(c); Serial.print(' ');
    }
    Serial.println();
  }
}

// ============================================================================
// 10. MÁQUINA DE ESTADOS (cada 20 ms)
// ============================================================================
// Mientras se acerca: última distancia buena y cuánto había avanzado cuando la leyó.
// Si el sensor pierde el vaso de vista (pasa mucho de cerca), el carro calcula cuánto le
// falta con lo que ya avanzó, y frena igual en vez de estrellarse.
static float acercaUltMM = 9999, acercaHechaEnLect = 0;
static uint32_t acercaTLect = 0;

// Vio algo mientras avanzaba: si ya pasó la mitad del camino, ya está en el cuadro nuevo.
// Luego se acerca despacio para leer el color.
static void entrarAcerca(float recorrido) {
  parar();
  if (recorrido >= CELDA_MM / 2.0f) {
    miX += DX[miDir]; miY += DY[miDir];
    llegarACelda();
    recorrido -= CELDA_MM;                               // negativo = quedó antes del centro
  }
  avanceDesdeCentro = recorrido;
  float r = frenteMM();
  Serial.printf("veo algo a %.0f mm: me acerco despacio a revisar el color\n", r);
  // Nunca avanza más de lo que lo separa del objeto (+2 cm), aunque deje de verlo.
  float maximo = r < 2000 ? clampf(r - DIST_REVISAR_MM + 20.0f, 0.0f, MAX_ACERCA_MM) : MAX_ACERCA_MM;
  acercaUltMM = r; acercaHechaEnLect = 0; acercaTLect = tTof;
  moverMM(maximo, true);
  cambiar(E_ACERCA);
}

static void decidir() {
  uint32_t enEstado = millis() - tEstado;
  // Sin sensor de distancia NO se mueve (antes andaba a ciegas y se estrellaba).
  if (estado != E_PARADO && estado != E_OBJETIVO && estado != E_BATERIA && estado != E_SENSOR && !tofSano()) {
    parar(); apagarMotores();
    Serial.println(F("el VL53L0X no manda datos: me detengo. Revisa VIN, GND, SDA (21) y SCL (22)."));
    cambiar(E_SENSOR);
    return;
  }
  switch (estado) {
    case E_INICIO:
      if (enEstado > 3000) { Serial.println(F("¡A buscar!")); cambiar(E_DECIDE); }
      break;

    case E_DECIDE: {
      int d = elegirDireccion();
      if (d < 0) {                                       // encerrado: olvido los cuadros ocupados
        for (uint8_t y = 0; y < GRID_N; y++) for (uint8_t x = 0; x < GRID_N; x++) ocupada[y][x] = false;
        Serial.println(F("sin salida: vuelvo a revisar los cuadros ocupados"));
        break;
      }
      dirPend = (uint8_t)d;
      if (dirPend != miDir) { girarA(dirPend); cambiar(E_GIRA); }
      else cambiar(E_MIRA);
      break;
    }

    case E_GIRA:
      if (mov == MOV_NADA) { miDir = dirPend; cambiar(E_MIRA); }
      break;

    case E_MIRA: {                                       // ¿hay algo en el cuadro de enfrente?
      if (enEstado < 250) break;                         // esperar lecturas frescas
      avanceDesdeCentro = 0;
      float r = frenteMM(), pared = paredEsperadaMM();
      if (r < UMBRAL_OBJETO_MM && r < pared - MARGEN_PARED_MM) {
        entrarAcerca(0);
      } else {
        float avance = CELDA_MM;
        // Si lo que ve es la PARED del espacio, la usa para saber dónde está de verdad:
        // si la pared está más cerca de lo que debería, avanza menos (y al revés).
        if (r < 1200 && fabsf(r - pared) < 100) {
          avance = clampf(CELDA_MM + (r - pared), CELDA_MM - 100.0f, CELDA_MM + 100.0f);
          if (fabsf(r - pared) > 15) Serial.printf("corrijo con la pared: avanzo %.0f mm\n", avance);
        }
        moverMM(avance, false);
        cambiar(E_AVANZA);
      }
      break;
    }

    case E_AVANZA: {
      float r = frenteMM();
      float paredAhora = paredEsperadaMM() - distHecha;
      float lim = max((float)DIST_FRENO_MM, UMBRAL_OBJETO_MM - distHecha);
      // Algo apareció en el cuadro al que voy (y no es la pared del espacio): frenar y revisar.
      if (r < lim && r < paredAhora - MARGEN_PARED_MM) { entrarAcerca(distHecha); break; }
      // Muy cerca de algo (aunque parezca la pared): frenar para no estrellarse.
      if (r < DIST_FRENO_MM - 20) {
        if (distHecha >= CELDA_MM / 2.0f) {              // ya casi llegaba: cuenta como llegado
          parar();
          miX += DX[miDir]; miY += DY[miDir];
          llegarACelda();
          cambiar(E_DECIDE);
        } else entrarAcerca(distHecha);
        break;
      }
      if (mov == MOV_NADA) {
        miX += DX[miDir]; miY += DY[miDir];
        llegarACelda();
        cambiar(E_DECIDE);
      }
      break;
    }

    case E_ACERCA: {                                     // despacio hasta tener el objeto cerca
      if (tTof != acercaTLect && frenteMM() < 2000) {    // lectura nueva y buena
        acercaUltMM = frenteMM(); acercaHechaEnLect = distHecha; acercaTLect = tTof;
      }
      float falta = acercaUltMM - (distHecha - acercaHechaEnLect);   // lo que estimo que falta
      if (falta <= DIST_REVISAR_MM || mov == MOV_NADA) {
        avanceDesdeCentro += distHecha; parar(); apagarMotores();
        Serial.printf("frené frente al objeto (sensor %u mm, calculo %.0f mm): leo el color\n", frenteMM(), falta);
        cambiar(E_REVISA);
      }
      break;
    }

    case E_REVISA: {
      if (enEstado < 300) break;                         // quieto antes de leer
      apagarMotores();
      if (!tcsOk && !iniciarColor())                     // sin sensor de color no puede reconocer nada
        Serial.println(F("  (sin sensor de color no puedo reconocer el vaso)"));
      // Lee el color 3 veces: es su vaso si al menos 2 de las 3 dicen que sí.
      float dif = 1.0f; int c = -1, votos = 0;
      for (int i = 0; i < 3; i++) {
        float d, d2; int ci = clasificarColor(d, &d2);
        bool si = esMiVaso(ci, d, d2);
        Serial.printf("  color leído: vaso %d (dif %.3f, el 2.º más parecido %.3f) -> %s\n",
                      ci + 1, d, d2, si ? "ES MI VASO" : "no");
        if (si) votos++;
        if (d < dif) { dif = d; c = ci; }
      }
      revisados++;
      if (votos >= 2) {
        parar(); apagarMotores();
        cambiar(E_OBJETIVO);
        Serial.printf("¡ENCONTRÉ MI VASO! (vaso %d, $%u, dif %.3f). Me quedo quieto.\n",
                      COLOR_OBJETIVO + 1, VASO_VALOR[COLOR_OBJETIVO], dif);
      } else {
        if (dif <= UMBRAL_COLOR) Serial.printf("es el vaso %d (dif %.3f), no el mío: sigo buscando\n", c + 1, dif);
        else Serial.printf("no es un vaso conocido (dif %.3f): sigo buscando\n", dif);
        marcarOcupadaEnfrente();
        // volver al centro del cuadro (nunca hacia adelante: ahí está el objeto)
        moverMM(avanceDesdeCentro > 0 ? -avanceDesdeCentro : 0, true);
        cambiar(E_RETROCEDE);
      }
      break;
    }

    case E_RETROCEDE:
      if (mov == MOV_NADA) cambiar(E_DECIDE);
      break;

    case E_OBJETIVO:
    case E_BATERIA:
      parar();
      break;

    case E_PARADO:                                       // quieto; acepta pruebas por serial
      break;

    case E_SENSOR:                                       // quieto hasta que el VL53L0X vuelva
      parar();
      if (enEstado > 2000) {
        if (iniciarTof()) { Serial.println(F("VL53L0X otra vez OK: sigo buscando")); cambiar(E_DECIDE); }
        else { Serial.println(F("VL53L0X sigue sin responder (reintento en 2 s)")); cambiar(E_SENSOR); }
      }
      break;
  }
}

static void actualizarLED() {
  switch (estado) {
    case E_INICIO:    ponerLED(30, 30, 0); break;                        // amarillo
    case E_DECIDE: case E_GIRA: case E_MIRA: case E_AVANZA:
                      ponerLED(0, 20, 50); break;                        // celeste: explorando
    case E_ACERCA: case E_REVISA:
                      ponerLED(40, 0, 40); break;                        // morado: revisando color
    case E_RETROCEDE: ponerLED(50, 15, 0); break;                        // naranja: no era su color
    case E_OBJETIVO:  ponerLED(0, (millis() / 250) % 2 ? 80 : 10, 0); break;   // verde titilando: ¡lo encontró!
    case E_PARADO:    ponerLED(20, 20, 20); break;                       // blanco tenue
    case E_BATERIA:   ponerLED((millis() / 300) % 2 ? 60 : 0, 0, 0); break;   // rojo titilando
    case E_SENSOR:    ponerLED(0, 0, (millis() / 300) % 2 ? 60 : 0); break;   // azul titilando: sin VL53L0X
  }
}

// ============================================================================
// 11. SERIAL (calibración y pruebas)
// ============================================================================
static void imprimirColores() {
  Serial.println(F("static float CENT[5][4] = {"));
  for (int k = 0; k < 5; k++)
    Serial.printf("  {%.3ff, %.3ff, %.3ff, %.3ff},   // vaso %d ($%u)\n",
                  CENT[k][0], CENT[k][1], CENT[k][2], CENT[k][3], k + 1, VASO_VALOR[k]);
  Serial.println(F("};"));
  Serial.printf("static float gainRGB[3] = {%.3ff, %.3ff, %.3ff};\n", gainRGB[0], gainRGB[1], gainRGB[2]);
  Serial.printf("static float BLANCO_SUMA = %.1ff;\n", BLANCO_SUMA);
  Serial.println(F("^ Copia esas lineas y pegalas en el codigo en lugar de las que estan (seccion 4), luego sube."));
  Serial.printf("static float MM_S_CRUCERO = %.1ff;\nstatic float MM_S_LENTO   = %.1ff;\nstatic float VBAT_VEL = %.2ff;\n",
                MM_S_CRUCERO, MM_S_LENTO, VBAT_VEL);
  Serial.println(F("^ Y estas tres van en la seccion 3 (movimiento)."));
  // Aviso si dos vasos quedaron casi iguales: el carro los va a confundir.
  for (int a = 0; a < 5; a++) for (int b = a + 1; b < 5; b++) {
    float d = distanciaColor(CENT[a], b);
    if (d < 2 * UMBRAL_COLOR)
      Serial.printf("OJO: los vasos %d y %d se parecen mucho (dif %.3f). Calibra más cerca o cambia uno de color.\n", a + 1, b + 1, d);
  }
}
// Avanza derecho 'ms' (o hasta tener algo a menos de 10 cm). Devuelve los segundos que anduvo.
static float moverDosSegundos(bool lento, uint32_t ms = 2000) {
  if (estado != E_PARADO) cambiar(E_PARADO);
  rumboObjetivo = rumbo;                                 // seguir derecho hacia donde mira
  moverMM(100000, lento);
  uint32_t t0 = millis(), tPrev = millis();
  while (millis() - t0 < ms) {
    uint32_t ahora = millis();
    if (ahora - tPrev >= 20) { lazoControl((ahora - tPrev) / 1000.0f); tPrev = ahora; }
    if (actualizarTof() && tofCrudo < 100) { Serial.println(F("algo a menos de 10 cm: freno")); break; }
    delay(2);
  }
  float t = (millis() - t0) / 1000.0f;
  parar(); apagarMotores();
  return t;
}
// 'c': el carro mide su propia velocidad con el VL53L0X mirando una pared.
static void calibrarVelocidad() {
  if (!tofOk) { Serial.println(F("sin VL53L0X no puedo medir: usa 'v' y 'n' con un metro")); return; }
  parar(); apagarMotores(); cambiar(E_PARADO); delay(300);
  float d0 = leerTofPromedio(8);
  // Tu VL53L0X llega a ~50 cm: la prueba es corta (1 s lento + 1 s rápido) para caber ahí.
  if (d0 < 350 || d0 > 1200) {
    Serial.printf("la pared está a %.0f mm. Pon el carro mirando DE FRENTE una pared o caja plana BLANCA a 40-50 cm.\n", d0);
    return;
  }
  Serial.println(F("midiendo lento..."));
  float t1 = moverDosSegundos(true, 1000); delay(700);
  float d1 = leerTofPromedio(8);
  if (d1 < 200) { Serial.println(F("quedó muy cerca de la pared para la parte rápida: aléjalo un poco más y repite")); return; }
  Serial.println(F("midiendo rápido..."));
  float t2 = moverDosSegundos(false, 1000); delay(700);
  float d2 = leerTofPromedio(8);
  if (d1 < 0 || d2 < 0) { Serial.println(F("perdí la pared durante la prueba: intenta de nuevo más derecho")); return; }
  float vl = (d0 - d1) / t1, vc = (d1 - d2) / t2;
  Serial.printf("pared: %.0f -> %.0f -> %.0f mm | lento %.0f mm/s, rápido %.0f mm/s\n", d0, d1, d2, vl, vc);
  if (vl < 20 || vl > 500 || vc < 40 || vc > 800 || vc < vl) {
    Serial.println(F("esos valores no tienen sentido (¿se torció o patinó?): no los guardo. Intenta de nuevo."));
    return;
  }
  MM_S_LENTO = vl; MM_S_CRUCERO = vc;
  VBAT_VEL = leerVBat() > 3.0f ? leerVBat() : 0.0f;      // con qué batería se midió
  Serial.println(F("guardadas (hasta que lo apagues). Para dejarlas fijas escribe 'p' y pega las líneas de la seccion 3."));
}
// 'i': lista las direcciones I2C que responden en un bus.
static void escanearI2C(TwoWire& bus, const char* nombre) {
  Serial.printf("bus %s:", nombre);
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    bus.beginTransmission(a);
    if (bus.endTransmission() == 0) { Serial.printf(" 0x%02X", a); n++; }
  }
  Serial.println(n ? "" : " NADA (revisa cables, alimentación y soldaduras)");
}
// 'f': muestra el VL53L0X durante 5 s para ver si funciona.
static void probarTof() {
  if (!tofOk) { Serial.println(F("el VL53L0X no respondió al prender: revisa VIN (3V3 o 5V), GND, SDA 21, SCL 22")); return; }
  if (estado != E_PARADO) { parar(); apagarMotores(); cambiar(E_PARADO); }
  Serial.println(F("5 s: acerca y aleja la mano del sensor. Debe cambiar la distancia."));
  uint32_t t0 = millis(), tImp = 0; int n = 0;
  while (millis() - t0 < 5000) {
    if (actualizarTof()) n++;
    if (millis() - tImp >= 200) {
      tImp = millis();
      if (tofCrudo >= 2000) Serial.printf("  nada enfrente (estado %u)\n", tofEstado);
      else Serial.printf("  %4u mm (estado %u%s)\n", tofCrudo, tofEstado, tofEstado ? " = dudosa" : "");
    }
    delay(5);
  }
  Serial.printf("%d lecturas en 5 s (deberían ser ~100). %s\n", n,
                n < 20 ? "MUY POCAS: el sensor no está midiendo bien." : "El sensor funciona.");
}
static void leerSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c >= '1' && c <= '5') {                          // '1' = vaso 1 ... '5' = vaso 5
      int k = c - '1';
      float v[4];
      if (leerColorPromedio(v, 20)) {                    // 20 lecturas promediadas
        for (int j = 0; j < 4; j++) CENT[k][j] = v[j];
        Serial.printf("color del vaso %d ($%u) guardado: r %.3f g %.3f b %.3f brillo %.3f\n",
                      k + 1, VASO_VALOR[k], v[0], v[1], v[2], v[3]);
        if (BLANCO_SUMA <= 0) Serial.println(F("  (haz primero 'w' con una hoja blanca para usar también el brillo)"));
      } else { Serial.println(F("el sensor de color no responde:")); iniciarColor(); }
    } else if (c == 'p') imprimirColores();
    else if (c == 'w') {
      balanceBlancos();
      Serial.printf("ganancias RGB %.3f %.3f %.3f | brillo del blanco %.1f\n", gainRGB[0], gainRGB[1], gainRGB[2], BLANCO_SUMA);
      Serial.println(F("Ahora guarda otra vez los 5 vasos (1..5): los colores dependen de este balance."));
    }
    else if (c == 't') {
      float v[4] = {0, 0, 0, 0}; leerColorPromedio(v, 3);
      float dif, dif2; int col = clasificarColor(dif, &dif2);
      Serial.printf("ToF %u mm (estado %u%s) | color r %.3f g %.3f b %.3f brillo %.2f\n",
                    tofMm, tofEstado, tofSano() ? "" : ", SIN DATOS", v[0], v[1], v[2], v[3]);
      Serial.printf("  se parece más al vaso %d (dif %.3f; el 2.º más parecido %.3f) -> %s | vbat %.2f V | rumbo %.1f°\n",
                    col + 1, dif, dif2,
                    esMiVaso(col, dif, dif2) ? "ES MI VASO" :
                      (dif > UMBRAL_COLOR ? "no se parece lo suficiente a ninguno" :
                       (col == COLOR_OBJETIVO ? "parecido a mi vaso pero no seguro" : "no es mi vaso")),
                    leerVBat(), rumbo);
    } else if (c == 'f') probarTof();
    else if (c == '+' || c == '-') {                     // ajuste fino del largo de cada cuadro
      // '+' si con 'a' se PASÓ de 25 cm (el carro cree que va más lento de lo que va).
      // '-' si con 'a' se QUEDÓ CORTO.
      float f = (c == '+') ? 1.05f : 0.95f;
      MM_S_CRUCERO *= f; MM_S_LENTO *= f;
      if (leerVBat() > 3.0f) VBAT_VEL = leerVBat();
      Serial.printf("MM_S_CRUCERO = %.1f, MM_S_LENTO = %.1f. Prueba otra vez con 'a'. Cuando quede bien, 'p'.\n",
                    MM_S_CRUCERO, MM_S_LENTO);
    }
    else if (c == 'r') {                                 // volver a empezar desde la esquina
      parar(); apagarMotores(); cambiar(E_PARADO);
      for (uint8_t y = 0; y < GRID_N; y++) for (uint8_t x = 0; x < GRID_N; x++) { feromona[y][x] = 0; ocupada[y][x] = false; }
      miX = INICIO_X; miY = INICIO_Y; miDir = INICIO_DIR;
      feromona[miY][miX] = FEROMONA_DEPOSITO;
      rumbo = rumboObjetivo = RUMBO_DIR[miDir]; pasos = revisados = 0;
      Serial.printf("reiniciado: pon el carro en el cuadro (%u,%u) mirando %c y escribe 'x'\n", miX, miY, LETRA_DIR[miDir]);
    }
    else if (c == 'c') calibrarVelocidad();
    else if (c == 'i') {                                 // ¿qué sensores I2C ve el ESP32?
      if (estado != E_PARADO) { parar(); apagarMotores(); cambiar(E_PARADO); }
      escanearI2C(Wire,  "pines 21/22 (VL53L0X 0x29 + MPU6050 0x68)");
#if SENSOR_COLOR != 3200
      escanearI2C(Wire1, "pines 18/19 (sensor de color 0x29)");
#endif
      if (!tcsOk) iniciarColor();                        // por si lo reconectaste
    }
    else if (c == 'm') {
      if (estado != E_PARADO) { parar(); cambiar(E_PARADO); }
      Serial.println(F("motor IZQUIERDO adelante 1 s...")); motorIzqDirecto(PWM_CRUCERO); delay(1000); motorIzqDirecto(0); delay(300);
      Serial.println(F("motor DERECHO adelante 1 s..."));   motorDerDirecto(PWM_CRUCERO); delay(1000); motorDerDirecto(0);
      Serial.println(F("Las DOS ruedas deben ir hacia ADELANTE. Si una fue hacia atrás, intercambia sus 2 cables en el L298N."));
      Serial.println(F("Si giró primero la rueda DERECHA, los motores están cruzados: intercambia OUT1/OUT2 con OUT3/OUT4."));
    } else if (c == 'v') {
      Serial.println(F("avanzo 2 s. Mide los mm recorridos y divide entre 2: ese número va en MM_S_CRUCERO."));
      moverDosSegundos(false);
    } else if (c == 'n') {
      Serial.println(F("avanzo 2 s lento. Mide los mm y divide entre 2: ese número va en MM_S_LENTO."));
      moverDosSegundos(true);
    }
    else if (c == 'g') {
      if (estado != E_PARADO) { parar(); cambiar(E_PARADO); }
      miDir = (miDir + 1) & 3; girarA(miDir);
      Serial.println(F("girando 90° a la derecha (debe girar UNA vez y detenerse)"));
    } else if (c == 'a') {
      if (estado != E_PARADO) { parar(); cambiar(E_PARADO); }
      moverMM(CELDA_MM, false);
      Serial.printf("avanzando un cuadro (%u mm)\n", CELDA_MM);
    }
    else if (c == 'd') imprimirMapa();
    else if (c == 'l') { ponerLED(60, 0, 0); delay(400); ponerLED(0, 60, 0); delay(400); ponerLED(0, 0, 60); delay(400); ponerLED(0, 0, 0); Serial.println(F("LED: rojo, verde, azul")); }
    else if (c == 'x') {
      if (estado == E_PARADO || estado == E_OBJETIVO) {
        if (AUTO_SIGNO_GYRO && mpuOk && !signoDetectado) {   // la prueba del arranque falló: otra vez
          calibrarGyro(); detectarSignoGyro(); rumbo = rumboObjetivo = RUMBO_DIR[miDir];
        }
        cambiar(E_DECIDE); Serial.println(F("arranca"));
      }
      else { parar(); cambiar(E_PARADO); Serial.println(F("parado")); }
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
  iniciarColor();                                        // sensor de color (TCS3200 o TCS34725)
  ponerLED(30, 30, 0);

  // 100 kHz: más lento pero aguanta mejor el ruido de los motores en los cables largos.
  Wire.begin(PIN_SDA, PIN_SCL); Wire.setClock(100000);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0);   // despierta el MPU6050
  mpuOk = (Wire.endTransmission() == 0);
  if (mpuOk) Serial.println(F("MPU6050 OK"));
  else Serial.println(F("MPU6050 (giroscopio) NO responde: revisa VCC, GND, SDA (21) y SCL (22). Sin él no gira bien ni va derecho."));
  if (!iniciarTof())
    Serial.println(F("VL53L0X no responde: revisa VIN, GND, SDA (21) y SCL (22). El carro NO arranca sin él."));
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);

  Serial.printf("Carro buscador: busco el vaso %d ($%u) en un espacio de %ux%u cuadros de %u mm.\n",
                COLOR_OBJETIVO + 1, VASO_VALOR[COLOR_OBJETIVO], GRID_N, GRID_N, CELDA_MM);
  Serial.println(F("Déjalo QUIETO 1 s (calibro el giroscopio)..."));
  delay(500); calibrarGyro();
  if (AUTO_SIGNO_GYRO) detectarSignoGyro();

  for (uint8_t y = 0; y < GRID_N; y++) for (uint8_t x = 0; x < GRID_N; x++) { feromona[y][x] = 0; ocupada[y][x] = false; }
  miX = INICIO_X; miY = INICIO_Y; miDir = INICIO_DIR;
  feromona[miY][miX] = FEROMONA_DEPOSITO;
  rumbo = rumboObjetivo = RUMBO_DIR[miDir];
  vbatActual = leerVBat();
  cambiar(tofOk ? E_INICIO : E_SENSOR);
  Serial.printf("Listo en el cuadro (%u,%u) mirando %c. Arranco en 3 s. Escribe 'x' para parar.\n", miX, miY, LETRA_DIR[miDir]);
}

void loop() {
  static uint32_t tCtrl = 0, tDec = 0, tLed = 0, tBat = 0;
  uint32_t now = millis();

  leerSerial();
  actualizarTof();                                       // VL53L0X en modo continuo (no bloquea)
  // dt máximo 60 ms: si algo demoró el programa (leer el color), no "inventa" distancia recorrida.
  if (now - tCtrl >= 20) { lazoControl(min((now - tCtrl) / 1000.0f, 0.06f)); tCtrl = now; }
  if (now - tDec >= 20) { decidir(); tDec = now; }       // revisa seguido para frenar a tiempo
  if (now - tLed >= 100) { actualizarLED(); tLed = now; }

  // Batería: solo se mide con los motores quietos (al arrancar, el voltaje cae un momento).
  if (now - tBat >= 1000 && mov == MOV_NADA) {
    static bool avisoSinBateria = false;
    vbatActual = leerVBat();
    if (vbatActual < 3.0f) {
      vbatBajas = 0;
      if (!avisoSinBateria) { Serial.printf("No leo la batería (%.2f V): ¿solo USB? Sigo sin protección.\n", vbatActual); avisoSinBateria = true; }
    } else {
      avisoSinBateria = false;
      vbatBajas = vbatActual < VBAT_MIN ? vbatBajas + 1 : 0;
      if (vbatBajas >= 3 && estado != E_BATERIA && estado != E_PARADO) {
        parar(); cambiar(E_BATERIA);
        Serial.printf("batería baja (%.2f V): me detengo\n", vbatActual);
      }
    }
    tBat = now;
  }
}
