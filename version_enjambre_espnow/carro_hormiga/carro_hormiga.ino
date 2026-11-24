/*
 * CARRO HORMIGA — firmware del ESP32 de cada carro (3 carros, 5 vasos)
 * ---------------------------------------------------------------------------
 * Qué hace (versión SIN pinza: el vaso va fijo en el chasis y las monedas caen en él):
 *   1. Elige su OBJETIVO: un vaso (el más lleno y cercano que nadie más haya reclamado),
 *      o siempre el mismo si pones VASO_FIJO. El color de ese vaso es el "color objetivo".
 *   2. Viaja hasta ese vaso con el algoritmo de hormigas (aco.h), celda por celda.
 *   3. Se acerca despacio y lee el color con el TCS3200:
 *        - si ES su color objetivo: para los motores y se queda quieto para siempre;
 *        - si NO es: se devuelve y sigue buscando en otro vaso.
 *      Además, mientras viaja, si tiene algo muy cerca al frente revisa el color: si es su
 *      color objetivo también se detiene para siempre; si no, sigue moviéndose.
 *   4. En cada llegada deposita feromona en su ruta y se la manda a los otros carros por ESP-NOW.
 *   5. Si encuentra un obstáculo que no está en el plano, lo ve con el VL53L0X, bloquea ese
 *      paso y les avisa a los demás.
 *
 * Hardware: ESP32 DevKit V1 · L298N + 2 motores (o TB6612 + N20 con encoder) · VL53L0X
 *   TCS3200 · MPU6050 · LED WS2812 · divisor de batería · buck MP1584 (sin código). SIN servos.
 *   Todas las conexiones están en CONEXIONES_Y_PRUEBAS.md.
 *
 * Librería (Gestor de librerías del Arduino IDE): "Adafruit_VL53L0X". Ya no hace falta ESP32Servo.
 * Placa: "ESP32 Dev Module". Probado que compila con el core ESP32 3.3.x.
 *
 * ANTES DE SUBIR: en config.h cambia CARRO_ID (0, 1 o 2) y revisa las celdas de la meta y los vasos.
 * ESTADO: compila, pero NO se ha probado en un carro real. Lo marcado con [CALIBRAR] se mide en el carro.
 *
 * Comandos por el monitor serie (115200). Escribe 'x' apenas arranque para que no salga solo:
 *   x  parar / arrancar          t  leer sensores           m  probar motores (y encoders)
 *   v  avanzar 500 mm (calibrar distancia)                  a  avanzar una celda
 *   g  girar 90° a la derecha    d  diagnóstico (estado, ruta aprendida, otros carros)
 *   l  probar el LED (rojo, verde, azul)
 *   w  balance de blancos (tarjeta blanca frente al TCS3200)
 *   0..4  guardar el color del vaso 1..5 (ponlo frente al sensor)     p  imprimir colores guardados
 *   n  (solo sin encoder) avanzar 2 s a velocidad lenta para medirla
 */
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Adafruit_VL53L0X.h>

#include "config.h"
#include "aco.h"
#if MODO_SIMULACION
#include "simulacion.h"
#endif

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif
#if ESP_ARDUINO_VERSION_MAJOR >= 3
typedef const esp_now_recv_info_t* NowOrigen;   // core 3.x
#else
typedef const uint8_t* NowOrigen;                // core 2.x
#endif

// ============================================================================
// 0. ¿QUÉ MOTORES TIENE ESTE CARRO?
// ============================================================================
// 1 = TB6612FNG + motorreductores N20 CON encoder (la lista de materiales). Recomendado.
// 0 = driver L298N + motores SIN encoder: el avance se calcula por tiempo.
#ifndef CON_ENCODER
#define CON_ENCODER 0          // L298N + motores sin encoder (lo que se montó)
#endif

// ============================================================================
// 1. PINES (lámina 5b del repositorio)
// ============================================================================
// Motores. Con TB6612: PWMA/AIN1/AIN2 y PWMB/BIN1/BIN2. Con L298N: ENA/IN1/IN2 y ENB/IN3/IN4.
static const uint8_t PIN_VEL_I = 25, PIN_DIR_I1 = 26, PIN_DIR_I2 = 27;   // motor izquierdo
static const uint8_t PIN_VEL_D = 14, PIN_DIR_D1 = 32, PIN_DIR_D2 = 4;    // motor derecho
#if CON_ENCODER
static const uint8_t PIN_STBY = 13;                                      // TB6612: alto = encendido
static const uint8_t PIN_ENC_I_A = 34, PIN_ENC_I_B = 35, PIN_ENC_D_A = 36, PIN_ENC_D_B = 39;
#endif
static const uint8_t PIN_TCS_S2 = 18, PIN_TCS_S3 = 19, PIN_TCS_OUT = 23; // S0 a 3V3, S1 a GND
static const uint8_t PIN_SDA = 21, PIN_SCL = 22;                         // VL53L0X 0x29 + MPU6050 0x68
static const uint8_t PIN_LED = 5, PIN_VBAT = 33;                         // GPIO16 y 17 (servos) quedan libres

// ============================================================================
// 2. MOVIMIENTO [CALIBRAR]
// ============================================================================
#if CON_ENCODER
// Los encoders miden cuánto gira cada rueda: distancia exacta y control de velocidad (PI).
static const float RUEDA_D_MM = 42.0f;                  // ruedas Ø42 × 15 mm
static const float TROCHA_MM = 110.0f;                  // distancia entre ruedas
static const float CUENTAS_POR_VUELTA = 1400.0f;        // 7 pulsos x 100:1 x 2 flancos de A. Ajustar a tu N20
static const float MM_POR_CUENTA = (PI * RUEDA_D_MM) / CUENTAS_POR_VUELTA;
static const int8_t SIGNO_ENC_I = +1, SIGNO_ENC_D = -1; // si las cuentas salen negativas en 'm', cámbialo
static const float V_MAX_MM_S = 250.0f;                 // velocidad máxima (para la ganancia directa)
static const float V_CRUCERO = 160.0f;                  // mm/s entre celdas
static const float V_LENTA = 70.0f;                     // mm/s para acercarse al vaso y entregar
static const float PWM_MAX_FRAC = 0.80f;                // N20 de 6 V con batería de 7,4 V: máximo 80 %
static const uint32_t PWM_FREQ = 20000;
// Choque: si manda avanzar y las ruedas van a menos de este % de lo pedido durante T_CHOQUE_MS,
// hay algo adelante. Así detecta obstáculos aunque el vaso tape el VL53L0X.
static const float CHOQUE_FRAC = 0.25f;
static const uint32_t T_CHOQUE_MS = 500;
#else
// Sin encoder: distancia = velocidad real (mm/s) × tiempo. Mide la velocidad con 'v' y 'n'.
// El L298N se come ~2 V: con 7,4 V a 255 al motor le llegan ~5 V. Por debajo de ~180 los N20 solo zumban.
static const int   PWM_CRUCERO = 230;                   // 0..255
static const int   PWM_LENTO   = 190;
static const float MM_S_CRUCERO = 150.0f;               // [CALIBRAR] con el comando 'v'
static const float MM_S_LENTO   = 70.0f;                // [CALIBRAR] con el comando 'n'
static const int   PWM_GIRO_MIN = 185, PWM_GIRO_MAX = 240;
static const float KP_GIRO  = 3.0f;
static const int   AJUSTE_IZQ = 0, AJUSTE_DER = 0;      // si un motor es más rápido, réstale aquí (p. ej. -10)
static const float VBAT_REFERENCIA = 7.3f;             // batería con la que mediste las velocidades
static const float V_CRUCERO = MM_S_CRUCERO;
static const float V_LENTA = MM_S_LENTO;
static const uint32_t PWM_FREQ = 1000;                  // el L298N es lento: 1 kHz
#endif
static const float KP_RECTO = 4.0f;                     // corrección por grado al ir derecho (solo sin encoder)

static const uint16_t UMBRAL_OBSTACULO_MM = 170;        // algo más cerca que esto = la celda de enfrente está ocupada
static const uint16_t DIST_EMERGENCIA_MM = 80;          // en movimiento: frenar ya
static const uint32_t T_PAUSA_MAX_MS = 3000;            // frenado más de esto: devolverse a la celda
static const uint16_t DIST_REVISAR_MM = 45;             // junto al vaso: se acerca hasta esta distancia y lee el color
static const float MAX_ACERCA_MM = 120.0f;              // máximo que avanza buscando el vaso

// ---- COLOR OBJETIVO ----
// -1 = cada carro elige su vaso (los 3 carros se reparten vasos distintos).
// 0..4 = este carro SIEMPRE busca el color del vaso 1..5 (p. ej. 1 = el color guardado con '1').
static const int8_t VASO_FIJO = -1;
// En el camino: si algo está a menos de esto al frente, revisa si es su color objetivo.
static const uint16_t DIST_COLOR_MM = 100;
// Qué tan parecido debe ser el color (diferencia con el guardado). Más pequeño = más exigente.
// El comando 't' muestra la diferencia ("dif"). [CALIBRAR]
static const float UMBRAL_COLOR = 0.04f;

static const float VBAT_MIN = 6.4f;
static const float DIVISOR_VBAT = (100.0f + 47.0f) / 47.0f;
// Corrección del ADC: (voltaje del multímetro) / (voltaje que muestra 't'). [CALIBRAR]
// Sin condensador en GPIO33 se midió 7,2 V real contra 5,27 V leído = 1,36.
// Si pones el condensador de 100 nF entre GPIO33 y GND, vuelve a medir: probablemente quede en ~1,0.
static const float VBAT_CAL = 1.36f;

// Colores de los 5 vasos: cromaticidad (r,g,b)/(r+g+b). Valores de EJEMPLO:
// [CALIBRAR] con los comandos '0'..'4' y pega aquí lo que imprime 'p'.
static float CENT[NUM_VASOS][3] = {
  {0.461f, 0.261f, 0.278f},   // vaso 1 ($50)
  {0.336f, 0.334f, 0.331f},   // vaso 2 ($100)
  {0.276f, 0.338f, 0.386f},   // vaso 3 ($200)
  {0.356f, 0.322f, 0.322f},   // vaso 4 ($500)
  {0.430f, 0.320f, 0.251f},   // vaso 5 ($1000)
};

// ============================================================================
// 3. TIPOS (van antes de cualquier función por cómo Arduino arma los prototipos)
// ============================================================================
enum Estado : uint8_t { E_INICIO, E_ELIGE_VASO, E_NAVEGA, E_REVISA, E_OBJETIVO, E_PARADO, E_BATERIA };
static const char* NOMBRE_ESTADO[] = {"inicio", "elige_vaso", "navega", "revisa", "objetivo", "parado", "bateria"};
enum Nav : uint8_t { NAV_DECIDIR, NAV_GIRANDO, NAV_MIRAR, NAV_AVANZANDO };
enum Mov : uint8_t { MOV_NADA, MOV_GIRO, MOV_RECTO };

// Mensajes ESP-NOW (todos broadcast)
enum : uint8_t { MSG_FEROMONA = 1, MSG_BLOQUEO = 2, MSG_ESTADO = 3, MSG_RECOGIDO = 4, MSG_VASO_LISTO = 10 };
struct __attribute__((packed)) MsgFeromona { uint8_t tipo, carro, capa, n; uint8_t nodos[MAX_CAMINO]; };
struct __attribute__((packed)) MsgBloqueo  { uint8_t tipo, carro, nodo, dir, ticks; };
struct __attribute__((packed)) MsgEstado   { uint8_t tipo, carro, nodo, siguiente, vaso, estado; };
struct __attribute__((packed)) MsgRecogido { uint8_t tipo, carro, vaso; };
struct __attribute__((packed)) MsgVasoListo { uint8_t tipo, vaso, monedas; };   // lo manda la estación (opcional)

struct Entrante { uint8_t buf[250]; uint8_t len; };
struct Par { uint8_t nodo, siguiente, vaso, estado; uint32_t t; };

// ============================================================================
// 4. ESTADO GLOBAL
// ============================================================================
static ColoniaACO colonia;                 // mi copia del mapa de feromona (~25 KB)

static Estado estado = E_INICIO;
static uint32_t tEstado = 0;
static uint8_t paso = 0;                   // sub-paso dentro de REVISA
static uint32_t tPaso = 0;
static Nav nav = NAV_DECIDIR;

static uint8_t miNodo = 0;                 // celda donde estoy
static uint8_t miDir = SALIDA_DIR;         // hacia dónde miro (0 N, 1 E, 2 S, 3 O)
static uint8_t miVaso = NODO_NINGUNO;      // vaso (posición) al que voy
static int8_t colorBuscado = -1;           // color objetivo: índice del vaso cuyo color busco (-1 = ninguno)
static uint8_t dirPend = 0, sigPend = NODO_NINGUNO;
static bool retroPend = false;
static bool esperando = false;
static uint32_t tEsperaIni = 0, tMirar = 0;
static float distAcercada = 0;

// vasos
static uint16_t monedas[NUM_VASOS] = {1, 1, 1, 1, 1};
static uint32_t disponibleDesde[NUM_VASOS] = {0, 0, 0, 0, 0};
static uint32_t vetoHasta[NUM_VASOS] = {0, 0, 0, 0, 0};
static uint16_t entregas = 0;

// movimiento
static Mov mov = MOV_NADA;
static float rumbo = 0;                    // grados, del giroscopio (0 = Este, +90 = Sur)
static float rumboObjetivo = 0;
static float distObjetivo = 0, distHecha = 0, velMov = 0;
static bool ignorarFrente = false, movAbortado = false, pausado = false, choque = false;
static uint32_t tGiroOk = 0, tPausa = 0, tMovIni = 0, tLento = 0;
static float v_cmd = 0, w_cmd = 0;
static float gz_bias = 0;
static float vbatActual = 0;
#if CON_ENCODER
static float iI = 0, iD = 0, velI = 0, velD = 0;
static volatile long encI = 0, encD = 0;
static long encIprev = 0, encDprev = 0;
#endif

// sensores
static Adafruit_VL53L0X lox;
static bool tofOk = false;
static uint16_t tofMm = 8190;
static float gainRGB[3] = {0.991,1.067,0.949};
static uint8_t vbatBajas = 0;

// radio
static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static Entrante cola[8];
static volatile uint8_t colaIn = 0, colaOut = 0;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static Par pares[NUM_CARROS];

#if MODO_SIMULACION
static SimulacionEnjambre* sim = nullptr;
#endif

// ============================================================================
// 5. UTILIDADES
// ============================================================================
static inline float envolver180(float a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static float azar() { return (esp_random() >> 8) / 16777216.0f; }

static void cambiar(Estado e) { estado = e; tEstado = millis(); paso = 0; tPaso = millis(); nav = NAV_DECIDIR; }

static void ponerLED(uint8_t r, uint8_t g, uint8_t b) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  rgbLedWrite(PIN_LED, r, g, b);
#else
  neopixelWrite(PIN_LED, r, g, b);
#endif
}

// ============================================================================
// 6. MOTORES, ENCODERS Y GIROSCOPIO
// ============================================================================
// Canales PWM FIJOS 4 y 5 para los motores (así no chocan con nada más que use PWM).
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

#if CON_ENCODER
static void IRAM_ATTR isrEncI() { encI = encI + ((digitalRead(PIN_ENC_I_A) == digitalRead(PIN_ENC_I_B)) ? 1 : -1); }
static void IRAM_ATTR isrEncD() { encD = encD + ((digitalRead(PIN_ENC_D_A) == digitalRead(PIN_ENC_D_B)) ? 1 : -1); }
static const int AJUSTE_IZQ = 0, AJUSTE_DER = 0;         // con encoder el PI ya iguala los motores
#endif

// duty en [-255, 255]: positivo = adelante
static void motorIzq(int duty) {
  if (duty != 0) duty += (duty > 0 ? AJUSTE_IZQ : -AJUSTE_IZQ);
  duty = constrain(duty, -255, 255);
  digitalWrite(PIN_DIR_I1, duty > 0); digitalWrite(PIN_DIR_I2, duty < 0);
  pwmEscribir(PIN_VEL_I, CH_A, abs(duty));
}
static void motorDer(int duty) {
  if (duty != 0) duty += (duty > 0 ? AJUSTE_DER : -AJUSTE_DER);
  duty = constrain(duty, -255, 255);
  digitalWrite(PIN_DIR_D1, duty > 0); digitalWrite(PIN_DIR_D2, duty < 0);
  pwmEscribir(PIN_VEL_D, CH_B, abs(duty));
}

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
  gz_bias = s / N;
}

// Distancia libre al frente (mm) según el VL53L0X.
static uint16_t frenteMM() {
  if (tofOk && tofMm < 2000) return tofMm;
  return 9999;
}

// ---- órdenes de movimiento (no bloquean: el lazo de control las ejecuta) ----
static void parar() { v_cmd = 0; w_cmd = 0; mov = MOV_NADA; }
static void girarA(uint8_t dir) { rumboObjetivo = DIR_RUMBO_GRADOS[dir]; mov = MOV_GIRO; tGiroOk = 0; }
// mm > 0 adelante, mm < 0 atrás. Mantiene el rumbo actual con el giroscopio.
static void moverMM(float mm, float vel, bool sinFrente) {
  distObjetivo = mm; distHecha = 0; velMov = vel; ignorarFrente = sinFrente;
  movAbortado = false; choque = false; pausado = false; tMovIni = millis(); tLento = 0; mov = MOV_RECTO;
}
// Devolverse a la celda de donde salió (algo bloquea el paso)
static void abortarMovimiento(bool porChoque) {
  movAbortado = true; choque = porChoque; pausado = false; tLento = 0;
  distObjetivo = -distHecha; distHecha = 0;
}

// Lógica común de la maniobra: decide v_cmd (mm/s, signo = sentido) y w_cmd (rad/s, + = horario).
// Devuelve true si el carro debe estar quieto en este ciclo.
static bool maniobra(float vMedida) {
  v_cmd = 0; w_cmd = 0;
  if (mov == MOV_GIRO) {
    float err = envolver180(rumboObjetivo - rumbo);
    if (fabsf(err) < 3.0f) {
      if (!tGiroOk) tGiroOk = millis();
      else if (millis() - tGiroOk > 150) { mov = MOV_NADA; rumbo = rumboObjetivo; }   // se "ajusta" a la cuadrícula
      return true;
    }
    tGiroOk = 0;
    w_cmd = clampf(0.06f * err, -2.5f, 2.5f);            // [CALIBRAR]
    if (fabsf(w_cmd) < 0.6f) w_cmd = w_cmd > 0 ? 0.6f : -0.6f;
    return false;
  }
  if (mov != MOV_RECTO) return true;

  float falta = fabsf(distObjetivo) - fabsf(distHecha);
  float signo = distObjetivo >= 0 ? 1.0f : -1.0f;
  if (falta <= 0) { parar(); return true; }

  if (signo > 0 && !ignorarFrente && frenteMM() < DIST_EMERGENCIA_MM) {   // algo se atravesó
    if (!pausado) { pausado = true; tPausa = millis(); }
    if (millis() - tPausa > T_PAUSA_MAX_MS) abortarMovimiento(false);
    return true;
  }
  if (pausado) { pausado = false; tMovIni = millis(); }

#if CON_ENCODER
  // Choque: va hacia adelante pero las ruedas casi no giran (sirve aunque el vaso tape el VL53L0X)
  if (signo > 0 && !ignorarFrente && millis() - tMovIni > 400 && vMedida < CHOQUE_FRAC * velMov) {
    if (!tLento) tLento = millis();
    else if (millis() - tLento > T_CHOQUE_MS) { abortarMovimiento(true); return true; }
  } else tLento = 0;
#else
  (void)vMedida;
#endif

  v_cmd = signo * velMov * (falta < 30 ? 0.6f : 1.0f);
  w_cmd = clampf(0.04f * envolver180(rumboObjetivo - rumbo), -1.0f, 1.0f);   // mantener la línea recta [CALIBRAR]
  return false;
}

#if CON_ENCODER
// Cada 20 ms: rumbo (giroscopio), distancia y velocidad (encoders), maniobra y PI por rueda.
static void lazoControl(float dt) {
  noInterrupts(); long ei = encI, ed = encD; interrupts();
  float dI = SIGNO_ENC_I * (ei - encIprev) * MM_POR_CUENTA;
  float dD = SIGNO_ENC_D * (ed - encDprev) * MM_POR_CUENTA;
  encIprev = ei; encDprev = ed;
  velI = dI / dt; velD = dD / dt;
  distHecha += 0.5f * (dI + dD);
  rumbo = envolver180(rumbo - (leerGyroZ() - gz_bias) * dt);   // + horario visto desde arriba

  maniobra(0.5f * (velI + velD));

  float vObjI = v_cmd + w_cmd * TROCHA_MM * 0.5f;      // w > 0 (horario) => rueda izquierda más rápida
  float vObjD = v_cmd - w_cmd * TROCHA_MM * 0.5f;
  if (estado == E_BATERIA || estado == E_OBJETIVO) { vObjI = vObjD = 0; }
  const float KP = 0.55f, KI = 1.8f, KFF = 255.0f / V_MAX_MM_S;   // [CALIBRAR]
  float eI = vObjI - velI, eD = vObjD - velD;
  iI = clampf(iI + eI * dt, -80, 80); iD = clampf(iD + eD * dt, -80, 80);
  float uI = KFF * vObjI + KP * eI + KI * iI;
  float uD = KFF * vObjD + KP * eD + KI * iD;
  if (fabsf(vObjI) < 1) { uI = 0; iI = 0; }
  if (fabsf(vObjD) < 1) { uD = 0; iD = 0; }
  const float tope = 255.0f * PWM_MAX_FRAC;
  motorIzq((int)clampf(uI, -tope, tope));
  motorDer((int)clampf(uD, -tope, tope));
}
#else
// Cada 20 ms: rumbo (giroscopio), distancia ESTIMADA por tiempo y PWM de cada motor (L298N).
static void lazoControl(float dt) {
  rumbo = envolver180(rumbo - (leerGyroZ() - gz_bias) * dt);
  bool quieto = maniobra(0);
  float compensa = (vbatActual > 5.0f) ? clampf(VBAT_REFERENCIA / vbatActual, 0.85f, 1.30f) : 1.0f;
  float uI = 0, uD = 0;
  if (!quieto && mov == MOV_GIRO) {
    float err = envolver180(rumboObjetivo - rumbo);
    float p = clampf(KP_GIRO * fabsf(err), (float)PWM_GIRO_MIN, (float)PWM_GIRO_MAX) * compensa;
    float s = err > 0 ? 1.0f : -1.0f;
    uI = s * p; uD = -s * p;                             // giro en el sitio
  } else if (!quieto && mov == MOV_RECTO) {
    bool lento = fabsf(v_cmd) < V_CRUCERO * 0.9f;
    float signo = v_cmd >= 0 ? 1.0f : -1.0f;
    float base = (lento ? PWM_LENTO : PWM_CRUCERO) * compensa;
    float corr = clampf(KP_RECTO * envolver180(rumboObjetivo - rumbo), -60.0f, 60.0f);
    uI = signo * base + corr; uD = signo * base - corr;
    distHecha += signo * (lento ? MM_S_LENTO : MM_S_CRUCERO) * dt;   // SIN ENCODER: velocidad × tiempo
  }
  if (estado == E_BATERIA || estado == E_OBJETIVO) { uI = uD = 0; }   // en el objetivo: quieto para siempre
  motorIzq((int)uI);
  motorDer((int)uD);
}
#endif

// ============================================================================
// 7. COLOR (TCS3200)
// ============================================================================
static float leerCanal(bool s2, bool s3) {
  digitalWrite(PIN_TCS_S2, s2); digitalWrite(PIN_TCS_S3, s3);
  delayMicroseconds(400);
  unsigned long suma = 0; int n = 0;
  for (int i = 0; i < 6; i++) { unsigned long t = pulseIn(PIN_TCS_OUT, LOW, 20000); if (t) { suma += t; n++; } }
  if (!n) return 0;
  return 1.0e6f / (2.0f * (float)suma / n);             // Hz (proporcional a la luz)
}
static bool leerCromaticidad(float out[3]) {
  float r = leerCanal(LOW, LOW) * gainRGB[0];
  float b = leerCanal(LOW, HIGH) * gainRGB[2];
  float g = leerCanal(HIGH, HIGH) * gainRGB[1];
  float s = r + g + b;
  if (s <= 0) return false;
  out[0] = r / s; out[1] = g / s; out[2] = b / s;
  return true;
}
// Color más parecido (índice de vaso) a una lectura promedio de 5.
// En 'dif' deja qué tan distinto es del color guardado (0 = idéntico).
static int clasificarColor(float& dif) {
  float acc[3] = {0, 0, 0}, v[3]; int n = 0;
  dif = 1.0f;
  for (int i = 0; i < 5; i++) if (leerCromaticidad(v)) { for (int k = 0; k < 3; k++) acc[k] += v[k]; n++; }
  if (!n) return -1;
  int mejor = -1; float dMejor = 1e9f;
  for (int k = 0; k < NUM_VASOS; k++) {
    float d0 = acc[0] / n - CENT[k][0], d1 = acc[1] / n - CENT[k][1], d2 = acc[2] / n - CENT[k][2];
    float d = d0 * d0 + d1 * d1 + d2 * d2;
    if (d < dMejor) { dMejor = d; mejor = k; }
  }
  dif = sqrtf(dMejor);
  return mejor;
}
// ¿Lo que tengo enfrente es mi color objetivo?
static bool veoMiColor() {
  if (colorBuscado < 0) return false;
  float dif; int c = clasificarColor(dif);
  return c == colorBuscado && dif <= UMBRAL_COLOR;
}
static void balanceBlancos() {
  gainRGB[0] = gainRGB[1] = gainRGB[2] = 1;
  float r = leerCanal(LOW, LOW), b = leerCanal(LOW, HIGH), g = leerCanal(HIGH, HIGH);
  if (r > 0 && g > 0 && b > 0) { float m = (r + g + b) / 3; gainRGB[0] = m / r; gainRGB[1] = m / g; gainRGB[2] = m / b; }
}
static float leerVBat() {
  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(PIN_VBAT);   // promedio de 8 lecturas
  return mv / 8.0f / 1000.0f * DIVISOR_VBAT * VBAT_CAL;
}

// ============================================================================
// 8. RADIO (ESP-NOW): feromona, bloqueos, posición y vasos
// ============================================================================
static void alRecibir(NowOrigen, const uint8_t* data, int len) {   // corre en la tarea de WiFi: solo encolar
  if (len <= 0 || len > (int)sizeof(cola[0].buf)) return;
  portENTER_CRITICAL(&mux);
  uint8_t sig = (colaIn + 1) & 7;
  if (sig != colaOut) { memcpy(cola[colaIn].buf, data, len); cola[colaIn].len = len; colaIn = sig; }
  portEXIT_CRITICAL(&mux);
}

static void enviar(const void* m, size_t len) { esp_now_send(BCAST, (const uint8_t*)m, len); }

static void enviarFeromona(uint8_t capa, const uint8_t* nodos, uint16_t n) {
  MsgFeromona m;
  m.tipo = MSG_FEROMONA; m.carro = CARRO_ID; m.capa = capa; m.n = (uint8_t)n;
  memcpy(m.nodos, nodos, n);
  enviar(&m, 4 + n);
}
static void enviarBloqueo(uint8_t nodo, uint8_t dir) {
  MsgBloqueo m = {MSG_BLOQUEO, CARRO_ID, nodo, dir, BLOQUEO_SENSOR_TICKS};
  enviar(&m, sizeof(m));
}
static void enviarEstado() {
  bool moviendo = (estado == E_NAVEGA && nav != NAV_DECIDIR);
  // El vaso sigue "reclamado" aunque ya haya llegado: así los otros carros buscan otro.
  MsgEstado m = {MSG_ESTADO, CARRO_ID, miNodo, moviendo ? sigPend : NODO_NINGUNO, miVaso, (uint8_t)estado};
  enviar(&m, sizeof(m));
}

static void procesarMensajes() {
  while (colaOut != colaIn) {
    Entrante e;
    portENTER_CRITICAL(&mux);
    e = cola[colaOut]; colaOut = (colaOut + 1) & 7;
    portEXIT_CRITICAL(&mux);
    const uint8_t tipo = e.buf[0];
    if (tipo == MSG_FEROMONA && e.len >= 4) {
      MsgFeromona* m = (MsgFeromona*)e.buf;
      if (m->carro != CARRO_ID && m->n <= MAX_CAMINO && e.len >= 4 + m->n)
        colonia.depositarViaje(m->capa, m->nodos, m->n);          // la feromona de otra hormiga
    } else if (tipo == MSG_BLOQUEO && e.len >= (int)sizeof(MsgBloqueo)) {
      MsgBloqueo* m = (MsgBloqueo*)e.buf;
      if (m->carro != CARRO_ID && m->nodo < NUM_NODOS && m->dir < 4) colonia.bloquearArista(m->nodo, m->dir, m->ticks);
    } else if (tipo == MSG_ESTADO && e.len >= (int)sizeof(MsgEstado)) {
      MsgEstado* m = (MsgEstado*)e.buf;
      if (m->carro < NUM_CARROS && m->carro != CARRO_ID)
        pares[m->carro] = Par{m->nodo, m->siguiente, m->vaso, m->estado, millis()};
    } else if (tipo == MSG_RECOGIDO && e.len >= (int)sizeof(MsgRecogido)) {
      MsgRecogido* m = (MsgRecogido*)e.buf;
      if (m->vaso < NUM_VASOS) { disponibleDesde[m->vaso] = millis() + T_REPOSICION_MS; monedas[m->vaso] = 1; }
    } else if (tipo == MSG_VASO_LISTO && e.len >= (int)sizeof(MsgVasoListo)) {
      MsgVasoListo* m = (MsgVasoListo*)e.buf;
      if (m->vaso < NUM_VASOS) { disponibleDesde[m->vaso] = 0; monedas[m->vaso] = m->monedas; }
    }
  }
}

static bool parVigente(uint8_t i) { return i != CARRO_ID && pares[i].t && millis() - pares[i].t < VIGENCIA_PAR_MS; }

// Nodos donde están o a donde van los otros carros
static uint8_t ocupados(uint8_t* out) {
  uint8_t n = 0;
  for (uint8_t i = 0; i < NUM_CARROS; i++) {
    if (!parVigente(i)) continue;
    out[n++] = pares[i].nodo;
    if (pares[i].siguiente != NODO_NINGUNO) out[n++] = pares[i].siguiente;
  }
  return n;
}
static bool hayParEn(uint8_t nodo) {
  uint8_t o[2 * NUM_CARROS]; uint8_t n = ocupados(o);
  for (uint8_t i = 0; i < n; i++) if (o[i] == nodo) return true;
  return false;
}
// ¿Otro carro con prioridad (ID menor) reclamó mi mismo vaso?
static bool meQuitaronElVaso() {
  if (miVaso == NODO_NINGUNO || estado == E_OBJETIVO) return false;
  for (uint8_t i = 0; i < CARRO_ID; i++) if (parVigente(i) && pares[i].vaso == miVaso) return true;
  return false;
}

// ============================================================================
// 9. MÁQUINA DE ESTADOS (cada 50 ms)
// ============================================================================
static void alLlegar() {
  uint8_t capa = colonia.capa;
  colonia.depositarViaje(capa, colonia.camino, colonia.largo);   // deposito en MI mapa...
  enviarFeromona(capa, colonia.camino, colonia.largo);           // ...y en el de los demás
  Serial.printf("[carro %u] llegó a %s en %u pasos (ruta de %u)\n", CARRO_ID,
                capa == CAPA_META ? "la META" : "el vaso", colonia.pasosTotales, colonia.largo - 1);
  cambiar(E_REVISA);                                             // llegué al vaso: revisar su color
}

// Vi mi color objetivo: motores quietos para siempre (hasta reiniciar o escribir 'x' dos veces).
static void llegueAlObjetivo(const char* donde) {
  parar(); motorIzq(0); motorDer(0);
  cambiar(E_OBJETIVO);
  Serial.printf("[carro %u] veo mi color objetivo (vaso %d, $%u) %s. Me quedo quieto.\n",
                CARRO_ID, colorBuscado + 1, VASO_VALOR[colorBuscado], donde);
}

static void avisarObstaculo(uint8_t nodo, uint8_t dir, const char* como) {
  colonia.bloquearArista(nodo, dir, BLOQUEO_SENSOR_TICKS);
  enviarBloqueo(nodo, dir);
  Serial.printf("obstáculo (%s) en (%d,%d)->%c, aviso a los demás\n", como, nodoX(nodo), nodoY(nodo), DIR_LETRA[dir]);
}

static void pasoNavegacion() {
  switch (nav) {
    case NAV_DECIDIR: {
      if (meQuitaronElVaso()) { Serial.println(F("otro carro tomó mi vaso: elijo otro")); miVaso = NODO_NINGUNO; cambiar(E_ELIGE_VASO); return; }
      uint8_t ocup[2 * NUM_CARROS]; uint8_t nOcup = ocupados(ocup);
      bool retro;
      int8_t d = colonia.elegir(azar(), azar(), ocup, nOcup, retro);
      if (d == ELEGIR_LLEGO) { parar(); alLlegar(); return; }
      if (d == ELEGIR_SIN_SALIDA) { colonia.iniciarViaje(miNodo, colonia.capa); return; }
      if (d == ELEGIR_ESPERAR) {
        if (!esperando) { esperando = true; tEsperaIni = millis(); }
        else if (millis() - tEsperaIni > T_ESPERA_MAX_MS) { colonia.iniciarViaje(miNodo, colonia.capa); esperando = false; }
        return;
      }
      esperando = false;
      dirPend = (uint8_t)d; retroPend = retro; sigPend = vecino(miNodo, dirPend);
      if (dirPend != miDir) { girarA(dirPend); nav = NAV_GIRANDO; }
      else { nav = NAV_MIRAR; tMirar = 0; }
      return;
    }
    case NAV_GIRANDO:
      if (mov == MOV_NADA) { miDir = dirPend; nav = NAV_MIRAR; tMirar = millis(); }
      return;
    case NAV_MIRAR:
      if (tMirar && millis() - tMirar < 150) return;     // esperar una lectura fresca tras girar
      if (hayParEn(sigPend)) { nav = NAV_DECIDIR; return; }
      if (frenteMM() < UMBRAL_OBSTACULO_MM) {             // obstáculo que no está en el mapa
        avisarObstaculo(miNodo, dirPend, "VL53L0X");
        nav = NAV_DECIDIR;
        return;
      }
      moverMM(CELDA_MM, V_CRUCERO, false);
      nav = NAV_AVANZANDO;
      return;
    case NAV_AVANZANDO:
      if (mov != MOV_NADA) return;
      if (movAbortado) {                                 // volví a mi celda
        movAbortado = false;
        if (choque) avisarObstaculo(miNodo, dirPend, "choque");          // las ruedas se frenaron: hay algo
        else colonia.bloquearArista(miNodo, dirPend, 2);                 // algo se atravesó: bloqueo corto y local
        choque = false;
        nav = NAV_DECIDIR;
        return;
      }
      miNodo = sigPend;
      colonia.avanzar(dirPend, retroPend);
      nav = NAV_DECIDIR;
      return;
  }
}

static void decidir() {
  uint32_t enPaso = millis() - tPaso;
  switch (estado) {
    case E_INICIO:
      if (millis() - tEstado > 3000) cambiar(E_ELIGE_VASO);
      break;

    case E_ELIGE_VASO: {
      bool disp[NUM_VASOS]; uint8_t recl[NUM_VASOS];
      for (uint8_t v = 0; v < NUM_VASOS; v++) {
        disp[v] = (int32_t)(millis() - disponibleDesde[v]) >= 0 && (int32_t)(millis() - vetoHasta[v]) >= 0;
        recl[v] = NODO_NINGUNO;
      }
      for (uint8_t i = 0; i < NUM_CARROS; i++)
        if (parVigente(i) && pares[i].vaso < NUM_VASOS) recl[pares[i].vaso] = i;
      int v;
      if (VASO_FIJO >= 0 && VASO_FIJO < NUM_VASOS) {
        // Color fijo: primero va a donde debería estar ese vaso; si ahí no estaba, revisa los otros.
        colorBuscado = VASO_FIJO;
        v = disp[VASO_FIJO] ? VASO_FIJO : elegirVaso(colonia, miNodo, monedas, disp, recl, CARRO_ID);
      } else {
        v = elegirVaso(colonia, miNodo, monedas, disp, recl, CARRO_ID);
        colorBuscado = (int8_t)v;
      }
      if (v < 0) { parar(); break; }                     // no hay vaso libre: esperar aquí
      miVaso = (uint8_t)v;
      colonia.iniciarViaje(miNodo, capaVaso(miVaso));
      Serial.printf("[carro %u] busco el color del vaso %d ($%u); voy a revisar la posición del vaso %u\n",
                    CARRO_ID, colorBuscado + 1, VASO_VALOR[colorBuscado], miVaso + 1);
      cambiar(E_NAVEGA);
      break;
    }

    case E_NAVEGA:
      pasoNavegacion();
      break;

    case E_REVISA:                                       // estoy en la celda del vaso: ¿es mi color?
      switch (paso) {
        case 0: girarA(VASO_DIR[miVaso]); paso = 1; break;
        case 1: if (mov == MOV_NADA) { miDir = VASO_DIR[miVaso]; paso = 2; tPaso = millis(); } break;
        case 2: if (enPaso > 300) { moverMM(MAX_ACERCA_MM, V_LENTA, true); paso = 3; } break;
        case 3:                                          // acercarse despacio hasta tener el vaso cerca
          if ((tofOk && tofMm <= DIST_REVISAR_MM) || mov == MOV_NADA) {
            distAcercada = distHecha; parar(); paso = 4; tPaso = millis();
          }
          break;
        case 4: {
          if (enPaso < 300) break;
          float dif; int color = clasificarColor(dif);
          if (color == colorBuscado && dif <= UMBRAL_COLOR) {
            entregas++;
            llegueAlObjetivo("junto al vaso");
          } else {
            Serial.printf("aquí veo el color del vaso %d (dif %.3f), busco el %d: sigo buscando\n",
                          color + 1, dif, colorBuscado + 1);
            vetoHasta[miVaso] = millis() + T_VETO_VASO_MS;   // no vuelvo a esta posición por un rato
            moverMM(-distAcercada, V_LENTA, true); paso = 5;
          }
          break;
        }
        case 5: if (mov == MOV_NADA) { miVaso = NODO_NINGUNO; cambiar(E_ELIGE_VASO); } break;
      }
      break;

    case E_OBJETIVO:                                     // encontró su color: quieto para siempre
      parar();
      break;

    case E_PARADO:                                       // quieto, pero acepta pruebas por serial ('a', 'g', 'v'...)
      break;
    case E_BATERIA:
      parar();
      break;
  }
}

static void actualizarLED() {
  switch (estado) {
    case E_INICIO:     ponerLED(30, 30, 0); break;
    case E_ELIGE_VASO: ponerLED(0, 0, 40); break;
    case E_NAVEGA:     ponerLED(0, 20, 50); break;
    case E_REVISA:     ponerLED(40, 0, 40); break;
    case E_OBJETIVO:   ponerLED(0, 60, 0); break;        // verde fijo: encontró su color
    case E_PARADO:     ponerLED(20, 20, 20); break;
    case E_BATERIA:    ponerLED((millis() / 300) % 2 ? 60 : 0, 0, 0); break;
  }
}

// ============================================================================
// 10. SERIAL (calibración y diagnóstico)
// ============================================================================
static void imprimirColores() {
  Serial.println(F("static float CENT[NUM_VASOS][3] = {"));
  for (int k = 0; k < NUM_VASOS; k++)
    Serial.printf("  {%.3ff, %.3ff, %.3ff},   // vaso %d ($%u)\n", CENT[k][0], CENT[k][1], CENT[k][2], k + 1, VASO_VALOR[k]);
  Serial.println(F("};"));
}
static void diagnostico() {
  Serial.printf("[carro %u] %s | celda (%d,%d) mira %c | rumbo %.1f° | busco color del vaso %d | voy a la posición %d\n",
                CARRO_ID, NOMBRE_ESTADO[estado], nodoX(miNodo), nodoY(miNodo), DIR_LETRA[miDir], rumbo,
                colorBuscado + 1, miVaso == NODO_NINGUNO ? 0 : miVaso + 1);
  if (miVaso != NODO_NINGUNO) {
    uint8_t ruta[MAX_CAMINO];
    uint16_t n = colonia.rutaAprendida(miNodo, capaVaso(miVaso), ruta, MAX_CAMINO);
    int opt = colonia.distanciaBFS(miNodo, nodoDe(VASO_CELDA[miVaso]));
    if (n) Serial.printf("ruta aprendida a ese vaso: %u pasos (óptimo con lo que conozco: %d)\n", n - 1, opt);
    else   Serial.printf("la feromona aún no marca ruta a ese vaso (óptimo: %d pasos)\n", opt);
  }
  for (uint8_t i = 0; i < NUM_CARROS; i++)
    if (parVigente(i)) Serial.printf("  carro %u en (%d,%d) estado %s\n", i, nodoX(pares[i].nodo), nodoY(pares[i].nodo),
                                     NOMBRE_ESTADO[pares[i].estado < 7 ? pares[i].estado : 0]);
}
static void leerSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c >= '0' && c <= '4') {
      float acc[3] = {0, 0, 0}, v[3]; int n = 0;
      for (int i = 0; i < 20; i++) if (leerCromaticidad(v)) { for (int k = 0; k < 3; k++) acc[k] += v[k]; n++; delay(30); }
      if (n) { for (int k = 0; k < 3; k++) CENT[c - '0'][k] = acc[k] / n; Serial.printf("color del vaso %d guardado (%d lecturas)\n", c - '0' + 1, n); }
    } else if (c == 'p') imprimirColores();
    else if (c == 'w') { balanceBlancos(); Serial.printf("ganancias RGB %.3f %.3f %.3f\n", gainRGB[0], gainRGB[1], gainRGB[2]); }
    else if (c == 't') {
      float v[3] = {0, 0, 0}; leerCromaticidad(v);
      float dif; int col = clasificarColor(dif);
      Serial.printf("ToF %u mm | rgb %.3f %.3f %.3f (vaso %d, dif %.3f%s) | vbat %.2f V | rumbo %.1f°\n",
                    tofMm, v[0], v[1], v[2], col + 1, dif, dif <= UMBRAL_COLOR ? "" : " = no se parece a ninguno",
                    leerVBat(), rumbo);
    } else if (c == 'm') {                               // cada rueda por separado, hacia ADELANTE
#if CON_ENCODER
      long a = encI, b = encD;
#endif
      Serial.println(F("motor IZQUIERDO adelante 1 s...")); motorIzq(230); delay(1000); motorIzq(0); delay(300);
      Serial.println(F("motor DERECHO adelante 1 s..."));   motorDer(230); delay(1000); motorDer(0);
#if CON_ENCODER
      Serial.printf("cuentas: izq %ld  der %ld  (las dos POSITIVAS y parecidas; si una es negativa cambia SIGNO_ENC_*)\n",
                    (encI - a) * SIGNO_ENC_I, (encD - b) * SIGNO_ENC_D);
#endif
      Serial.println(F("Si alguna rueda giró hacia atrás, intercambia los 2 cables de ese motor en el driver."));
    } else if (c == 'v') {
#if CON_ENCODER
      moverMM(500, V_CRUCERO, true);
      Serial.println(F("avanzo 500 mm según los encoders. Mide con una regla: si no son 500, ajusta RUEDA_D_MM o CUENTAS_POR_VUELTA."));
#else
      moverMM(10000, V_CRUCERO, true);                  // se corta a los 2 s
      Serial.println(F("avanzo 2 s. Mide los mm recorridos y divide entre 2: ese número va en MM_S_CRUCERO."));
      uint32_t t0 = millis(); while (millis() - t0 < 2000) { lazoControl(0.01f); delay(10); } parar(); motorIzq(0); motorDer(0);
#endif
    }
#if !CON_ENCODER
    else if (c == 'n') {
      moverMM(10000, V_LENTA, true);
      Serial.println(F("avanzo 2 s lento. Mide los mm y divide entre 2: ese número va en MM_S_LENTO."));
      uint32_t t0 = millis(); while (millis() - t0 < 2000) { lazoControl(0.01f); delay(10); } parar(); motorIzq(0); motorDer(0);
    }
#endif
    else if (c == 'g') { girarA((miDir + 1) & 3); miDir = (miDir + 1) & 3; Serial.println(F("girando 90° a la derecha")); }
    else if (c == 'a') { moverMM(CELDA_MM, V_CRUCERO, false); Serial.println(F("avanzando una celda (100 mm)")); }
    else if (c == 'd') diagnostico();
    else if (c == 'l') { ponerLED(60, 0, 0); delay(400); ponerLED(0, 60, 0); delay(400); ponerLED(0, 0, 60); delay(400); ponerLED(0, 0, 0); Serial.println(F("LED: rojo, verde, azul")); }
    else if (c == 'x') {
      if (estado == E_PARADO) { miVaso = NODO_NINGUNO; cambiar(E_ELIGE_VASO); Serial.println(F("arranca")); }
      else { parar(); cambiar(E_PARADO); Serial.println(F("parado")); }
    }
  }
}

// ============================================================================
// 11. SETUP / LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(300);

#if MODO_SIMULACION
  Serial.println(F("MODO_SIMULACION = 1: no se mueven motores, solo se simula el enjambre."));
  sim = new SimulacionEnjambre(7);
  sim->iniciar();
  sim->correr(3000);
  Serial.println(F("Fin de la simulación. Pon MODO_SIMULACION en 0 para usar el carro."));
  return;
#endif

  pinMode(PIN_DIR_I1, OUTPUT); pinMode(PIN_DIR_I2, OUTPUT); pinMode(PIN_DIR_D1, OUTPUT); pinMode(PIN_DIR_D2, OUTPUT);
#if CON_ENCODER
  pinMode(PIN_STBY, OUTPUT); digitalWrite(PIN_STBY, LOW);
#endif
  pwmIniciar(PIN_VEL_I, CH_A); pwmIniciar(PIN_VEL_D, CH_B);
  motorIzq(0); motorDer(0);
#if CON_ENCODER
  pinMode(PIN_ENC_I_A, INPUT); pinMode(PIN_ENC_I_B, INPUT); pinMode(PIN_ENC_D_A, INPUT); pinMode(PIN_ENC_D_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_I_A), isrEncI, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_D_A), isrEncD, CHANGE);
#endif
  pinMode(PIN_TCS_S2, OUTPUT); pinMode(PIN_TCS_S3, OUTPUT); pinMode(PIN_TCS_OUT, INPUT);
  ponerLED(30, 30, 0);

  Wire.begin(PIN_SDA, PIN_SCL); Wire.setClock(400000);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0); Wire.endTransmission();   // despierta el MPU6050
  tofOk = lox.begin();
  if (tofOk) lox.startRangeContinuous(50);
  else Serial.println(F("VL53L0X no responde: revisa SDA/SCL"));

  analogSetPinAttenuation(PIN_VBAT, ADC_11db);

  Serial.printf("Carro %u. Dejar QUIETO 1 s para calibrar el giroscopio...\n", CARRO_ID);
  delay(500); calibrarGyro();

  WiFi.mode(WIFI_STA); WiFi.disconnect();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);        // todos los carros en el canal 1
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(alRecibir);
    esp_now_peer_info_t peer = {}; memcpy(peer.peer_addr, BCAST, 6); peer.channel = 0; peer.encrypt = false;
    esp_now_add_peer(&peer);
  } else Serial.println(F("ESP-NOW no inició"));

  colonia.iniciar();
  miNodo = nodoDe(SALIDA[CARRO_ID]);
  miDir = SALIDA_DIR;
  rumbo = rumboObjetivo = DIR_RUMBO_GRADOS[miDir];
  vbatActual = leerVBat();
#if CON_ENCODER
  digitalWrite(PIN_STBY, HIGH);                          // habilita el TB6612
#endif
  cambiar(E_INICIO);
  Serial.printf("Listo en la celda (%d,%d). Arranca en 3 s. Escribe 'x' para parar.\n", nodoX(miNodo), nodoY(miNodo));
}

void loop() {
#if MODO_SIMULACION
  delay(1000);
  return;
#endif
  static uint32_t tCtrl = 0, tDec = 0, tEvap = 0, tEst = 0, tLed = 0, tBat = 0, tColor = 0;
  uint32_t now = millis();

  leerSerial();
  procesarMensajes();
  if (tofOk && lox.isRangeComplete()) {                  // VL53L0X en modo continuo (no bloquea)
    uint16_t r = lox.readRange();
    tofMm = (lox.readRangeStatus() == 0) ? r : 8190;
  }
  if (now - tCtrl >= 20) { lazoControl((now - tCtrl) / 1000.0f); tCtrl = now; }
  // En el camino: si algo está muy cerca al frente, ¿es mi color objetivo? Si sí, me quedo; si no, sigo.
  if (estado == E_NAVEGA && now - tColor >= 300 && tofOk && tofMm <= DIST_COLOR_MM) {
    tColor = now;
    if (veoMiColor()) llegueAlObjetivo("en el camino");
  }
  if (now - tDec >= 50) { decidir(); tDec = now; }
  if (now - tEvap >= EVAP_PERIODO_MS) { colonia.evaporar(); tEvap = now; }
  if (now - tEst >= ESTADO_PERIODO_MS) { enviarEstado(); tEst = now; }
  if (now - tLed >= 100) { actualizarLED(); tLed = now; }
  if (now - tBat >= 1000) {
    static bool avisoSinBateria = false, avisoBajaParado = false;
    vbatActual = leerVBat();
    if (vbatActual < 3.0f) {
      // Lee casi 0 V: no hay batería (solo USB) o el divisor no está conectado. No se bloquea.
      vbatBajas = 0;
      if (!avisoSinBateria) {
        Serial.printf("No leo la batería en GPIO33 (%.2f V): ¿solo USB o divisor sin conectar? Sigo sin protección de batería.\n", vbatActual);
        avisoSinBateria = true;
      }
    } else {
      avisoSinBateria = false;
      vbatBajas = vbatActual < VBAT_MIN ? vbatBajas + 1 : 0;
      if (vbatBajas >= 3 && estado != E_BATERIA) {
        if (estado == E_PARADO) {                        // ya está quieto: solo avisa una vez
          if (!avisoBajaParado) { Serial.printf("batería baja (%.2f V): cárgala antes de arrancar\n", vbatActual); avisoBajaParado = true; }
        } else {
          parar(); cambiar(E_BATERIA);
          Serial.printf("batería baja (%.2f V): me detengo\n", vbatActual);
        }
      }
      if (vbatBajas == 0) avisoBajaParado = false;
    }
    tBat = now;
  }
}
