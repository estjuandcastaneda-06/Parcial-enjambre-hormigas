/*
 * aco.h — Núcleo del algoritmo de colonia de hormigas (ACO, Ant Colony Optimization)
 * ---------------------------------------------------------------------------
 * C++ puro: NO usa nada de Arduino, así compila igual en el ESP32 y en el PC.
 *
 * IDEA GENERAL
 *  - La arena es una cuadrícula (config.h). Cada celda es un nodo; desde cada nodo salen
 *    4 aristas: N, E, S, O.
 *  - Cada viaje de un carro es una "hormiga": sale de un nodo y va paso a paso hasta su
 *    destino (la meta o un vaso), eligiendo cada paso al azar pero con más probabilidad
 *    por donde hay más feromona y por donde se acerca al destino.
 *  - Hay una CAPA de feromona por destino: capa 0 = META, capa 1..5 = vaso 0..4.
 *    Así la feromona de "cómo llegar a la meta" no se mezcla con "cómo llegar al vaso 3".
 *  - Al llegar, el carro deposita feromona en todo su camino (más si el camino fue corto)
 *    y manda ese camino a los otros carros, que hacen el mismo depósito en su copia.
 *    Como el camino al revés sirve para volver, también se deposita en la capa del origen.
 *  - La feromona se evapora con el tiempo, así las rutas viejas o malas se olvidan.
 *  - Lista tabú: en un mismo viaje no se vuelve a pisar un nodo (evita dar vueltas).
 *  - Callejón sin salida: el carro retrocede un nodo y castiga la arista que lo metió ahí.
 *  - Obstáculo visto por el sensor: se bloquea esa arista (en las dos direcciones) y se avisa.
 */
#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "config.h"

static_assert(NUM_NODOS < 255, "La cuadricula tiene demasiados nodos para guardarlos en uint8_t");
static_assert(MAX_CAMINO <= 240, "MAX_CAMINO no cabe en un mensaje ESP-NOW (250 bytes)");

// ---------------------------------------------------------------------------
// Direcciones y utilidades de la cuadrícula
// ---------------------------------------------------------------------------
enum : uint8_t { NORTE = 0, ESTE = 1, SUR = 2, OESTE = 3 };
static const int8_t DIR_DX[4] = {0, 1, 0, -1};
static const int8_t DIR_DY[4] = {-1, 0, 1, 0};           // y crece hacia abajo (como el plano)
static const char   DIR_LETRA[4] = {'N', 'E', 'S', 'O'};
static const float  DIR_RUMBO_GRADOS[4] = {-90.0f, 0.0f, 90.0f, 180.0f};

static const uint8_t NODO_NINGUNO = 255;
static const uint8_t BLOQUEO_PERMANENTE = 255;

static const uint8_t NUM_CAPAS = 1 + NUM_VASOS;
static const uint8_t CAPA_META = 0;
static inline uint8_t capaVaso(uint8_t vaso) { return 1 + vaso; }

static inline uint8_t dirOpuesta(uint8_t d) { return (d + 2) & 3; }
static inline uint8_t nodoDe(int x, int y) { return (uint8_t)(y * GRID_W + x); }
static inline uint8_t nodoDe(PuntoGrid p) { return nodoDe(p.x, p.y); }
static inline int nodoX(uint8_t n) { return n % GRID_W; }
static inline int nodoY(uint8_t n) { return n / GRID_W; }
static inline int manhattan(uint8_t a, uint8_t b) { return abs(nodoX(a) - nodoX(b)) + abs(nodoY(a) - nodoY(b)); }

static inline uint8_t vecino(uint8_t n, uint8_t d) {
  int x = nodoX(n) + DIR_DX[d], y = nodoY(n) + DIR_DY[d];
  if (x < 0 || y < 0 || x >= GRID_W || y >= GRID_H) return NODO_NINGUNO;
  return nodoDe(x, y);
}
// Dirección para ir de a a b (deben ser vecinos). 255 si no lo son.
static inline uint8_t dirEntre(uint8_t a, uint8_t b) {
  for (uint8_t d = 0; d < 4; d++) if (vecino(a, d) == b) return d;
  return 255;
}

// ¿El centro de la celda (x,y) choca con un obstáculo del plano o queda pegado a la pared?
static inline bool celdaChocaConPlano(int x, int y) {
  int cx = x * CELDA_MM + CELDA_MM / 2, cy = y * CELDA_MM + CELDA_MM / 2;
  if (cx < MARGEN_PARED_MM || cy < MARGEN_PARED_MM ||
      cx > ARENA_W_MM - MARGEN_PARED_MM || cy > ARENA_H_MM - MARGEN_PARED_MM) return true;
  for (uint8_t i = 0; i < NUM_OBSTACULOS; i++) {
    const RectMM& r = OBSTACULOS_MM[i];
    if (cx >= r.x0 - MARGEN_OBST_MM && cx <= r.x1 + MARGEN_OBST_MM &&
        cy >= r.y0 - MARGEN_OBST_MM && cy <= r.y1 + MARGEN_OBST_MM) return true;
  }
  return false;
}

// Resultados especiales de ColoniaACO::elegir()
static const int8_t ELEGIR_LLEGO      = -1;   // ya está en el destino
static const int8_t ELEGIR_ESPERAR    = -2;   // la única salida la ocupa otro carro: esperar
static const int8_t ELEGIR_SIN_SALIDA = -3;   // encerrado: hay que reiniciar el viaje

// ---------------------------------------------------------------------------
// La colonia: feromona + bloqueos + el viaje en curso de ESTE carro
// ---------------------------------------------------------------------------
class ColoniaACO {
 public:
  // Mapa compartido (cada carro tiene su copia y se sincronizan por radio)
  float    tau[NUM_CAPAS][NUM_NODOS][4];   // feromona de la arista (nodo, dirección) en cada capa
  bool     celdaFija[NUM_NODOS];           // obstáculo del plano, pared o vaso: nunca se pisa
  uint8_t  bloqueo[NUM_NODOS][4];          // 0 = libre, 1..254 = evaporaciones restantes, 255 = siempre
  uint8_t  destinoCapa[NUM_CAPAS];         // nodo destino de cada capa
  uint16_t mejorLargo[NUM_CAPAS];          // mejor ruta conocida por capa (en pasos)

  // Viaje en curso (la "hormiga" actual de este carro)
  uint8_t  camino[MAX_CAMINO];
  uint16_t largo = 0;                      // nodos en camino[] (el último es donde estoy)
  bool     visitado[NUM_NODOS];            // lista tabú
  uint8_t  capa = CAPA_META;
  uint16_t pasosTotales = 0;               // pasos reales del viaje (incluye retrocesos)

  // -------------------------------------------------------------------------
  void iniciar() {
    for (uint8_t c = 0; c < NUM_CAPAS; c++) {
      for (uint16_t n = 0; n < NUM_NODOS; n++)
        for (uint8_t d = 0; d < 4; d++) tau[c][n][d] = ACO_TAU_INI;
      mejorLargo[c] = 0xFFFF;
    }
    memset(bloqueo, 0, sizeof(bloqueo));
    for (int y = 0; y < GRID_H; y++)
      for (int x = 0; x < GRID_W; x++) celdaFija[nodoDe(x, y)] = celdaChocaConPlano(x, y);
    for (uint8_t v = 0; v < NUM_VASOS; v++) celdaFija[nodoDe(VASO_POS[v])] = true;
    // los puntos de parada nunca se consideran bloqueados aunque el margen los toque
    celdaFija[nodoDe(META_CELDA)] = false;
    for (uint8_t v = 0; v < NUM_VASOS; v++) celdaFija[nodoDe(VASO_CELDA[v])] = false;

    destinoCapa[CAPA_META] = nodoDe(META_CELDA);
    for (uint8_t v = 0; v < NUM_VASOS; v++) destinoCapa[capaVaso(v)] = nodoDe(VASO_CELDA[v]);
    largo = 0;
  }

  bool celdaLibre(uint8_t n) const { return n != NODO_NINGUNO && !celdaFija[n]; }

  bool aristaLibre(uint8_t n, uint8_t d) const {
    uint8_t v = vecino(n, d);
    return celdaLibre(v) && bloqueo[n][d] == 0;
  }

  // El sensor vio algo entre n y su vecino en d: se bloquea en las dos direcciones.
  void bloquearArista(uint8_t n, uint8_t d, uint8_t ticks) {
    uint8_t v = vecino(n, d);
    if (v == NODO_NINGUNO) return;
    if (ticks > bloqueo[n][d]) bloqueo[n][d] = ticks;
    uint8_t o = dirOpuesta(d);
    if (ticks > bloqueo[v][o]) bloqueo[v][o] = ticks;
  }

  // -------------------------------------------------------------------------
  // Viaje
  // -------------------------------------------------------------------------
  void iniciarViaje(uint8_t desde, uint8_t capaDestino) {
    capa = capaDestino;
    memset(visitado, 0, sizeof(visitado));
    camino[0] = desde;
    largo = 1;
    visitado[desde] = true;
    pasosTotales = 0;
  }

  uint8_t nodoActual() const { return camino[largo - 1]; }
  uint8_t destino() const { return destinoCapa[capa]; }
  bool    llego() const { return largo > 0 && nodoActual() == destino(); }

  // Elige la próxima dirección.
  //   azar1, azar2: dos números al azar en [0,1)
  //   ocupados:     nodos donde están (o a donde van) los otros carros
  //   retroceso:    sale en true si la dirección elegida es volver atrás (callejón)
  // Devuelve 0..3 (N, E, S, O) o ELEGIR_LLEGO / ELEGIR_ESPERAR / ELEGIR_SIN_SALIDA.
  int8_t elegir(float azar1, float azar2, const uint8_t* ocupados, uint8_t nOcupados, bool& retroceso) const {
    retroceso = false;
    uint8_t n = nodoActual();
    uint8_t dst = destino();
    if (n == dst) return ELEGIR_LLEGO;

    float peso[4] = {0, 0, 0, 0};
    float suma = 0, pesoMejor = -1;
    int8_t mejor = -1;
    bool hayOcupado = false;
    int distAhora = manhattan(n, dst);

    for (uint8_t d = 0; d < 4; d++) {
      if (!aristaLibre(n, d)) continue;
      uint8_t v = vecino(n, d);
      if (visitado[v]) continue;                         // lista tabú
      if (estaOcupado(v, ocupados, nOcupados)) { hayOcupado = true; continue; }
      float eta = (manhattan(v, dst) < distAhora) ? ACO_ETA_ACERCA : ACO_ETA_ALEJA;
      peso[d] = powf(tau[capa][n][d], ACO_ALFA) * powf(eta, ACO_BETA);
      suma += peso[d];
      if (peso[d] > pesoMejor) { pesoMejor = peso[d]; mejor = (int8_t)d; }
    }

    if (suma > 0) {
      if (azar1 < ACO_Q0) return mejor;                 // explotación: la mejor arista
      float r = azar2 * suma;                            // exploración: ruleta
      for (uint8_t d = 0; d < 4; d++) {
        if (peso[d] <= 0) continue;
        r -= peso[d];
        if (r <= 0) return (int8_t)d;
      }
      return mejor;
    }

    if (hayOcupado) return ELEGIR_ESPERAR;               // hay salida, pero la tapa otro carro
    if (largo <= 1) return ELEGIR_SIN_SALIDA;            // encerrado desde el inicio

    // Callejón sin salida: volver al nodo anterior
    uint8_t d = dirEntre(n, camino[largo - 2]);
    if (d == 255) return ELEGIR_SIN_SALIDA;
    retroceso = true;
    return (int8_t)d;
  }

  // El carro YA se movió en la dirección d (llamar solo después de moverse de verdad).
  void avanzar(uint8_t d, bool retroceso) {
    pasosTotales++;
    if (retroceso && largo >= 2) {
      // castigar la arista anterior -> actual, que llevó al callejón
      uint8_t anterior = camino[largo - 2];
      uint8_t dEntrada = dirOpuesta(d);
      float& t = tau[capa][anterior][dEntrada];
      t = fmaxf(ACO_TAU_MIN, t * ACO_CASTIGO_CALLEJON);
      largo--;                                           // el callejón queda marcado como visitado
      return;
    }
    uint8_t v = vecino(nodoActual(), d);
    if (v == NODO_NINGUNO) return;
    if (largo >= MAX_CAMINO) {                           // camino demasiado largo: olvidar y seguir
      uint16_t p = pasosTotales;
      iniciarViaje(nodoActual(), capa);
      pasosTotales = p;
    }
    camino[largo++] = v;
    visitado[v] = true;
  }

  // -------------------------------------------------------------------------
  // Feromona
  // -------------------------------------------------------------------------
  // Deposita un viaje completo que terminó en el destino de 'capaViaje'.
  // Se usa igual para MI viaje y para el viaje que me manda otro carro por radio.
  // Además, el camino al revés sirve para llegar al ORIGEN, así que si el origen es la
  // meta o un vaso, también se deposita (al revés) en esa capa.
  void depositarViaje(uint8_t capaViaje, const uint8_t* nodos, uint16_t n) {
    if (n < 2 || capaViaje >= NUM_CAPAS) return;
    depositarEnCapa(capaViaje, nodos, n, false);
    for (uint8_t c = 0; c < NUM_CAPAS; c++)
      if (c != capaViaje && destinoCapa[c] == nodos[0]) depositarEnCapa(c, nodos, n, true);
  }

  // Evaporación: tau <- max(TAU_MIN, tau·(1-RHO)). También descuenta los bloqueos temporales.
  void evaporar() {
    const float f = 1.0f - ACO_RHO;
    for (uint8_t c = 0; c < NUM_CAPAS; c++)
      for (uint16_t n = 0; n < NUM_NODOS; n++)
        for (uint8_t d = 0; d < 4; d++) tau[c][n][d] = fmaxf(ACO_TAU_MIN, tau[c][n][d] * f);
    for (uint16_t n = 0; n < NUM_NODOS; n++)
      for (uint8_t d = 0; d < 4; d++)
        if (bloqueo[n][d] > 0 && bloqueo[n][d] != BLOQUEO_PERMANENTE) bloqueo[n][d]--;
  }

  // -------------------------------------------------------------------------
  // Herramientas
  // -------------------------------------------------------------------------
  // Distancia en pasos por el mapa conocido (búsqueda en anchura). -1 si no hay camino.
  int distanciaBFS(uint8_t desde, uint8_t hasta) const {
    if (desde == hasta) return 0;
    uint8_t cola[NUM_NODOS];
    int16_t dist[NUM_NODOS];
    for (uint16_t i = 0; i < NUM_NODOS; i++) dist[i] = -1;
    uint16_t ini = 0, fin = 0;
    cola[fin++] = desde; dist[desde] = 0;
    while (ini < fin) {
      uint8_t n = cola[ini++];
      for (uint8_t d = 0; d < 4; d++) {
        if (!aristaLibre(n, d)) continue;
        uint8_t v = vecino(n, d);
        if (dist[v] >= 0) continue;
        dist[v] = dist[n] + 1;
        if (v == hasta) return dist[v];
        cola[fin++] = v;
      }
    }
    return -1;
  }

  // La ruta que "aprendió" la colonia: desde 'desde', seguir siempre la arista con más feromona
  // de la capa (sin repetir nodos). Devuelve cuántos nodos tiene, o 0 si no llega al destino.
  uint16_t rutaAprendida(uint8_t desde, uint8_t capaRuta, uint8_t* out, uint16_t maxNodos) const {
    bool vis[NUM_NODOS];
    memset(vis, 0, sizeof(vis));
    uint16_t k = 0;
    uint8_t n = desde;
    out[k++] = n; vis[n] = true;
    while (n != destinoCapa[capaRuta] && k < maxNodos) {
      int8_t mejor = -1; float t = -1;
      for (uint8_t d = 0; d < 4; d++) {
        if (!aristaLibre(n, d)) continue;
        uint8_t v = vecino(n, d);
        if (vis[v]) continue;
        if (tau[capaRuta][n][d] > t) { t = tau[capaRuta][n][d]; mejor = (int8_t)d; }
      }
      if (mejor < 0) return 0;
      n = vecino(n, (uint8_t)mejor);
      out[k++] = n; vis[n] = true;
    }
    return (n == destinoCapa[capaRuta]) ? k : 0;
  }

 private:
  static bool estaOcupado(uint8_t v, const uint8_t* ocupados, uint8_t nOcupados) {
    for (uint8_t i = 0; i < nOcupados; i++) if (ocupados[i] == v) return true;
    return false;
  }

  void depositarEnCapa(uint8_t c, const uint8_t* nodos, uint16_t n, bool alReves) {
    uint16_t pasos = n - 1;
    float delta = ACO_Q / pasos;
    if (pasos <= mejorLargo[c]) { mejorLargo[c] = pasos; delta *= ACO_BONO_MEJOR; }
    for (uint16_t i = 0; i + 1 < n; i++) {
      uint8_t a = alReves ? nodos[n - 1 - i] : nodos[i];
      uint8_t b = alReves ? nodos[n - 2 - i] : nodos[i + 1];
      uint8_t d = dirEntre(a, b);
      if (d == 255) continue;                            // mensaje corrupto: se ignora ese tramo
      float& t = tau[c][a][d];
      t = fminf(ACO_TAU_MAX, t + delta);
    }
  }
};

// ---------------------------------------------------------------------------
// ¿Qué vaso recoger? (3 carros, 5 vasos)
// ---------------------------------------------------------------------------
// Puntaje = monedas del vaso / (1 + pasos hasta él). Se saltan los vasos que no están
// disponibles o que ya reclamó otro carro. Devuelve el índice del vaso o -1 si no hay.
static inline int elegirVaso(const ColoniaACO& col, uint8_t desde,
                             const uint16_t monedas[NUM_VASOS],
                             const bool disponible[NUM_VASOS],
                             const uint8_t reclamadoPor[NUM_VASOS], uint8_t miId) {
  int mejor = -1;
  float puntajeMejor = -1;
  for (uint8_t v = 0; v < NUM_VASOS; v++) {
    if (!disponible[v]) continue;
    if (reclamadoPor[v] != NODO_NINGUNO && reclamadoPor[v] != miId) continue;
    int dist = col.distanciaBFS(desde, nodoDe(VASO_CELDA[v]));
    if (dist < 0) continue;                              // no se puede llegar
    float m = monedas[v] > 0 ? (float)monedas[v] : 1.0f;
    float puntaje = m / (1.0f + dist);
    if (puntaje > puntajeMejor) { puntajeMejor = puntaje; mejor = v; }
  }
  return mejor;
}
