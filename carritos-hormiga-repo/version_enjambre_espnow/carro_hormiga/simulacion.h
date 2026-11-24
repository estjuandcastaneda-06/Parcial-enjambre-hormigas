/*
 * simulacion.h — Simulación del enjambre (3 carros) SIN hardware
 * ---------------------------------------------------------------------------
 * Usa exactamente el mismo núcleo (aco.h) que el carro real. Cada carro tiene su
 * propia copia del mapa de feromona, como en la vida real, y los "mensajes de radio"
 * se simulan llamando a la misma función en las 3 copias.
 *
 * Corre en el PC (prueba_pc/prueba_aco.cpp) o en el mismo ESP32 con MODO_SIMULACION = 1.
 * Imprime con printf (en el ESP32 sale por el monitor serie).
 *
 * Un "tick" = cada carro hace una acción (avanzar una celda, girar/esperar o recoger).
 */
#pragma once
#include <stdio.h>
#include "aco.h"

// Obstáculo SORPRESA: no está en config.h. Los carros no lo conocen; lo descubren con el
// "sensor" (en la simulación: al intentar entrar a una celda ocupada en el mundo real).
static const RectMM SORPRESA_MM = {600, 500, 700, 800};

#define SIM_EVAP_CADA_TICKS     4     // ≈ EVAP_PERIODO_MS si un paso de celda tarda ~1,2 s
#define SIM_REPOSICION_TICKS   40     // ticks hasta que un vaso recogido vuelve a estar lleno
#define SIM_ESPERA_MAX          6     // ticks esperando a otro carro antes de cambiar de camino
#define SIM_VENTANA            15     // entregas por renglón del informe

// Generador pseudoaleatorio propio: misma semilla = misma corrida en PC y en ESP32.
struct AzarSim {
  uint32_t s;
  explicit AzarSim(uint32_t semilla) : s(semilla ? semilla : 1) {}
  float sig() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return (s >> 8) / 16777216.0f; }
  uint32_t entero(uint32_t n) { return (uint32_t)(sig() * n) % n; }
};

class SimulacionEnjambre {
 public:
  ColoniaACO col[NUM_CARROS];              // la copia del mapa que lleva cada carro
  bool mundo[NUM_NODOS];                   // lo que hay DE VERDAD en la arena

  struct Carro {
    uint8_t  nodo;
    uint8_t  vaso;                         // vaso que reclamó o lleva
    bool     enViaje, cargado;
    uint16_t espera;
    uint16_t monedasLlevadas;
  } car[NUM_CARROS];

  uint16_t monedas[NUM_VASOS];
  uint32_t disponibleEn[NUM_VASOS];
  uint8_t  reclamadoPor[NUM_VASOS];

  AzarSim  az;
  uint32_t tick = 0;

  // estadísticas
  uint32_t entregas = 0, dinero = 0, monedasTot = 0, bloqueos = 0, esperas = 0, reinicios = 0;
  uint32_t vPasos = 0, vOptimo = 0, vN = 0;   // ventana actual
  float    primeraRelacion = 0, ultimaRelacion = 0;

  explicit SimulacionEnjambre(uint32_t semilla) : az(semilla) {}

  void iniciar() {
    for (int y = 0; y < GRID_H; y++)
      for (int x = 0; x < GRID_W; x++) {
        uint8_t n = nodoDe(x, y);
        mundo[n] = celdaChocaConPlano(x, y) || dentroSorpresa(x, y);
      }
    for (uint8_t v = 0; v < NUM_VASOS; v++) mundo[nodoDe(VASO_POS[v])] = true;
    mundo[nodoDe(META_CELDA)] = false;
    for (uint8_t v = 0; v < NUM_VASOS; v++) mundo[nodoDe(VASO_CELDA[v])] = false;

    for (uint8_t i = 0; i < NUM_CARROS; i++) {
      col[i].iniciar();
      car[i] = Carro{nodoDe(SALIDA[i]), NODO_NINGUNO, false, false, 0, 0};
    }
    for (uint8_t v = 0; v < NUM_VASOS; v++) {
      monedas[v] = 1 + az.entero(10);
      disponibleEn[v] = 0;
      reclamadoPor[v] = NODO_NINGUNO;
    }
  }

  void correr(uint32_t ticks) {
    printf("\n=== SIMULACION DEL ENJAMBRE: %d carros, %d vasos, cuadricula %dx%d de %d mm ===\n",
           NUM_CARROS, NUM_VASOS, GRID_W, GRID_H, CELDA_MM);
    printf("Mapa real (los carros NO conocen la X: la descubren con el sensor)\n");
    imprimirMapa(nullptr, 0);
    printf("\nCada renglon = %d entregas. 'relacion' = pasos reales / pasos de la ruta optima (1.00 = perfecto)\n",
           SIM_VENTANA);

    for (uint32_t t = 0; t < ticks; t++) {
      tick++;
      for (uint8_t i = 0; i < NUM_CARROS; i++) pasoCarro(i);
      if (tick % SIM_EVAP_CADA_TICKS == 0)
        for (uint8_t i = 0; i < NUM_CARROS; i++) col[i].evaporar();
    }
    informeFinal();
  }

 private:
  static bool dentroSorpresa(int x, int y) {
    int cx = x * CELDA_MM + CELDA_MM / 2, cy = y * CELDA_MM + CELDA_MM / 2;
    return cx >= SORPRESA_MM.x0 - MARGEN_OBST_MM && cx <= SORPRESA_MM.x1 + MARGEN_OBST_MM &&
           cy >= SORPRESA_MM.y0 - MARGEN_OBST_MM && cy <= SORPRESA_MM.y1 + MARGEN_OBST_MM;
  }

  bool ocupadoPorOtro(uint8_t nodo, uint8_t yo) const {
    for (uint8_t j = 0; j < NUM_CARROS; j++) if (j != yo && car[j].nodo == nodo) return true;
    return false;
  }

  void pasoCarro(uint8_t i) {
    Carro& c = car[i];
    ColoniaACO& m = col[i];

    if (!c.enViaje) {
      if (!c.cargado) {
        bool disp[NUM_VASOS];
        for (uint8_t v = 0; v < NUM_VASOS; v++) disp[v] = tick >= disponibleEn[v];
        int v = elegirVaso(m, c.nodo, monedas, disp, reclamadoPor, i);
        if (v < 0) return;                         // no hay vaso libre: esperar
        reclamadoPor[v] = i;                       // "radio": los demás ven el reclamo
        c.vaso = (uint8_t)v;
        m.iniciarViaje(c.nodo, capaVaso(c.vaso));
      } else {
        m.iniciarViaje(c.nodo, CAPA_META);
      }
      c.enViaje = true;
      c.espera = 0;
    }

    uint8_t ocup[NUM_CARROS];
    uint8_t nOcup = 0;
    for (uint8_t j = 0; j < NUM_CARROS; j++) if (j != i) ocup[nOcup++] = car[j].nodo;

    bool retro;
    int8_t d = m.elegir(az.sig(), az.sig(), ocup, nOcup, retro);

    if (d == ELEGIR_LLEGO) { llegar(i); return; }
    if (d == ELEGIR_SIN_SALIDA) { m.iniciarViaje(c.nodo, m.capa); reinicios++; return; }
    if (d == ELEGIR_ESPERAR) { esperarOtro(i); return; }

    uint8_t v = vecino(c.nodo, (uint8_t)d);
    if (mundo[v]) {                                 // el sensor ve un obstáculo: bloquear y avisar a TODOS
      for (uint8_t k = 0; k < NUM_CARROS; k++) col[k].bloquearArista(c.nodo, (uint8_t)d, BLOQUEO_SENSOR_TICKS);
      bloqueos++;
      if (bloqueos <= 5)
        printf("  [tick %4lu] carro %u: obstaculo en (%d,%d)->%c, avisa a los demas\n",
               (unsigned long)tick, i, nodoX(c.nodo), nodoY(c.nodo), DIR_LETRA[d]);
      return;
    }
    if (ocupadoPorOtro(v, i)) { esperarOtro(i); return; }   // pasa al retroceder

    c.nodo = v;
    m.avanzar((uint8_t)d, retro);
    c.espera = 0;
  }

  void esperarOtro(uint8_t i) {
    esperas++;
    if (++car[i].espera > SIM_ESPERA_MAX) {          // se cansó de esperar: replanea desde aquí
      col[i].iniciarViaje(car[i].nodo, col[i].capa);
      car[i].espera = 0;
      reinicios++;
    }
  }

  void llegar(uint8_t i) {
    Carro& c = car[i];
    ColoniaACO& m = col[i];
    uint8_t capaViaje = m.capa;

    // "radio": el camino le llega a los 3 carros (incluido este) y todos depositan igual
    for (uint8_t k = 0; k < NUM_CARROS; k++) col[k].depositarViaje(capaViaje, m.camino, m.largo);

    if (capaViaje == CAPA_META) {
      entregas++;
      monedasTot += c.monedasLlevadas;
      dinero += (uint32_t)c.monedasLlevadas * VASO_VALOR[c.vaso];
      int opt = bfsMundo(nodoDe(VASO_CELDA[c.vaso]), nodoDe(META_CELDA));
      if (opt > 0 && m.camino[0] == nodoDe(VASO_CELDA[c.vaso])) {   // solo viajes limpios vaso -> meta
        vPasos += m.pasosTotales; vOptimo += opt; vN++;
      }
      if (entregas % SIM_VENTANA == 0) cerrarVentana();
      c.cargado = false;
      c.vaso = NODO_NINGUNO;
    } else {
      uint8_t v = c.vaso;
      c.cargado = true;
      c.monedasLlevadas = monedas[v];
      monedas[v] = 1 + az.entero(10);                // el divisor vuelve a llenar el vaso
      disponibleEn[v] = tick + SIM_REPOSICION_TICKS;
      reclamadoPor[v] = NODO_NINGUNO;
    }
    c.enViaje = false;
  }

  void cerrarVentana() {
    if (vN == 0) return;
    float rel = (float)vPasos / (float)vOptimo;
    if (primeraRelacion == 0) primeraRelacion = rel;
    ultimaRelacion = rel;
    printf("entregas %4lu-%4lu | pasos prom %6.1f | optimo prom %5.1f | relacion %.2f | bloqueos %lu\n",
           (unsigned long)(entregas - SIM_VENTANA + 1), (unsigned long)entregas,
           (float)vPasos / vN, (float)vOptimo / vN, rel, (unsigned long)bloqueos);
    vPasos = vOptimo = vN = 0;
  }

  // Distancia óptima en el mundo REAL (para comparar)
  int bfsMundo(uint8_t desde, uint8_t hasta) const {
    uint8_t cola[NUM_NODOS];
    int16_t dist[NUM_NODOS];
    for (uint16_t k = 0; k < NUM_NODOS; k++) dist[k] = -1;
    uint16_t a = 0, b = 0;
    cola[b++] = desde; dist[desde] = 0;
    while (a < b) {
      uint8_t n = cola[a++];
      if (n == hasta) return dist[n];
      for (uint8_t d = 0; d < 4; d++) {
        uint8_t v = vecino(n, d);
        if (v == NODO_NINGUNO || mundo[v] || dist[v] >= 0) continue;
        dist[v] = dist[n] + 1;
        cola[b++] = v;
      }
    }
    return -1;
  }

  void imprimirMapa(const uint8_t* ruta, uint16_t nRuta) const {
    for (int y = 0; y < GRID_H; y++) {
      printf("  ");
      for (int x = 0; x < GRID_W; x++) {
        uint8_t n = nodoDe(x, y);
        char ch = '.';
        if (mundo[n]) ch = dentroSorpresa(x, y) && !celdaChocaConPlano(x, y) ? 'X' : '#';
        for (uint8_t v = 0; v < NUM_VASOS; v++) {
          if (n == nodoDe(VASO_POS[v])) ch = (char)('1' + v);
          if (n == nodoDe(VASO_CELDA[v])) ch = 'v';
        }
        for (uint16_t k = 0; k < nRuta; k++) if (ruta[k] == n && ch == '.') ch = '*';
        if (n == nodoDe(META_CELDA)) ch = 'M';
        printf("%c ", ch);
      }
      printf("\n");
    }
    printf("  # obstaculo/pared   X obstaculo sorpresa   1-5 vasos   v donde se para el carro   M meta   * ruta\n");
  }

  void informeFinal() {
    printf("\n=== RESULTADO tras %lu ticks ===\n", (unsigned long)tick);
    printf("Entregas: %lu | monedas: %lu | dinero: $%lu | obstaculos detectados: %lu | esperas: %lu | replaneos: %lu\n",
           (unsigned long)entregas, (unsigned long)monedasTot, (unsigned long)dinero,
           (unsigned long)bloqueos, (unsigned long)esperas, (unsigned long)reinicios);
    printf("Relacion pasos/optimo: al inicio %.2f -> al final %.2f\n", primeraRelacion, ultimaRelacion);

    printf("\nRuta aprendida por la feromona (copia del carro 0) de cada vaso a la meta:\n");
    uint8_t ruta[MAX_CAMINO];
    int vasoMostrar = NUM_VASOS / 2;
    uint16_t nMostrar = 0;
    uint8_t rutaMostrar[MAX_CAMINO];
    for (uint8_t v = 0; v < NUM_VASOS; v++) {
      uint16_t n = col[0].rutaAprendida(nodoDe(VASO_CELDA[v]), CAPA_META, ruta, MAX_CAMINO);
      int opt = bfsMundo(nodoDe(VASO_CELDA[v]), nodoDe(META_CELDA));
      if (n) printf("  vaso %u ($%u): %3u pasos   (optimo %d)%s\n", v + 1, VASO_VALOR[v], n - 1, opt,
                    (int)(n - 1) == opt ? "  <- optima" : "");
      else   printf("  vaso %u ($%u): la feromona todavia no marca una ruta completa (optimo %d)\n",
                    v + 1, VASO_VALOR[v], opt);
      if ((int)v == vasoMostrar) { nMostrar = n; for (uint16_t k = 0; k < n; k++) rutaMostrar[k] = ruta[k]; }
    }
    if (nMostrar) {
      printf("\nRuta aprendida del vaso %d a la meta:\n", vasoMostrar + 1);
      imprimirMapa(rutaMostrar, nMostrar);
    }
  }
};
