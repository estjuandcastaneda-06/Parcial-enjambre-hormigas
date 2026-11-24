/*
 * config.h — TODO lo que se ajusta del proyecto está en este archivo.
 * ---------------------------------------------------------------------------
 * Lo usan el firmware del carro (carro_hormiga.ino), el núcleo del algoritmo (aco.h)
 * y la simulación (simulacion.h). Si cambias la arena, cambias solo esto.
 *
 * Marco de referencia (el mismo del plano de la arena, lámina 4):
 *   origen en la esquina superior izquierda, x hacia la derecha, y hacia ABAJO, en mm.
 *   Rumbo: 0° = mirando al ESTE (+x), 90° = SUR (+y), 180° = OESTE, -90° = NORTE.
 */
#pragma once
#include <stdint.h>

// ============================================================================
// 1. IDENTIDAD DEL CARRO
// ============================================================================
#ifndef CARRO_ID
#define CARRO_ID 0          // <- CAMBIAR antes de subir a cada carro: 0, 1 o 2
#endif
#define NUM_CARROS 3
#define NUM_VASOS  5        // $50, $100, $200, $500, $1000

// 1 = el ESP32 NO mueve motores: corre la simulación del enjambre (3 carros) y la imprime
//     por el monitor serie a 115200. Sirve para probar el algoritmo sin armar nada.
// 0 = firmware normal del carro.
#ifndef MODO_SIMULACION
#define MODO_SIMULACION 0
#endif

// ============================================================================
// 2. ARENA Y CUADRÍCULA
// ============================================================================
// La arena se divide en celdas cuadradas. Cada celda es un NODO del grafo y desde cada
// nodo salen 4 ARISTAS (Norte, Este, Sur, Oeste). Las hormigas (carros) caminan de nodo
// en nodo y dejan feromona en las aristas.
#define ARENA_W_MM 2100
#define ARENA_H_MM 1200
#define CELDA_MM   100
#define GRID_W     (ARENA_W_MM / CELDA_MM)   // 21 columnas
#define GRID_H     (ARENA_H_MM / CELDA_MM)   // 12 filas
#define NUM_NODOS  (GRID_W * GRID_H)         // 252 nodos (debe ser < 255)

// Una celda queda bloqueada si su centro está a menos de este margen de un obstáculo o
// de la pared (≈ medio ancho del carro). [CALIBRAR] con el ancho real del carro.
#define MARGEN_OBST_MM  60
#define MARGEN_PARED_MM 60

struct RectMM { int16_t x0, y0, x1, y1; };

// Obstáculos CONOCIDOS (lámina 4 del plano). Los obstáculos que NO estén aquí los
// descubre el carro con sus sensores y se los avisa a los demás.
static const RectMM OBSTACULOS_MM[] = {
  { 900,   0, 1000,  750},   // obstáculo 1
  {1300, 450, 1400, 1200},   // obstáculo 2
  {1650,   0, 1750,  750},   // obstáculo 3
};
static const uint8_t NUM_OBSTACULOS = sizeof(OBSTACULOS_MM) / sizeof(OBSTACULOS_MM[0]);

struct PuntoGrid { uint8_t x, y; };   // columna, fila

// --- META: donde se entregan los vasos ------------------------------------------------
// [CALIBRAR] Pon la celda real de la meta en tu arena.
static const PuntoGrid META_CELDA = {19, 6};     // centro ≈ (1950, 650) mm
static const uint8_t   META_DIR   = 1;           // hacia dónde mira el carro para soltar (0 N, 1 E, 2 S, 3 O)

// --- VASOS: salen del divisor de monedas ------------------------------------------------
// VASO_POS   = celda donde está físicamente el vaso (queda bloqueada para circular).
// VASO_CELDA = celda donde se para el carro para recogerlo.
// VASO_DIR   = hacia dónde mira el carro para agarrarlo.
// [CALIBRAR] Pon las celdas reales de los 5 vasos bajo el divisor.
static const PuntoGrid VASO_POS[NUM_VASOS]   = {{1, 2}, {1, 4}, {1, 6}, {1, 8}, {1, 10}};
static const PuntoGrid VASO_CELDA[NUM_VASOS] = {{2, 2}, {2, 4}, {2, 6}, {2, 8}, {2, 10}};
static const uint8_t   VASO_DIR[NUM_VASOS]   = {3, 3, 3, 3, 3};      // mirando al OESTE
static const uint16_t  VASO_VALOR[NUM_VASOS] = {50, 100, 200, 500, 1000};

// --- SALIDA de cada carro ---------------------------------------------------------------
static const PuntoGrid SALIDA[NUM_CARROS] = {{4, 3}, {4, 6}, {4, 9}};
static const uint8_t   SALIDA_DIR = 1;           // todos arrancan mirando al ESTE

// ============================================================================
// 3. PARÁMETROS DEL ALGORITMO DE HORMIGAS (ACO)
// ============================================================================
// Probabilidad de tomar la arista d:   p(d) ∝ tau(d)^ALFA · eta(d)^BETA
//   tau = feromona de esa arista en la capa del destino
//   eta = visibilidad: ETA_ACERCA si el paso acerca al destino, ETA_ALEJA si lo aleja
#define ACO_ALFA        1.0f    // peso de la feromona
#define ACO_BETA        1.0f    // peso de la visibilidad
#define ACO_ETA_ACERCA  1.0f
#define ACO_ETA_ALEJA   0.35f
#define ACO_Q0          0.60f   // probabilidad de ir directo por la mejor arista (explotación)
#define ACO_Q           20.0f   // feromona total que deja un viaje: cada arista recibe Q / largo
#define ACO_BONO_MEJOR  2.0f    // multiplica el depósito si el viaje empató o mejoró la mejor ruta
#define ACO_RHO         0.05f   // evaporación por cada llamada a evaporar()
#define ACO_TAU_INI     1.0f
#define ACO_TAU_MIN     0.10f   // nunca baja de aquí: siempre queda algo de exploración
#define ACO_TAU_MAX     12.0f   // nunca sube de aquí: evita que una ruta se "congele"
#define ACO_CASTIGO_CALLEJON 0.40f  // al salir de un callejón, la arista que entró se multiplica por esto
#define MAX_CAMINO      160     // máximo de nodos por viaje (si se pasa, la hormiga "olvida" y sigue)

// Bloqueos vistos por el sensor: duran este número de evaporaciones (255 = para siempre).
// Así, si el "obstáculo" era algo temporal (una mano, una caja que se quitó), se vuelve a probar.
#define BLOQUEO_SENSOR_TICKS 120   // con EVAP_PERIODO_MS = 5 s, son 10 minutos
// Valores ajustados con la simulación (6 corridas de 3000 ticks): con estos parámetros los
// viajes quedan en ≈1,3 veces la ruta óptima; sin feromona (ALFA = 0) quedan en ≈2,5 veces
// y se hacen ≈30 % menos entregas.

// ============================================================================
// 4. TIEMPOS DEL FIRMWARE
// ============================================================================
#define EVAP_PERIODO_MS      5000   // cada cuánto se evapora la feromona
#define ESTADO_PERIODO_MS     300   // cada cuánto cada carro anuncia dónde está
#define VIGENCIA_PAR_MS      1500   // si no sé nada de un carro en este tiempo, lo ignoro
#define T_REPOSICION_MS     60000   // tras recoger un vaso, cuánto se espera para que haya otro lleno
#define T_VETO_VASO_MS      20000   // si el color no coincide, no volver a ese vaso por este tiempo
#define T_ESPERA_MAX_MS      4000   // si espero a otro carro más que esto, cambio de camino
