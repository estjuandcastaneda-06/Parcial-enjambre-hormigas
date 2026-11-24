// Prueba del algoritmo en el PC, sin hardware.
//   g++ -O2 -std=c++17 prueba_aco.cpp -o prueba_aco
//   ./prueba_aco            (semilla 7)
//   ./prueba_aco 123        (otra semilla)
// Usa el MISMO aco.h y config.h que el carro.
#include <stdlib.h>
#include "../carro_hormiga/simulacion.h"

int main(int argc, char** argv) {
  uint32_t semilla = argc > 1 ? (uint32_t)atoi(argv[1]) : 7;
  uint32_t ticks   = argc > 2 ? (uint32_t)atoi(argv[2]) : 3000;
  static SimulacionEnjambre sim(semilla);   // static: son ~75 KB, mejor fuera de la pila
  sim.iniciar();
  sim.correr(ticks);
  return 0;
}
