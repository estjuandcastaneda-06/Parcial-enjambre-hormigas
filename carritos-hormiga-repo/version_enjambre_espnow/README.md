# Carros hormiga: algoritmo de colonia de hormigas (ACO) para transportar monedas

Proyecto final de Micros. Tres carros con ESP32 recogen los 5 vasos de monedas que salen
del divisor y los llevan a la **meta**. Para elegir la ruta usan un algoritmo de colonia de
hormigas: dejan **feromona** en los caminos buenos y se la comparten por radio (ESP-NOW).

## Archivos

```
proyecto_final_hormigas/
├── carro_hormiga/            <- se abre con el Arduino IDE (la carpeta entera)
│   ├── carro_hormiga.ino     firmware del carro: motores, sensores, pinza, radio y máquina de estados
│   ├── aco.h                 núcleo del algoritmo de hormigas (C++ puro, sin Arduino)
│   ├── config.h              TODO lo que se ajusta: ID del carro, arena, meta, vasos, parámetros
│   └── simulacion.h          simulación de los 3 carros sin hardware
├── prueba_pc/prueba_aco.cpp  corre la simulación en el PC con g++
├── CONEXIONES_Y_PRUEBAS.md   qué va a cada pin del ESP32, cómo conectar cada sensor y cómo probarlo
└── README.md
```

Hardware (la lista de materiales del carro, láminas 5 y 5b del repositorio): ESP32 DevKit V1,
**TB6612FNG + 2 motorreductores N20 con encoder**, VL53L0X, TCS3200, MPU6050, 2 servos SG90,
LED WS2812 con divisor de batería, buck MP1584 (sin código), LiPo 2S. **No usa ultrasonido.**

- **Distancia:** la miden los encoders, y un control PI iguala la velocidad de las dos ruedas.
- **Giros de 90° e ir derecho:** los controla el giroscopio.
- **Obstáculos sin vaso:** el VL53L0X los ve antes de avanzar a la siguiente celda.
- **Obstáculos con el vaso agarrado:** la pinza tapa el VL53L0X, así que los detectan los
  **encoders**. Si el carro manda avanzar y las ruedas casi no giran durante 0,5 s, es un
  choque: se devuelve, bloquea ese paso y avisa a los demás.

Si en vez de eso usas **L298N y motores sin encoder**, pon `#define CON_ENCODER 0` en
`carro_hormiga.ino`. En ese modo el avance se calcula por tiempo (ver el anexo de la guía).

## Comparación con la versión del chat anterior

Los archivos sueltos en `Parcial micros/` (`aco.h`, `aco.cpp`, `carro_hormiga.ino`,
`config.h`, `test_aco.cpp`) eran un **esqueleto**: la idea del algoritmo es la misma
(cuadrícula, capas por destino, tabú, callejones, depósito Q/L, evaporación, reparto de vasos),
pero:

| | Versión anterior | Esta versión (definitiva) |
|---|---|---|
| Hardware | Funciones vacías (`TODO`): motores, giro, sensores y radio | Todo escrito: motores con TB6612 y encoders (PI de velocidad y detección de choque), giros y línea recta con giroscopio, VL53L0X, TCS3200, pinza, LED, batería y ESP-NOW |
| Pista | 6×6 celdas como máximo (la lista tabú usa 64 bits) | 21×12 celdas de 100 mm (la arena real de 2100×1200), con los 3 obstáculos del plano |
| Feromona al ir al vaso | No se deposita | Se deposita, y la ruta al revés también sirve para volver |
| Radio | Un mensaje por cada arista | Un solo mensaje con toda la ruta |
| Reclamo de vasos | Nunca se libera en los otros carros | Se renueva cada 300 ms y caduca; desempata el ID menor |
| Choques entre carros | No se tienen en cuenta | Cada carro anuncia su celda y la siguiente, y espera o rodea |
| Obstáculos | Bloqueo para siempre | Bloqueo de 10 min (si se quita la caja, se vuelve a usar el camino) |
| Estructura de carpetas | El `.ino` suelto: el Arduino IDE no lo abre así | Carpeta `carro_hormiga/` lista para abrir |
| Probado | — | Compila para ESP32; simulado con 3 carros |

Los archivos anteriores se dejaron donde estaban, sin tocarlos.

## Cómo funciona el algoritmo

1. **Mapa:** la arena (2100 × 1200 mm) se divide en celdas de 100 mm, 21 × 12 = 252 nodos.
   Cada nodo tiene 4 aristas (N, E, S, O). Los 3 obstáculos del plano (lámina 4) ya vienen
   marcados en `config.h`.
2. **Cada viaje es una hormiga:** en cada celda el carro elige la siguiente al azar, con
   probabilidad
   `p(d) ∝ feromona(d)^α · visibilidad(d)^β`.
   La visibilidad vale 1 si el paso lo acerca al destino y 0,35 si lo aleja. Con probabilidad
   `q0 = 0,6` toma directamente la mejor arista (explota lo aprendido); si no, sortea (explora).
3. **Capas de feromona:** hay una capa por destino, la **meta** y los **5 vasos**. Así el rastro
   para "ir a la meta" no se mezcla con el de "ir al vaso 3".
4. **Lista tabú y callejones:** en un viaje no se repite celda. Si el carro queda encerrado,
   retrocede una celda y castiga la arista que lo metió ahí.
5. **Depósito:** al llegar, el carro deposita `Q / largo` en cada arista de su ruta (el doble si
   empata o mejora la mejor ruta conocida) y manda la ruta a los demás, que hacen el mismo
   depósito en su copia. La ruta al revés sirve para volver, así que también se deposita en la
   capa del origen.
6. **Evaporación:** cada 5 s toda la feromona se multiplica por `1 − ρ` (`ρ = 0,05`), con un
   mínimo y un máximo para que nunca se pierda del todo la exploración.
7. **Obstáculos nuevos:** si el sensor ve algo en la celda de enfrente que no está en el plano
   y no es otro carro, se bloquea esa arista por 10 minutos y se avisa a los demás.
8. **Tráfico:** cada carro anuncia cada 300 ms dónde está y a qué celda va. Los demás no
   eligen esas celdas, y si esperan más de 4 s, buscan otro camino.
9. **3 carros, 5 vasos:** cada carro elige el vaso con mejor `monedas / (1 + distancia)` que
   nadie haya reclamado. Si dos reclaman el mismo, se lo queda el de ID menor. Al recoger un
   vaso, este queda "vacío" 60 s, hasta que el divisor lo vuelva a llenar. Si hay una estación
   que avise cuándo un vaso está lleno, puede mandar el mensaje `MSG_VASO_LISTO`.

### Qué hace el carro paso a paso (versión sin pinza: el vaso va fijo en el chasis)

```
INICIO (3 s) → ELIGE_VASO (color objetivo) → NAVEGA (hormiga hacia ese vaso) → REVISA
   ├─ es mi color   → OBJETIVO: motores quietos para siempre (LED verde)
   └─ no es mi color → se devuelve → ELIGE_VASO (otra posición) …
```

- **ELIGE_VASO:** el color objetivo es el del vaso que elige (el más lleno y cercano que no
  haya reclamado otro carro), o siempre el mismo si `VASO_FIJO` vale 0..4.
- **NAVEGA:** gira con el giroscopio a la dirección elegida, mira con el sensor de distancia y
  avanza una celda (100 mm), manteniendo la línea recta con el giroscopio. Si algo se atraviesa
  frena, y si no se quita en 3 s se devuelve a su celda. Si algo está a menos de 100 mm al
  frente, revisa su color: si es su color objetivo se detiene para siempre; si no, sigue.
- **REVISA:** mira hacia el vaso, avanza despacio hasta que el VL53L0X lea unos 45 mm y lee el
  color con el TCS3200.
- **OBJETIVO:** se queda quieto. Sigue anunciando su vaso por radio, así los otros carros buscan
  otros. Para que vuelva a moverse: botón EN (reinicio) o `x` dos veces.

## Resultado de la simulación

Se probó con 6 corridas de 3000 ticks y 3 carros, con un obstáculo "sorpresa" (`X`) que los
carros no conocen y tienen que descubrir con el sensor.

| | Entregas (6 corridas) | Pasos reales / pasos óptimos |
|---|---|---|
| **Con feromona** | 429 | 1,3 |
| Sin feromona (α = 0) | 309 | 2,5 |

Ejemplo de ruta aprendida (vaso 3 → meta, 33 pasos, la óptima):

```
  # # # # # # # # # # # # # # # # # # # # #
  # . . . . . . . # # # . . . . . # # . . #
  # 1 v . . . . . # # # . . . . . # # . . #
  # . . . . . . . # # # * * * * * # # . . #
  # 2 v . . X X X # # # * # # # * # # . . #
  # . . . . X X X # # # * # # # * # # . . #
  # 3 v * * X X X # # # * # # # * # # . M #
  # . . . * X X X # # # * # # # * # # . * #
  # 4 v . * X X X * * * * # # # * * * * * #
  # . . . * * * * * . . . # # # . . . . . #
  # 5 v . . . . . . . . . # # # . . . . . #
  # # # # # # # # # # # # # # # # # # # # #
  # obstáculo/pared  X sorpresa  1-5 vasos  v donde se para el carro  M meta  * ruta
```

Esta tabla salió de una traducción a C# del mismo `aco.h` y `simulacion.h`, porque en el
computador donde se escribió no había compilador de C++ para PC. Para comprobarla con el
código original, corre la prueba (sección siguiente).

## Cómo probarlo

**Opción 1, en el ESP32 sin armar el carro:** en `config.h` pon `#define MODO_SIMULACION 1`,
sube el programa a cualquier ESP32 y abre el monitor serie a 115200. Imprime el mapa, el
avance y las rutas aprendidas. Luego vuelve a ponerlo en `0`.

**Opción 2, en el PC** (con g++, por ejemplo MinGW o w64devkit):

```bash
cd prueba_pc
g++ -O2 -std=c++17 prueba_aco.cpp -o prueba_aco
./prueba_aco 7 3000
```

El primer número es la semilla y el segundo la cantidad de ticks.

## Cómo subirlo a cada carro

1. Arduino IDE → placa **ESP32 Dev Module**.
2. Librerías (Gestor de librerías): **Adafruit_VL53L0X** y **ESP32Servo**. El LED WS2812 usa
   la función `rgbLedWrite` del core ESP32 y no necesita librería.
3. En `config.h` pon `CARRO_ID` en 0, 1 o 2 (uno distinto por carro).
4. En `config.h` revisa las celdas de la **meta** (`META_CELDA`) y de los **vasos**
   (`VASO_POS`, `VASO_CELDA`, `VASO_DIR`) para que coincidan con la arena real. Las que
   vienen son un ejemplo: los vasos a la izquierda (salida del divisor) y la meta a la derecha,
   detrás de los 3 obstáculos.
5. Sigue las pruebas de **[CONEXIONES_Y_PRUEBAS.md](CONEXIONES_Y_PRUEBAS.md)** en orden. Los comandos
   del monitor serie son estos:
   - `t` lee los sensores, y `m` prueba cada motor y muestra las cuentas de los encoders (deben ser positivas).
   - `v` avanza 500 mm según los encoders (para calibrar la distancia), y `a` avanza una celda.
   - `g` gira 90°, para ajustar las ganancias de giro y de línea recta.
   - `c` abre o cierra la pinza (cerrada = "llevo vaso"), y `l` prueba el LED.
   - `w` hace el balance de blancos con una tarjeta blanca. Luego `0`…`4` guarda el color de cada vaso, y `p` imprime la tabla para pegarla en `CENT`.
   - `d` muestra el estado y la ruta aprendida, y `x` para o arranca el carro.

## Pendiente y advertencias

- **Compila, pero no se ha probado en un carro real.** Las constantes marcadas `[CALIBRAR]`
  (cuentas por vuelta del encoder, ganancias de giro y línea recta, umbrales de distancia y de
  choque, ángulos de la pinza y colores) hay que medirlas en el carro.
- **Con el vaso agarrado** el carro no "ve" adelante: solo nota un obstáculo cuando lo toca y
  las ruedas se frenan. Los obstáculos que ya descubrió cualquier carro viajando sin vaso quedan
  en el mapa compartido, así que normalmente los evita antes de tocarlos.
- **Odometría por celdas:** el carro asume que siempre queda en el centro de una celda y mirando
  a 0°, 90°, 180° o 270°. Si se desvía mucho conviene añadir una referencia (marcas en el piso
  o alinearse contra una pared) para corregir el error.
- El firmware anterior del repositorio (`firmware/carrito_hormiga`) era para 5 carros con un
  corredor fijo. Este lo reemplaza para 3 carros, con la ruta aprendida por las hormigas.
