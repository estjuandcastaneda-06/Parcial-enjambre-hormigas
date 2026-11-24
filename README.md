# Carritos hormiga: buscador de vasos por color

Proyecto final de Microcontroladores (UMNG).

Cada carrito, con un ESP32, recorre un espacio de **6 × 6 cuadros** (1,5 m × 1,5 m) buscando el **vaso de su color**. Para explorar usa la idea de una hormiga: deja "feromona" en los cuadros que pisa y prefiere los cuadros con menos feromona, o sea, los que no ha visitado.

Con el sensor de distancia ve los objetos que tiene enfrente. Frena frente a cada uno y lee su color. Si es su vaso, se queda quieto con el LED verde titilando. Si no, se devuelve, marca ese cuadro como ocupado y sigue buscando.

---

## Contenido del repositorio

```
carritos-hormiga-repo/
├── README.md                                   este archivo
├── carritos_hormiga_1/
│   └── carritos_hormiga_1.ino                  carro con el sensor de color GRANDE (TCS3200, 4 LEDs)
├── carrito_hormigas_sensor_color_pequeno/
│   └── carrito_hormigas_sensor_color_pequeno.ino   carro con el sensor PEQUEÑO morado (TCS34725, I2C)
├── docs/
│   ├── 01_conexiones.md                        qué cable va a cada pin del ESP32
│   ├── 02_como_funciona.md                     explicación del programa: estados, mapa, feromona, color
│   ├── 03_calibracion_y_pruebas.md             paso a paso para dejar el carro funcionando
│   ├── 04_problemas_y_soluciones.md            fallas que tuvimos y cómo se resolvieron
│   └── conexiones_carro_hormiga.pdf            documento de conexiones con fotos y esquemas
└── version_enjambre_espnow/                    versión anterior: 3 carros que se comunican por ESP-NOW
```

Los dos programas principales son **el mismo código**. Solo cambia esta línea de la sección 2:

| Carpeta | Sensor de color | Línea en el código |
|---|---|---|
| `carritos_hormiga_1` | TCS3200 (grande, 4 LEDs, pines S0–S3 y OUT) | `#define SENSOR_COLOR 3200` |
| `carrito_hormigas_sensor_color_pequeno` | TCS34725 / CJMCU-34725 (pequeño, morado, I2C) | `#define SENSOR_COLOR 34725` |

Los dos sensores se conectan distinto. El TCS3200 da el color como una frecuencia en el pin OUT. El TCS34725 lo manda por I2C. Por eso cada carro necesita la versión que corresponde a su sensor.

## Hardware

| Pieza | Para qué sirve |
|---|---|
| ESP32 DevKit | El cerebro del carro |
| L298N + 2 motorreductores (sin encoder) | Mover las ruedas |
| VL53L0X (sensor de distancia láser, I2C) | Ver los objetos que tiene enfrente y frenar |
| TCS3200 **o** TCS34725 | Leer el color del vaso |
| MPU6050 (giroscopio, I2C) | Girar 90° exactos y avanzar derecho |
| LED WS2812 | Mostrar qué está haciendo el carro |
| Divisor 100 kΩ / 47 kΩ en GPIO33 | Medir el voltaje de la batería |
| 2 pilas 18650 (≈ 7,4 V) + switch | Alimentación |

Las conexiones completas están en [docs/01_conexiones.md](docs/01_conexiones.md).

## Cómo subir el código

1. Instala el **Arduino IDE 2** y, en el gestor de placas, el paquete **esp32 de Espressif**.
2. En el gestor de librerías, instala **Adafruit_VL53L0X**.
3. Abre la carpeta del carro que corresponde a tu sensor de color. Abre el `.ino`: el archivo tiene que quedar dentro de una carpeta con su mismo nombre.
4. Elige la placa **ESP32 Dev Module** y el puerto COM del ESP32.
5. En la sección 1 del código, pon en `VASO_OBJETIVO` el vaso que busca ese carro, del 1 al 5.
6. Sube el código y abre el **monitor serie a 115200**.

Después sigue [docs/03_calibracion_y_pruebas.md](docs/03_calibracion_y_pruebas.md): hay que calibrar los colores y la distancia de cada cuadro **en cada carro**.

## Colores del LED

| LED | Qué está haciendo el carro |
|---|---|
| Amarillo | Acaba de prender; arranca en 3 s |
| Celeste | Explorando los cuadros |
| Morado | Frenó frente a algo y está leyendo el color |
| Naranja | No era su vaso; se está devolviendo |
| **Verde titilando** | **Encontró su vaso y se quedó quieto** |
| Blanco tenue | Parado (con la tecla `x`) |
| Rojo titilando | Batería baja |
| Azul titilando | El sensor de distancia no responde |

## Comandos por el monitor serie

| Tecla | Qué hace |
|---|---|
| `x` | Parar / arrancar |
| `r` | Reiniciar el mapa: el carro vuelve a contar desde la esquina de inicio |
| `d` | Mostrar el mapa (C = carro, # = ocupado, . = sin visitar, 1-9 = feromona) |
| `t` | Leer todos los sensores y decir a qué vaso se parece lo que tiene enfrente |
| `f` | Ver el sensor de distancia durante 5 s |
| `i` | Ver qué sensores I2C encuentra el ESP32 (para revisar cables) |
| `m` | Probar cada motor por separado |
| `a` | Avanzar un cuadro (para calibrar) |
| `+` / `-` | El cuadro le quedó largo / corto: ajusta la velocidad estimada |
| `c` | Medir la velocidad solo, mirando una pared blanca a 40–50 cm |
| `v` / `n` | Avanzar 2 s rápido / lento (para medir la velocidad a mano) |
| `g` | Girar 90° a la derecha |
| `w` | Balance de blancos, con una hoja blanca enfrente |
| `1`..`5` | Guardar el color del vaso 1..5 |
| `p` | Imprimir los valores calibrados para pegarlos en el código |
| `l` | Probar el LED |

## Limitaciones conocidas

- **No tiene encoders.** El carro calcula cuánto avanzó con *velocidad × tiempo*. Si las ruedas patinan o la batería baja mucho, se equivoca. El código compensa el voltaje de la batería, pero conviene calibrar con la batería cargada.
- **No ve el piso.** Sabe en qué cuadro está contando desde la esquina de inicio. Si lo arrancas en otro lugar, todo el mapa queda corrido. Con bordes de cartón alrededor del espacio y `HAY_PAREDES = true`, el carro usa esas paredes para corregirse.
- **El VL53L0X que usamos llega solo a unos 50 cm.** Para buscar vasos basta, porque solo necesita ver el cuadro de enfrente (28 cm).
- **El color depende de la luz y de la distancia.** Por eso se calibra con el carro en el espacio real y con el vaso a la misma distancia a la que el carro frena.
