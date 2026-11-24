# 1. Conexiones

Todas las tierras (GND) van unidas: la de las pilas, la del L298N, la del ESP32 y la de los sensores.

## Alimentación

```
Pilas 18650 (+) ──> switch ──┬──> L298N borne 12V
                             └──> resistencia 100 kΩ ──┬──> GPIO33 (medición de batería)
                                                       └──> resistencia 47 kΩ ──> GND
Pilas (−) ──────────────────────> GND común
L298N 5V (o el buck) ───────────> VIN del ESP32
```

- El divisor 100 kΩ / 47 kΩ baja el voltaje de las pilas (≈ 7,4 V) a unos 2,3 V, que el ESP32 sí puede leer.
- Un condensador de 100 nF entre GPIO33 y GND ayuda a que la lectura de batería sea estable.
- El switch tiene que aguantar la corriente de los motores. Uno malo hace caer el voltaje y los motores solo zumban: lo vimos en la sección 4.

## Motores (L298N)

| L298N | ESP32 | Función |
|---|---|---|
| ENA | GPIO25 | Velocidad del motor izquierdo (PWM) |
| IN1 | GPIO26 | Sentido del motor izquierdo |
| IN2 | GPIO27 | Sentido del motor izquierdo |
| ENB | GPIO14 | Velocidad del motor derecho (PWM) |
| IN3 | GPIO32 | Sentido del motor derecho |
| IN4 | GPIO4 | Sentido del motor derecho |
| OUT1/OUT2 | — | Motor izquierdo |
| OUT3/OUT4 | — | Motor derecho |

Quita los jumpers de ENA y ENB para que el ESP32 controle la velocidad.

## Sensores I2C principales (pines 21/22)

El VL53L0X y el MPU6050 comparten los mismos dos cables.

| Sensor | VIN/VCC | GND | SDA | SCL | Dirección |
|---|---|---|---|---|---|
| VL53L0X (distancia) | 3V3 | GND | GPIO21 | GPIO22 | 0x29 |
| MPU6050 (giroscopio) | 3V3 | GND | GPIO21 | GPIO22 | 0x68 |

En el VL53L0X, los pines GPIO1 y XSHUT quedan sin conectar.

## Sensor de color — opción A: TCS3200 (grande, 4 LEDs) → `carritos_hormiga_1`

| TCS3200 | ESP32 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| S0 | 3V3 |
| S1 | GND |
| S2 | GPIO18 |
| S3 | GPIO19 |
| OUT | GPIO23 |
| LED / OE | Sin conectar, o OE a GND |

S0 en alto y S1 en bajo fijan la salida al 20 % de la frecuencia máxima, que el ESP32 mide cómodo.

## Sensor de color — opción B: TCS34725 (pequeño, morado) → `carrito_hormigas_sensor_color_pequeno`

| TCS34725 | ESP32 |
|---|---|
| VIN | 3V3 |
| GND | GND |
| SDA | **GPIO18** |
| SCL | **GPIO19** |
| 3V3, LED, INT | Sin conectar |

**Por qué no va en los pines 21/22:** el TCS34725 usa la misma dirección I2C (0x29) que el VL53L0X. Si los dos van en los mismos cables se pisan y ninguno funciona. Por eso el sensor de color va en su propio bus I2C, en los pines 18 y 19.

Con el LED sin conectar, el LED blanco del módulo queda siempre prendido e ilumina el vaso.

## Otros

| Pieza | ESP32 |
|---|---|
| LED WS2812, entrada de datos | GPIO5 |

## Montaje de los sensores (importante)

- El **sensor de distancia** y el **sensor de color** van **juntos, en el centro del frente**. Si quedan en esquinas distintas, el carro frena por algo que ve un sensor y lee el color con el otro, que está mirando a otro lado.
- Los dos van **a la altura de los vasos** y **mirando derecho hacia adelante**.
- El sensor de distancia va **1–2 cm más atrás** que el de color. Así, cuando frena a 4 cm, el sensor de color queda a unos 2 cm del vaso.
- **Nada encima del chip** del VL53L0X: ni cinta, ni silicona, ni la película protectora de fábrica.

## Revisar los cables con el código

Escribe `i` en el monitor serie. Debe aparecer:

```
bus pines 21/22 (VL53L0X 0x29 + MPU6050 0x68): 0x29 0x68
bus pines 18/19 (sensor de color 0x29): 0x29        <- solo en la versión TCS34725
```

Si falta una dirección, revisa los cables y las soldaduras de ese sensor.
