# Conexiones y pruebas de cada carro

Esta guía es para **un carro**. Los 3 carros se arman igual, y lo único que cambia es
`CARRO_ID` en `config.h`.

> **Versión actual: SIN pinza ni servos.** El vaso va fijo en el chasis (ahí caen las monedas).
> Todo lo que esta guía dice de los **servos SG90, la pinza, el comando `c`, el condensador de
> 1000 µF de los servos y la "entrega en la meta" ya no aplica**: no conectes nada a GPIO16 y 17.
> Ahora el carro va hasta un vaso, lee su color y, si es su **color objetivo**, se queda quieto
> para siempre (LED verde fijo). Si no lo es, sigue buscando. Mientras viaja, si tiene algo a
> menos de 100 mm al frente y es su color objetivo, también se detiene. El objetivo se elige
> solo, o fijo con `VASO_FIJO` en `carro_hormiga.ino`. El comando `t` muestra `dif`: si es
> mayor que `UMBRAL_COLOR` (0,04) el carro no lo toma como su color.

## Materiales de cada carro

| Pieza | Para qué sirve |
|---|---|
| ESP32 DevKit V1 | cerebro: algoritmo de hormigas + radio ESP-NOW con los otros carros |
| 2 motorreductores N20 **con encoder** + soportes | moverse; el encoder mide cuánto avanzó cada rueda |
| Driver TB6612FNG | da la potencia a los motores (la ESP32 sola no puede) |
| Ruedas Ø42 × 15 mm + rueda loca (bola Ø10) | ruedas |
| Giroscopio MPU6050 | girar 90° exactos e ir derecho |
| Sensor de distancia VL53L0X | ver obstáculos adelante y acercarse al vaso |
| Sensor de color TCS3200 | saber de qué color (denominación) es el vaso |
| 2 servos SG90 + dedos de pinza impresos | agarrar el vaso |
| LED WS2812 + R/C/divisor | mostrar el estado; medir la batería |
| Batería LiPo 2S 850 mAh + interruptor + portafusible | energía (7,4 V) |
| Regulador buck MP1584 a 5 V | baja 7,4 V a 5 V para la ESP32 y los servos |
| Placas de acrílico 3 mm y 2 mm + separadores M3 × 30 mm | chasis de 2 pisos |

> **No hace falta ultrasonido.** El VL53L0X ve los obstáculos cuando el carro va sin vaso. Con el
> vaso agarrado, la pinza tapa el VL53L0X, y el carro detecta los obstáculos con los **encoders**:
> si manda avanzar y las ruedas casi no giran durante medio segundo, es que chocó. Entonces se
> devuelve a su celda, bloquea ese paso y les avisa a los otros carros.

---

## 1. Alimentación (lo primero y lo más importante)

```
LiPo 2S (7,4 V) ──► interruptor ──► fusible 3 A ──┬──► VM del TB6612FNG          (riel VBAT)
                                                   ├──► IN+ del buck MP1584
                                                   └──► R1 100 kΩ ─┬─► GPIO33     (medir batería)
                                                                   └─ R2 47 kΩ ─► GND

MP1584 OUT+ (ajustado a 5,0 V) ──┬──► VIN de la ESP32                             (riel 5 V)
                                 ├──► + (rojo) de los 2 servos SG90
                                 ├──► VDD del LED WS2812
                                 └──► condensador 1000 µF / 16 V a GND (junto a los servos)

Pin 3V3 de la ESP32 ──► VCC de: TCS3200, VL53L0X, MPU6050, encoders de los N20,
                        y VCC (lógica) del TB6612                                  (riel 3V3)

GND: TODOS los GND unidos en UN solo punto, junto al buck
     (batería, buck, ESP32, TB6612 y todos los módulos).
```

> ⚠️ **Antes de conectar la ESP32:** alimenta solo el buck con la batería, mide la salida con
> el multímetro y gira el potenciómetro hasta que marque **5,0 V**. Si sale a 7 V o más, quema
> la ESP32 y los servos.
>
> ⚠️ **Para programar** basta el USB: puedes dejar la batería apagada. Para probar los motores
> enciende la batería. Las DevKit V1 traen un diodo que protege el USB, así que puedes tener
> los dos conectados mientras pruebas.

---

## 2. Tabla de conexiones a la ESP32 (DevKit V1 de 30 pines)

| GPIO | Va a | Pin del módulo | Notas |
|---|---|---|---|
| **25** | TB6612FNG | PWMA | velocidad del motor izquierdo |
| **26** | TB6612FNG | AIN1 | dirección del motor izquierdo |
| **27** | TB6612FNG | AIN2 | dirección del motor izquierdo |
| **14** | TB6612FNG | PWMB | velocidad del motor derecho |
| **32** | TB6612FNG | BIN1 | dirección del motor derecho |
| **4** | TB6612FNG | BIN2 | dirección del motor derecho |
| **13** | TB6612FNG | STBY | alto = driver encendido |
| **34** | Encoder motor izquierdo | C1 (A) | solo entrada |
| **35** | Encoder motor izquierdo | C2 (B) | solo entrada |
| **36** (VP) | Encoder motor derecho | C1 (A) | solo entrada |
| **39** (VN) | Encoder motor derecho | C2 (B) | solo entrada |
| **18** | TCS3200 | S2 | filtro de color |
| **19** | TCS3200 | S3 | filtro de color |
| **23** | TCS3200 | OUT | frecuencia según el color |
| **21** | VL53L0X **y** MPU6050 | SDA | bus I2C compartido |
| **22** | VL53L0X **y** MPU6050 | SCL | bus I2C compartido |
| **16** | Servo izquierdo SG90 | señal (naranja) | pinza |
| **17** | Servo derecho SG90 | señal (naranja) | pinza (va espejado) |
| **5** | LED WS2812 | DIN | color = estado del carro |
| **33** | Divisor 100k / 47k | punto medio | mide la batería |
| VIN | Riel 5 V | — | sale del buck |
| 3V3 | Riel 3V3 | — | alimenta sensores y encoders |
| GND | GND común | — | — |

**No uses** GPIO0, 2, 12 ni 15 (son pines de arranque) ni GPIO1 y 3 (son del USB).

---

## 3. Cómo conectar cada módulo

### Driver TB6612FNG

```
              ┌────────── TB6612FNG ──────────┐
  VBAT ──────►│ VM                       PWMA │◄── GPIO25
  3V3  ──────►│ VCC                      AIN2 │◄── GPIO27
  GND  ──────►│ GND                      AIN1 │◄── GPIO26
 Motor IZQ ◄──│ AO1                      STBY │◄── GPIO13
 Motor IZQ ◄──│ AO2                      BIN1 │◄── GPIO32
 Motor DER ◄──│ BO2                      BIN2 │◄── GPIO4
 Motor DER ◄──│ BO1                      PWMB │◄── GPIO14
  GND  ──────►│ GND                       GND │◄── GND
              └───────────────────────────────┘
   (el orden físico de los pines cambia según el módulo: guíate por lo impreso)
```

| TB6612 | Conectar a |
|---|---|
| VM | VBAT (7,4 V, después del fusible) |
| VCC | 3V3 |
| GND (todos) | GND |
| PWMA / AIN1 / AIN2 | GPIO25 / GPIO26 / GPIO27 |
| PWMB / BIN1 / BIN2 | GPIO14 / GPIO32 / GPIO4 |
| STBY | GPIO13 |
| AO1, AO2 | M1 y M2 del **motor izquierdo** |
| BO1, BO2 | M1 y M2 del **motor derecho** |

### Motorreductores N20 con encoder (cable de 6 hilos)

Guíate por lo **impreso en la plaquita** del encoder, no por el color del cable.

| Plaquita del encoder | Motor izquierdo | Motor derecho |
|---|---|---|
| **M1** (motor +) | AO1 del TB6612 | BO1 del TB6612 |
| **M2** (motor −) | AO2 del TB6612 | BO2 del TB6612 |
| **VCC** | 3V3 (**no** 5 V) | 3V3 |
| **GND** | GND | GND |
| **C1** (A) | GPIO34 | GPIO36 |
| **C2** (B) | GPIO35 | GPIO39 |

- GPIO34, 35, 36 y 39 no tienen resistencia de pull-up interna. Si en la prueba `m` las cuentas
  salen en 0, pon una resistencia de **10 kΩ de cada salida C1 y C2 a 3V3**.
- El código asume N20 de **reducción 100:1** con encoder de **7 pulsos por vuelta**. Si los tuyos
  son de otra reducción, cambia `CUENTAS_POR_VUELTA = 7 × reducción × 2` en
  `carro_hormiga.ino`, o calíbralo con la prueba 8.

### Giroscopio MPU6050

| MPU6050 | Conectar a |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO21 |
| SCL | GPIO22 |
| AD0 | GND (dirección 0x68) |
| INT, XDA, XCL | sin conectar |

Va **plano**, cerca del centro del carro y bien pegado, para que no vibre.

### Sensor de distancia VL53L0X

| VL53L0X | Conectar a |
|---|---|
| VIN / VCC | 3V3 |
| GND | GND |
| SDA | GPIO21 (mismo bus del MPU6050) |
| SCL | GPIO22 |
| XSHUT, GPIO1 | sin conectar |

Va al frente, mirando hacia adelante a la altura de los vasos, en la placa superior y a unos
105 mm del borde trasero (lámina 1 del repositorio). Con el vaso agarrado entre los dedos debe
leer **~45 mm**.

### Sensor de color TCS3200

| TCS3200 | Conectar a |
|---|---|
| VCC | 3V3 |
| GND | GND |
| S0 | 3V3 (fijo) |
| S1 | GND (fijo) → escala de frecuencia al 20 % |
| S2 | GPIO18 |
| S3 | GPIO19 |
| OUT | GPIO23 |
| OE (si tu módulo lo trae) | GND |
| LED (si tu módulo lo trae) | 3V3 (luces encendidas) |

Va en la "palma" de la pinza, a **~15 mm del vaso**. De más lejos no lee bien.

### Pinza: 2 servos SG90

| Servo | Café / negro | Rojo | Naranja (señal) |
|---|---|---|---|
| Izquierdo | GND | 5 V | GPIO16 |
| Derecho | GND | 5 V | GPIO17 |

Los servos van al **5 V del buck**, nunca al 3V3 de la ESP32. El condensador de 1000 µF va
cerca de ellos: sin él, el pico de corriente de los servos puede reiniciar la ESP32.

### LED WS2812 + R/C/divisor

| Pieza | Conexión |
|---|---|
| LED VDD / 5V | 5 V |
| LED GND | GND |
| LED DIN | GPIO5, con una resistencia de **330 Ω** en serie |
| **R1 100 kΩ** | de VBAT a GPIO33 |
| **R2 47 kΩ** | de GPIO33 a GND |
| **C 1000 µF / 16 V** | entre el riel 5 V y GND (pata − a GND) |

```
VBAT ──[R1 100 kΩ]──┬──► GPIO33
                    └──[R2 47 kΩ]──► GND
```

Con la batería llena (8,4 V), GPIO33 recibe 2,7 V, que está dentro del rango de la ESP32.

Colores del LED: amarillo = iniciando · azul = eligiendo vaso · azul claro = va al vaso ·
verde = lleva el vaso · morado = recogiendo · cian = entregando · gris = parado ·
rojo intermitente = batería baja.

---

## 4. Software

1. Arduino IDE → **Herramientas → Placa → ESP32 Dev Module.** En tu computador ya tienes el
   core ESP32 3.3 instalado.
2. **Herramientas → Gestionar bibliotecas:** instala **Adafruit_VL53L0X**. **ESP32Servo** ya la
   tienes.
3. Abre `proyecto_final_hormigas/carro_hormiga/carro_hormiga.ino`. Se abren solas las pestañas
   `aco.h`, `config.h` y `simulacion.h`.
4. En `carro_hormiga.ino` revisa que diga `#define CON_ENCODER 1` (TB6612 + N20 con encoder).
5. En `config.h` pon `CARRO_ID` en **0** para el primer carro, **1** para el segundo y **2**
   para el tercero.

---

## 5. Cómo probar, paso a paso

Haz las pruebas **en este orden**: cada una necesita que la anterior funcione. Abre el monitor
serie a **115200 baudios**. Apenas arranque, escribe **`x`** para que el carro no salga solo
(arranca a los 3 s), y luego escribe la letra de cada prueba.

### Prueba 0: el algoritmo solo (sin armar nada)
1. En `config.h` pon `#define MODO_SIMULACION 1`.
2. Sube el programa a una ESP32 suelta, conectada solo por USB.
3. En el monitor serie debe salir el mapa, los obstáculos que van descubriendo los carros,
   renglones de `entregas … relacion …` y, al final, las rutas aprendidas con `<- optima`.
4. Vuelve a poner `MODO_SIMULACION 0`.

✅ Pasa si la "relacion" baja hacia ~1,1–1,4 y aparecen rutas óptimas.

### Prueba 1: alimentación
1. Sin ESP32 ni módulos, conecta la batería, el interruptor, el fusible y el buck.
2. Ajusta el buck a **5,0 V** con el multímetro.
3. Mide que el riel VBAT tenga entre 7,4 V y 8,4 V.

### Prueba 2: ESP32 y sensores I2C
1. Conecta la ESP32, el VL53L0X y el MPU6050. Sube el programa por USB.
2. Debe salir `Carro 0. Dejar QUIETO 1 s...` y luego `Listo en la celda (4,3)`.
   Si sale `VL53L0X no responde`, revisa SDA/SCL y el 3V3.
3. Escribe **`x`** (parar).
4. Escribe **`t`** varias veces:
   - `ToF` debe cambiar al acercar la mano (en mm).
   - `rumbo` debe cambiar al girar el carro con la mano: +90° a la derecha y −90° a la
     izquierda, visto desde arriba. Si va al revés, en `lazoControl` cambia `rumbo - …` por
     `rumbo + …`.

### Prueba 3: batería
Escribe **`t`** y compara `vbat` con lo que marca el multímetro en la batería. Si difieren en
más de 0,2 V, ajusta `DIVISOR_VBAT`.

### Prueba 4: LED y pinza
- **`l`**: el LED debe hacer rojo, verde y azul.
- **`c`**: la pinza abre y cierra. Ajusta `PINZA_ABIERTA` y `PINZA_CERRADA` hasta que agarre
  el vaso firme sin forzar los servos.

### Prueba 5: motores y encoders (con el carro LEVANTADO, ruedas en el aire)
1. Conecta el TB6612 y los motores, y enciende la batería.
2. Escribe **`m`**: primero gira 1 s la rueda **izquierda** y después la **derecha**.
   - Las dos deben girar **hacia adelante**. Si una gira hacia atrás, intercambia sus cables
     M1/M2 en el TB6612.
   - Al final imprime `cuentas: izq ___ der ___`. Las dos deben ser **positivas y parecidas**
     (cientos o miles). Si una es negativa, cambia su signo en `SIGNO_ENC_I` o `SIGNO_ENC_D`.
     Si sale 0, revisa los cables C1/C2/VCC del encoder o pon las resistencias de 10 kΩ.

### Prueba 6: color de los vasos
1. Pon una tarjeta **blanca** a 15 mm del TCS3200 y escribe **`w`** (balance de blancos).
2. Pon el vaso 1 frente al sensor y escribe **`0`**. Haz lo mismo con el vaso 2 y **`1`**,
   y así hasta el vaso 5 con **`4`**.
3. Escribe **`p`** y copia la tabla que imprime en `CENT` dentro de `carro_hormiga.ino`.
   Vuelve a subir el programa.
4. Comprueba con **`t`**: al poner cada vaso debe decir `(vaso N)` con el número correcto.

### Prueba 7: avanzar en el piso
1. Pon el carro junto a una cinta métrica y escribe **`v`**: avanza **500 mm** según los
   encoders.
2. Mide cuánto avanzó de verdad. Si no son 500 mm, corrige `CUENTAS_POR_VUELTA` en
   proporción. Por ejemplo, si avanzó 550 mm, multiplícalo por 550/500 = 1,1.
3. Escribe **`a`**: debe avanzar exactamente **100 mm** (una celda).

### Prueba 8: girar
1. Escribe **`g`**: gira 90° a la derecha. Repítelo 4 veces y debe quedar como empezó.
   - Si oscila (se pasa y vuelve), baja el `0.06f` de la ganancia de giro en `maniobra()`.
   - Si gira muy lento o se queda trabado, sube el mínimo `0.6f`.
2. Escribe **`a`** 10 veces seguidas: debe quedar a ~1 m y en línea recta. Si se tuerce, sube
   el `0.04f` de "mantener la línea recta".

### Prueba 9: choque (detección de obstáculos con el vaso agarrado)
1. Pon un vaso entre los dedos y escribe **`c`** para cerrar la pinza. Así el carro se comporta
   como si llevara el vaso: ignora el VL53L0X, que está viendo el vaso.
2. Pon una caja pesada justo delante del vaso y escribe **`a`**.
3. El carro debe empujar medio segundo, notar que no avanza y **devolverse**. Durante la
   navegación real, esto imprime `obstáculo (choque) …` y avisa a los demás.
   - Si no reacciona, sube `CHOQUE_FRAC` (por ejemplo, 0,35).
   - Si "choca" sin que haya nada, bájalo (por ejemplo, 0,15) o sube `T_CHOQUE_MS`.

### Prueba 10: un carro solo en la pista
1. Mide tu pista y pon en `config.h` las celdas reales de la **meta** (`META_CELDA`) y de los
   **vasos** (`VASO_POS`, `VASO_CELDA`, `VASO_DIR`). Cada celda mide 100 mm, y la celda
   (0,0) es la esquina de arriba a la izquierda. Pon también los obstáculos fijos en
   `OBSTACULOS_MM`.
2. Pon el carro en su celda de salida (`SALIDA[CARRO_ID]`), **mirando al Este**, que es la
   derecha del plano.
3. Enciéndelo y espera 3 s (sin escribir `x`). Debe ir por un vaso, revisarlo, agarrarlo,
   llevarlo a la meta y volver.
4. Con el USB conectado, **`d`** muestra en qué celda cree estar y la ruta aprendida.

✅ Pasa si después de varios viajes los recorridos se vuelven más cortos y directos.

### Prueba 11: obstáculo sorpresa
Con el carro andando **sin vaso**, pon una caja en su camino. En el monitor serie debe salir
`obstáculo (VL53L0X) en (x,y)->E, aviso a los demás`, y el carro debe buscar otra ruta.

### Prueba 12: los 3 carros juntos
1. Sube el programa a cada carro con su `CARRO_ID` (0, 1 y 2).
2. Con uno conectado por USB, escribe **`d`**: debe listar a los otros dos carros con su celda.
   Si no aparecen, no les está llegando la radio: revisa que los 3 tengan el mismo programa.
3. Ponlos en la pista: cada uno debe ir a un vaso **distinto**, no deben chocar (se esperan o
   rodean) y, al bloquear un paso, los otros también lo evitan.

---

## 6. Problemas comunes

| Síntoma | Causa probable |
|---|---|
| Los motores no se mueven | STBY (GPIO13) sin conectar, falta VM o GND común |
| La ESP32 se reinicia al mover la pinza | Falta el condensador de 1000 µF o los servos están en 3V3 |
| `VL53L0X no responde` | SDA/SCL invertidos, falta 3V3 o GND |
| Las cuentas de los encoders salen en 0 | C1/C2 mal conectados, VCC del encoder sin 3V3, o faltan los pull-up de 10 kΩ |
| El carro gira sin parar o va de lado | Una cuenta negativa: corrige `SIGNO_ENC_*` (prueba 5) |
| `rumbo` cambia solo con el carro quieto | El carro se movió durante la calibración inicial: reinícialo quieto |
| Se detiene y retrocede sin obstáculo | Detección de choque muy sensible: baja `CHOQUE_FRAC` (prueba 9) |
| Lee mal el color | Repite la prueba 6 con la luz real de la pista, y el sensor a 15 mm |
| Los carros no se ven entre sí | Ninguna ESP32 debe estar conectada a un Wi-Fi; todas usan el canal 1 |

---

## Anexo: si usas L298N y motores sin encoder

Pon `#define CON_ENCODER 0` en `carro_hormiga.ino`. Conexión:

| L298N | Conectar a |
|---|---|
| 12V | VBAT · **GND** al GND común · **5V** sin conectar |
| ENA / IN1 / IN2 | GPIO25 / GPIO26 / GPIO27 (**quita el jumper de ENA**) |
| ENB / IN3 / IN4 | GPIO14 / GPIO32 / GPIO4 (**quita el jumper de ENB**) |
| OUT1, OUT2 / OUT3, OUT4 | motor izquierdo / motor derecho |

En este modo el avance se calcula por tiempo:
- Con **`v`** y **`n`** mides la velocidad real (avanza 2 s; divide los mm entre 2) y la pones en
  `MM_S_CRUCERO` y `MM_S_LENTO`.
- `VBAT_REFERENCIA` es el voltaje de la batería cuando mediste.
- Sin encoders **no hay detección de choque**: con el vaso agarrado, el carro confía en el mapa
  que comparten los carros.
