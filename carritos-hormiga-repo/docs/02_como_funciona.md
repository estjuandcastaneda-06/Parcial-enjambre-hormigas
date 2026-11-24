# 2. Cómo funciona el programa

El código está dividido en 12 secciones numeradas. Aquí se explica qué hace cada parte y por qué.

## 2.1 El espacio: una cuadrícula de 6 × 6

```
        x = 0   1   2   3   4   5
y = 0     C → .   .   .   .   .       C = carro en el cuadro (0,0) mirando al Este
    1     .   .   .   .   .   .
    2     .   .   #   .   .   .       # = cuadro ocupado por un objeto que no era su vaso
    3     .   .   .   .   .   .
    4     .   .   .   .   .   .
    5     .   .   .   .   .   .
```

- Cada cuadro mide `CELDA_MM` = 250 mm, así que el espacio completo mide 1,5 m × 1,5 m.
- El carro guarda en qué cuadro está (`miX`, `miY`) y hacia dónde mira (`miDir`: 0 Norte, 1 Este, 2 Sur, 3 Oeste).
- Nunca elige un cuadro fuera del 6 × 6. Así respeta los bordes **siempre que haya arrancado en la esquina correcta**.

**Cómo sabe dónde está:** el carro no tiene encoders ni ve el piso, así que cuenta. Cada vez que avanza un cuadro suma 1 en `x` o en `y`. La distancia recorrida se estima con `velocidad × tiempo` (`MM_S_CRUCERO`) y se corrige según el voltaje de la batería. Los giros de 90° los mide el giroscopio. Si el espacio tiene paredes (`HAY_PAREDES = true`), además mide la distancia a la pared y ajusta cuánto avanza.

## 2.2 Exploración tipo hormiga (feromona)

Cada vez que llega a un cuadro:
1. Toda la feromona del mapa se **evapora** un 2 % (`FEROMONA_EVAPORA`).
2. Deja **1 unidad** de feromona en el cuadro donde está (`FEROMONA_DEPOSITO`).

Para elegir el siguiente cuadro, a cada vecino le da un peso:

```
peso = 1 / (1 + feromona)²
       × 8    si es seguir derecho        (PREFERIR_DERECHO: menos giros, líneas rectas)
       × 0,3  si es devolverse
       = 0    si está fuera del espacio o está ocupado
```

Después escoge uno **al azar, según los pesos**. Así casi siempre va a los cuadros menos visitados, pero sin repetir siempre el mismo recorrido. Es la misma idea de una colonia de hormigas, solo que aquí la feromona *repele* en vez de atraer, porque lo que se busca es explorar.

## 2.3 Máquina de estados

El carro siempre está en uno de estos estados. `decidir()` se ejecuta cada 20 ms y decide si cambia de estado.

```
INICIO ──3 s──> DECIDE ──> GIRA ──> MIRA ──nada enfrente──> AVANZA ──llegó──> DECIDE
                                     │                        │
                                     └──ve algo──> ACERCA <───┘ (ve algo mientras avanza)
                                                     │
                                                  REVISA ──es su vaso──> OBJETIVO (quieto, LED verde)
                                                     │
                                                     └──no es──> RETROCEDE ──> DECIDE
```

| Estado | Qué hace |
|---|---|
| `E_INICIO` | Espera 3 s después de prender |
| `E_DECIDE` | Elige el siguiente cuadro con la feromona |
| `E_GIRA` | Gira hasta el rumbo de ese cuadro, controlado por el giroscopio |
| `E_MIRA` | Mira si hay algo en el cuadro de enfrente (hasta 28 cm) |
| `E_AVANZA` | Avanza un cuadro; si algo aparece adelante, frena |
| `E_ACERCA` | Se acerca despacio hasta quedar a 4 cm del objeto |
| `E_REVISA` | Lee el color 3 veces |
| `E_RETROCEDE` | No era su vaso: vuelve al centro del cuadro |
| `E_OBJETIVO` | Encontró su vaso: motores apagados, LED verde titilando |
| `E_PARADO` | Detenido con la tecla `x` |
| `E_BATERIA` | Batería baja: se detiene |
| `E_SENSOR` | El sensor de distancia no responde: se detiene y lo reintenta cada 2 s |

## 2.4 Movimiento (`lazoControl`, cada 20 ms)

- **Rumbo:** integra el giroscopio (`rumbo += velocidad angular × dt`). Al prender, el carro da un giro corto para averiguar si el MPU6050 quedó montado al revés (`detectarSignoGyro`).
- **Girar:** gira en el sitio con fuerza proporcional a lo que le falta (`KP_GIRO`), hasta quedar a menos de 4° del rumbo.
- **Ir derecho:** si se tuerce, frena un poco una rueda y acelera la otra (`KP_RECTO`).
- **Arranque suave:** el PWM sube de a 30 cada 20 ms (`RAMPA_PWM`), para que el pico de corriente no haga caer la batería.
- **Deriva del giroscopio:** con el carro quieto, corrige poco a poco el error del giroscopio (`gzBias`).

## 2.5 Sensor de distancia (VL53L0X)

- Mide en modo continuo cada 50 ms, sin bloquear el programa.
- Acepta las lecturas "dudosas" (estado 1–3), porque muy cerca de un objeto casi todas salen así. Solo descarta el estado 4, que significa que no hay nada enfrente.
- Usa **la menor de las 2 últimas lecturas**: si vio el vaso en una de las dos, frena.
- Al acercarse, si deja de ver el vaso, calcula cuánto le falta con lo que ya avanzó, y nunca avanza más que la última distancia medida.

## 2.6 Reconocimiento del color

Cada lectura se convierte en 4 números:

```
rojo   = R / (R+G+B)          proporción de cada color: casi no cambia con la distancia
verde  = G / (R+G+B)
azul   = B / (R+G+B)
brillo = (R+G+B) / blanco     qué tan claro es, comparado con la hoja blanca de 'w'
```

Antes de dividir, cada canal se multiplica por `gainRGB`, el **balance de blancos**, para que una hoja blanca dé R = G = B.

La diferencia con cada vaso guardado (`CENT`) es:

```
dif = √( Δrojo² + Δverde² + Δazul² + (0,15 · Δbrillo)² )
```

El carro acepta que es **su** vaso solo si se cumplen tres cosas:
1. su vaso es el más parecido de los 5;
2. `dif ≤ UMBRAL_COLOR` (0,035);
3. `dif ≤ 0,6 × dif del segundo más parecido`, es decir, que se parezca claramente más a su vaso que a cualquier otro.

Además lee **3 veces** y necesita que al menos **2** digan que sí.

Con eso deja de aceptar "cualquier azul": un azul que no se registró queda lejos de todos los vasos (condición 2), y un azul parecido a dos vasos no cumple la condición 3.

## 2.7 Seguridad

- **Sin sensor de distancia no se mueve** (LED azul).
- **Batería baja** (menos de 6,4 V tres veces seguidas): se detiene (LED rojo).
- La batería solo se mide con los motores quietos, porque al arrancar el voltaje cae un momento.
