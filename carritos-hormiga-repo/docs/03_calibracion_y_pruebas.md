# 3. Calibración y pruebas, paso a paso

Hazlo **en cada carro**, con la **batería cargada**, el **switch prendido** y el carro **en el piso del espacio real**, con la luz de la presentación.

## Paso 1 — Revisar que todo responde

Sube el código, abre el monitor serie a 115200 y presiona EN/RST en el ESP32. Deben salir estas líneas:

```
TCS3200 OK                (o TCS34725 OK)
MPU6050 OK
Giroscopio: giró 30°, sentido = 1 (listo)
Listo en el cuadro (0,0) mirando E. Arranco en 3 s.
```

Escribe `x` para dejarlo parado mientras calibras.

- Si algún sensor no responde, escribe `i` y revisa [01_conexiones.md](01_conexiones.md).
- Si sale `Giroscopio: casi no giró`, el switch estaba apagado o las ruedas no tocaban el piso. El carro lo vuelve a probar cuando escribes `x` para arrancar.

## Paso 2 — Motores

Escribe `m`. Primero gira la rueda **izquierda** y después la **derecha**, y las dos deben ir **hacia adelante**.

- Si una va hacia atrás, intercambia los 2 cables de ese motor en el L298N.
- Si la primera que gira es la derecha, intercambia OUT1/OUT2 con OUT3/OUT4.

## Paso 3 — Sensor de distancia

Escribe `f` y acerca y aleja un vaso entre 5 y 30 cm. La distancia debe cambiar, y al final debe decir "El sensor funciona".

## Paso 4 — Largo de cada cuadro (que respete el 6 × 6)

1. Marca en el piso una línea de inicio y otra a 25 cm.
2. Pon el carro con su centro sobre la línea de inicio y escribe `a`. El carro avanza un cuadro.
3. Mide hasta dónde llegó:
   - **Se pasó de 25 cm:** escribe `+`.
   - **Se quedó corto:** escribe `-`.
   - Cada tecla cambia el avance más o menos un 5 %.
4. Repite hasta que avance 25 cm ±1 cm.
5. Escribe `p` y copia las líneas `MM_S_CRUCERO`, `MM_S_LENTO` y `VBAT_VEL` en la **sección 3** del código.

Hay otra opción: pon el carro mirando de frente una pared blanca a 40–50 cm y escribe `c`. El carro mide su velocidad solo.

## Paso 5 — Colores

El color hay que calibrarlo a la **misma distancia a la que el carro frena**: unos 4 cm del sensor de distancia, o sea, 2 cm del sensor de color.

1. Pon una **hoja blanca** frente al sensor de color, a esa distancia, y escribe `w`.
2. Pon el **vaso 1** enfrente y escribe `1`. Haz lo mismo con los vasos 2, 3, 4 y 5.
3. Escribe `p`. Copia las líneas `CENT`, `gainRGB` y `BLANCO_SUMA` en la **sección 4** del código y vuelve a subirlo.
4. Si `p` dice **"OJO: los vasos X y Y se parecen mucho"**, el carro va a confundir esos dos vasos. Calibra más cerca, mejora la luz o cambia uno de los vasos por otro color.

**Comprobar:** pon cada vaso enfrente y escribe `t`. Debe decir:

```
se parece más al vaso 3 (dif 0.012; el 2.º más parecido 0.080) -> ES MI VASO
```

Esto si es el vaso que busca ese carro. Para los otros vasos debe decir "no es mi vaso". Prueba también con un objeto que **no** es un vaso, por ejemplo otra cosa azul: debe decir "no se parece lo suficiente a ninguno".

**Si es muy exigente o muy flojo**, ajusta en la sección 4:

| Problema | Qué cambiar |
|---|---|
| No reconoce su propio vaso | Sube `UMBRAL_COLOR` (0,035 → 0,05) |
| Acepta objetos que no son su vaso | Baja `UMBRAL_COLOR` (0,035 → 0,025) o `MARGEN_COLOR` (0,6 → 0,5) |
| Confunde azul claro con azul oscuro | Sube `PESO_BRILLO` (0,15 → 0,3) |

## Paso 6 — Elegir qué vaso busca

En la sección 1, pon `VASO_OBJETIVO` con el número del vaso de ese carro (1..5) y súbelo.

## Paso 7 — A buscar

1. Pon el carro en el **cuadro (0,0)**, la esquina de arriba a la izquierda, **mirando al Este**, o sea, a lo largo del borde de arriba.
2. Escribe `r` para reiniciar el mapa y luego `x` para arrancar.
3. Con `d` ves el mapa mientras busca.

Si quieres arrancar en otra esquina o en otra dirección, cambia `INICIO_X`, `INICIO_Y` e `INICIO_DIR` en la sección 1.

## Paso opcional — Paredes

Pon bordes de cartón de unos 10 cm de alto alrededor del espacio y cambia `HAY_PAREDES` a `true` en la sección 4. Cuando el carro mira hacia una pared que tiene a menos de unos 50 cm, compara lo que mide con lo que debería medir y corrige cuánto avanza. Así no se va saliendo del espacio poco a poco.
