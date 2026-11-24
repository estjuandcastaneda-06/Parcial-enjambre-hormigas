# 4. Problemas que tuvimos y cómo se resolvieron

Este es el registro de las fallas que salieron al armar y probar el carro. Sirve para no repetirlas y para diagnosticar más rápido.

| # | Síntoma | Causa | Solución |
|---|---|---|---|
| 1 | Los motores zumban pero no se mueven | El L298N pierde 2–3 V y el PWM estaba muy bajo | Subir el PWM: `PWM_CRUCERO` 230, giros entre 185 y 240 |
| 2 | Con los motores andando, las pilas caían de 7,3 V a 6,2 V y menos | Pilas con poca corriente | Cambiar a una batería que dé más corriente |
| 3 | En el switch se perdía 1,1 V con los motores andando | Switch malo, que no aguanta la corriente | Cambiar el switch, o puentearlo para probar |
| 4 | Decía "batería baja (5,27 V)" con las pilas en 7,2 V | Con resistencias de 100k/47k, el ADC del ESP32 lee de menos | Corrección `VBAT_CAL = 1,36` y un condensador de 100 nF en GPIO33 |
| 5 | El carro solo daba vueltas en círculo | El giroscopio medía el giro al revés, así que el carro "corregía" hacia el lado equivocado | Al prender, el carro hace una prueba de giro y averigua solo el sentido; también se montó el MPU6050 plano |
| 6 | Giraba a cada rato en vez de ir derecho | La exploración no prefería seguir derecho | `PREFERIR_DERECHO = 8` |
| 7 | Se estrellaba con todo | Frenaba solo a 4 cm, y el código descartaba las lecturas "dudosas" del VL53L0X | Aceptar las lecturas con estado 1–3, revisar todo el cuadro de enfrente (28 cm) y frenar a 8 cm |
| 8 | El VL53L0X llegaba solo a 30–50 cm | Señal débil: película protectora, cinta encima del chip o superficie oscura | Quitar lo que tape el chip; el código usa la menor de las 2 últimas lecturas para no perder los vasos |
| 9 | Frenaba frente a un objeto pero nunca reconocía el color | El sensor de distancia y el de color estaban en esquinas opuestas | Montar los dos juntos, en el centro del frente |
| 10 | Se iba torciendo de la cuadrícula | Después de cada giro se ignoraba el error que quedaba, hasta 4° por giro | El rumbo ya no se "redondea" después de girar |
| 11 | No respetaba el espacio 6 × 6 | Distancia por tiempo sin calibrar; la batería cambia la velocidad | Calibrar con `a` y `+`/`-`, compensar el voltaje de la batería (`VBAT_VEL`) y, si es posible, poner paredes (`HAY_PAREDES`) |
| 12 | Aceptaba "cualquier azul" como su vaso | Solo comparaba el tono, con un umbral amplio | Comparar también el brillo, usar un umbral más estricto y exigir que su vaso sea claramente el más parecido |
| 13 | "TCS34725 no responde" | Mismo bus I2C que el VL53L0X, o pines sin soldar | Bus propio en los pines 18/19, soldar los pines y revisar con `i` |

## Cómo diagnosticar rápido

1. **¿Qué dice al prender?** Busca los mensajes `OK` de cada sensor.
2. **`i`:** muestra qué sensores I2C responden.
3. **`m`:** muestra si los motores giran hacia el lado correcto.
4. **`f`:** muestra si el sensor de distancia ve los objetos.
5. **`t`:** muestra a qué vaso se parece lo que tiene enfrente, y por qué lo acepta o lo rechaza.
6. **`d`:** muestra dónde cree el carro que está y qué cuadros ha visitado.
