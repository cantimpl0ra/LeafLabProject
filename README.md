- Cal calibrar els llindars d'activació del sensor d'humitat de sòl per a que s'ajusti a les necessitats de cada context (planta, terra, test, posició del sensor, etc).

- La configuració MQTT es fa per serial al arrancar el programa.

- El tòpic que s'utilitza per canviar de fase és: leaflab/CMD/phase; la dada és tipus raw {1,2,3,4}.	

- Els codis "codiPrincipal-versioX" està pensat per muntar el projecte de manera progressiva. És a dir, és recomanable no passar a la versió 2 fins que no et funcioni la versió 1.

===== ESQUEMA DEL PROJECTE =====

A quin actuador fa referència cada botó de la iot carrier.

BOTON 00 = PUMP = RELAY 7 

BOTON 01 = HUMIDIFICADOR = RELAY 1

BOTON 02 = HEAT = RELAY 2

BOTON 03 = IOT RELAY = RELAY 3

== CABLEAJAT ==

POWER = COM

ACATUADOR = NC


===== LLINDARS ACTIVACIÓ =====

| FASE | ETAPA (DURACIÓN) | TEMPERATURA (°C) | HUMEDAD (%) |
| :---: | :--- | :--- | :--- |
| **1** | **Plántula**<br>*(7–10 días)* | **< 20** → Activar calefactor | **> 75** → Activar deshumidificador (IoT relay)<br>**< 65** → Activar humidificador (I2C relay) |
| **2** | **Crecimiento vegetativo**<br>*(2–4 semanas)* | **> 24** → No permitido<br>**< 20** → Activar calefactor | **> 70** → Activar deshumidificador (IoT relay)<br>**< 55** → Activar humidificador (I2C relay) |
| **3** | **Prefloración**<br>*(1–2 semanas)* | **> 26** → Activar refrigerador “Pingüino” (IoT)<br>**< 18** → Activar calefactor | **Siempre activo** → Deshumidificador (IoT)<br>**< 50** → Activar humidificador |
| **4** | **Floración**<br>*(4–7 semanas)* | **> 26** → Activar refrigerador “Pingüino” (IoT)<br>**< 18** → Activar calefactor | **Siempre activo** → Deshumidificador (IoT)<br>**< 40** → Activar humidificador |
