
- The activation thresholds of the soil moisture sensor must be calibrated so that they adapt to the needs of each context (plant, soil, pot, sensor position, etc).

- MQTT configuration is done via serial when the program starts.

- The topic used to change phase is: leaflab/CMD/phase; the data type is raw {1,2,3,4}.

- The codes "mainCode-versionX" are intended to assemble the project progressively. That is, it is recommended not to move on to version 2 until version 1 is working correctly.

===== PROJECT SCHEME =====

Which actuator each button on the IoT carrier refers to.

BUTTON 00 = PUMP = RELAY 7

BUTTON 01 = HUMIDIFIER = RELAY 1

BUTTON 02 = HEAT = RELAY 2

BUTTON 03 = IOT RELAY = RELAY 3

== WIRING ==

POWER = COM

ACTUATOR = NC


===== ACTIVATION THRESHOLDS =====

PHASE | DURATION                  | TEMPERATURE (°C)                                     | HUMIDITY (%)                                          |
------|---------------------------|------------------------------------------------------|-------------------------------------------------------|
1     | Seedling (7–10 days)      | < 20  → Activate heater                              | > 75 → Activate dehumidifier (IoT relay)              |
      |                           |                                                      | < 65 → Activate humidifier (I2C relay)                |
------|---------------------------|------------------------------------------------------|-------------------------------------------------------|
2     | Vegetative growth         | > 24 → Not allowed                                   | > 70 → Activate dehumidifier (IoT relay)              |
      | (2–4 weeks)               | < 20 → Activate heater                               | < 55 → Activate humidifier (I2C relay)                |
------|---------------------------|------------------------------------------------------|-------------------------------------------------------|
3     | Pre-flowering             | > 26 → Activate “Penguin” cooler (IoT)               | Dehumidifier always active (IoT always on)            |
      | (1–2 weeks)               | < 18 → Activate heater                               | < 50 → Activate humidifier                            |
------|---------------------------|------------------------------------------------------|-------------------------------------------------------|
4     | Flowering                 | > 26 → Activate “Penguin” cooler (IoT)               | Dehumidifier always active (IoT always on)            |
      | (4–7 weeks)               | < 18 → Activate heater                               | < 40 → Activate humidifier                            |
