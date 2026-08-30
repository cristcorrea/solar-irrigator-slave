# ESPECIFICACIÓN — Detección de acople al HUB (GPIO10) en el firmware de la esfera

**Proyecto:** `solar-irrigator-slave` (ESP32-C3, ESP-IDF)
**Alcance:** solo firmware de la esfera. No se modifica hardware ni el proceso de montaje.
**Estado:** ✅ **VALIDADA** — esquemáticos revisados y señal verificada por medida directa en banco.

---

## 1. Problema que se resuelve

El HUB descubre esferas emitiendo por **ESP-NOW un broadcast** `HELLO_ESFERA,<MACHUB>` a `FF:FF:FF:FF:FF:FF`, cada 500 ms, mientras detecta una esfera acoplada físicamente.

Hoy **cualquier** esfera que esté escuchando en ese canal responde `HELLO_HUB,<su MAC>`, esté acoplada o no. El hub valida únicamente que la MAC declarada coincida con la MAC remitente: **no existe ningún vínculo con la presencia física**. Consecuencias reales:

1. El hub puede emparejarse con una esfera que **no** es la que el usuario acopló.
2. La carrera está sesgada **en contra** de la esfera correcta: el hub empieza a emitir ~100 ms tras el acople, pero la esfera acoplada tarda ~3,2 s en tener el callback de ESP-NOW registrado. Durante esos ~3 s la única que puede contestar es una esfera ajena.
3. Una esfera **ya emparejada** también puede ser capturada por otro hub: responde a `HELLO_ESFERA` sin condición, y además `peer_manager_on_data_recv()` sobrescribe la MAC del hub guardada ante **cualquier** mensaje de **cualquier** origen.
4. Una esfera sin emparejar **nunca duerme**: rota canales reiniciándose cada ~5 s indefinidamente, agotando la batería.

**Objetivo:** que la esfera solo participe en el descubrimiento cuando está físicamente acoplada a un hub.

---

## 2. Hechos de hardware verificados

> Confirmados sobre los esquemáticos y por medida en banco. No hace falta volver a deducirlos.

### 2.1 El acople tiene 3 contactos

Del esquemático del HUB, bloque `Connect detection` (componente `U8`, marcado *Add into BOM: no* porque son pads de contacto, no un componente real):

| Pin | Red | Función |
|-----|-----|---------|
| 1 | `VCHAR` | Alimentación hacia la esfera |
| 2 (medio) | `DEV_DETEC` | Detección de presencia. **Pull-up R11 15 kΩ a 3V3 del hub** |
| 3 | `GND` | Masa |

En la esfera, el contacto del medio se **puentea a mano contra GND**. Es un cortocircuito pasivo: tira `DEV_DETEC` a nivel bajo y así el HUB detecta presencia en su `GPIO5`. **La esfera no obtiene ninguna información de ese puente**: su MCU no está conectado a ese pad.

**No modificar este puenteado.** Es una propiedad *fail-safe*: la detección funciona aunque la esfera esté apagada, descargada o con el firmware colgado, y es lo que permite que una esfera con la batería agotada se recargue al acoplarla.

### 2.2 Cadena de alimentación — la señal que sí puede leer la esfera

```
HUB GPIO4 (PS_ENB) ──► EN del load switch LPW5209AB5F (U7)
                       VBUS 5V ──► VIN
                                   VOUT = VCHAR ──► pad Vcc del acople
                                                    ──► VIN1 (esfera)
                                                        ──► R14 10 kΩ ──► VIN1_UC ──► GPIO10
```

**El circuito en la esfera es una única resistencia en serie, NO un divisor:**

```
VIN1 ──[ R14 = 10 kΩ ]── VIN1_UC ── IO10 (pin 16 del ESP32-C3-MINI-1-N4)
```

No hay rama a masa. R14 solo limita corriente; el nivel bajo lo tiene que definir el **pull-down interno** del MCU (ver 4.1).

- `VIN1` lo alimentan **el hub (`VCHAR`)** y **el puerto USB de flasheo** (comparten red).
- **El panel solar entra por `VIN2`**, red distinta, y `D1` bloquea la realimentación hacia `VIN1`.
  **Verificado por medida: con el panel al sol y sin USB, GPIO10 se queda a 0 V.**

**Lógica normal (no invertida):**

| Estado físico | `VIN1` | `GPIO10` |
|---|---|---|
| **Acoplada al hub** (o USB conectado) | presente | **ALTO (~3,3 V)** |
| **Sin acoplar** (incluso con sol) | ausente | **BAJO (~0 V)** |

⇒ En el código: **`acoplada == (gpio_get_level(GPIO_POWER_CONNECTED) == 1)`**

### 2.3 No confundir con `GPIO4` (CHG) — es la resistencia de al lado

```
CHG ──[ R12 = 1 kΩ ]── IO4      (net GPIO_CHARGER_STATUS en el firmware)
```

`CHG` es la salida **open-drain** de estado de carga del BQ24090: se pone a nivel bajo mientras carga, **con cualquier fuente (hub, USB o solar)**, y queda en alto en reposo por el pull-up.

Esto la hace **inútil para detectar acople** y, además, se comporta de forma **inversa y sensible al sol**, justo lo contrario que `VIN1_UC`. Ambas resistencias están físicamente juntas en el PCB (`R14 R12 R15 R11`), así que es fácil confundirlas al medir.

| Condición | `VIN1_UC` (R14 → IO10) | `CHG` (R12 → IO4) |
|---|---|---|
| Nada conectado | **BAJO** | ALTO |
| USB o hub | **ALTO** | BAJO |
| Solo sol | **BAJO** | BAJO |

**Para identificar R14 en el PCB:** con el USB enchufado, es la única del grupo que tiene **~5 V en una de sus patas**. La otra pata es `VIN1_UC`, la que va al GPIO10.

### 2.4 El pin ya existe en el firmware y nunca se lee

En `components/power_manager/power_manager.h`:

```c
#define GPIO_CHARGER_STATUS  GPIO_NUM_4
#define GPIO_POWER_CONNECTED GPIO_NUM_10
```

`gpio_power_init()` (en `power_manager.c`) los configura como entrada **y no se hace `gpio_get_level()` sobre ninguno de los dos en todo el proyecto**. No hay que cablear nada: solo leerlo.

### 2.5 Restricción crítica — GPIO10 no despierta del deep sleep

Confirmado en `soc_caps.h` del ESP32-C3 (ESP-IDF v5.5.5):

```c
#define SOC_GPIO_DEEP_SLEEP_WAKE_VALID_GPIO_MASK   (0ULL | BIT0|BIT1|BIT2|BIT3|BIT4|BIT5)
#define SOC_GPIO_DEEP_SLEEP_WAKE_SUPPORTED_PIN_CNT (6)
```

Solo **GPIO0–GPIO5** pueden despertar de deep sleep. **GPIO10 queda fuera.**

⇒ El diseño debe ser **por sondeo (polling)**, nunca por wakeup de GPIO sobre el pin 10. No intentar `esp_deep_sleep_enable_gpio_wakeup()` con GPIO10: devuelve `ESP_ERR_INVALID_ARG`.

El botón (`BUTTON_GPIO = GPIO_NUM_0`) **sí** es válido como fuente de despertar y ya está configurado como tal en `enter_deep_sleep()`.

### 2.6 Cortar VCHAR no reinicia la esfera

La esfera siempre se alimenta de la batería (`BAT → VBAT1 → SW1 → VBAT2 → AMS1117 → VCC`). `VIN1`/`VCHAR` **solo entra al cargador BQ24090**, no alimenta el micro. Cortar `PS_ENB` desde el hub no reinicia ni apaga la esfera: solo pausa la carga y hace bajar `GPIO10`.

---

## 3. Restricciones — qué NO hacer

1. **No** modificar el puenteado del contacto del medio (`DEV_DETEC`) hacia GND (ver 2.1).
2. **No** usar GPIO10 como fuente de wakeup de deep sleep (ver 2.5).
3. **No** usar `GPIO4`/`CHG` como señal de acople (ver 2.3).
4. **No** alterar el ciclo normal de telemetría, el riego ni el cálculo de deep sleep por máscara de días.
5. **No** tocar el firmware del hub en esta tarea.

---

## 4. Trabajo requerido

### 4.0 Verificación de la señal — COMPLETADA ✅

Medido en la pata de `R14` que va al micro (`VIN1_UC`), en voltios DC, con masa en el **terminal negativo del portapilas** y batería puesta:

| Estado | Lectura medida | Nivel lógico |
|---|---|---|
| **USB conectado** (`VIN1` presente) | **~3,3 V** | **ALTO** |
| **Panel solar al sol, sin USB ni acople** | **~0 V** | **BAJO** |

La segunda medida es la decisiva: **confirma que el panel solar no contamina la señal**, porque entra por `VIN2` y `D1` bloquea la realimentación hacia `VIN1`.

### 4.1 Nuevo accesor en `power_manager`

En `components/power_manager/`, añadir una función pública que devuelva si la esfera está acoplada (o conectada por USB). Nombre sugerido: `power_manager_is_hub_connected(void) → bool`.

Requisitos:

- Lee `GPIO_POWER_CONNECTED` (GPIO10). Devuelve `true` cuando el nivel es **1** (lógica normal).
- **Cambiar la configuración del pin en `gpio_power_init()` a `GPIO_PULLDOWN_ENABLE`.** Hoy tiene ambos pulls deshabilitados, y como **no hay resistencia externa a masa** (ver 2.2), el nivel bajo quedaría indefinido. El pull-down interno (~45 kΩ) no compromete el nivel alto: 5 V a través de R14 de 10 kΩ da de sobra, y el diodo de protección del pin lo limita a ~3,3 V, como confirma la medida.
- **Antirrebote obligatorio**: `VIN1` lleva C3 = 10 µF + C4 = 100 nF y los flancos son lentos. Exigir N lecturas consecutivas coincidentes antes de dar por bueno un cambio de estado (sugerido: 5 lecturas separadas 20 ms ⇒ ventana de ~100 ms).

### 4.2 Regla única de aceptación en `peer_manager`

Fichero: `components/peer_manager/peer_manager.c`

Implementar esta regla en `peer_manager_on_data_recv()`, lo antes posible dentro de la función:

> **Si la esfera NO está acoplada, solo se procesan mensajes cuyo MAC de origen coincida exactamente con la MAC del hub guardada en NVS. Cualquier otro mensaje se descarta con un log de aviso.**
> **Si la esfera SÍ está acoplada, se procesan todos los mensajes (comportamiento actual).**

Esta única regla cubre los tres agujeros a la vez. En concreto, estando **no acoplada**, debe impedir que:

1. **Se sobrescriba la MAC del hub guardada.** Hoy ocurre incondicionalmente en el bloque *"1) Guardar MAC del HUB si no existe o cambió"*, cerca del inicio de `peer_manager_on_data_recv()`. Esa llamada a `peer_manager_save_hub_mac()` debe quedar detrás de la compuerta.
2. **Se responda a `HELLO_ESFERA` con `HELLO_HUB`.** Ocurre en `handle_non_json_payload()`. Conviene añadir ahí una segunda comprobación explícita como defensa en profundidad.
3. **Se acepte configuración de un hub ajeno.**

**Caso que hay que preservar:** una esfera ya emparejada, en su ciclo normal (no acoplada), recibe la configuración de *su* hub. Como el MAC de origen coincide con el guardado, pasa la compuerta y todo sigue igual.

**Semántica adoptada:** *acoplar es el gesto explícito de emparejamiento.* Si se acopla una esfera ya emparejada a un hub distinto, se re-empareja con el nuevo. Es intencionado.

### 4.3 Compuerta del bucle de descubrimiento

Fichero: `main/solar_irrigator_slave.c`

- `wait_hub_first_link_blocking()` **solo debe ejecutarse si la esfera está acoplada**.
- Si `first_boot == true` (no hay MAC de hub o no hay configuración válida) **y no está acoplada**: la esfera **no debe entrar en el bucle de rotación de canal ni reiniciarse**. Debe esperar el acople (ver 4.4) sin emitir ni responder nada por radio.
- Si `first_boot == true` **y sí está acoplada**: comportamiento actual sin cambios (LED rojo parpadeando, timeout de 2 s, `switch_channel_and_reboot()` con rotación 1 → 6 → 11).

  Esto es correcto tal cual: el hub mantiene `PS_ENB` activo mientras `DEV_DETEC` siga a masa, así que `VIN1` se mantiene a través de los reinicios y la esfera vuelve a entrar en descubrimiento tras cada uno.

### 4.4 Sustituir el bucle de reinicios por espera de bajo consumo

Estado actual: una esfera sin emparejar y sin acoplar se reinicia cada ~5 s indefinidamente y **nunca duerme**.

Comportamiento requerido cuando `first_boot == true` y **no** está acoplada:

1. Apagar el LED.
2. Entrar en **deep sleep con despertador por temporizador**, intervalo sugerido **15–30 s** (parámetro en `#define`, no número mágico).
3. Al despertar, leer `power_manager_is_hub_connected()`:
   - Si sigue sin acoplar → volver a dormir sin encender radio ni sensores.
   - Si está acoplada → continuar con el flujo de emparejamiento normal (4.3).
4. Reutilizar la infraestructura de `enter_deep_sleep()` ya existente, respetando el apagado de periféricos (`BOOST_GPIO` a 0, retención de GPIO, wakeup por botón).

Compromiso aceptado: al acoplar, la esfera puede tardar hasta un intervalo completo (15–30 s) en reaccionar. Es irrelevante para una operación manual, y baja el consumo de «despierta el 100 % del tiempo» a ~1 %.

⚠️ El wakeup sigue siendo **por temporizador** (más el botón en GPIO0, que ya existe). No se puede usar GPIO10 como fuente de wakeup (ver 2.5).

### 4.5 Indicación por LED

- No acoplada, esperando: **LED apagado**.
- Acoplada, buscando hub: **LED rojo parpadeando** (comportamiento actual).
- Emparejada correctamente: **animación verde** (actual, `led_manager_start_animation_2(0,20,0)`).

---

## 5. Cambio independiente y muy recomendado: desactivar la ventana AT en producción

Ficheros: `components/test_manager/Kconfig`, `sdkconfig` / perfil de producción.

Hoy `CONFIG_TEST_MANAGER` y `CONFIG_TEST_MANAGER_BOOT_AT_WINDOW` están a `default y`, con `CONFIG_TEST_MANAGER_BOOT_AT_MS = 3000`. Efecto: **cada arranque —incluido cada despertar del deep sleep— instala el driver USB-Serial-JTAG y se queda 3 segundos escuchando comandos AT antes de inicializar Wi-Fi y medir.**

Impacto directo sobre esta especificación:

- Crea el hueco ciego de ~3 s descrito en 1.2 (la esfera acoplada no puede contestar mientras el hub ya emite).
- Duplica el coste de cada rotación de canal (3 s de ventana + 2 s de espera por canal).
- Con el sondeo de 4.4, añadiría 3 s de consumo a **cada** despertar, arruinando el ahorro.

**Acción:** crear un perfil de producción con `CONFIG_TEST_MANAGER_BOOT_AT_WINDOW=n` (o `CONFIG_TEST_MANAGER=n`), manteniendo la CLI AT disponible en el perfil de desarrollo. Es un cambio de configuración, no de código, y mejora los tres problemas a la vez.

---

## 6. Criterios de aceptación

Se necesitan **dos esferas** (A y B) y **un hub**, con los tres monitores serie a la vista.

| # | Escenario | Resultado esperado |
|---|---|---|
| 1 | Esfera A sin emparejar y **sin acoplar**. Se acopla la esfera B al hub. | En el log del hub aparece **únicamente la MAC de B**. La esfera A no debe emitir `HELLO_HUB` ni aparecer en el log del hub. |
| 2 | Esfera A sin emparejar, **acoplada** al hub. | Emparejamiento normal: `HELLO_ESFERA` → `HELLO_HUB` → config + `ts` → `CFG_OK` → `ACK_END` → LED verde → deep sleep. |
| 3 | Esfera A **ya emparejada** al hub 1, despierta y **sin acoplar**. Un hub 2 emite descubrimiento. | La MAC del hub guardada en A **no cambia**. Verificar con el log de arranque (`MAC del hub cargada desde NVS`). |
| 4 | Esfera A emparejada, ciclo normal sin acoplar. | Telemetría, recepción de configuración de **su** hub, sincronización de hora por `ts` y deep sleep: **sin cambios respecto de hoy**. |
| 5 | Esfera A sin emparejar y sin acoplar, dejada 10 minutos. | No se reinicia en bucle. Duerme y despierta según el intervalo definido. Consumo medio muy inferior al actual. |
| 6 | Esfera A sin emparejar y sin acoplar. Se acopla. | Detecta el acople en ≤ 1 intervalo de sondeo y entra en emparejamiento. |
| 7 | **Esfera A sin emparejar, a pleno sol, sin acoplar.** | **No entra en emparejamiento.** `GPIO10` debe leer bajo. Es la comprobación de que el panel no contamina la señal. |
| 8 | Esfera conectada por USB para flashear. | `GPIO10` alto: la esfera cree estar acoplada y entra en descubrimiento. **Es esperado y benigno** (ningún hub responde). No tratarlo como bug. |

---

## 7. Fase 2 — no implementar ahora, solo contexto

Queda un caso sin cubrir: **dos hubs a pocos metros, cada uno con una esfera acoplada**. Ambas esferas están legítimamente en escucha y pueden cruzarse. La solución, viable con este hardware, es un **nonce por la línea de alimentación**:

- El hub pulsa `PS_ENB` (su GPIO4) para emitir un número aleatorio; la esfera lo lee en `GPIO10`.
- La esfera devuelve ese nonce dentro de su `HELLO_HUB`.
- El hub solo registra a quien devuelva su nonce.

Es viable porque:

- `DEV_DETEC` (puenteado a masa) es **independiente** de `PS_ENB`: el hub puede cortar y restaurar `VCHAR` sin que su propia detección de presencia se vea afectada, y sin reiniciar la esfera (ver 2.6).
- `GPIO10` sigue solo a `VIN1`, así que **la modulación es visible también a pleno sol** (ver 2.2).

Requiere medir antes el tiempo de bajada de `VCHAR` al cortar `PS_ENB` (los 10 µF de C3 pueden hacerlo lento), lo que fija la duración de símbolo. Implica tocar el firmware del hub. **Fuera del alcance de esta tarea.**

Alternativa complementaria, también fuera de alcance: **filtro RSSI en el hub**. `esp_now_recv_info_t` expone `rx_ctrl->rssi` (dBm), pero el puntero solo es válido dentro del callback y hoy `espnow_recv_cb()` descarta ese dato.

---

## 8. Resumen para el implementador

1. ~~Verificar la señal~~ **Ya hecho** (4.0): lógica **normal** (alto = acoplada) y **inmune al sol**.
2. Añadir `power_manager_is_hub_connected()` con antirrebote, y activar el **pull-down interno** en GPIO10 (4.1).
3. Compuerta en `peer_manager_on_data_recv()`: sin acople, solo mensajes del hub guardado (4.2).
4. Compuerta del descubrimiento en `app_main()` (4.3).
5. Sustituir el bucle de reinicios por deep sleep + sondeo por temporizador (4.4).
6. Desactivar la ventana AT en el perfil de producción (5).
7. **Leer la sección 9 antes de tocar nada**: riesgos de regresión.
8. Validar con los 8 escenarios de la sección 6.

---

## 9. Riesgos de regresión — leer antes de implementar

Estos puntos no son parte del cambio, son cosas que **el cambio puede romper**. Cada uno indica qué hacer.

### 9.1 `gpio_power_init()` no se ejecuta en modo TEST

`gpio_power_init()` solo se llama en el flujo normal de `app_main()`. En modo TEST (`test_manager_start_cli()`) **no se llama**, así que GPIO10 quedaría sin configurar.

**Acción:** que `power_manager_is_hub_connected()` sea seguro de llamar aunque `gpio_power_init()` no se haya ejecutado (auto-inicialización perezosa, con un flag estático). Si no, cualquier lectura desde el CLI de test daría basura.

### 9.2 NO reordenar `app_main()`

Es tentador comprobar el acople al principio para ahorrar la inicialización de Wi-Fi. **No hacerlo.** El orden actual es:

```
... → Wi-Fi/ESP-NOW init → button_init() → button_erase() → gpio_power_init() → sensores → if (first_boot)
```

`button_erase()` es el **borrado de fábrica manteniendo el botón 8 s al arrancar**. Si la esfera se duerme antes de llegar ahí, **una esfera sin acoplar ya no se podrá resetear nunca**, porque siempre dormirá antes.

**Acción:** poner la comprobación de acople **dentro de la rama `first_boot`**, en su sitio actual. Se paga una inicialización de Wi-Fi innecesaria antes de dormir (unos cientos de ms); es un precio barato frente a reestructurar `app_main()`.

### 9.3 `pm_tx_unlock()` al principio de `peer_manager_on_data_recv()`

La función arranca con `pm_tx_unlock()`, que libera el guard de transmisión. Si la compuerta hace `return` antes de esa llamada, el guard queda tomado hasta que venza su temporizador de 7 s, y se pierden envíos.

**Acción:** dejar `pm_tx_unlock()` **antes** de la compuerta. Liberar de más es inofensivo; no liberar bloquea.

### 9.4 Esfera nueva sin MAC de hub guardada

Si no hay MAC de hub en NVS **y** no está acoplada, la regla de 4.2 no tiene contra qué comparar.

**Acción:** en ese caso, **descartar todos los mensajes**. Es exactamente el comportamiento buscado: una esfera nueva en un cajón no debe responder a nadie.

### 9.5 `cfg_ready_sem` espera con `portMAX_DELAY` (bug preexistente)

En el emparejamiento, tras encontrar el hub, `app_main()` espera la configuración con `xSemaphoreTake(cfg_ready_sem, portMAX_DELAY)`. Si el hub nunca la envía, **la esfera se queda despierta para siempre**, gastando batería.

Hoy es difícil de alcanzar; con los cambios pasa a ser el camino normal de todo emparejamiento fallido.

**Acción:** añadir un timeout (sugerido 30 s). Al vencer: LED apagado y volver a dormir, sin marcar emparejamiento.

### 9.6 Retención de GPIO tras el deep sleep

`enter_deep_sleep()` llama a `gpio_deep_sleep_hold_en()`. Verificar que la retención no deje GPIO10 congelado y que la lectura tras despertar refleje el estado real.

**Acción:** si hace falta, liberar la retención al arrancar (`gpio_hold_dis()` sobre los pines afectados) antes de la primera lectura.

### 9.7 `enter_deep_sleep()` con la radio ya apagada

`enter_deep_sleep()` llama a `esp_now_deinit()` y `esp_wifi_stop()`. Siguiendo 9.2 la radio ya estará inicializada, así que funciona. Pero si en algún momento se añade un camino de sueño anterior a la inicialización de Wi-Fi, esas llamadas devolverán error.

**Acción:** no envolver esas llamadas en `ESP_ERROR_CHECK`. Hoy no lo están; mantenerlo así.

### 9.8 Acoplar una esfera ya emparejada la re-empareja

Con la semántica adoptada («acoplar es el gesto de emparejar»), acoplar una esfera del hub B sobre el hub A la pasa a A. Si el usuario solo quería cargarla, el cambio es silencioso.

**Acción:** es intencionado, pero debe quedar documentado de cara al usuario. Si no se quiere, hay que añadir una condición extra (p. ej. exigir además pulsar el botón).

### 9.9 Lo que NO se debe tocar

- El ciclo normal de telemetría, riego y cálculo de sueño por máscara de días.
- El handshake `CFG_OK` / `ACK_END`.
- La rotación de canal 1 → 6 → 11 **cuando sí está acoplada**.
- El firmware del hub.
