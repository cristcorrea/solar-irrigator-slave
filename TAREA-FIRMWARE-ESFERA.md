> [!WARNING]
> **Documento desactualizado (2026-09-02).** Las tareas C1 a C5 estan implementadas y verificadas en
> el codigo. La prohibicion numero 5 (preservar la rotacion de canal `1 -> 6 -> 11`) esta contradicha
> por un cambio sin commitear en `main/solar_irrigator_slave.c` que barre los 13 canales: hay que
> decidir cual de los dos manda. Estado real: `D:\Firmware\SISTEMA-SMARTGROW.md`.

# TAREA — Compuerta de acople en el firmware de la esfera

**Repositorio a modificar:** `solar-irrigator-slave` (ESP32-C3, ESP-IDF).
**NO tocar:** el firmware del hub (`solar-irrigator-hub`). No hay cambios de hardware.

**Objetivo:** que la esfera solo escuche y responda al descubrimiento por ESP-NOW cuando está físicamente acoplada a un hub. Hoy responde siempre, y el hub puede emparejarse con una esfera equivocada.

**Señal de acople:** `GPIO10` (`GPIO_POWER_CONNECTED`, ya definido en `power_manager.h`).
Lógica **normal**, verificada por medida: **alto (~3,3 V) = acoplada** (o USB conectado); **bajo (~0 V) = no acoplada**. El panel solar **no** la afecta.

---

## C1 — `power_manager`: nuevo accesor de acople

**Ficheros:** `components/power_manager/power_manager.c` y `power_manager.h`

1. En `gpio_power_init()`, en la configuración de `GPIO_POWER_CONNECTED` (GPIO10), cambiar:
   - `.pull_down_en = GPIO_PULLDOWN_DISABLE` → **`GPIO_PULLDOWN_ENABLE`**
   - Dejar `.pull_up_en = GPIO_PULLUP_DISABLE`.
   - Motivo: en la placa hay solo una resistencia serie (R14, 10 kΩ) desde VIN1; **no hay resistencia a masa**, así que el nivel bajo hay que definirlo con el pull-down interno.
2. **No modificar** la configuración de `GPIO_CHARGER_STATUS` (GPIO4).
3. Añadir `#include <stdbool.h>` en `power_manager.h` (hoy la cabecera no tiene ningún include).
4. Añadir **dos** funciones públicas:

```c
bool power_manager_is_hub_connected(void);         // NO bloqueante
bool power_manager_is_hub_connected_stable(void);  // con antirrebote, bloquea ~100 ms
```

**`power_manager_is_hub_connected()` — no bloqueante**

- Una sola lectura: `gpio_get_level(GPIO_POWER_CONNECTED) == 1`. Sin retardos.
- Es la que se usa en **C2**, porque se invoca desde el callback de recepción de ESP-NOW, que corre en la tarea de Wi-Fi: **bloquearla puede provocar pérdida de paquetes**.
- No hace falta antirrebote ahí: la red `VIN1` ya está filtrada por C3 (10 µF) y C4 (100 nF).

**`power_manager_is_hub_connected_stable()` — con antirrebote**

- 5 lecturas con 20 ms de espera entre ellas (4 esperas, ~80 ms de bloqueo). Solo acepta un cambio de estado si las 5 coinciden; si no, conserva el último estado estable.
- **Estado inicial: `false`.** Si el primer conjunto no coincide, se asume no acoplada. Es la opción segura: evita emparejamientos accidentales.
- Es la que se usa en **C3**, desde `app_main()`, donde bloquear ~80 ms es irrelevante.
- **C4 no la usa**: en el camino de timeout no hace falta volver a consultar GPIO10, solo apagar el LED y dormir.

**Auto-inicialización perezosa (ambas):** con un flag estático, si `gpio_power_init()` no se ha ejecutado todavía, configurar el pin en la primera llamada. Necesario porque en modo TEST `gpio_power_init()` nunca se llama.
**El flag debe marcarse también dentro de `gpio_power_init()`**, no solo en el accesor, para que no se reconfigure dos veces.

---

## C2 — `peer_manager`: compuerta de aceptación de mensajes

**Ficheros:** `components/peer_manager/peer_manager.c` y `components/peer_manager/CMakeLists.txt`
**Función:** `peer_manager_on_data_recv()`

Antes de nada, añadir la dependencia. Hoy `CMakeLists.txt` no la tiene y la compilación fallaría:

```cmake
PRIV_REQUIRES led_manager CJSON time_sync power_manager
```

E incluir `power_manager.h` en `peer_manager.c`.

Insertar la compuerta **justo después de la llamada existente a `pm_tx_unlock()`** y **antes** del bloque comentado `/* 1) Guardar MAC del HUB si no existe o cambió */`.

Pseudocódigo exacto:

```c
if (!power_manager_is_hub_connected()) {
    uint8_t stored[6];
    if (peer_manager_load_hub_mac(stored) != 1) {
        ESP_LOGW(TAG, "Sin acople y sin hub guardado: mensaje descartado");
        return;
    }
    if (memcmp(stored, recv_info->src_addr, 6) != 0) {
        ESP_LOGW(TAG, "Sin acople: mensaje de hub ajeno descartado");
        return;
    }
}
```

⚠️ `pm_tx_unlock()` debe quedar **antes** de este bloque. Si se hace `return` sin haberlo llamado, el guard de transmisión queda tomado 7 s y se pierden envíos.

Además, en `handle_non_json_payload()`, en la rama que responde a `HELLO_ESFERA`: **no responder** si `power_manager_is_hub_connected()` es `false`. Es defensa en profundidad; la compuerta anterior ya debería haberlo filtrado.

**Efecto esperado:** estando **no acoplada**, la esfera solo procesa mensajes de su hub guardado. Estando **acoplada**, procesa todo (comportamiento actual, sin cambios).

---

## C3 — `app_main`: compuerta del descubrimiento

**Fichero:** `main/solar_irrigator_slave.c`
**Ubicación:** dentro de la rama `if (first_boot) { ... }` existente, **al principio**.

```c
#define DOCK_POLL_INTERVAL_S 20   // parámetro, no número mágico

if (first_boot) {
    if (!power_manager_is_hub_connected_stable()) {   // versión CON antirrebote
        led_manager_set_rgb(0, 0, 0);
        enter_deep_sleep((uint64_t)DOCK_POLL_INTERVAL_S * 1000000ULL);
        return;
    }
    // ... comportamiento actual sin cambios ...
}
```

⚠️ **No mover esta comprobación más arriba en `app_main()`.** Tiene que ejecutarse después de `button_init()` y `button_erase()`. Si la esfera se duerme antes, se pierde el borrado de fábrica manteniendo el botón 8 s al arrancar, y una esfera sin acoplar **no se podría resetear nunca**.

Se paga una inicialización de Wi-Fi innecesaria antes de dormir. Es aceptable: no reestructurar `app_main()`.

---

## C4 — Timeout en la espera de configuración

**Fichero:** `main/solar_irrigator_slave.c`, misma rama `first_boot`.

Cambiar la espera del semáforo de configuración:

- De: `xSemaphoreTake(cfg_ready_sem, portMAX_DELAY)`
- A: `xSemaphoreTake(cfg_ready_sem, pdMS_TO_TICKS(30000))`

Si vence el timeout: apagar el LED y entrar en deep sleep con `DOCK_POLL_INTERVAL_S`. **No** marcar la esfera como emparejada.

Motivo: hoy, si el hub no envía la configuración, la esfera se queda despierta indefinidamente. Con estos cambios ese sería el camino de todo emparejamiento fallido.

---

## C5 — Perfil de producción sin CLI de test

**Fichero:** `sdkconfig` de producción (o `sdkconfig.defaults`).

- `CONFIG_TEST_MANAGER=n`

Alternativa si se quiere conservar la CLI: `CONFIG_TEST_MANAGER_BOOT_AT_WINDOW=n`.

Motivo: hoy cada arranque —incluido cada despertar del deep sleep— gasta **3 segundos** escuchando comandos AT antes de inicializar la radio. Es cambio de configuración, no de código.

---

## Prohibiciones

1. No reordenar `app_main()` (ver C3).
2. No llamar a `esp_deep_sleep_enable_gpio_wakeup()` con GPIO10: **no es válido**. En el ESP32-C3 solo GPIO0–GPIO5 pueden despertar del deep sleep. El despertar sigue siendo por temporizador, más el botón en GPIO0 que ya está configurado.
3. No usar `GPIO4` (`GPIO_CHARGER_STATUS`) como señal de acople: es el estado de carga del BQ24090, se activa también con el panel solar y tiene lógica inversa.
4. No envolver `esp_now_deinit()` ni `esp_wifi_stop()` de `enter_deep_sleep()` en `ESP_ERROR_CHECK`.
5. No modificar: el ciclo normal de telemetría, la lógica de riego, el cálculo de sueño por máscara de días, el handshake `CFG_OK` / `ACK_END`, ni la rotación de canal 1 → 6 → 11 cuando **sí** está acoplada.
6. No tocar el repositorio del hub.

---

## Alcance del trabajo

**Lo que debe hacer quien implemente:**

1. Aplicar C1 a C5.
2. Compilar para ESP32-C3 y dejar la compilación limpia.
3. Confirmar que no se tocó ningún fichero fuera de los autorizados.
4. Entregar y parar.

**Lo que NO debe intentar:** ejecutar o validar los escenarios de la tabla siguiente. Requieren hardware
real (dos esferas, un hub, un panel solar al sol, un multímetro) y los ejecuta una persona. Una
compilación limpia solo demuestra integración y sintaxis, no comportamiento.

---

## Verificación en hardware (la ejecuta una persona, no la IA)

Dos esferas (A y B) y un hub, con los monitores serie a la vista.

| # | Escenario | Resultado esperado |
|---|---|---|
| 1 | A sin acoplar; se acopla B | El hub solo registra la MAC de **B**. A no debe emitir `HELLO_HUB` |
| 2 | A acoplada, sin emparejar | Empareja normal: LED verde y deep sleep |
| 3 | A emparejada al hub 1, sin acoplar; el hub 2 emite | La MAC guardada en A **no cambia** |
| 4 | A emparejada, ciclo normal sin acoplar | Telemetría y configuración **igual que hoy** |
| 5 | A sin acoplar, 10 minutos | No se reinicia en bucle; duerme y despierta cada `DOCK_POLL_INTERVAL_S` |
| 6 | A sin acoplar → se acopla | Reacciona en ≤ 1 intervalo de sondeo |
| 7 | A a pleno sol, sin acoplar | **No empareja.** GPIO10 debe leer bajo |
| 8 | A conectada por USB | GPIO10 alto: cree estar acoplada y busca hub. **Es correcto, no es bug** |

El escenario **7** es la regresión crítica a vigilar: confirma que el panel solar no dispara el emparejamiento.

---

## Contexto adicional

El fondo técnico (esquemáticos, medidas, por qué GPIO10 y no otro pin) está en `SPEC-DETECCION-ACOPLE.md`, en este mismo repositorio. No hace falta leerlo para implementar esta tarea.
