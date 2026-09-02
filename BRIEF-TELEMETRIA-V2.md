# Brief — Telemetría v2, energía y recuperación de canal

> Destinatario: agente de firmware de la ESFERA (`solar-irrigator-slave`, ESP32-C3).
> Contraparte: `solar-irrigator-hub/BRIEF-TELEMETRIA-V2-HUB.md`, que se implementa en paralelo.
> **El contrato de la sección 1 es vinculante**: los dos lados se implementan contra él, carácter a
> carácter.
> Contexto del sistema completo: `D:\Firmware\SISTEMA-SMARTGROW.md`.
>
> Como en los briefs anteriores: **no se flashea**. Entregá el código compilado y el reporte.
>
> Nota: `SPEC-DETECCION-ACOPLE.md` y `TAREA-FIRMWARE-ESFERA.md` están **obsoletos**; no los uses como
> referencia. En particular su prohibición de tocar la rotación de canal queda derogada por este brief.

---

## Por qué este trabajo

Tres agujeros que hoy hacen que el sistema no se pueda diagnosticar ni operar en producción:

1. **El campo `riego` de la telemetría siempre vale 0.** `irrigation_done` se declara `false` en
   [main/solar_irrigator_slave.c:604](main/solar_irrigator_slave.c:604) y nunca se actualiza tras
   regar. Ni el hub ni la app pueden saber si una esfera regó. Hay una alerta de fallo de riego ya
   escrita en la app que por esto nunca puede resolverse.
2. **Los ml realmente entregados se pierden.** El caudalímetro los cuenta
   (`ML_PER_PULSE = 0.00545`, `pump_controller.c:13`) pero `pump_controller_irrigate(int ml)`
   devuelve `void` y el dato solo sale por el log serie. La app descuenta el depósito con los ml
   *programados*, no con los reales.
3. **Si el AHT20 falla no se envía nada**, ni siquiera el VBAT, que sí es válido
   ([main:679-682](main/solar_irrigator_slave.c:679)). Desde el hub, una esfera con el sensor roto es
   indistinguible de una muerta o de una fuera de cobertura.

---

## 1. Contrato de telemetría v2 (vinculante)

### Formato

```
hum,temp,vbat,riego,ml,st MACESFERA
```

Ejemplo real: `45.6,23.4,4.12,1,148,17 A0B1C2D3E4F5`

| Campo | Formato | Rango | Significado |
|---|---|---|---|
| `hum` | `%.1f` | 0.0 … 100.0 | % HR del aire. **0.0 si el AHT20 falló** |
| `temp` | `%.1f` | -50.0 … 100.0 | °C. **0.0 si el AHT20 falló** |
| `vbat` | `%.2f` | 0.00 … 20.00 | Volts de batería |
| `riego` | `%d` | 0 o 1 | 1 = se **intentó** un riego en este ciclo |
| `ml` | `%d` | 0 … 65535 | ml realmente entregados según el caudalímetro. 0 si no hubo riego |
| `st` | `%d` | 0 … 255 | Mapa de bits, tabla siguiente |

Separador entre `st` y la MAC: **un espacio**, igual que hoy. La MAC son 12 hex mayúsculas sin
separadores. Se sigue enviando sin terminador nulo (`strlen(payload)` bytes).

### Mapa de bits de `st`

| Bit | Valor | Significado |
|---|---|---|
| 0 | 1 | Lectura del AHT20 válida. **0 = sensor roto o fuera de rango** |
| 1 | 2 | El riego se cortó por falta de caudal (timeout de 5 s sin pulsos) |
| 2 | 4 | Batería por debajo del umbral de aviso |
| 3 | 8 | Riego omitido por batería baja |
| 4 | 16 | La esfera tiene hora válida (`time_sync_is_valid()`) |
| 5-7 | — | Reservados, siempre 0 |

`st = 17` (bits 0 y 4) es el valor de una esfera sana que no regó en este ciclo.

### Regla no negociable

**Siempre se envía telemetría.** El guarda `if (!sensors_ok)` que hoy omite el envío se elimina: si
el AHT20 falla se envía igual, con `hum=0.0`, `temp=0.0` y el bit 0 de `st` apagado. El VBAT y el
estado son datos válidos y son justamente los que hacen falta para diagnosticar en campo.

El buffer `char payload[64]` sigue alcanzando: el peor caso
(`100.0,100.0,20.00,1,65535,31 AABBCCDDEEFF`) son 41 caracteres.

---

## 2. ml reales del caudalímetro

`pump_controller_irrigate` cambia de firma:

```c
/* Devuelve los ml realmente entregados. `cut_by_flow` queda a true si se cortó
   por el timeout de 5 s sin pulsos. */
uint16_t pump_controller_irrigate(int ml, bool *cut_by_flow);
```

El cálculo ya existe dentro de la función (`pulse_count` y `ML_PER_PULSE`); hoy solo se imprime en el
log de `pump_controller.c:158-162`. Se trata de devolverlo en vez de descartarlo, saturando a 65535.

En el sitio de llamada ([main:667](main/solar_irrigator_slave.c:667)):

```c
bool cut_by_flow = false;
uint16_t ml_real = pump_controller_irrigate((int)cfg_ml, &cut_by_flow);
irrigation_done = true;            /* se intentó regar, independientemente del resultado */
if (cut_by_flow) st |= ST_FLOW_CUT;
```

Las cuatro condiciones de corte actuales (objetivo alcanzado, parada manual, timeout de caudal de
5 s, timeout absoluto de 10 min) no se tocan.

---

## 3. Umbrales de batería

Batería Li-ion 1S. Los umbrales viven **en el firmware, en volts**, no en la app: en modo offline el
teléfono puede pasar días sin conectarse, así que una decisión tomada allá no puede parar una bomba.

| Constante | Valor | Efecto |
|---|---|---|
| `VBAT_WARN_V` | **3.55 V** | Se enciende el bit 2 de `st`. No cambia el comportamiento |
| `VBAT_STOP_V` | **3.35 V** | **No se riega.** Se enciende el bit 3 de `st`. La telemetría se envía igual |
| `VBAT_RESUME_V` | **3.45 V** | Histéresis: por debajo de `STOP` no vuelve a regar hasta superar este valor |

Están separados 0,20 V a propósito. El ADC del ESP32-C3 con calibración por ajuste de curva tiene un
error del orden de ±2-3 %, que a 3,4 V son ~±0,1 V: umbrales más juntos oscilarían.

**Cómo medir:**

- **Antes de arrancar la bomba**, nunca durante ni justo después: la tensión se hunde bajo carga y
  daría un falso corte.
- **Mediana de 5 lecturas** consecutivas de `power_manager_get_battery_level()`, descartando los
  `-1.0` de error. Si las 5 fallan, no se toma ninguna decisión de corte (se riega normalmente) y se
  reporta `vbat = 0.00`.

El estado de histéresis se guarda en **RTC memory** (`RTC_DATA_ATTR`), no en NVS: sobrevive al deep
sleep, que es lo único que hace falta, y no desgasta la flash.

---

## 4. El bucle de LED no puede matar la batería

Hoy, si el semáforo de configuración llega pero la NVS no tiene una config válida, el firmware entra
en `for(;;)` parpadeando en rojo ([main:560-566](main/solar_irrigator_slave.c:560)) **con la CPU y la
radio encendidas y sin dormir nunca**. La esfera agota la batería en horas y después ni siquiera
parpadea, con lo cual el usuario pierde justamente la señal que ese bucle quería darle.

La señal visual se mantiene, el consumo no:

```
parpadeo rojo 250 ms on / 750 ms off durante 30 s
→ deep sleep de DOCK_POLL_INTERVAL_S (20 s)
→ al despertar se vuelve a entrar por el mismo camino
```

Visualmente es idéntico para el usuario, dura semanas en vez de horas, y como la esfera despierta
cada 20 s se recupera sola en cuanto se la acople al hub. No hace falta lógica extra: al despertar,
`app_main` recalcula `first_boot` y vuelve a pasar por la compuerta de acople.

Este comportamiento es intencional y se documenta en el manual de usuario: **parpadeo rojo lento =
esfera sin configurar**.

---

## 5. Recuperación de canal

### El problema

En modo online el canal de radio del hub lo impone el router, no el hub. Si el router cambia de
canal, las esferas quedan mudas para siempre: el reintento con cambio de canal existe pero está
**comentado** ([main:265](main/solar_irrigator_slave.c:265)).

### Por qué no basta con descomentarlo

`switch_channel_and_reboot()` escribe NVS y hace `esp_restart()`. En el ciclo normal, tras el
reinicio `first_boot` vale `false` (hay config y hay `hub_mac`), así que la esfera **salta la
compuerta de acople**, entra directo al ciclo normal, vuelve a fallar el envío y reinicia otra vez:
un bucle de reinicios que **nunca llega al deep sleep**. Con el hub apagado, eso vacía la batería en
horas. Además, un canal por ciclo significaría hasta 13 horas para reengancharse, y una escritura de
NVS por salto.

### Diseño a implementar

Dos contadores en RTC memory, que sobreviven al deep sleep:

```c
RTC_DATA_ATTR static uint8_t s_tx_fail_streak;   /* ciclos consecutivos sin poder entregar */
RTC_DATA_ATTR static uint8_t s_sweep_fail_count; /* barridos completos fallidos seguidos */
```

- **Envío correcto** ⇒ los dos contadores a 0.
- **Envío fallido** (los 3 reintentos de capa MAC) ⇒ `s_tx_fail_streak++`. Se duerme normalmente.
- **`s_tx_fail_streak >= 3`** ⇒ barrido **dentro del mismo despertar, sin reiniciar**:

```
para ch = canal_actual+1 … 13, 1 … canal_actual:
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE)
    reenviar la misma telemetría
    esperar el send_cb hasta 250 ms
    si hubo éxito: persistir ch en NVS, contadores a 0, salir
```

Coste del barrido completo: 13 × 250 ms ≈ **3,5 s** de radio adicional, una vez, en lugar de 13
horas.

- **Barrido completo fallido** ⇒ `s_sweep_fail_count++`, `s_tx_fail_streak = 0`.
- **`s_sweep_fail_count >= 5`** ⇒ dejar de barrer en cada ciclo y probar **una vez cada 6
  despertares**, para no gastar batería contra un hub que sencillamente está apagado.

**Prohibido `esp_restart()` en el camino del ciclo normal.** `switch_channel_and_reboot()` sigue
existiendo y sigue usándose **solo** en el emparejamiento (`wait_hub_first_link_blocking`), donde el
reinicio es correcto porque no hay estado que perder.

### Sobre el cambio sin commitear

El working tree ya trae la rotación de 1→2→…→13 en el camino de emparejamiento
([main:71-91](main/solar_irrigator_slave.c:71)). **Se conserva**: es la correcta, porque el router
puede haber puesto al hub en cualquier canal, no solo en 1, 6 u 11. Commitealo junto con este
trabajo.

---

## Fuera del alcance de este brief

- **Riego bajo demanda desde la app** ("regar ahora" / "cancelar riego"). Choca con el deep sleep: la
  esfera solo escucha 2,5 s por hora. Es una funcionalidad nueva que necesita que el hub encole la
  orden y la entregue en el próximo contacto. Va en un brief aparte.
- **Cifrado ESP-NOW** (PMK/LMK). No hay ninguno hoy en ninguno de los dos lados.
- **OTA**, en cualquier forma.
- El boost de la bomba (GPIO2) queda como está: es una decisión de hardware ya documentada.

---

## Qué reportar

1. Build limpio y tamaño del binario.
2. Cadena de telemetría real capturada en el log, en los cuatro casos: ciclo normal sin riego, ciclo
   con riego completo, riego cortado por falta de caudal, y AHT20 desconectado.
3. Consumo medido del barrido de canales completo, y confirmación de que **no hay ningún
   `esp_restart()` en el ciclo normal**.
4. Comportamiento con la batería por debajo de 3,35 V: que no riegue, que igual reporte, y que la
   histéresis no oscile.
5. Que el bucle de LED rojo duerme y que la esfera se recupera sola al acoplarla.
6. Plan de prueba: AHT20 desconectado en caliente, hub apagado durante 10 ciclos, cambio de canal del
   router con las esferas ya emparejadas, y batería bajando por debajo de los dos umbrales.
