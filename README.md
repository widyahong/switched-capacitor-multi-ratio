# Switched-Capacitor (Charge Pump) Multi Ratio (x1/2, x1, x2) with BJT Push-Pull Drivers

Open-loop, single flying capacitor. All switches are BJTs: wide, flexible supply
range, slower and less efficient than the MOSFET version.

---

## 1. Universal concept

### 1.1 Power stage

```
                    Vin
          +----------+-----------+
          |                      |
        [S1]                   [S3]
          |                      |
          A---------| |----------B----[S4]---- GND
          |       Cfly           |
        [S2a]                  [S2b]
          |                      |
          +----------+-----------+
                     |
                    Vout ---+--- Rload
                            |
                          Cout
                            |
                           GND
```

Cfly: `+` at node A. Cout: `+` at Vout.

| Switch | Between   | Device                                                    |
|--------|-----------|-----------------------------------------------------------|
| S1     | Vin - A   | 2x 2N3906 back-to-back (emitters tied, bases tied)        |
| S2a    | A - Vout  | 2x 2N3906 back-to-back (emitters tied, bases tied)        |
| S2b    | B - Vout  | 1x 2N3906 (emitter at Vout, collector at B)               |
| S3     | Vin - B   | 1x 2N3906 (emitter at Vin, collector at B)                |
| S4     | B - GND   | 1x 2N3904 (collector at B), GPIO-driven through 1k        |

Back-to-back = two PNP with **emitters tied together**, collectors to the two
terminals, bases tied. It blocks voltage in both directions. S1 and S2a see both
polarities, S2b, S3 and S4 do not.

### 1.2 Phases

| Mode        | Phase 1 ON   | Phase 2 ON   | Ideal Vout |
|-------------|--------------|--------------|------------|
| x2 (boost)  | S1, S4       | S3, S2a      | 2 x Vin    |
| x1/2 (buck) | S1, S2b      | S2a, S4      | Vin / 2    |
| x1 (bypass) | S1, S2a (static, no switching) | -  | Vin |

```
 |<- P1 (t1) ->|dead|<- P2 (t2) ->|dead|        period = 1 / freq
```

The ratio fixes Vout. `freq`, `duty` and `dead` only change output impedance,
ripple and switching loss. BJT storage time is several us, so keep `dead` >= 10 us.

### 1.3 Push-pull BJT driver (one per PNP switch, x4)

No level shifter and no Vhi rail. Each PNP switch is referenced to its own
emitter node `E` (the emitter of the power PNP, or the common emitter of the
back-to-back pair). The two GPIO of the pair drive a push-pull:

- `IN = 1, INb = 0`: Qn1 pulls the power base down -> switch **ON**
- `IN = 0, INb = 1`: Qn2 turns Qup on, Qup pushes the base up to `E` -> switch **OFF** (fast turn-off)

Per-switch netlist (same for S1, S2a, S2b, S3):

| Part | Type    | Connections                                                        |
|------|---------|--------------------------------------------------------------------|
| Qpw  | 2N3906 (x2 for S1, S2a) | power switch, base node = `Bp`                    |
| Qn1  | 2N3904  | base = IN via 2.2k, emitter = GND, collector -> `Rb` -> `Bp`       |
| Rb   | 1k (S1, S2a, S2b) / 2.2k (S3) | `Qn1` collector to `Bp`                      |
| Rbe  | 47k     | `Bp` to `E` (keeps the switch OFF if drivers float)                |
| Qup  | 2N3906  | emitter = `E`, collector = `Bp`                                    |
| Qn2  | 2N3904  | base = INb via 2.2k, emitter = GND, collector -> `Rqc` 4.7k -> Qup base |
| Rqb  | 10k     | Qup base to `E`                                                    |

Check: Qn1 and Qn2 are never on together (firmware never sets IN = INb = 1).

S4: GPIO -> 1k -> base of 2N3904, emitter GND, collector at B, plus 100k base-to-GND.

### 1.4 What the MCU must provide

- 9 push-pull GPIO (4 complementary pairs + 1 for S4), logic high >= 3.3 V
- a microsecond timer (`micros()`) and a serial port
- Nothing else: no ADC, no PWM peripheral, no library

### 1.5 Parameters you can change

In code (top of `firmware.ino`): pins, boot mode, default freq / dead / duty,
`AUTO_START`, and the limits. Over Serial at run time:

| Command                    | Effect                                      |
|----------------------------|---------------------------------------------|
| `mode half\|one\|double`   | x1/2, x1, x2 (also `0.5`, `1`, `2`)         |
| `freq <Hz>`                | 100 - 5000                                  |
| `dead <us>`                | 0 - 100 (must be <= period / 4)             |
| `duty <pct>`               | 10 - 90, phase 1 share of the active time   |
| `on` / `off`               | start / stop (off = every switch open)      |
| `status`, `help`           | print state / commands                      |

---

## 2. ESP32 example

Arduino-ESP32 core, `firmware.ino` as is. Serial monitor: 115200 baud, newline.

### 2.1 Bill of materials (all through-hole)

| Qty | Part | Value / model | Use |
|-----|------|---------------|-----|
| 10  | PNP BJT | 2N3906 (TO-92) | 6 power switches + 4 Qup |
| 9   | NPN BJT | 2N3904 (TO-92) | 8 drivers (Qn1, Qn2) + S4 |
| 1   | Capacitor | 10 uF / 25 V electrolytic | Cfly |
| 1   | Capacitor | 10 uF / 25 V electrolytic | Cout |
| 1   | Capacitor | 10 uF / 25 V electrolytic | Cin (at Vin) |
| 8   | Resistor | 2.2 kOhm | Qn1 / Qn2 base resistors |
| 3   | Resistor | 1 kOhm | Rb for S1, S2a, S2b |
| 1   | Resistor | 2.2 kOhm | Rb for S3 |
| 1   | Resistor | 1 kOhm | S4 base resistor |
| 4   | Resistor | 4.7 kOhm | Rqc |
| 4   | Resistor | 10 kOhm | Rqb |
| 4   | Resistor | 47 kOhm | Rbe |
| 1   | Resistor | 4.7 kOhm | test load |
| 10  | Resistor | 100 kOhm (recommended) | defined OFF state at reset (see below) |

Use PN2222 / 2N2907 or BC547 / BC557 if you prefer; check the pinout (EBC / CBE) of your parts.

Recommended: 100k pull-down on each `IN` pin and on the S4 base, 100k pull-up to 3.3 V
on each `INb` pin, so every switch is OFF while the ESP32 boots.

### 2.2 ESP32 pin map

| Signal | GPIO | | Signal | GPIO |
|--------|------|-|--------|------|
| S1 IN  | 16   | | S3 IN  | 23   |
| S1 INb | 17   | | S3 INb | 25   |
| S2a IN | 18   | | S4 base| 26   |
| S2a INb| 19   | | GND    | common with Vin supply |
| S2b IN | 21   | | | |
| S2b INb| 22   | | | |

Change them in the `PIN_IN`, `PIN_INB`, `PIN_S4` tables.

### 2.3 Bring-up

1. Build the circuit **without** Rload. Supply Vin = **9 V** from a bench supply (current limit 50 mA). Join its GND with the ESP32 GND.
2. Flash, open the serial monitor, check `state : STOPPED`.
3. `mode one`, `on`: Vout should be about Vin minus a few hundred mV.
4. `mode double`: Vout should approach 2 x Vin minus drops. `mode half`: about Vin / 2.
5. Connect the 4.7k load and compare Vout with and without it. Try `freq 200 / 1000 / 4000` and watch the droop and ripple change.
6. `off` before touching the circuit.

---

## 3. Limits and cautions

- **Vin range about 5 V - 11 V.** Upper limit: x2 puts 2 x Vin on Cfly / Cout (25 V caps); use higher-rated caps for more. The BJTs (Vceo 40 V) allow up to about 18 V Vin in x2.
- **Vin min about 5 V.** The base drive current is (E - 0.7 V) / Rb, so below that the PNP switches are not saturated.
- Veb of the 2N3906 is limited to 5 V; this design never applies more than about 0.7 V reverse across it, do not add other drive circuits on the bases.
- The inverted-mode PNP of each switch has low gain (about 5), so keep the load to **about 10 mA or less**. Vce(sat) drops add up: expect a few hundred mV loss per switch.
- BJT storage time makes turn-off slow: keep `dead` >= 10 us and `freq` <= 5 kHz.
- A 3.3 V GPIO drives the 2N3904 base fine through 2.2k / 1k.
- Software timing (`micros()` polling) has a few microseconds of jitter; this is fine for a test at 100 Hz - 5 kHz.
- Cfly and Cout are polarized: `+` at A and `+` at Vout. Never apply reverse voltage.
