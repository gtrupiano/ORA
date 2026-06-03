# ESP IMU Board Design Audit

Date: 2026-05-27

Scope: analysis only. This audit cross-references the provided documentation in `Documentation/`, the BOM in `Documentation/ESP_IMU_Board_BOM.xlsx`, and the native KiCad schematic/PCB files in `ESP_IMU_Board/`. No design files were changed as part of this review.

Primary files reviewed:

- `Documentation/PCB_Design_Notes.md`
- `Documentation/Images/Schematic/Schematic.png`
- `Documentation/Images/ESP32/Typical_Schematic.png`
- `Documentation/Images/ESP32/Boot_Pins_To_Avoid.png`
- `Documentation/Images/ESP32/I2C_Pin_Mapping.png`
- `Documentation/Images/BNO086/I2C_Connection_Diagram.png`
- `Documentation/Images/BNO086/Internal_Clock_Selection.png`
- `Documentation/Images/SparkFun_BNO086_IMU_Schematic.png`
- `Documentation/Images/TLV75733PDBVR/Typical_Application.png`
- `Documentation/Images/TLV75733PDBVR/Recommended_Operating_Conditions.png`
- `Documentation/Images/TLV75733PDBVR/Capacitor_Value_Proof.png`
- `Documentation/Images/TLV75733PDBVR/Voltage_Dropout.png`
- `Documentation/Images/AO3400A/Electrical_Characteristics.png`
- `Documentation/Images/AO3400A/Absolute_Maximum_Ratings.png`
- `ESP_IMU_Board/ESP_IMU_Board.kicad_sch`
- `ESP_IMU_Board/ESP_IMU_Board.kicad_pcb`

## Executive Summary

The design is broadly close, but there are two confirmed functional issues:

1. BNO086 pin 17, `SA0/H_MOSI`, is floating. In I2C mode this pin selects the I2C address and must be tied to a defined logic level.
2. The AO3400A low-side switch source is not directly connected to ground. It is connected through a 10 kOhm resistor, which puts that resistor in the light return path and prevents normal load current.

There are also robustness concerns:

- AO3400A has only 30 V `VDS` rating on a 24 V load system.
- ESP32-S3 bulk capacitance is lighter than Espressif's typical schematic.
- TLV75733 thermal headroom should be checked for worst-case ESP32-S3 Wi-Fi current.

## Confirmed Issues

### 1. BNO086 I2C Address Pin Is Floating

Severity: high

Component: `U3`, BNO086

Relevant netlist evidence from `ESP_IMU_Board/ESP_IMU_Board.kicad_pcb`:

```text
U3 pin 17: SA0/H_MOSI_17 -> unconnected-(U3-SA0{slash}H_MOSI-Pad17)
```

KiCad PCB reference:

```text
ESP_IMU_Board/ESP_IMU_Board.kicad_pcb:4858
```

Why this matters:

- In I2C mode, `SA0/H_MOSI` is used as the I2C address select input.
- Leaving the pin floating can make the device address undefined or unstable.
- This can result in the ESP32 not finding the IMU or the IMU intermittently appearing at the wrong address.

Required correction:

- Tie `SA0/H_MOSI` to `GND` or `+3.3V`.
- Prefer using a resistor footprint or solder jumper if address flexibility is useful.
- The SparkFun BNO086 reference schematic uses a configurable address strap approach, which is a good pattern.

Recommended implementation:

```text
U3 pin 17 SA0/H_MOSI -> 10k to GND
```

or:

```text
U3 pin 17 SA0/H_MOSI -> 10k to +3.3V
```

Better flexible option:

```text
U3 pin 17 SA0/H_MOSI -> selectable jumper/resistor to GND or +3.3V
```

### 2. AO3400A Source Is Not Directly Grounded

Severity: high

Component: `Q2`, AO3400A

Relevant netlist evidence from `ESP_IMU_Board/ESP_IMU_Board.kicad_pcb`:

```text
Q2 pin 1: G_1 -> Net-(Q2-G)
Q2 pin 2: S_2 -> Net-(Q2-S)
Q2 pin 3: D_3 -> AUTON_LIGHT_GND

R6 pin 1 -> GND
R6 pin 2 -> Net-(Q2-S)
R6 value -> 10k

J5 pin 1 -> AUTON_LIGHT_GND
J5 pin 2 -> GND
```

KiCad PCB references:

```text
ESP_IMU_Board/ESP_IMU_Board.kicad_pcb:9033
ESP_IMU_Board/ESP_IMU_Board.kicad_pcb:9043
ESP_IMU_Board/ESP_IMU_Board.kicad_pcb:9053
ESP_IMU_Board/ESP_IMU_Board.kicad_pcb:6637
ESP_IMU_Board/ESP_IMU_Board.kicad_pcb:7370
```

Intended low-side switch topology:

```text
+24V supply -> light -> switched return wire -> Q2 drain -> Q2 source -> board GND
```

Actual topology:

```text
+24V supply -> light -> switched return wire -> Q2 drain -> Q2 source -> R6 10k -> board GND
```

Why this matters:

- R6 is in series with the light return current.
- A 10 kOhm series resistor allows only a few milliamps at most.
- Approximate maximum current from a 24 V load through 10 kOhm:

```text
I = V / R = 24 V / 10000 ohm = 2.4 mA
```

- That is not enough for a normal stack light load.
- The resistor also raises the MOSFET source voltage as current attempts to flow, reducing `VGS` and further turning the MOSFET off.

Required correction:

```text
Q2 source -> direct GND
```

Keep these gate-side parts:

```text
ESP32 GPIO -> R4 1k -> Q2 gate
Q2 gate -> R5 10k -> GND
```

Remove or bypass R6 from the source path. If R6 was intended as a pulldown, it belongs on the gate, but R5 already performs that function.

## BNO086 IMU Audit

Component: `U3`, BNO086

BOM part:

```text
U3: BNO086
```

### Power Pins

Actual connections:

```text
U3 pin 3  VDD   -> +3.3V
U3 pin 28 VDDIO -> +3.3V
U3 pin 2  GND   -> GND
U3 pin 25 GNDIO -> GND
```

Assessment: correct for a 3.3 V design.

### Decoupling and CAP Pin

Actual connections:

```text
C12 100nF -> +3.3V to GND
C13 100nF -> +3.3V to GND
C11 100nF -> U3 CAP pin to GND
```

Relevant BNO086 connections:

```text
U3 pin 9 CAP -> Net-(U3-CAP)
C11 pin 1 -> Net-(U3-CAP)
C11 pin 2 -> GND
```

Assessment: matches the provided BNO086 reference diagrams.

### I2C Mode Selection

Actual connections:

```text
U3 pin 5 PS1      -> GND
U3 pin 6 PS0/WAKE -> GND
```

Assessment: correct for I2C mode based on the provided BNO086 connection diagram.

### I2C Host Bus

Actual connections:

```text
U3 pin 19 H_SCL/SCK/RX     -> IMU_SCL
U3 pin 20 H_SDA/H_MISO/TX  -> IMU_SDA

R12 2.2k -> +3.3V to IMU_SCL
R13 2.2k -> +3.3V to IMU_SDA

U1 ESP32 pin 17 IO9 -> IMU_SCL
U1 ESP32 pin 12 IO8 -> IMU_SDA
```

Assessment: correct.

Notes:

- `2.2k` pullups match the provided BNO086 connection diagram.
- SparkFun uses `4.7k` on its breakout. Either can be reasonable depending on bus capacitance and speed. `2.2k` is stronger and consistent with the datasheet diagram captured in this repo.

### I2C Address Select

Actual connection:

```text
U3 pin 17 SA0/H_MOSI -> unconnected
```

Assessment: incorrect. This is a confirmed issue.

Fix:

```text
Tie U3 pin 17 to GND or +3.3V.
```

### Reset

Actual connections:

```text
U3 pin 11 ~RST -> ~IMU_RST
R9 10k -> +3.3V to ~IMU_RST
U1 ESP32 pin 39 IO1 -> ~IMU_RST
```

Assessment: correct.

Note:

- Reset has a pullup and can be driven by the ESP32.

### Boot Pin

Actual connections:

```text
U3 pin 4 ~BOOT -> ~IMU_BOOTN
R14 10k -> +3.3V to ~IMU_BOOTN
U1 ESP32 pin 4 IO4 -> ~IMU_BOOTN
```

Assessment: reasonable.

Note:

- Pulling `BOOTN` high by default matches normal boot behavior.
- ESP32 GPIO4 is not one of the avoided ESP32 strapping pins listed in the local documentation.

### Interrupt Pin

Actual connections:

```text
U3 pin 14 ~H_INT -> ~IMU_INT
U1 ESP32 pin 5 IO5 -> ~IMU_INT
```

Assessment: reasonable.

Note:

- The design notes correctly mention that BNO086 I2C operation should use interrupt-driven reads rather than relying on polling.

### Internal Clock Selection

Actual connections:

```text
U3 pin 10 CLKSEL0        -> +3.3V
U3 pin 26 XOUT32/CLKSEL1 -> GND
U3 pin 27 XIN32          -> unconnected
```

Assessment: correct based on the provided internal clock selection diagram.

### Environmental Sensor I2C Pins

Actual connections:

```text
U3 pin 15 ENV_SCL -> Net-(U3-ENV_SCL)
R8 2.2k -> +3.3V to Net-(U3-ENV_SCL)

U3 pin 16 ENV_SDA -> Net-(U3-ENV_SDA)
R7 2.2k -> +3.3V to Net-(U3-ENV_SDA)
```

Assessment: likely acceptable.

Notes:

- These are pulled up but no external environmental sensor is connected in the current board.
- This matches the local design note discussion that the datasheet diagram uses `2.2k` while SparkFun uses `4.7k`.

### H_CS Pin

Actual connection:

```text
U3 pin 18 ~H_CS -> unconnected
```

Assessment: not flagged as a confirmed issue from the provided local diagrams.

Notes:

- The provided I2C connection diagram and internal clock selection diagram do not require `H_CS` to be tied for I2C mode.
- `SA0/H_MOSI` is the important missing strap identified by issue #3.

## ESP32-S3-WROOM-1 Audit

Component: `U1`, ESP32-S3-WROOM-1

BOM part:

```text
U1: ESP32-S3-WROOM-1-N8R2
```

### Power

Actual connections:

```text
U1 pin 2 3V3 -> +3.3V
U1 pins 1, 40, 41 -> GND
```

Assessment: correct.

### EN Pin

Actual connections:

```text
U1 pin 3 EN -> ~ESP_ENABLE
R3 10k -> +3.3V to ~ESP_ENABLE
C14 100nF -> ~ESP_ENABLE to GND
SW1 -> ~ESP_ENABLE to GND
```

Assessment: correct and consistent with the provided Espressif typical schematic style.

### Boot Pin / GPIO0

Actual connections:

```text
U1 pin 27 IO0 -> ~ESP_BOOT
SW2 -> ~ESP_BOOT to GND
```

Assessment: mostly correct.

Concern:

- The local ESP32 boot pin documentation identifies GPIO0 as a strapping pin.
- GPIO0 is intentionally used as a boot button, which is normal.
- No external pullup was found in the netlist; the design appears to rely on the module/internal weak pullup.

Recommendation:

- Consider adding an external pullup on GPIO0 if deterministic boot behavior is desired.

### USB

Actual connections:

```text
U1 pin 13 USB_D- -> USB-
U1 pin 14 USB_D+ -> USB+

J4 A7 D- -> USB-
J4 B7 D- -> USB-
J4 A6 D+ -> USB+
J4 B6 D+ -> USB+
```

Assessment: correct.

### I2C GPIOs

Actual connections:

```text
U1 pin 12 IO8 -> IMU_SDA
U1 pin 17 IO9 -> IMU_SCL
```

Assessment: correct. The local ESP32 pin mapping document shows these pins are usable GPIOs, and they are not listed as avoided strapping pins.

### IMU Control GPIOs

Actual connections:

```text
U1 pin 4 IO4  -> ~IMU_BOOTN
U1 pin 5 IO5  -> ~IMU_INT
U1 pin 39 IO1 -> ~IMU_RST
```

Assessment: reasonable.

Notes:

- GPIO4, GPIO5, and GPIO1 are not in the local "avoid these boot pins" list.

### UART Header

Actual connections:

```text
U1 pin 36 RXD0 -> UART_RX
U1 pin 37 TXD0 -> UART_TX

J2 pin 1 -> +3.3V
J2 pin 2 -> UART_RX
J2 pin 3 -> UART_TX
J2 pin 4 -> GND
```

Assessment: electrically sensible.

Note:

- Header labeling depends on intended cable perspective. If the header labels are from the ESP32 perspective, it is correct. If they are from the external adapter perspective, TX/RX may need to be crossed in documentation.

### Strapping Pins

Local documentation lists strapping pins to avoid:

```text
GPIO0
GPIO3
GPIO45
GPIO46
```

Actual usage:

```text
GPIO0  -> boot button
GPIO3  -> unconnected
GPIO45 -> unconnected
GPIO46 -> unconnected
```

Assessment: acceptable.

## TLV75733PDBVR Regulator Audit

Component: `U2`, TLV75733PDBVR

BOM part:

```text
U2: TLV75733PDBVR
```

### Pin Connections

Actual connections:

```text
U2 pin 1 IN  -> PWR_IN
U2 pin 2 GND -> GND
U2 pin 3 EN  -> PWR_IN
U2 pin 4 NC  -> unconnected
U2 pin 5 OUT -> +3.3V
```

Assessment: correct.

### Capacitors

Actual connections:

```text
C1 1uF -> PWR_IN to GND
C2 1uF -> +3.3V to GND
```

Assessment: correct and consistent with the provided TLV757 typical application and recommended operating conditions.

Additional bulk/decoupling:

```text
C5 100nF -> PWR_IN to GND
C6 10uF  -> PWR_IN to GND
C7 100nF -> +3.3V to GND
C8 10uF  -> +3.3V to GND
```

Assessment: beneficial.

### Dropout

Input path:

```text
J3 barrel input -> D2 Schottky -> PWR_IN
J4 USB VBUS -> D3 Schottky -> PWR_IN
```

Nominal voltage:

```text
USB/barrel nominal: 5 V
Regulator output: 3.3 V
```

Assessment: dropout margin appears acceptable at nominal 5 V even after a Schottky drop, based on the provided TLV757 dropout table.

### Thermal

Concern:

- The TLV757 is a linear regulator.
- Dissipation is approximately:

```text
P = (VIN - VOUT) * IOUT
```

Example with 5 V input and 3.3 V output:

```text
P = (5.0 V - 3.3 V) * IOUT = 1.7 V * IOUT
```

At 300 mA:

```text
P = 0.51 W
```

At 500 mA:

```text
P = 0.85 W
```

Assessment:

- The pinout and capacitors are correct.
- Thermal headroom should be checked for worst-case ESP32-S3 Wi-Fi current, IMU current, LED current, and ambient temperature.

Recommendation:

- Estimate worst-case 3.3 V rail current.
- Confirm the SOT-23 thermal dissipation is acceptable with the actual PCB copper.
- If the ESP32 will use Wi-Fi heavily, consider whether a buck regulator is more appropriate.

## AO3400A Light Switch Audit

Component: `Q2`, AO3400A

BOM part:

```text
Q2: AO3400A
```

### Gate Drive

Actual connections:

```text
U1 ESP32 pin 33 IO40 -> AUTON_LIGHT
R4 1k -> AUTON_LIGHT to Net-(Q2-G)
R5 10k -> Net-(Q2-G) to GND
Q2 gate -> Net-(Q2-G)
```

Assessment: correct gate-drive topology.

Notes:

- `R4` provides a small series gate resistor.
- `R5` provides a gate pulldown to keep the MOSFET off during reset/boot.
- ESP32 3.3 V GPIO drive is reasonable for AO3400A because the datasheet provides `RDS(on)` at `VGS = 2.5 V`.

### Drain and Source

Actual connections:

```text
J5 pin 1 -> AUTON_LIGHT_GND
Q2 drain -> AUTON_LIGHT_GND
Q2 source -> Net-(Q2-S)
R6 10k -> Net-(Q2-S) to GND
J5 pin 2 -> GND
```

Assessment: incorrect. Q2 source should be direct to GND.

Required correction:

```text
Q2 source -> GND
```

Do not place `10k` in the source return path for this low-side switch.

### MOSFET Voltage Rating

Datasheet information from local snippet:

```text
AO3400A VDS maximum: 30 V
Load system: 24 V
```

Assessment:

- Electrically valid for a clean 24 V LED load.
- Margin is only 6 V.

Recommendation:

- If the stack light cable is long, the environment is industrial/noisy, or the load internals are uncertain, use a higher-voltage logic-level MOSFET such as 40 V or 60 V.
- Consider a TVS or clamp on the switched node if transients are expected.

### MOSFET Current and Dissipation

Datasheet snippet shows:

```text
RDS(on) at VGS = 2.5 V, ID = 3 A:
typical 24 mOhm, maximum 48 mOhm
```

Power estimate at 3 A using max `RDS(on)`:

```text
P = I^2 * R
P = 3^2 * 0.048 = 0.432 W
```

Assessment:

- The original calculation in `PCB_Design_Notes.md` is reasonable once the source is directly grounded.
- With the current R6 source resistor, that calculation does not apply because the MOSFET source is not actually at ground.

## USB-C and Power Input Audit

Component: `J4`, USB-C receptacle

BOM part:

```text
J4: USB4085-GF-A
```

### USB-C CC Pins

Actual connections:

```text
J4 CC1 -> R2 5.1k -> GND
J4 CC2 -> R1 5.1k -> GND
```

Assessment: correct for a USB-C sink/device.

### USB-C Data Pins

Actual connections:

```text
J4 A6/B6 D+ -> USB+
J4 A7/B7 D- -> USB-
```

Assessment: correct.

### USB-C VBUS

Actual connections:

```text
J4 A4/A9/B4/B9 VBUS -> Net-(D3-A)
D3 Schottky -> PWR_IN
```

Assessment: conceptually correct for isolating USB VBUS from the shared `PWR_IN` rail.

### Shield

Actual connection:

```text
J4 shield -> GND
```

Assessment: acceptable for many simple designs.

Note:

- Some EMC-sensitive designs use chassis/earth strategies, RC/ESD networks, or split shield handling. This board appears to use direct shield-to-ground.

## Barrel Jack and Power ORing Audit

Component: `J3`, barrel jack

BOM part:

```text
J3: PJ-063AH
```

Actual connections:

```text
J3 pin 1 -> Net-(D2-A)
J3 pin 2 -> GND
D2 Schottky -> PWR_IN
```

Assessment: conceptually correct.

Notes:

- The schematic label says `5V_Barrel_Jack`.
- The selected footprint description mentions the connector is rated for 24 V, 8 A, but the design intent is 5 V input to the regulator.
- Make sure the actual external barrel supply used with this jack is 5 V, not 24 V. The TLV757 absolute/recommended input max from the local documentation is 5.5 V.

## LEDs and Indicators

Component: `D1`, LED

Actual connections:

```text
D1 anode -> +3.3V
D1 cathode -> R11 220 -> GND
```

Assessment: correct for a power indicator LED.

Current estimate:

Assuming red LED around 2 V forward voltage:

```text
I = (3.3 V - 2.0 V) / 220 ohm = 5.9 mA
```

Assessment: reasonable.

## Decoupling and Layout Considerations

### ESP32-S3 Bulk Capacitance

Local Espressif typical schematic shows:

```text
22uF + 0.1uF on VDD33
```

Current board has:

```text
C8 10uF  -> +3.3V to GND
C7 100nF -> +3.3V to GND
```

Assessment:

- This may work.
- It is lighter than the reference schematic.

Recommendation:

- Prefer matching the Espressif typical `22uF + 0.1uF` near the ESP32 module if board space allows.

### IMU Decoupling

Current board has:

```text
C12 100nF -> +3.3V to GND
C13 100nF -> +3.3V to GND
C11 100nF -> CAP to GND
```

Assessment: matches the local BNO086 reference diagrams.

### Regulator Decoupling

Current board has:

```text
C1 1uF input
C2 1uF output
```

Assessment: matches TLV757 requirements.

Additional bulk is present and beneficial.

## BOM Cross-Check

Important BOM entries:

```text
U1: ESP32-S3-WROOM-1-N8R2
U2: TLV75733PDBVR
U3: BNO086
Q2: AO3400A
R1/R2: 5.1k USB-C CC pulldowns
R12/R13: 2.2k host I2C pullups
R7/R8: 2.2k environmental I2C pullups
R3/R5/R6/R9/R14: 10k resistors
C1/C2: 1uF regulator caps
C6/C8: 10uF bulk caps
C5/C7/C11/C12/C13/C14: 100nF caps
```

Assessment:

- BOM values match the visible schematic and KiCad board connectivity.
- R6 is the problematic 10 kOhm source resistor in the light switch.
- There is no BOM item currently allocated to strap BNO086 `SA0/H_MOSI`.

## Recommended Fix List

### Required Before Rev A Fabrication

1. Strap BNO086 `SA0/H_MOSI` to a known logic level.
2. Connect AO3400A source directly to GND.
3. Remove or bypass R6 from the MOSFET source path.

### Strongly Recommended

1. Add a higher-voltage MOSFET or clamp protection if the 24 V light wiring may see transients.
2. Increase ESP32-S3 local bulk capacitance to match Espressif's typical `22uF + 0.1uF` recommendation.
3. Run thermal calculations for TLV75733 under worst-case ESP32-S3 Wi-Fi load.

### Optional Improvements

1. Add an external pullup for ESP32 GPIO0 boot pin.
2. Add selectable BNO086 address jumpers instead of a fixed strap.
3. Document UART header direction clearly: whether `UART_TX`/`UART_RX` are named from the ESP32 perspective or the external adapter perspective.

## Final Assessment

The board design is not ready for fabrication as-is because of the floating BNO086 address pin and the AO3400A source resistor issue. After those two items are fixed, the remaining concerns are mostly robustness and margin rather than basic functionality.

