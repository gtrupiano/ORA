# ESP IMU Board Issues and Solutions

Date: 2026-05-27

## High Severity

1. BNO086 `SA0/H_MOSI` address-select pin is floating.
   - Evidence: `U3` pin 17 is on `unconnected-(U3-SA0{slash}H_MOSI-Pad17)` in `ESP_IMU_Board/ESP_IMU_Board.kicad_pcb`.
   - Impact: the BNO086 I2C address is undefined or unstable.
   - Solution: tie `U3` pin 17 to a defined logic level. Use a 10 kOhm resistor to `GND` or `+3.3V`; if address flexibility is useful, add a solder jumper or selectable resistor footprint for `GND` / `+3.3V`.

2. AO3400A source is not directly connected to ground.
   - Evidence: `Q2` source is on `Net-(Q2-S)` and returns to `GND` through `R6 10k`.
   - Impact: the 10 kOhm resistor is in the light return path, limiting load current and preventing normal switching.
   - Solution: connect `Q2` source directly to `GND`. Remove or bypass `R6`; keep `R5 10k` as the gate pulldown and `R4 1k` as the gate series resistor.

3. The 24 V stack-light current path is routed with 0.2 mm tracks.
   - Evidence: `AUTON_LIGHT`, `AUTON_LIGHT_GND`, and `Net-(Q2-S)` segments use `width 0.2` in `ESP_IMU_Board/ESP_IMU_Board.kicad_pcb`.
   - Impact: this is not appropriate for the 3 A load current assumed in `Documentation/PCB_Design_Notes.md`; the traces may overheat or drop excessive voltage.
   - Solution: reroute the stack-light current path as a high-current path. Use wide copper pours or substantially wider traces for `J5 -> Q2 drain -> Q2 source -> GND`, sized for the real stack-light current, copper weight, allowable temperature rise, and board stackup. Add multiple vias if current changes layers.

4. The AO3400A has only a 30 V drain-source rating in a 24 V external-load system.
   - Evidence: `Documentation/PCB_Design_Notes.md` cites `BVDSS = 30V` for a 24 V stack light.
   - Impact: there is little margin for inductive/load transients, cable hot-plug events, or supply overshoot.
   - Solution: replace `Q2` with a logic-level N-channel MOSFET rated at least 40 V, preferably 60 V, with low `RDS(ON)` at `VGS = 2.5 V` or `3.3 V`. Recheck package power dissipation for the actual stack-light current.

## Medium Severity

5. The 5 V input and 3.3 V regulator feed are routed with 0.2 mm tracks.
   - Evidence: `PWR_IN` and many `+3.3V` segments use `width 0.2` in `ESP_IMU_Board/ESP_IMU_Board.kicad_pcb`.
   - Impact: the traces may be undersized for ESP32-S3 Wi-Fi current peaks and any USB/barrel input current.
   - Solution: widen `PWR_IN` and main `+3.3V` distribution traces or use pours. Size them for the regulator input current, ESP32-S3 peak current, copper weight, trace length, and acceptable voltage drop. Keep short, low-impedance routes from the regulator output to ESP32 bulk and decoupling capacitors.

6. ESP32-S3 bulk capacitance is lighter than the Espressif typical schematic shown in the documentation.
   - Evidence: the design has `C5 100nF`, `C6 10uF`, `C7 100nF`, and `C8 10uF`, while the local Espressif typical schematic shows more bulk capacitance.
   - Impact: Wi-Fi current bursts may cause 3.3 V rail droop or resets.
   - Solution: add bulk capacitance near the ESP32-S3 module 3.3 V pin, following the Espressif reference design more closely. Keep local 100 nF decoupling and add enough low-ESR bulk capacitance to support Wi-Fi transmit bursts.

7. TLV75733 thermal headroom is not proven for worst-case ESP32-S3 current.
   - Evidence: `PWR_IN` can be about 5 V and regulator output is 3.3 V, so the LDO dissipates `(VIN - 3.3 V) * IOUT`.
   - Impact: sustained high ESP32-S3 current could overheat the SOT-23 regulator.
   - Solution: calculate worst-case LDO dissipation and junction temperature using maximum input voltage, expected peak/sustained current, ambient temperature, and package thermal resistance. If margin is poor, replace the LDO with a higher-thermal-capacity regulator or a buck converter.

8. USB-C VBUS input has no fuse or current limiting.
   - Evidence: USB connector VBUS pads connect through `D3` into `PWR_IN`; no fuse/polyfuse/load switch is present in the reviewed schematic or PCB.
   - Impact: a board fault can draw excessive current from the USB source.
   - Solution: add a USB VBUS protection element before `PWR_IN`, such as a resettable fuse, eFuse, or current-limited load switch. Size it for the board's normal current and USB source expectations.

9. Barrel-jack input has no fuse or current limiting.
   - Evidence: barrel input connects through `D2` into `PWR_IN`; no fuse/polyfuse/load switch is present in the reviewed schematic or PCB.
   - Impact: a board fault can draw excessive current from the external 5 V supply.
   - Solution: add a fuse, resettable fuse, or current-limited input switch on the barrel-jack path before it joins `PWR_IN`. Choose the trip/limit current based on normal board load plus startup margin.

10. USB D+ and D- are routed as long, non-impedance-controlled board-spanning traces with vias.
    - Evidence: `USB+` and `USB-` route from the ESP32 area to the USB-C connector through long `0.2 mm` segments and vias in `ESP_IMU_Board/ESP_IMU_Board.kicad_pcb`.
    - Impact: USB signal integrity and enumeration reliability may suffer.
    - Solution: place the USB-C connector closer to the ESP32-S3 USB pins or reroute D+ and D- as a short differential pair over a continuous ground reference. Match lengths reasonably, avoid stubs, minimize vias, and set trace width/spacing for the intended differential impedance with the board stackup.

11. USB data lines have no visible ESD protection near the USB-C connector.
    - Evidence: no TVS/ESD component is present on `USB+` or `USB-` in the reviewed schematic or PCB.
    - Impact: cable ESD events can stress or damage the ESP32-S3 USB pins.
    - Solution: add a low-capacitance USB ESD protection array near `J4`, connected from `USB+` and `USB-` to ground with short return paths. Pick a part intended for USB 2.0 data lines.

12. External 24 V light connector has no visible transient suppression.
    - Evidence: no TVS, flyback clamp, or snubber is present on `AUTON_LIGHT` / `AUTON_LIGHT_GND`.
    - Impact: load or cable transients can overstress the AO3400A and nearby circuitry.
    - Solution: add transient protection at the light connector or MOSFET drain. Use a TVS diode rated for the 24 V system and/or a flyback path or RC snubber if the load is inductive. Coordinate the clamp voltage with the MOSFET voltage rating.

## Low Severity

13. ESP32 GPIO0 boot pin has no external pullup.
    - Evidence: `U1` pin 27 `IO0` connects to `~ESP_BOOT` and `SW2`, but no external pullup was found.
    - Impact: boot mode relies on the module/internal pullup and may be less deterministic.
    - Solution: add an external pullup from `~ESP_BOOT` / GPIO0 to `+3.3V`, typically 10 kOhm. Keep `SW2` pulling GPIO0 to `GND` for download mode.

14. UART header TX/RX labeling may be ambiguous.
    - Evidence: `J2` labels are `UART_RX` and `UART_TX` from the ESP32 net perspective.
    - Impact: if the silkscreen is interpreted from the external adapter perspective, TX/RX may be crossed incorrectly by the user.
    - Solution: make the header labeling explicit. Use labels such as `ESP_TX`, `ESP_RX`, `3V3`, and `GND`, or add board silkscreen/documentation that states TX/RX are from the ESP32 perspective.

15. USB-C shield is tied directly to board ground.
    - Evidence: `J4` shield pads are on `GND`.
    - Impact: this may be acceptable, but it can couple cable shield noise directly into board ground depending on enclosure and system grounding.
    - Solution: decide the shield strategy for the final mechanical system. If direct ground is intentional, document it. If noise/ESD isolation is desired, connect shield to ground through an RC network, capacitor, high-value resistor, or chassis-ground strategy appropriate for the enclosure.

16. Environmental-sensor I2C pullups are populated even though no external environmental sensor is connected.
    - Evidence: `R7` and `R8` pull `U3` `ENV_SDA` and `ENV_SCL` to `+3.3V`.
    - Impact: this adds small static current paths and unused nets; it is likely harmless but unnecessary unless future expansion is intended.
    - Solution: depopulate `R7` and `R8` if the environmental sensor bus will never be used. If future expansion is intended, keep the footprints but mark them DNP by default or add a connector/test pads for the environmental sensor bus.

17. BOM diode references do not match the KiCad schematic references.
    - Evidence: `Documentation/ESP_IMU_Board_BOM.xlsx` lists `D1,D3` as Schottky diodes and `D2` as the LED, while `ESP_IMU_Board/ESP_IMU_Board.kicad_sch` has `D1` as `LED` and `D2,D3` as `D_Schottky`.
    - Impact: parts can be ordered, placed, or checked against the wrong reference designators.
    - Solution: update the BOM so `D1` is the LED and `D2,D3` are Schottky diodes, or update the schematic references and regenerate the BOM. Then verify the PCB reference designators match the corrected BOM.

18. KiCad command-line ERC/DRC could not be run in this environment.
    - Evidence: `kicad-cli` is not installed.
    - Impact: this issue list has not been cross-checked against KiCad's automated electrical and layout rule checks.
    - Solution: run KiCad ERC and DRC from the KiCad GUI or install `kicad-cli` and run schematic ERC plus PCB DRC. Add any resulting violations to this document before fabrication.
