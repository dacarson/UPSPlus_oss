## UPSPlus HAT Chip Inventory (EP-0136)

This document lists the known ICs on the UPSPlus HAT and their roles. It is a
technical reference intended to be kept current as hardware variants are
confirmed.

## I2C Bus Devices

- (**STM32F030F4P6**)[https://www.st.com/resource/en/datasheet/stm32f030f4.pdf] (MCU, SW I2C slave: 0x17 runtime; 0x18 OTA) — **board ref U9**
  - Main microcontroller running the UPSPlus firmware.
  - Shares the I2C bus with the current monitors and RTC.
  - Confirmed this session: the MCU does **not** talk to the IP5328 over I2C
    (the IP5328's I2C interface is present on the IC but is a separate,
    unconnected bus — see Power/Charging below). MCU↔IP5328 interaction
    appears to be limited to discrete GPIO lines (`IP_EN`, `MT_EN`), not I2C
    register access.
  - **Open item:** the power rail feeding U9 itself has not yet been traced.
    This matters for root-causing the low-voltage collapse behaviour
    described under "Known Issues" below — see Notes.

- **INA219** x2 (current monitors, I2C slave: 0x40 & 0x45) — **board ref U5, U6**
  - One monitors the output/RPi rail (downstream of TPS61088, i.e. the
    boosted 5 V side), one monitors the battery path (raw battery voltage).
  - Each INA219 uses an onboard shunt resistor marked **R010**.
  - SOIC-8, TI logo visible on package. Top markings read approx.
    "I219A" / [TI logo]"1BM" / "ASCX G4" (G4 underlined) — consistent with
    an INA219AIDR device+date+lot code, but not yet cross-checked against
    TI's official part marking lookup (https://www.ti.com/packaging/docs/partlookup.tsp).
  - The battery-path INA219 is the source of the precise voltage readings
    (e.g. the 2.51 V reading discussed under "Known Issues") used to
    correlate MCU/telemetry loss against actual battery voltage during a
    low-battery event.

- **DS1307** (RTC, I2C slave 0x68) — **board ref unconfirmed**
  - Real-time clock device on the same I2C bus.
  - SOIC-8, located adjacent to crystal **Y1** near U5/U6. Top marking
    reads "DS1307N" with date/lot code "2214AG" / "+970AB".

## Power / Charging

- **IP5328** (Injoinic; battery charger / power management) — **board ref U2**
  - Fully-integrated power bank SoC ("Power Bank SoC with Integrated
    Multi-Protocol Input/Output Fast-Charge Functions"), QFN40 (6×6 mm,
    0.5 mm pitch, 41 pins incl. EPAD) — matches U2's package.
  - Handles battery charging, boost (discharge) power path, and battery
    fuel-gauge/level indication in one chip. Supports **USB-C PD 3.0**
    (Power Delivery) as well as QC2.0/QC3.0, FCP, AFC, SFCP, and MTK PE+
    fast-charge protocols.
  - Accepts 5 V and higher PD input voltages (e.g. 9 V when the charger
    negotiates PD); USB-C register readings around 8–9 V are normal with a
    PD charger. Per datasheet: input voltage range 4.5–14 V (recommended
    operating), absolute max 16 V.
  - I2C interface present on the IC but **not** connected to the HAT I2C
    bus (confirmed this session — see I2C Bus Devices above).
  - Charging can be enabled and disabled via MCU (`IP_EN` pin). Note:
    `IP_EN` does not correspond to a named pin in the official IP5328
    pinout (VING/VBUSG are the chip's own internal PMOS-gate-control pins
    for the VIN/VBUS input paths, already wired to Q1/Q2 — see MOSFETs
    below). `IP_EN` is therefore most likely a board-level signal gating
    the input path some other way (e.g. an additional switch, or a level
    on Q1/Q2's gate network) rather than a direct IP5328 die pin —
    **unconfirmed, worth tracing** if a respin is planned.
  - **VREG** (IP5328 pin 27) is documented as an *always-on* 3.1 V LDO,
    rated for only **30 mA** load capability. This session's board
    discussion indicates VREG is the supply feeding the **TPS61088**
    boost converter's input (i.e. the RPi's power path runs off VREG, not
    off VOUT1/VOUT2/VBUS). ⚠️ A Raspberry Pi + TPS61088 combination will
    draw far more than 30 mA — likely tens to hundreds of mA reflected
    back to VREG's 3.1 V rail — meaning this rail is probably being run
    well outside its datasheet rating. Worth scoping VREG under load to
    confirm it's holding regulation rather than sagging/current-limiting.
  - Because a charger and the RPi load are effectively present
    simultaneously and continuously, this board is necessarily running in
    IP5328's "charge-while-discharge" (pass-through) mode. The base
    datasheet flags this as a **custom-order-only** feature, not the
    chip's standard behaviour — standard-configuration IP5328 disables
    discharge output while charging is active. This is relevant context
    for the charge-stall issue below, since charge-while-discharge is a
    less-travelled code path than the chip's primary charge/discharge
    modes.
  - Low Battery Cutoff is roughly 2.4–2.6 V; recovery around 3.0 V. This
    matches both the datasheet's documented "low-voltage lockout / locked
    state" behaviour (chip disables discharge outputs and re-arms only
    once charging resumes) and the Recommended Operating Conditions
    battery-voltage floor of 3.0 V. Empirically confirmed this session: a
    live low-battery event showed the STM32/I2C-bus telemetry go silent
    at a measured battery voltage of **2.51 V**.
  - Charge-termination current (`ISTOP`) is ~300 mA typical — charging is
    considered "complete" only once *battery-side* charge current drops
    below this. Charge safety-timer (`TEND`) is 20/24/27 h (min/typ/max).
    See "Known Issues" below — a continuous downstream load on VREG may
    prevent the battery-side current from ever reading below `ISTOP`,
    so the charge cycle may never terminate normally and instead run out
    the `TEND` safety timer, which has no documented auto-recovery path.
  - Output-port light-load auto-shutdown: total output power below
    ~300 mW for ~32 s shuts off all discharge outputs; a per-port version
    (current-equivalent below ~180 mA at the onboard 10 mΩ sense resistor)
    shuts off that individual port after ~16 s. Only relevant to loads on
    VOUT1/VOUT2/VBUS — **not** VREG, which is documented as exempt from
    this port-switching logic.
  - **KEY** pin (IP5328 pin 26) is the button-detect input; also multiplexed
    with the WLED flashlight driver.

- **TPS61088** (TI; 10-A fully-integrated synchronous boost converter) — **board ref U1**
  - 20-pin VQFN (RHL), 2.7–12 V input, 4.5–12.6 V adjustable output, 10 A
    switch current. [Product page](https://www.ti.com/product/TPS61088) /
    [datasheet](https://www.ti.com/lit/gpn/tps61088).
  - Located mid-board on the "25A MAX" trace side, next to C9/R2, near
    inductor **L2**. Top marking reads "S61088A" (TI symbolization for
    TPS61088) / [TI logo]+date code / lot-trace code.
  - Input confirmed (this session) to be fed from the IP5328's **VREG**
    pin, not from VBUSG/VOUTG — generates the regulated 5 V rail for the
    RPi header, separate from IP5328's own VOUT1/VOUT2/VBUS boost/charge
    path — output gated onward by **Q7** (NCE20P70G) toward the Pi header.
  - **UVLO (undervoltage lockout), per TI datasheet SLVSCM8A:**
    `VIN_UVLO` falling threshold ≈2.4–2.5 V typ (disables switching below
    this), rising threshold ≈2.7 V (200 mV hysteresis); a second,
    independent `VCC_UVLO` falling threshold ≈2.1 V typ on the internal
    bias supply. TI states this circuit exists specifically "to prevent
    the device from malfunctioning at low input voltage and the battery
    from excessive discharge."
  - This UVLO is the leading explanation for why, during a live low-battery
    event, the RPi continued running for ~5 seconds *after* the
    STM32/I2C-bus telemetry (fed by a separate rail/protection point on
    the IP5328 side) had already gone silent at 2.51 V: TPS61088's own
    ~2.4 V VIN_UVLO trip point is lower than whatever cut the STM32 rail,
    so the Pi's supply persisted until the battery sagged that final
    ~0.1–0.2 V further. Whether VCC_UVLO (2.1 V) or VIN_UVLO (~2.4 V) is
    the one actually governing the cutoff depends on how TPS61088's VCC
    pin is biased on this board — not yet confirmed.

## MOSFETs / Switches

- **NCE20P70G** (Wuxi NCE Power; P-channel power MOSFET, −20 V / −70 A) — **board ref Q7**
  - Located near R28/C32, close to the gold RPi header — likely the
    high-side load switch gating power to the Pi header, driven by the
    MCU's `MT_EN` (RPi Power Enable) line.
- **FKBB3105** (FETek Technology Corp; P-channel 30 V fast-switching MOSFET) — **board ref Q1, Q2**
  - Package PRPAK3x3 (small leadless power package) — matches the top
    marking read as "B3105" (abbreviated from FKBB3105), second line
    "P33XL5" being a date/lot code.
  - BVDSS −30 V, RDS(ON) ≈14 mΩ typ. (VGS=−10V, ID=−30A), continuous
    ID up to −42 A (TC=25°C). [Datasheet](https://www.jasencan.com/Upload/PicFiles/2020.9.1_18.2.54_3247.pdf).
  - Positioned flanking the IP5328's VIN/VBUS input pins; matches the
    external input-path PMOS switches (VING/VBUSG) the IP5328
    application circuit expects.

## Known Issues / Failure Modes (Under Investigation)

These are working hypotheses from analysis of live Grafana/NUT telemetry,
cross-referenced against the IP5328 and TPS61088 datasheets — not yet
confirmed against register-level fault status or a schematic.

- **Long-duration charging stall (~24–27 h after charge appears complete).**
  Observed: charger connected, battery reaches ~100%, CHRG flag clears
  normally, then ~27 hours later — with no further charging activity in
  between and no real battery discharge in that window — the battery
  suddenly begins draining rapidly even though the UPS's `OL` (online)
  status stays asserted throughout, implying AC/mains genuinely remained
  present. Hypothesis: because VREG (feeding TPS61088/the Pi) draws
  current continuously and isn't a load the charge-current sensing can
  distinguish from "the battery isn't accepting charge," the *battery-side*
  charge current (as sensed by IP5328) may never drop below the ~300 mA
  `ISTOP` termination threshold, so the charger never cleanly declares the
  cycle "complete." Instead it likely runs out the `TEND` charge safety
  timer (20/24/27 h) — which the datasheet documents no automatic recovery
  path for (only the *normal*-termination case documents an automatic
  recharge trigger). This is consistent with anecdotal reports that newer
  vendor UPS firmware performs a hard reset of the IP5328 whenever it
  detects "charger present, not charging," as a workaround for this exact
  condition.
- **Rapid battery collapse / STM32 I2C-bus dropout ahead of RPi power loss.**
  Observed: during a fast battery-drain event (sawtoothing battery voltage,
  ~23%→0% in ~24 minutes), the STM32/I2C bus (and therefore INA219/DS1307
  telemetry) went silent at a measured battery voltage of **2.51 V**, while
  the RPi itself continued running for **~5 more seconds** before losing
  power. Working theory: two independent, unrelated low-voltage
  protections tripped in quick succession as the battery made its final
  collapse — IP5328's own low-battery lockout (~2.4–2.6 V, matching the
  existing spec note above) cutting whichever rail feeds U9/the I2C bus,
  followed a few seconds later by TPS61088's own VIN_UVLO (~2.4 V falling)
  cutting the boosted 5 V Pi rail once VREG sagged low enough. This does
  **not** currently look like the output-port light-load auto-shutdown
  (that mechanism is duration/current-magnitude based, not voltage-based,
  and wouldn't be expected to correlate this tightly with a specific
  voltage) — but this hasn't been ruled out for other, non-low-battery
  dropout events and should be considered separately if the STM32 rail is
  ever found to be fed from a switched VOUT port.

## Notes

- If additional regulators, protection ICs, or board revisions are identified,
  add them here with part numbers and roles.
- Reference designators above (U*, Q*) were read directly off the board via
  macro photography. Q1/Q2 (FKBB3105) confirmed against manufacturer
  datasheet; U5/U6's INA219 marking is still a best-effort read and should
  be re-verified against TI's part marking lookup before relying on it for
  a respin.
- DS1307's own reference designator wasn't visible in the photos taken so far —
  worth confirming (likely U3, U4, or U7 depending on board silkscreen order).
- **STM32 (U9) power source is unconfirmed** — needs board-level tracing to
  determine whether it's fed from IP5328's VREG (alongside TPS61088), from
  a switched VOUT port (subject to light-load auto-shutdown, ~16–32 s), or
  from a separate regulator entirely. This is the key open item needed to
  fully validate the "Known Issues" sequencing theory above.
- `IP_EN` and `MT_EN` net names are referenced from firmware/board
  labelling; their exact connection points on the IP5328/TPS61088 side are
  inferred, not confirmed from a schematic.
- DEBUG UPSPlus HAT pins, Pin 1 on the LHS
  Pin 1 - VDD MCU Pin 16
  Pin 2 - SWCLK MCU Pin 20
  Pin 3 - SWDIO MCU Pin 19
  Pin 4 - GND MCU Pin 15
