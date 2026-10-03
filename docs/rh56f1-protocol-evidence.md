# RH56F1 protocol evidence (Stage 5A/5B, revised 2026-09-23)

Purpose: decide from official sources whether the RH56F1 real transport can be
implemented. No device was opened for this work.

**Result**
- Interface: **RS-485**, per the user's statement of 2026-09-23 ("RS485로 확정").
  That puts the installed hands in the E2 family. CAN-FD was dropped from scope.
- **ANGLE_SET = 1040 (0x0410)**, settled from both official manuals (see
  "ANGLE_SET address ruling"). 1046 is FORCE_SET.
- **5B-1 offline RS-485 codec: done** (`protocol.hpp`, `rs485_codec.*`).
- **5B-2a serial transport layer: done, PTY-tested** (`serial_port.*`,
  `register_client.*`). It is **not linked into the plugin**, and writes are
  disabled by default.
- **5B-2b read-only diagnostic `rh56f1_diag`: done, PTY-tested.** It is dry-run
  by default and never writes.
- **Real transport in ros2_control (5B-2 gate): FAILED.** `Rs485Transport`
  stays a fail-closed stub.

## ANGLE_SET address ruling (1040 vs 1046)

| Evidence | 1040 (0x0410) | 1046 (0x0416) |
|---|---|---|
| M-CN V1.0.1 (docx; PRJ-02-TS-U-001), §2.5 table 30 | "各自由度的角度设置值" (angle set), 12 byte, W/R | "各自由度的力控阈值设置值" (force-control threshold set) |
| M-CN §2.5.11 table 35 / §2.5.12 table 37 | 1040–1045 = angle setting little…thumb rotation | 1046–1051 = force setting little…thumb rotation |
| M-CN §2.2.2 table 11 (write example "set angles") | bytes `12 10 04` → address LE `10 04` = 0x0410 | — |
| M-CN §3 example 6 ("force threshold = 1000") | — | bytes `12 16 04` → `16 04` = 0x0416 |
| M-CN §3 example 8 ("set angles = 1000") | bytes `12 10 04` | — |
| M-EN (RH56F1-User-ManualV1.2.pdf; PRJ-02-TS-U-015), table 30 on printed p.15 (pdf p.23) | "Set value of the angle for each DOF" | "Set value of force control threshold for each DOF" |
| M-EN table 35, printed p.20 / table 37, printed p.21 | 1040–1045 angle | 1046–1051 force |
| M-EN table 11 (printed p.9) and §3 ex. 8 (printed p.37) | `EB 90 01 0F 12 10 04 …` | ex. 6 (printed p.36): `EB 90 01 0F 12 16 04 … BE` |
| M §2.4 CAN write example ("set index angle", both) | address 1042, inside the 1040–1045 group | — |
| Vendor SDK `RH56F1_485_protocol.cpp:17-18` | `{"angleSet", 1040}` | `{"forceSet", 1046}` |
| `protocol.hpp` / golden tests | `kRegAngleSet = 1040`; `OfficialGoldenWriteAngleSetRequest` reproduces table 11 byte for byte | `kRegForceSet = 1046`; ex. 6 checksum BE reproduced |

- Both official editions agree, the tables agree with the example frames, and
  the SDK agrees independently.
- No local record ever assigned 1046 to ANGLE_SET. The value most likely came
  from the "register 1046, 1064" search keywords in an earlier request.
- The only related typo is §2.4, which labels 1042 (and 1066) "index" where
  tables 35/40 say "middle". It does not affect the 1040 group start.
- **Chosen: ANGLE_SET = 1040, FORCE_SET = 1046.** The write encode can
  therefore represent ANGLE_SET, but no code path in this package enables a
  write outside tests.

## Lab settings found (past code, NOT verified on the units)

| File | Setting | Status |
|---|---|---|
| `sim2real/vendor/inspire_ws/src/config/RH56F1.yaml` (identical copy in `urdf/vendor/...`) | `RH56F1_485`, `/dev/ttyUSB0`, 115200, `Hand_ID: 1` | vendor sample config |
| `sim2real/vendor/inspire_ws/src/ros2/src/driver/config/device_protocol_config.yaml` | left ID 1, right ID 2, both `/dev/ttyUSB0` | vendor sample (also mislabelled RH5DG2_485) |
| `sim2real/isaacsim_bridge/config/rh56f1_hand_calibration.yaml` | right `hand_id: 2`, left `hand_id: 1` | legacy project guess, `TODO(hardware)` |
| `sim2real/USAGE_ISAACSIM_ROS2.md:149` | example `hand_id: 2` | doc example |
| vendor SDK | reply timeout 25 ms | not official |
| `sim2real/docs/HEAD_COMPLIANT_SETUP.md:32,55` | `/dev/ttyUSB0` = **head DYNAMIXEL bus** (U2D2, FT232H) on local5090 | shows why `/dev/ttyUSB*` names must not be used for the hands |
| `emergency_open.py`, `teleop_left_arm.py`, `setup_usb_lowlatency.sh` | not found in any readable location or git history (possibly only under the off-limits `robot_control-jazzy`) | not examined |

None of these IDs is treated as the units' real configuration.

## Wiring confirmed by the user (2026-09-23)

| Side | Adapter | Serial | Stable path |
|---|---|---|---|
| left | FTDI FT232R USB UART | BG0327KL | `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG0327KL-if00-port0` |
| right | FTDI FT232R USB UART | BG033STU | `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG033STU-if00-port0` |

- Each hand has its own adapter, so both may use the same bus ID without
  conflict. Every frame still carries the target ID, and replies are checked
  against it.
- The paths are documented here and in the bringup doc only. No library code
  hard-codes them.
- The bus IDs are still unknown (factory default 1, not verified on the units).

## Sources

| Code | Source | Location | Version / date | Official? |
|---|---|---|---|---|
| M-CN | 因时 RH56F1系列灵巧手用户手册 | local `sim2real/vendor/inspire_ws/src/document/因时机器人-RH56F1系列灵巧手用户手册V1.0.0.docx` | Doc PRJ-02-TS-U-001. File name says V1.0.0, but the content is V1.0.1 (2025-9 revision row) and the docx was modified 2025-09-18 | Yes (Inspire header). Note that RH56 series manuals share the same doc number. |
| M-EN | Dexterous Hands User Manual for RH56F1 | https://en.inspire-robots.com/wp-content/uploads/2026/09/RH56F1-User-ManualV1.2.pdf | File name V1.2. Cover: "V1.0, June 2025", doc PRJ-02-TS-U-015 (one header reads U-031). PDF created 2026-04-29 | Yes (official download page) |
| P-EN | RH56F1 product page | https://en.inspire-robots.com/product/rh56f1/ | fetched 2026-09-23 | Yes |
| P-CN | CN product page at the `RH56F1-series` URL | https://www.inspire-robots.com/dexterous%20hands/RH56F1-series/ | fetched 2026-09-23 | Yes. **It now shows RH56F2** (E2/E4, 24–60 V); RH56F1 is not listed on the CN site |
| HS-CN | 灵巧手上位机软件操作说明 (host software guide) | https://www.inspire-robots.com/d/file/p/2026/09-22/dexterous%20hands.pdf (older: …/2026/06-25/swjrjczsc.pdf) | 2026-09-22 | Yes |
| FAQ | Dexterous Hands FAQ | https://en.inspire-robots.com/faq/ | fetched 2026-09-23 | Yes |
| DL | Download centres | https://en.inspire-robots.com/download/ , https://www.inspire-robots.com/support/download/dexterous%20hands/ | fetched 2026-09-23 | Yes. Beyond M-EN there is nothing RH56F1-specific: no SDK, protocol note or ROS package. The Windows host-software .exe and USB driver archives were not run. |
| SDK | `inspire_control_ros2`, `RH56F1_485_protocol` | local `sim2real/vendor/inspire_ws/src/` | 2026-02/03 | Vendor-supplied, unattributed (placeholder maintainer) |
| GH | GitHub `INSPIRE-ROBOTS` account; Gitee search | github.com/INSPIRE-ROBOTS | — | Account has 0 public repositories and an unverified identity. Gitee has no results. The only other "RH56F1" repo found (Quok-it/inspire_RH56F1) is empty and third-party. **No official public SDK exists.** |
| DFX-CAN | RH56 CAN 增补协议 | https://www.inspire-robots.com/d/file/p/2024/12-06/… | PRJ-02-TS-U-005 V0.0.2 | Official, but for **RH56DFX, a different product** (different ID layout and register map). Comparison only. |
| U | User statement | this session, 2026-09-23 | — | Operator confirmation of RS-485 |

## Model codes (P-EN, re-verified)

| Code | Meaning |
|---|---|
| RH56F1-**E2**x | EtherCAT + RS485 |
| RH56F1-**E4**x | EtherCAT + CAN FD |
| …**R** / …**L** | right / left hand model |
| …**-T1** | with tactile sensors (8), 630 g; without T1: 615 g |

The product family offers both interfaces. The hands delivered to the lab are
RS-485 (E2) according to U. Which exact codes they are (E2R/E2L, with or
without T1) and their serial numbers still come only from the nameplates.

## Evidence table (RS-485 scope)

Columns: Item | Value | Model | Interface | Source | Location (table/page/line) | Version | Official | Conf. (H/M/L) | Implementable | Needs unit check | Conflicts

| Item | Value | Model | IF | Src | Location | Ver | Off. | Conf | Impl | Unit | Conflicts |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Interface options | E2 = EtherCAT+RS485, E4 = EtherCAT+CAN FD | F1 all | — | P-EN, M-CN/EN | P-EN tables; manual table 1 | 2026-09 | Y | H | — | exact code on nameplate | P-CN now lists RH56F2 |
| Installed interface | RS-485 | lab units | RS485 | U | — | 2026-09-23 | operator | H | Y | nameplate to record | — |
| Connector | 1BZ1G08CLL00000 (≈ LEMO FGG-1B-8P). Pin 3 485_A/CANFD_H, 4 485_B/CANFD_L, 1 GND, 2 VCC, 5–8 EtherCAT | F1 | both | M-CN/EN | §1.4.1 table 2/3 | V1.0/1.0.1 | Y | H | — | — | — |
| Adapter | cable with USB converter on the RS485/CANFD lead (fig. 3); host software opens a serial COM port with a "RS232/485" mode | F1 | RS485 | M, HS-CN | fig. 3; HS-CN fig. 2 | 2026-09-22 | Y | M | — | adapter model | — |
| Supply | 24–48 V; RS485 quiescent 194±10 mA @24 V; peak 5 A @24 V | F1 E2 | — | P-EN, M | P-EN; table 1 | — | Y | H | — | — | M says 0.2 A static, 4.0 A max grip |
| Baud | 115200 default; reg 1001: 0=115200, 1=57600, 2=19200, 3=921600 | F1 | RS485 | M | §2.2, §2.5.2 | both | Y | H | Y | current setting | — |
| Data/parity/stop | 8 / none / 1 | F1 | RS485 | M | §2.2 | both | Y | H | Y | — | — |
| Flow control | none (SDK only) | F1 | RS485 | SDK | `serial_port.cpp:21` | 2026-02 | N | L | Y | — | not in M |
| Bus ID | reg 1000, 1–254, default 1, saved only with SAVE | F1 | RS485 | M | §2.5.1, ex. 1 | both | Y | H | Y | **per-hand ID** | — |
| Left/right on bus | only by ID; no side register | F1 | RS485 | M | §1.4.2, table 30 | both | Y | H | Y | ID↔side | — |
| Header | request EB 90, reply 90 EB | F1 | RS485 | M | tables 4/5/8/9 | both | Y | H | Y (codec) | — | — |
| Commands | 0x11 read, 0x12 write | F1 | RS485 | M | same | both | Y | H | Y | — | — |
| Address | 16-bit word address, LE | F1 | RS485 | M | tables 4, 6 | both | Y | H | Y | — | — |
| Length byte | read req 0x04; write req/read reply = data bytes + 3; write reply 0x04 | F1 | RS485 | M | tables 4/5/8/9 | both | Y | H | Y | — | — |
| Payload | INT16 little-endian per register | F1 | RS485 | M | tables 7, 11 | both | Y | H | Y | — | — |
| Checksum | low byte of sum of bytes from ID to last data byte | F1 | RS485 | M | §2.2.1 | both | Y | H | Y | — | examples 1 and 5 are misprinted (below) |
| Write ACK | 90 EB ID 04 12 AL AH 01 CS | F1 | RS485 | M | table 9, 12 | both | Y | H | Y | — | §3 ex. 2–4 echo address+1 |
| SAVE result | extra frame 1 s later, byte7 00 ok / FF fail | F1 | RS485 | M | table 10 | both | Y | M | not used | — | §3 ex. 4 prints cmd 11 in it |
| Error/NAK frame | **not defined** | F1 | RS485 | M | — | — | — | — | codec rejects anything else | — | — |
| Read/write timeout | not published. SDK waits 25 ms per reply | F1 | RS485 | SDK | `RH56F1_485_protocol.cpp:656,804` | 2026-03 | N | L | N | ask vendor | — |
| Partial frames | not described. SDK scans for 90 EB, then waits for LEN+5 bytes | F1 | RS485 | SDK | `RH56F1_485_protocol.cpp:169-300` | — | N | L | Y (codec does the same, tested) | — | — |
| DOF order | little, ring, middle, index, thumb bend, thumb rotation | F1 | all | M | tables 31–48, 51 | both | Y | H | Y | — | CAN example calls 1066/1042 "index" (M §2.4) |
| ANGLE_SET | 1040, 0.1°, `-1` = DOF does not move; **a valid value moves at once**; not saved | F1 | RS485 | M | §2.5.11 | both | Y | H | Y | — | — |
| ANGLE_ACT | 1064, 0.1°, read-only | F1 | RS485 | M | §2.5.15 | both | Y | H | Y | — | ranges (below) |
| POS_SET/ACT | 1034/1058, stroke 0 (open) … 2000 (closed); vendor advises against POS_SET | F1 | RS485 | M | §2.5.10/14 | both | Y | H | — | — | — |
| FORCE_SET/ACT | 1046/1070, g | F1 | RS485 | M | §2.5.12/16 | both | Y | M | read only | — | ranges differ (below) |
| SPEED_SET | 1052, 0–4000; 2000 ≙ full stroke in 1000 ms unloaded | F1 | RS485 | M | §2.5.13 | both | Y | H | — | — | — |
| CURRENT_ACT | 1076, mA, 0–1500 | F1 | RS485 | M | §2.5.17 | both | Y | H | Y | — | — |
| ERROR | 1082: bit0 stall, 1 over-temp, 2 over-current, 3 motor, 4 comm | F1 | RS485 | M | table 44 | both | Y | H | Y | — | — |
| STATUS | 1088: 0 opening, 1 closing, 2 pos reached, 3 force reached, 5 current prot., 6 stall, 7 fault, 8 E-stop | F1 | RS485 | M | table 46 | both | Y | M | Y | — | code 8 only in M-CN |
| TEMP | 1094, °C, 0–100 | F1 | RS485 | M | §2.5.20 | both | Y | H | Y | — | no limit given |
| Clear error | write 1 to 1003; over-temp not clearable | F1 | RS485 | M | §2.5.3 | both | Y | H | — | — | — |
| PAUSE / E-STOP | 1130 (1 = pause), 1131 (1 = E-stop, other value releases it) | F1 | RS485 | M | table 30 | both | Y | M | not used | — | resume behavior undocumented |
| Enable/disable | none on RS-485 (EtherCAT ENABLE_SET only) | F1 | RS485 | M | tables 30, 51 | both | Y | H | software gate only | — | — |
| Mode | 1100: 0 speed/force-protect, 1 force loop, 2 impedance | F1 | RS485 | M | §2.5.21 | both | Y | H | — | current mode | — |
| Power-on defaults | 1022 speed / 1028 force thresholds applied at power-on; HS-CN shows 2000 / 600 | F1 | RS485 | M, HS-CN | §2.5.8/9; fig. 5 | — | Y | M | — | actual values | whether the hand *moves* at power-on is not stated |
| Performance | four fingers >107°/s, thumb flex >37°/s, rotation >155°/s, closing ≤0.8 s | F1 | — | P-EN | spec table | 2026-09 | Y | H | not a safety limit | — | — |
| "1 kHz real-time communication" | marketing text on P-EN (EtherCAT context) | F1 | EtherCAT | P-EN | intro | — | Y | L | **not** an RS-485 command rate | — | — |
| Sensor refresh | "Sensor data refresh rate after the control board is 50 Hz" | not model-specific | — | FAQ | — | — | Y | L | no | ask vendor | — |

### Raw angle ranges: conflicting official sources

| DOF | ANGLE_SET M-CN V1.0.1 | ANGLE_SET M-EN | ANGLE_ACT M-CN | ANGLE_ACT M-EN | P-EN | HS-CN screenshot (fig. 5) |
|---|---|---|---|---|---|---|
| 4 fingers | 900–1740 | 900–1740 | 900–1740 | **900–1720** | — | 1757, 1764, 1751, 1759 |
| thumb bend | 1100–1350 | 1100–1350 | 1100–1350 | **1200–1450** | — | 1572 |
| thumb rotation | **600–1750** | 600–1800 | 600–1750 | **500–1700** | 60–180° | 1650 |

Force ranges also differ: M-CN gives 0–1500 (thumb 0–2000) for set and actual,
while M-EN gives set 0–1000 and actual −1000…1000 (thumb −1000…1500). The
screenshot values are above both manuals' maxima. **No single official raw
limit set exists**, so none is used as a protocol constant.

### Raw ↔ URDF

| Layer | What it is | Official source |
|---|---|---|
| 1. Vendor finger angle | 0.1° angle between a finger line (P1–P2, P3–P4, P5–P6) and the metacarpal plane (∠E, ∠D, ∠A); larger = more open | M table 36 figures, §2.5.12 |
| 2. Actuator raw | ANGLE_SET/ACT (the angle above) or POS 0–2000 stroke | M §2.5.10–15 |
| 3. URDF independent joint | `…_{thumb_1,thumb_2,index_1,middle_1,ring_1,little_1}_joint` | project URDF |
| 4. Passive/mimic | `…_2` joints etc. (12 joints / 6 DOF, M table 1) | project URDF; coupling law not published |
| 5. Canonical rad | `r_hj_*`/`l_hj_*` via profile | project |

- No official formula maps layer 1/2 to layer 3.
- Span comparison: URDF thumb_1 spans 120°, which equals the 60–180° rotation
  span in M-EN/P-EN, and thumb_2 spans 27.2°, close to the 25° bend span. That
  *suggests* thumb_1 = rotation and thumb_2 = bend, but it is an inference, not
  evidence. The URDF finger span (87.6°) matches neither 84° nor 82°.
- Sign, offset and left/right differences: M does not distinguish left from
  right hands anywhere.
- No calibration procedure beyond force-sensor zeroing (reg 1007).
- **Linear interpolation between raw min/max and URDF limits is not
  justified**, and it is not implemented.

## Manual inconsistencies (both editions)

- §3 example 1 `EB 90 01 05 12 E8 03 02 00 06`: the rule gives checksum 05.
- §3 example 5 `EB 90 01 05 12 F1 03 01 00 0B`: the printed checksum 0B is
  correct for address **1007** (`EF 03`, table 30). The printed address `F1 03`
  (1009) is the inconsistent byte. (The official FAQ mentions register 1009 for
  RH56E2 force calibration.) *The previous revision of this document wrongly
  called the checksum the misprint.*
- §3 examples 2–4: the ACK echoes address+1, contradicting table 9.
- §2.4 CAN example labels 1066/1042 "index", while tables 35/40 assign them to
  the middle finger.
- §2.5.11 example text: "1200,1200,-1,1500,…" then "index moves to 1200".

## Golden vectors (official, both editions)

| Frame | Bytes | Used in test |
|---|---|---|
| Read ANGLE_ACT (table 6, ex. 9) | `EB 90 01 04 11 28 04 0C 4E` | `OfficialGoldenReadAngleActRequest` |
| Reply (table 7) | `90 EB 01 0F 11 28 04 E8 03 E8 03 E8 03 E8 03 78 05 E8 03 61` | `OfficialGoldenReadAngleActResponse` |
| Write ANGLE_SET (table 11) | `EB 90 01 0F 12 10 04 E8 03 E8 03 E8 03 E8 03 78 05 E8 03 4A` | `OfficialGoldenWriteAngleSetRequest` |
| ACK (table 12, ex. 8) | `90 EB 01 04 12 10 04 01 2C` | `OfficialGoldenWriteAcknowledgement` |
| Ex. 6/7/8 (FORCE/SPEED/ANGLE_SET = 1000) + ACKs | CS BE/32, C4/38, B8/2C | `OfficialGoldenSection3Examples6To8` |

All other codec tests use frames constructed in the test, with checksums from
an independent helper. They are not labelled as official.

## Classification

**A. Settled from official online/manual sources:** frame format, commands,
LE encoding, checksum, register map, DOF order, bus ID mechanism (1–254,
default 1), serial settings and default baud, error bits, status codes, clear
error, PAUSE/E-STOP registers, absence of an RS-485 enable register, speed
register meaning, model-code meaning (E2/E4/L/R/T1).

**B. From the units, nameplates, cables or order (user):**
1. Exact model of each hand: RH56F1-E2R / E2L, with or without -T1, and serial numbers.
2. ~~Adapter / port~~: **resolved** (FT232R BG0327KL left, BG033STU right, separate adapters).
3. ~~Shared bus~~: **resolved** (separate buses).

**C. From unit settings or from Inspire:**
1. Configured bus ID and baud register of each hand. The factory default is ID 1 / 115200. The hands are on separate adapters, so equal IDs are fine, but the actual value is unverified. The planned read-only `rh56f1_diag` run answers this for a given ID (a reply means the ID and baud match), without any scan.
2. Official raw↔URDF-joint relation (offset, sign, thumb_1/thumb_2 ↔ rotation/bend, mimic coupling) or an accepted calibration procedure.
3. Which raw range applies to our firmware (M-CN vs M-EN vs screenshot values).
4. Recommended command and poll rate, reply timeout, behavior on communication loss (hold last target?), power-on behavior (does it move?), PAUSE/E-STOP resume semantics.
5. Whether RH56F1 is superseded by RH56F2 (CN site), and the current manual revision.

## Gate 5B-1 (offline codec): PASS → implemented

Frame, checksum, endian, read/write, reply validation and DOF-group decoding
are all official and covered by golden vectors. An official error frame does
not exist, so the codec rejects every non-conforming reply with a specific
status: bad header/length/checksum, wrong ID/command/address/payload length,
or write not acknowledged.

## Gate 5B-2 (real transport in ros2_control): FAIL

| # | Item | Status |
|---|---|---|
| 1 | Exact E2/E4 model of the lab units | Partial: E2 by U; R/L/T1 and serial numbers not recorded (B1) |
| 2 | Interface | PASS (U) |
| 3 | Frame/register | PASS |
| 4 | Checksum | PASS |
| 5 | Position read/write | PASS (registers) |
| 6 | Left/right identification method | PASS (bus ID) |
| 7 | ID of each hand | **FAIL** (C1). Adapters and paths are known; the IDs are not. |
| 8 | Six-actuator order | PASS (vendor order); URDF assignment of thumb_1/thumb_2 **FAIL** (C2) |
| 9 | Raw ↔ canonical rad | **FAIL** (C2) |
| 10 | Sign and offset | **FAIL** (C2) |
| 11 | Position limits | **FAIL**: official ranges conflict (C3) |
| 12 | Safe command/poll rate | **FAIL** (C4) |
| 13 | Timeout/stale criteria | **FAIL**: SDK 25 ms only (C4) |
| 14 | Power-on and comm-loss behavior | **FAIL** (C4) |
| 15 | Command enable or equivalent software gate | PASS: no device enable exists; the Stage 4 software gate (explicit activation, dwell, fresh reads, fault latch) is in place |

## Stage 5B-2a/2b: what is built (not connected to the plugin)

| Layer | File | Role | Tested by |
|---|---|---|---|
| Protocol constants | `include/rh56f1_hardware/protocol.hpp`, `src/protocol.cpp` | official registers, DOF order, error bits, status codes | codec tests |
| Frame codec | `rs485_codec.*` | pure encode/decode | `test_rs485_codec` (19) |
| Serial I/O | `serial_port.*` | explicit path/baud, 8N1 raw, no flow control, `flock` + `TIOCEXCL` single owner, poll-based partial read/write with EINTR/EAGAIN, input discard, fd closed on every error path and in the destructor, no bytes on open | `test_serial_transport` |
| Register client | `register_client.*` | one request, one validated reply (ID/cmd/address/length/checksum); stale input discarded before sending; foreign frames skipped; noise resync; NAK distinct from timeout/disconnect; **writes disabled by default**; no reconnect | `test_serial_transport` (PTY fake) |
| Diagnostic | `diag.*`, `rh56f1_diag_main.cpp` → `ros2 run rh56f1_hardware rh56f1_diag` | read-only raw snapshot | `test_diag` |
| Test fixture | `test/fake_rh56f1.hpp` | PTY fake RH56F1 whose request parser and reply builder are independent of the codec | — |

- These are all built into the static library `rh56f1_rs485`, which is linked
  only into `rh56f1_diag` and the tests. `librh56f1_hardware.so` contains no
  codec/serial/client symbols (checked with `nm`).
- The PTY fake covers: normal read with the exact official frame, fragmented
  reply, noise prefix, stale input before the request, stale frame before the
  good one, two replies in one burst, truncated frame, bad header/length/
  checksum, wrong ID/command/address, silence (timeout), peer hang-up, write
  ACK/NAK (tests only, using FORCE_SET on the fake), left/right independence
  with one side silent, second-owner refusal, and no fd leak.
- Tool limits in `rh56f1_diag` (≤ 20 samples, 0.5–10 s period, 100 ms default
  reply wait) are conservative choices for a diagnostic, **not vendor values**.

## Placeholder values still in use (not RH56F1 values)

`openarm_description/config/rh56f1/real_bringup_safety.yaml` `hand:`:
2.0 rad/s, 0.02 rad per write (≤ 0.2 rad/s effective), 0.1 s write period,
0.02 s poll, 0.25 s stale, 2.0 s dwell, 3 fresh reads. All are Tesollo- or
project-derived.
