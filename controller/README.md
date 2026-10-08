# 🎮 Julia-x2 Joy — handheld controller

WiFi joystick controller for the julia-X2 robot. Seeed XIAO ESP32C3, PlatformIO + Arduino framework.

## Changelog

Newest first. Every change to the controller gets an entry here.

| Date | Version | Change |
|---|---|---|
| 2026-10-08 | 1.1.0 | **Robot control.** The controller now joins the robot dashboard's Socket.IO server (UNO Q, port 7000) as one more client. It sends the web joystick's own `joystick {x,y}` / `stop` messages, so the analog mapping, the dashboard speed slider, manual vs self-balancing handling and arming checks all come from the existing `python/main.py`. **No robot-side changes.** About 16 Hz while deflected, the same 0.15 radial dead zone, `stop` on release, held until the stick is centered after each (re)connect. The link runs in its own FreeRTOS task (connect timeout 2 s). New `robot_host` / `robot_port` in `platformio.ini`. **Buzzer** on D6: beep on button press, chirps on link up/down, double-beep entering TEST mode. **DRIVE/TEST run modes:** DRIVE by default; hold the button at power-up or send `t` for TEST (the step-1 bench tools, never sends to the robot). Calibration commands are TEST-only. OLED drive screen: robot mode/state, ToF distance (with avoidance limit), robot action, sent command, dashboard speed limit. Libraries: WebSockets 2.6.1, ArduinoJson 7.4.2. Builds; **not yet flashed or tested against the robot** (controller off USB, robot not found on the LAN). |
| 2026-10-08 | 1.0.3 | OLED status screen (U8g2 2.36.18, SSD1306 128×64 I2C assumed; SH1106 is a one-line switch in `src/display.cpp`). Shows a stick-position box (dot filled while the button is pressed), calibrated X/Y, button state, direction, WiFi signal bars, and a CALIBRATING view with raw values. The I2C bus is probed at 0x3C/0x3D at boot; with no display found it logs `[OLED] no display found` and the firmware runs normally. OTA start/failure messages are shown on screen. **Built, not yet flashed:** the board disconnected from USB (COM18 vanished) before the upload. |
| 2026-10-08 | 1.0.2 | Added a `[STATUS]` heartbeat every 10 s (fw, SSID, IP, RSSI, OTA state), because the boot log is printed before the USB monitor attaches and the IP was never visible. Orientation set from the hardware test: `JOY_SWAP_XY`, `JOY_INVERT_X`, `JOY_INVERT_Y` = `true`. RSSI on this run: −83/−84 dBm (marginal). |
| 2026-10-08 | 1.0.1 | Added the `w` serial command (WiFi scan that marks the configured SSID and shows RSSI) to diagnose `not connected (status 1)`. USB upload speed lowered from 921600 to 460800 after a mid-upload failure. Documented running `pio` when it isn't on PATH. First hardware run: USB flash OK, joystick reads OK, WiFi connected at about −72 dBm. First OTA attempt failed with `No response from the ESP`: the PC was on `HIVC2-5GHZ`, the board on `HIVC2.4G`, and the router doesn't pass traffic between them. |
| 2026-10-08 | docs | Troubleshooting: added the "no section headers" error (`[secrets]` line missing from `secrets.ini`). |
| 2026-10-08 | docs | Added the step-by-step `secrets.ini` guide and this changelog. |
| 2026-10-08 | 1.0.0 | Step 1: WiFi + password-protected OTA (`usb` / `ota` build envs), joystick reading (oversampled, smoothed, −1000…1000 with dead zone), button click/long-press, per-half-axis range calibration saved to NVS, center re-measured at boot, serial console. |

## Project layout

```
controller/
├── platformio.ini        build config: usb + ota environments
├── secrets.ini.example   template for your credentials (committed)
├── secrets.ini           your real credentials (gitignored, you create it)
├── include/config.h      pins and tuning constants
└── src/
    ├── main.cpp          setup/loop, serial console, button handling
    ├── ota.cpp/.h        WiFi + ArduinoOTA
    ├── joystick.cpp/.h   reading, filtering, calibration
    ├── display.cpp/.h    OLED drive + test screens
    ├── buzzer.cpp/.h     non-blocking beeps
    └── robot_link.cpp/.h Socket.IO link to the robot dashboard (own task)
```

## Creating `secrets.ini`

`secrets.ini` holds your WiFi network and OTA password. `platformio.ini` reads it in two places:

- **At build time:** the values are compiled into the firmware so the controller can join WiFi and check the OTA password.
- **At OTA upload time:** your PC sends `ota_password` to the board. If it doesn't match the password in the running firmware, the board rejects the upload.

The file is gitignored, so your passwords never reach git. **Nothing builds until it exists.** Without it you get:

```
InvalidProjectConfError: ... 'No section: 'secrets''
```

### Step 1: copy the template

From the `controller/` folder, in the VS Code terminal (PowerShell):

```powershell
Copy-Item secrets.ini.example secrets.ini
```

Or in VS Code's Explorer: right-click `secrets.ini.example` → Copy, paste it, and rename the copy to `secrets.ini`.

### Step 2: fill in your values

Open `secrets.ini` and replace the placeholders. Example:

```ini
[secrets]
wifi_ssid = MyHomeWiFi
wifi_password = sunflower2024
ota_password = joy-ota-7731
```

| Key | What to put | Example |
|---|---|---|
| `wifi_ssid` | Name of the WiFi network the **robot (UNO Q) is on**. The controller must be on the same network to reach it later. Case-sensitive. | `MyHomeWiFi` |
| `wifi_password` | That network's password. Leave it empty for an open network. | `sunflower2024` |
| `ota_password` | A password **you make up**. It protects wireless flashing, so nobody else on the network can flash your controller. It isn't your WiFi password. | `joy-ota-7731` |

Rules:
- **No quotes.** Write `wifi_ssid = My Home WiFi`, not `wifi_ssid = "My Home WiFi"`. Quotes become part of the value and break the build.
- **Spaces are fine** in the SSID and passwords, e.g. `wifi_ssid = My Home WiFi`. Spaces around `=` and at the ends of a value are ignored.
- **Special characters** (each tested against a real build):

  | Character | Works? | How to write it |
  |---|---|---|
  | `@ ! # & * - _ .` and `;` with no space before it | ✅ | as-is |
  | `%` | ✅ with escape | write it twice: `50%off` → `50%%off` |
  | space followed by `;` | ❌ | the ` ;` starts a comment and **silently cuts off** the rest: `ab ;cd` becomes `ab` |
  | `$` | ❌ | **silently dropped**, along with the text after it (`ab$cd` → `ab`). `$$` doesn't help. |
  | `"` or `\` | ❌ | breaks the C string |

  A silently cut-off password still builds fine, but WiFi then never connects. If your **WiFi** password needs a ❌ character, tell me and I'll switch secrets to a generated header that accepts anything. For `ota_password` you choose the value, so stick to letters, digits and `-`.
- **2.4 GHz only.** The ESP32-C3 can't see 5 GHz networks. On a dual-band router with a separate 5 GHz name, use the 2.4 GHz one.

More examples:

```ini
; Network name with spaces
[secrets]
wifi_ssid = Robotics Lab 2.4G
wifi_password = l@b-p4ss!
ota_password = julia-joy
```

```ini
; Open network (no WiFi password)
[secrets]
wifi_ssid = Workshop-Guest
wifi_password =
ota_password = julia-joy
```

```ini
; Disable WiFi/OTA entirely (USB-only bench testing): leave the SSID empty
[secrets]
wifi_ssid =
wifi_password =
ota_password =
```

With an empty SSID the board prints `[WiFi] no SSID configured (secrets.ini) - OTA disabled` and runs joystick-only.

### Step 3: check the file is ignored by git

```powershell
git status --short
```

`secrets.ini` must **not** be listed. `secrets.ini.example` is the committed template, so keep real passwords out of it.

### Step 4: build, flash over USB, and confirm

```powershell
pio run -e usb -t upload
pio device monitor
```

You usually **won't see the boot log** (`=== Julia-x2 Joy ===`, `[WiFi] connected ip=...`). It's printed in the first 1–2 seconds after reset, before the monitor has attached to the XIAO's USB port. Instead, wait up to 10 s for the status heartbeat, or send `i` at any time:

```
[STATUS] fw 1.0.2 | WiFi "HIVC2.4G" ip 192.168.1.226 -84 dBm | OTA julia-x2-joy.local ready
```

(The values are from the first real run; yours will differ.) If it says `NOT connected`, send `w` to scan. See [Troubleshooting secrets](#troubleshooting-secrets).

Signal strength guide: −50 to −65 dBm is good, −65 to −75 is OK, and **below −75 is marginal**: expect dropouts and lag, which matter for driving. Check the antenna is attached and move closer to the router.

Write down the IP. You'll need it if `.local` names don't resolve on your PC.

### Step 5: flash over WiFi

```powershell
pio run -e ota -t upload
# or, by IP:
pio run -e ota -t upload --upload-port 192.168.1.47
```

### Changing secrets later

- **New WiFi network:** edit `wifi_ssid` / `wifi_password`, then reflash. If the board can't reach the old network any more, reflash over **USB**.
- **New OTA password:** the board still holds the *old* one. Either flash over USB, or put the old password back in `secrets.ini`, flash over OTA once with the new password compiled in, then change the file to the new one.

### Troubleshooting secrets

| Symptom | Cause / fix |
|---|---|
| `No section: 'secrets'` | `secrets.ini` is missing or isn't in `controller/`. |
| `Invalid 'secrets.ini' ... File contains no section headers. line: 1` | The `[secrets]` line is missing. It must be the first line of the file, above the three keys. |
| `No option 'wifi_password' in section: 'secrets'` | A key is misspelled or missing. All three keys must be present, even if empty. |
| `'%' must be followed by '%' or '('` | A single `%` in a value. Write `%%`. |
| Build error near `WIFI_SSID` / `WIFI_PASSWORD` | A `"` or `\` in a value. |
| Stuck at `[WiFi] connecting to ...` | Wrong SSID or password (both case-sensitive), a password cut off by `$` or ` ;` (see the table above), a 5 GHz network, or weak signal. Send `i` to see the status code. |
| OTA upload: `Authentication Failed` | `ota_password` in `secrets.ini` doesn't match the password compiled into the firmware running on the board. |

## Step 1: OTA + joystick reading/calibration (firmware v1.0.0)

### Wiring

| Joystick module | XIAO ESP32C3 | Note |
|---|---|---|
| GND | GND | |
| +5V / VCC | **3V3** | **Do not use 5V**: the ADC pins are not 5 V tolerant |
| VRx | D1 / A1 (GPIO3) | ADC1 |
| VRy | D2 / A2 (GPIO4) | ADC1 |
| SW | D3 (GPIO5) | internal pull-up, active low |

| OLED module | XIAO ESP32C3 | Note |
|---|---|---|
| GND | GND | |
| VCC | 3V3 | |
| SDA | D4 (GPIO6) | I2C |
| SCL | D5 (GPIO7) | I2C |

| Buzzer | XIAO ESP32C3 | Note |
|---|---|---|
| + (signal) | D6 (GPIO21) | passive buzzer by default; set `BUZZER_ACTIVE = true` in `config.h` for an active one |
| − | GND | |

D6 is also the chip's UART TX pin, so the buzzer may click briefly at power-up. That's harmless. A buzzer drawing more than about 20 mA needs a transistor driver.

Why these pins: on the C3, ADC2 (GPIO5/A3) can't be read while WiFi is running, and GPIO2 (A0) is a boot strapping pin.
To change a pin, edit `include/config.h`.

### Running `pio`

If PowerShell says `The term 'pio' is not recognized`, PlatformIO is installed but not on your PATH. The VS Code extension only adds it inside its own terminal. Pick one:

- **PlatformIO terminal (easiest):** in the PlatformIO sidebar (alien icon), go to *Quick Access → Miscellaneous → New Terminal*. `pio` works there.
- **Full path, any terminal:**
  ```powershell
  & "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e usb -t upload
  ```
- **Add it to PATH permanently**, then open a new terminal:
  ```powershell
  [Environment]::SetEnvironmentVariable("Path", [Environment]::GetEnvironmentVariable("Path","User") + ";$env:USERPROFILE\.platformio\penv\Scripts", "User")
  ```

To find the board's port, run `pio device list`. The XIAO shows up as `USB VID:PID=303A:1001`. Bluetooth COM ports can't be used for uploads, so if several ports are listed, add `--upload-port COMxx`.

### Setup

1. Create `secrets.ini`. See [Creating `secrets.ini`](#creating-secretsini).
2. **The first flash must be over USB:** `pio run -e usb -t upload`, then `pio device monitor`.
   If the upload fails, hold **BOOT**, tap **RESET**, release BOOT, and retry.
3. Within 10 s the monitor shows a `[STATUS] ... ip ... OTA ... ready` line (the boot log itself scrolls past before the monitor attaches). Send `i` to print it on demand.

### Test 1: OTA

1. Change `fw_version` in `platformio.ini` (e.g. `1.0.1`).
2. Run `pio run -e ota -t upload`. It should reach 100% and the board should reboot.
3. Unplug USB, power the board from a battery, and repeat.
4. Check the boot banner (`=== Julia-x2 Joy fw 1.0.1 ===`) or send `i`.

If something goes wrong:
- `OTA_CONNECT_ERROR` or the upload stalls at 0%: the Windows firewall is blocking the incoming connection from the board. Allow Python (PlatformIO's `penv\Scripts\python.exe`) on private networks.
- `No response from the ESP`: the PC can't reach the board at all, and `ping <board ip>` gives `Destination host unreachable`. This happens on dual-band routers that isolate the 2.4 GHz network from the 5 GHz one, or that have "AP/client isolation" turned on. Put the PC on the same 2.4 GHz network as the controller, or turn off isolation in the router. **The robot must also be reachable from the controller's network**, or drive commands will be blocked the same way.
- `.local` doesn't resolve: use the printed IP, `pio run -e ota -t upload --upload-port 192.168.x.y`.
- `i` shows `not connected (status 1)`: the board can't see the network. Send `w`. If your SSID isn't marked `*`, check its spelling and case and that it's 2.4 GHz. If it's marked but weaker than about −75 dBm, make sure the antenna is clicked into the U.FL connector and move closer to the router. Status 4 means it found the network but the connection failed, usually a wrong password.
- WiFi won't associate: some XIAO C3 boards need lower TX power. Add `WiFi.setTxPower(WIFI_POWER_8_5dBm);` after `WiFi.mode()` in `src/ota.cpp`.
- USB always works as the recovery path, even if a bad OTA image breaks WiFi.

### Test 2: raw reading and orientation

Serial streams at 20 Hz:
```
raw X=2041 Y=1987 | x=    0 y=    0 | btn=0 | STOP
```
- At rest, `raw` should sit near mid-scale (~1500–2300) and `x`/`y` should read `0`.
- Push **away from you**. You should see `y → +1000` and `FORWARD`. If you get `x` instead, set `JOY_SWAP_XY = true`. If the sign is wrong, set `JOY_INVERT_Y = true`.
- Push **right**. You should see `x → +1000` and `RIGHT`. If the sign is wrong, set `JOY_INVERT_X = true`.
- Press the stick: `btn=1`, and `[BTN] click` on release.

### Test 3: calibration

1. Long-press the stick for 1.5 s, or send `c`.
2. Sweep the stick around its full edge 3–4 times, then release it.
3. Click the stick, or send `c` again. The range is checked (each direction must travel at least 400 counts from center) and saved to NVS, so it survives reboots and OTA updates.
4. Check that each direction now reaches ±1000 at full throw and returns to exactly 0 at rest.

The center is measured again at every boot, so keep the stick released at power-on. Send `r` to recenter later, `p` to print the calibration, and `x` to reset it to defaults.

| Key | Action |
|---|---|
| h | help |
| i | firmware / WiFi / OTA info |
| w | scan WiFi networks: the configured one is marked `*`; shows channel and RSSI |
| s | stream on/off |
| p | print calibration |
| r | recenter |
| c | start / finish range calibration |
| x | reset calibration (erases NVS) |

### Test 4: OLED display (fw 1.0.3)

From fw 1.1.0 this stick-box screen is the **TEST mode** screen: hold the button at power-up, or send `t`. DRIVE mode shows the robot screen (Step 2).

1. Wire the OLED as in the table above, then flash over USB.
2. The serial log should show `[OLED] ready at 0x3C` (or `0x3D`). Send `i` or wait for the `[STATUS]` line if you missed it.
3. The screen shows:
   ```
   Julia-x2 Joy                  ▂▄▆█   <- WiFi signal bars ("--" when disconnected)
   ──────────────────────────────────
   ┌──────────┐  X     0
   │    +  o  │  Y     0              <- o = stick position, filled while pressed
   │          │  BTN off
   └──────────┘  STOP
   ```
4. Move the stick. The dot should follow your thumb, numbers should reach ±1000 at full throw, and the direction word should match.
5. Long-press to start range calibration. The screen switches to **CALIBRATING** and shows raw ADC values.

If something goes wrong:
- `[OLED] no display found`: check SDA/SCL aren't swapped and VCC is on 3V3.
- Display found but the screen is garbled or shifted 2 px: it's an SH1106 (common on 1.3" modules). In `src/display.cpp`, change `U8G2_SSD1306_128X64_NONAME_F_HW_I2C` to `U8G2_SH1106_128X64_NONAME_F_HW_I2C`.
- Upside down: change `U8G2_R0` to `U8G2_R2` in the same line.

## Step 2: driving the robot (firmware 1.1.0)

### How it works

```
 Julia-x2 Joy (XIAO)  ──WiFi──►  UNO Q python/main.py (Socket.IO, :7000)  ──Bridge──►  MCU sketch
   "joystick" {x,y} ~16 Hz          on_joystick(): dead zone, speed slider,           wheels
   "stop" on release                 MANUAL → drive(L,R)  /  BALANCE → balance_cmd
   ◄── "balance" "tof" "action" telemetry (mode, state, distance, direction)
```

The controller is just another dashboard client, so it drives the robot **exactly like the web joystick**:
- **Speed:** the further you push, the faster the robot goes, up to the dashboard's *max speed* slider.
- **Manual mode:** arcade drive straight to the wheels.
- **Self-balancing mode:** forward/steer inputs to the balance loop. It only responds once the robot is **armed on the dashboard**; until then the OLED shows `arm on dashboard`.
- **Unchanged on the robot:** mode switching, arming, IMU calibration and all tuning stay on the web dashboard. The robot keeps its own safety: if commands stop for 500 ms (controller off, WiFi lost) it stops, and ToF obstacle avoidance still limits forward motion.

Don't drive from the dashboard and the controller at the same time. Both send commands, and the robot follows whichever arrived last.

### Setup

1. Find the robot's address: on the UNO Q run `hostname -I`, or use the address you open the dashboard with (`http://<ip>:7000`).
2. In `platformio.ini` under `[joy]`, set it, e.g. `robot_host = 192.168.1.50` (or `robot_host = myunoq.local`).
3. **The controller must be able to reach the robot.** Put both on the same 2.4 GHz WiFi. On this router the 2.4 GHz and 5 GHz networks don't talk to each other (see the OTA troubleshooting).
4. Flash, and power up without touching the button.

### Run modes

| Mode | How to enter | What it does |
|---|---|---|
| **DRIVE** (default) | normal power-up, or send `t` from TEST | drives the robot; OLED shows robot telemetry |
| **TEST** | hold the stick button while powering up, or send `t` | step-1 bench tools: value stream, test screen, calibration. **Sends nothing to the robot.** |

### Drive screen

```
MANUAL                    ▂▄▆█    <- robot mode/state (BALANCE: on/armed/off, FAULT, Connecting...)
──────────────────────────────
ToF  342 mm                60%    <- distance; right side = forward speed allowed by avoidance (STOP at 0)
Robot: Driving forward            <- what the robot reports it's doing
Cmd: FORWARD 80%                  <- what the controller is sending (release stick = after reconnect)
max speed 100%                    <- dashboard slider / "arm on dashboard" / "disarm on dashboard"
```

ToF shows `--` with no telemetry, `n/a` when no sensor is connected, `>1.2m` when nothing is in range, and `err` on a sensor timeout.

### Buzzer

| Sound | Meaning |
|---|---|
| short beep | button pressed |
| rising two-tone | connected to the robot |
| falling two-tone | link to the robot lost |
| double beep | TEST mode entered |

### Test 5: buzzer (no robot needed)

1. Press the stick button. You should hear a short beep each time.
2. If it's silent: check D6/GND. If it only clicks, you probably have an active buzzer: set `BUZZER_ACTIVE = true`. If the tone is weak, try `BUZZER_FREQ_HZ` between 2000 and 4000.

### Test 6: robot link (wheels off the ground first)

1. Run the robot app. **Prop the robot up so the wheels spin freely.**
2. Power the controller without touching the button. Expect the rising chirp, and the header changes from `Connecting...` to `MANUAL`.
3. Serial `i` prints `[ROBOT] 192.168.1.50:7000 connected | mode 0 state 0 | ToF ...`.
4. Push the stick gently. `Cmd: FORWARD 20%` should appear, the wheels turn slowly, and the dashboard shows the same action. Push harder and the wheels go faster. Release: they stop.
5. Turn the robot off, or walk out of range. Expect the falling chirp, `Connecting...`, and the wheels stop within 0.5 s (robot-side timeout).
6. Switch to self-balancing on the dashboard. The OLED shows `BALANCE: off` / `arm on dashboard`, and stick input is ignored until the robot is armed.
7. Hold an object in front of the ToF sensor. The distance updates, and the forward % drops to STOP at the dashboard's stop distance.
