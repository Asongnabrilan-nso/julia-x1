# 🎮 Julia-x2 Joy — handheld controller

WiFi joystick controller for the julia-X2 robot. Seeed XIAO ESP32C3, PlatformIO + Arduino framework.

## Changelog

Newest first. Every change to the controller gets an entry here.

| Date | Version | Change |
|---|---|---|
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
    └── joystick.cpp/.h   reading, filtering, calibration
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

A working setup prints:

```
=== Julia-x2 Joy  fw 1.0.0 ===
[WiFi] connecting to "MyHomeWiFi"...
[WiFi] connected  ip=192.168.1.47  rssi=-52 dBm
[OTA] ready: julia-x2-joy.local:3232
```

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
| `No section: 'secrets'` | `secrets.ini` is missing, isn't in `controller/`, or doesn't start with the `[secrets]` line. |
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

Reserved for later steps: OLED SDA = D4, SCL = D5; buzzer = D10.

Why these pins: on the C3, ADC2 (GPIO5/A3) can't be read while WiFi is running, and GPIO2 (A0) is a boot strapping pin.
To change a pin, edit `include/config.h`.

### Setup

1. Create `secrets.ini`. See [Creating `secrets.ini`](#creating-secretsini).
2. **The first flash must be over USB:** `pio run -e usb -t upload`, then `pio device monitor`.
   If the upload fails, hold **BOOT**, tap **RESET**, release BOOT, and retry.
3. The boot log prints `[WiFi] connected ip=...` and `[OTA] ready: julia-x2-joy.local:3232`.

### Test 1: OTA

1. Change `fw_version` in `platformio.ini` (e.g. `1.0.1`).
2. Run `pio run -e ota -t upload`. It should reach 100% and the board should reboot.
3. Unplug USB, power the board from a battery, and repeat.
4. Check the boot banner (`=== Julia-x2 Joy fw 1.0.1 ===`) or send `i`.

If something goes wrong:
- `OTA_CONNECT_ERROR` or the upload stalls at 0%: the Windows firewall is blocking the incoming connection from the board. Allow Python (PlatformIO's `penv\Scripts\python.exe`) on private networks.
- `.local` doesn't resolve: use the printed IP, `pio run -e ota -t upload --upload-port 192.168.x.y`.
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
| s | stream on/off |
| p | print calibration |
| r | recenter |
| c | start / finish range calibration |
| x | reset calibration (erases NVS) |
