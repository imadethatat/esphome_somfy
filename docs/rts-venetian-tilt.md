# RTS Venetian blind tilt

This document describes the hardware-verified 80-bit RTS support for Venetian
blinds controlled by a Somfy Telis 4 Mod/Var remote. Position remains the blind
height; tilt is exposed independently as the slat angle in ESPHome and Home
Assistant.

## Verified setup

- ESPHome 2026.8.0
- ESP32-WROOM using ESP-IDF
- CC1101 at 433.42 MHz, ASK/OOK
- Somfy Telis 4 Mod/Var
- 72-second opening and closing travel
- individual physical remotes plus one group remote

Both ordinary lift commands and wheel commands from this remote are 80-bit RTS
frames. A Venetian cover therefore sends 80-bit UP, DOWN, MY, STEP_UP, and
STEP_DOWN frames. Covers without `tilt_steps` continue to use the unchanged
56-bit transmitter.

## Configuration

Enable tilt independently on each cover by setting `tilt_steps`. The value is
the number of wheel detents between the two useful slat endpoints.

```yaml
external_components:
  - source: github://YOUR_GITHUB_USER/esphome_somfy@feature/rts-venetian-tilt
    components: [somfy]
    refresh: 0s

logger:
  level: DEBUG
  logs:
    somfy.rts.hub: DEBUG
    somfy.rts: DEBUG

cover:
  - platform: somfy
    type: rts
    id: blind_left
    name: "Blind Left"
    device_class: blind
    somfy_id: rts_radio
    open_duration: 72s
    close_duration: 72s
    remote_code: 0xA1B2C1
    storage_key: BlindLeft
    prog_button: prog_left
    detected_remote: somfy_rx_last
    tilt_steps: 12
    tilt_inverted: false
    allowed_remotes:
      - 0x112233
      - 0x445566

  - platform: somfy
    type: rts
    id: blind_centre
    name: "Blind Centre"
    device_class: blind
    somfy_id: rts_radio
    open_duration: 72s
    close_duration: 72s
    remote_code: 0xA1B2C2
    storage_key: BlindCentre
    prog_button: prog_centre
    detected_remote: somfy_rx_last
    tilt_steps: 12
    tilt_inverted: false
    allowed_remotes:
      - 0x223344
      - 0x445566

  - platform: somfy
    type: rts
    id: blind_right
    name: "Blind Right"
    device_class: blind
    somfy_id: rts_radio
    open_duration: 72s
    close_duration: 72s
    remote_code: 0xA1B2C3
    storage_key: BlindRight
    prog_button: prog_right
    detected_remote: somfy_rx_last
    tilt_steps: 12
    tilt_inverted: false
    allowed_remotes:
      - 0x334455
      - 0x445566
```

The three `remote_code` values above are already-paired virtual transmitters.
Never change a paired `remote_code`, and preserve each cover's unique
`storage_key`. The rolling-code storage format and keys are shared with the
standard RTS implementation and are not reset when tilt is enabled.

The group remote is included in every cover's `allowed_remotes`, so one group
wheel event updates each cover's independent tilt estimate.

## Individual calibration

Calibration is per cover. Different blinds may use different values:

```yaml
# Left
tilt_steps: 12
tilt_inverted: false

# Centre
tilt_steps: 11
tilt_inverted: false

# Right
tilt_steps: 13
tilt_inverted: true
```

To calibrate:

1. Move one blind to a useful slat endpoint with its physical wheel.
2. Count individual detents until it reaches the other endpoint.
3. Set that count as `tilt_steps` for this cover.
4. Move the Home Assistant tilt control by one step (`100 / tilt_steps` percent).
5. If the reported percentage moves opposite to the desired convention, set
   `tilt_inverted: true`.

RTS provides no absolute motor feedback. Position and tilt are estimates and
can drift if a command is missed or the blind is moved while the ESP32 is
offline. A physical group event applies the same relative step to all covers;
it does not force mechanically different blinds to an identical absolute
angle.

## Captured frame structure

The direction bit was determined from two consecutive one-detent captures from
an anonymized remote `0x112233`, not inferred solely from another
implementation. The remote address was replaced without changing the captured
command, rolling-code, or extension fields:

```text
wheel up:
RTS80 decoded: B3 B6 03 F9 11 22 33 84 30 1E

wheel down:
RTS80 decoded: B4 B2 03 FA 11 22 33 84 38 16
```

The base command is `0x0B` in both frames. Bit 3 of extension byte 8 is the
only direction-bearing difference:

- clear (`0x30`): protocol `STEP_DOWN` (`0x0B`)
- set (`0x38`): protocol `STEP_UP` (`0x8B`)

The physical wheel labels are opposite to the protocol names in this captured
installation: wheel-up produced protocol STEP_DOWN, and wheel-down produced
protocol STEP_UP. `tilt_inverted` maps protocol direction to the desired Home
Assistant percentage direction.

The seven-bit step magnitude is assembled from extension byte 8 bits 0..2 and
extension byte 9 bits 4..7. Both captures carry magnitude one. Bytes 7..9 have
a separate nibble checksum and, unlike bytes 0..6, are not XOR-chain
obfuscated.

## Behaviour and rolling codes

- STEP_UP and STEP_DOWN update tilt only and never start lift position travel.
- Physical UP/DOWN/MY commands continue to update or stop the height estimate.
- A Home Assistant tilt target is quantized to `tilt_steps`.
- Multiple required detents are encoded as one native step-magnitude command.
- One tilt target therefore consumes exactly one rolling code.
- Venetian UP/DOWN/MY commands use the 80-bit repeat framing expected by the
  motor, including repeat extension bytes and their checksums.
- Ordinary covers without `tilt_steps` retain their original 56-bit behaviour.

## On-device verification

After changing the external component, run **Clean Build Files**, compile, and
install the firmware. Test one blind before testing the group:

1. Send 0% position and verify full closing.
2. Send 100% position and verify full opening.
3. Send 50% and verify a proportional 36-second movement with 72-second travel.
4. Press MY during movement and verify an immediate stop.
5. Move tilt by one configured step in each direction.
6. Send 0%, 50%, and 100% tilt and verify both endpoints and the midpoint.
7. Repeat using the physical individual remote.
8. Repeat using the configured group remote and verify all three estimates update.

Useful logs include:

```text
RTS80 air: ...
RTS80 decoded: ...
RTS80 fields: remote=... command_base=... extension=... byte8_bit3=...
80-bit lift TX: command=... rolling=...
Tilt TX: STEP_..., ... step(s), rolling=...
RX tilt: STEP_..., ... step(s) -> ...%
```

## Upstream contribution notes

Keep the implementation commits separate when preparing a pull request:

1. complete 80-bit RX diagnostics;
2. STEP_UP/STEP_DOWN decoding and native tilt support;
3. 80-bit lift transmission for Venetian covers.

The pull request should include the two captured Telis vectors, host-side tests
for 56-bit regression and 80-bit UP/DOWN/MY/STEP_UP/STEP_DOWN, and a statement
that the virtual remote IDs and rolling-code storage format remain unchanged.
Do not make 80-bit transmission the default for existing roller covers;
`tilt_steps` is the explicit opt-in boundary.

## Development provenance

This feature, its regression tests, and this guide were developed and reviewed
with assistance from OpenAI Codex. The maintainer directed the implementation,
provided the physical Telis captures, performed the on-device verification,
and is responsible for the submitted result. This disclosure is intentional:
AI-assisted work should be reviewable under the same standards as any other
contribution, with its provenance stated plainly.
