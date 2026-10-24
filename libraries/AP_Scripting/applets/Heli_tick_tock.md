# Heli Tick-Tock

This applet flies a tick-tock (tic-toc) on a traditional helicopter. The
rotor disc is rocked back and forth through vertical, about the pitch or the
roll axis, while the collective reverses on every swing so the vertical part
of the rotor thrust always points up. The heli bounces in place like a
pendulum.

Raise the aux switch while flying. The script switches to Guided, swings the
heli up into the tick-tock, rocks it for `TTOC_CYCLES` cycles (or while the
switch stays high), brings it back upright and returns to the mode it was
flown in. Moving the roll, pitch or yaw stick hands control straight back to
that mode.

## Requirements

- A traditional helicopter with a 3D collective setup. The tock half of each
  swing is flown on negative collective, so `H_COL_ANG_MIN` must give about
  as much negative thrust as `H_COL_ANG_MAX` gives positive thrust, and
  `H_COL_ANG_MIN`, `H_COL_ANG_MAX` and `H_COL_ZERO_THRST` must match the
  measured blade pitch.
- `H_COL_HOVER` learned in a hover, since the collective is worked out from it.
- Firmware in which Guided gives a heli's commanded collective the full range
  once airborne. Older firmware holds collective above `H_COL_LAND_MIN` while
  hovering in Guided, which clips the tock half of every swing.
- A position estimate (GPS or other), and enough height: the tick-tock will not
  start below `TTOC_ALT_MIN` above home.

## Setup

1. Set `SCR_ENABLE` to 1 and reboot.
2. Copy `Heli_tick_tock.lua` to the `scripts` directory on the SD card and reboot.
3. Set `TTOC_ENABLE` to 1.
4. Set `RCx_OPTION` to 300 (Scripting1) on a switch channel, or to whichever
   scripting function `TTOC_AUX_FUNC` names.
5. Choose the axis and side with `TTOC_AXIS` and `TTOC_DIR`, and the swing with
   `TTOC_ANGLE` and `TTOC_PERIOD`.

The script checks before starting that the collective range can hold the heli
up over the planned swing, and that the swing's angular acceleration fits
within `ATC_ACC_P_MAX` (or `ATC_ACC_R_MAX` for a roll tick-tock). If not, it
says why on the GCS and does nothing. A faster tick-tock needs those limits
raised to what the heli can actually do.

## How it flies

The planned attitude is a rotation about the chosen axis by an angle that
swings sinusoidally between `90 - TTOC_ANGLE` and `90 + TTOC_ANGLE` degrees
from upright. The script commands body rates and throttle through Guided's
rate controller, and corrects its rates towards the planned attitude using the
AHRS attitude, so it never relies on Euler angles that fold at 90 degrees of
pitch.

The collective is set from where the thrust axis actually points: it is the
thrust that holds the heli up at that attitude, limited by the collective
range, and it reverses smoothly as the disc passes within `TTOC_REV_ANG` of
vertical. Near vertical the collective runs out, so the thrust at the ends of
the swing is raised until the mean over a swing holds the heli up.

Drift is corrected by tilting the whole manoeuvre a few degrees towards where
the heli should be, and height by raising or lowering the thrust. The swing
itself moves the heli back and forth once a period and bounces it up and down
at every crossing, so horizontal position and velocity are averaged over
exactly one period and height over half a period before they are corrected.
The correction therefore starts half a period into the swing for height and a
full period in for position. A heli flying fast along its own thrust axis gets
less thrust from its rotor, so a slow tick-tock loses some height before the
height correction catches up; shorter periods lose less.

Swinging straight up from a hover into the tick-tock would push the heli
sideways the whole way and leave it drifting, so the entry first dips the
other way by an amount that cancels that push. The exit mirrors the entry.

While the tick-tock runs the script sets `ATC_ANGLE_BOOST` to 0, since heli
angle boost rescales and flips the collective, and sets `GUID_TIMEOUT` to
0.3 seconds, so that if the script stops the heli does not keep rotating at the
last commanded rate for long. Both are put back when the manoeuvre ends. They
are not saved, so a reboot also restores them.

If the heli is more than `TTOC_ABORT_ANG` from the planned attitude for half a
second, the script abandons the tick-tock, levels the heli holding height, and
then returns to the pilot's mode.

The script logs a `TTOC` message each update: state, planned swing angle,
attitude error, thrust axis component along the wanted direction, thrust
ratio, collective, wanted and aimed thrust ratios, and the correction
acceleration.

## Parameters

- `TTOC_ENABLE`: set to 1 to enable the applet.
- `TTOC_AUX_FUNC`: the `RCx_OPTION` scripting function (300 to 307) that starts the tick-tock.
- `TTOC_AXIS`: 0 rocks about pitch (elevator), 1 about roll (aileron).
- `TTOC_DIR`: which way the heli is turned to stand the disc vertical. For pitch, 1 is nose up (tail down) and -1 is nose down. For roll, 1 is right side down and -1 is left side down.
- `TTOC_ANGLE`: how far the disc swings either side of vertical, in degrees. Larger angles spend less time near vertical and need less collective.
- `TTOC_PERIOD`: time for one full tick and tock, in seconds.
- `TTOC_CYCLES`: number of cycles to fly. 0 keeps going while the switch is held high.
- `TTOC_REV_ANG`: the collective reverses smoothly while the disc is within this many degrees of vertical.
- `TTOC_ATT_P`: rate correction per radian of attitude error.
- `TTOC_POS_P`, `TTOC_VEL_P`, `TTOC_VEL_I`: position and velocity gains for the drift and height correction, on the averaged position and velocity.
- `TTOC_ACC_MAX`: largest acceleration the drift correction, and separately the height correction, may ask for.
- `TTOC_ABORT_ANG`: attitude error, held for half a second, that abandons the tick-tock.
- `TTOC_ALT_MIN`: the tick-tock will not start below this height above home.
- `TTOC_STK_ABORT`: roll, pitch or yaw stick input beyond this fraction of full throw hands control back to the pilot.
