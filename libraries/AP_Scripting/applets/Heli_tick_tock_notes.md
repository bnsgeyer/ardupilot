# Heli tick-tock: development notes

Where the work on `Heli_tick_tock.lua` stands, as of 2026-10-25 on branch
`pr-heli-tick-tock`. These are working notes, not user documentation (that
is `Heli_tick_tock.md`); fold what matters into the PR description and drop
this file before submitting upstream.

## Goal

A Lua applet that flies a tick-tock on a traditional heli from an aux
switch: the rotor disc rocks back and forth through vertical while the
collective reverses on every swing, so the heli bounces in place. The
script takes Guided, flies body rates closed on the AHRS quaternion, sets
the collective from where the thrust axis actually points, and hands back
to the pilot's mode.

## What is on the branch

- RealFlight harness, cherry-picked from `auto-acro-4.7`: FlightAxis address
  from `REALFLIGHT_IPADDR`, the RealFlight autotest framework (adapted to
  master's `vehicleinfo.json`), the Rise255 name fixes, terrain off, and
  `AP_RCProtocol_UDP: only yield to FDM if enabled`. Without that last one
  master flies RealFlight on RealFlight's own transmitter channels and
  ignores the harness's RC.
- `realflight-autotest-extra.parm` centres the sticks on 1500, since a tune
  calibrated to a real transmitter otherwise fails the neutral-stick arming
  check.
- Copter: guided's direct-thrust path (scripting rate/angle + throttle,
  `SET_ATTITUDE_TARGET` thrust) gets the heli's full collective range once
  airborne, instead of the `H_COL_LAND_MIN` floor applied while slower than
  2.5 m/s.
- The applet and its doc.
- `HeliTickTock` (SITL heli) and `RealFlightTickTock` (Thunder Tiger Raptor
  E720 FBL in RealFlight). Runs 1-11 below flew a RealFlight tune from
  Copter 4.6.3, converted with master's own conversion tables; the
  defaults are now a newer 4.8-dev acro tune of the same model.

## Running the tests

    ./waf configure --board sitl && ./waf heli
    python3 Tools/autotest/autotest.py test.Helicopter.HeliTickTock
    python3 Tools/autotest/autotest.py --realflight-address 127.0.0.1 \
        test.Helicopter.RealFlightTickTock

RealFlight must be running with "Thunder Tiger Raptor E720 FBL - CPPM APM"
loaded and the FlightAxis link enabled. The `.RFX` is not in the repo.

## What we found

- Guided held a hovering heli's collective above `H_COL_LAND_MIN`, which
  clips every tock. Same SITL flight: 18.7 m lost with the floor, 5.6 m
  without, and the floor stayed until the swing's own horizontal speed
  latched `dynamic_flight`.
- Heli angle boost flips and rescales the commanded collective, so the
  script turns `ATC_ANGLE_BOOST` off while it flies. It also cuts
  `GUID_TIMEOUT` to 0.3 s, as guided keeps flying the last rate otherwise.
- `AC_AttitudeControl::input_quaternion` drops the caller's rate
  feedforward (since eaeae81b91) and integrates an earth-frame rate as a
  body-frame increment (since 7047e11090). The script uses the rate path
  to avoid both. Not yet reported upstream.
- Above speedup 1, SITL wakes the scripting thread late and the script
  runs every 20-35 ms instead of 10 ms, so the SITL test drops to speedup 1
  for the manoeuvre. The SITL Blade 360 model is linearised about hover and
  wrong with negative collective; the tests use the conventional heli.
- A rotor moving along its own axis loses thrust. In RealFlight the rotor
  gave 55-66% of what a static thrust model predicts over a swing, so the
  script learns that share per half period and starts at half, which gives
  the first swing full collective.
- At tick-tock collective the tail rotor pushes the heli sideways at about
  4 m/s/s on this heli, so the sideways correction runs from the start of
  the manoeuvre rather than after a full period of averaging.
- Handing back to LOITER while the heli was still tilted and turning
  (15-30 deg, about 100 deg/s) took it to a 56-87 deg tilt, so the script
  now holds level until it has stopped turning.
- The harness's failure cleanup reboots SITL in the air, which crashes the
  RealFlight model, so `RealFlightTickTock` always lands in a `finally`.

## RealFlight results

Raptor E720, tail up from run 4 on (tail down before), 10 deg of cyclic
and the 4.6.3 tune unless noted. Runs 12 and 13 were flown back to back
as an A/B of the two tunes.

| Run | Swing | Collective, rpm | Result |
|-----|-------|-----------------|--------|
| 1 | 50 deg, 2 s | +-13, 1600 | sank 5-8 m/s, rotor gave 66% of planned thrust, crashed |
| 2 | 50 deg, 1 s | +-13, 1600 | pitch output at its limit 38% of the swing, fell 60 deg behind, aborted |
| 3 | 50 deg, 1 s, full cyclic | +-13, 1600 | tracked within 17-30 deg, sank 3-5 m/s, stopped 10 m down |
| 4 | 50 deg, 1 s | +-10.5, 1600 | sank 10 m in 3 s, stopped |
| 5 | 50 deg, 1 s | +-10.5, 2000 | passed, 1.4 m lost, drifted 17 m sideways |
| 6 | 60 deg, 1 s | +-10.5, 2000 | passed, 1.3 m lost, LOITER tipped it to 87 deg after hand-back |
| 7 | 60 deg, 1 s | +-10.5, 2000 | tail lost it at the tock end (620 deg/s yaw), recovered |
| 8 | 60 deg, 1 s | +-13, 2000 | passed, 1.3 m lost, tail held |
| 9 | 40 deg, 0.6 s, 14 deg cyclic, gains x10/14 | +-13, 2000 | lagged without saturating, aborted |
| 10 | 40 deg, 0.6 s, 14 deg cyclic | +-13, 2000 | pitch overshoot, 584 deg/s yaw, aborted |
| 11 | 45 deg, 0.8 s, 14 deg cyclic | +-13, 2000 | pitch lags 0.1 s and overshoots, tail saturates, aborted |
| 12 | 60 deg, 1 s, 4.8-dev acro tune | +-13, 2000 | passed, 1.3 m lost; pitch lag 23 ms, cyclic at its limit 23% of the swing, yaw under 209 deg/s, handed back at 1.6 m/s |
| 13 | 60 deg, 1 s (run 8 again) | +-13, 2000 | passed, 1.4 m lost; pitch lag 55 ms, cyclic at its limit 55% of the swing, 372 deg/s yaw with the tail at its limit, handed back at 6.7 m/s |

`RealFlightTickTock` flies run 8's swing and collective on the 4.8-dev
acro tune (run 12). That tune filters the pitch rate target at 30 Hz
instead of 4 Hz and the gyro at 40 Hz instead of 20 Hz, and against the
4.6.3 tune on the same swing it more than halved the pitch lag and the
time the cyclic spent at its limit, kept the tail off its limit, and
levelled after the exit in 0.5 s instead of timing out at 3 s. Mean
attitude error over the swing fell from 30 to 23 deg. Pitch still
reaches only about two thirds of the rate asked for, with the 10 deg
cyclic at its limit for a quarter of the swing. On the SITL
heli model the same settings abort 3 s in, as that heli needs about twice
the tune's pitch feedforward. `HeliTickTock` passes in SITL with about
6 m of height lost on a 2.5 s swing.

## Where it stands, and what is next

- On the 4.6.3 tune the level-hold after the exit timed out after 3 s
  and handed back with the heli still moving at 6-9 m/s, leaving LOITER
  20-30 m to stop. The 4.8-dev tune levels in 0.5 s and hands back at
  1.6 m/s, but the hold should still brake to a stop as well as level.
- Faster than 1 s needs more pitch authority, not script changes: on the
  4.8-dev tune the 10 deg cyclic is at its limit for a quarter of a 1 s
  swing. Runs 9-11 raised it to 14 deg on the 4.6.3 tune, whose pitch
  loop lagged about 0.1 s and overshot and whose tail kicked at every
  collective reversal (`H_COL2YAW` is 0 in both tunes); the 14 deg model
  has not been flown on the 4.8-dev tune. The test
  logs at full rate (`LOG_BITMASK` 0x10FFFF) to make that tuning possible.
  A pilot's tick-tock, from a slow-motion Trex 700 video, looks smaller and
  faster than ours, about +-30-40 deg, and may be on roll rather than
  pitch; worth trying `TTOC_AXIS` 1.
- Two cherry-picked commits use prefixes CI's branch check rejects:
  `SIM_FlightAxis:` and `AP_RCProtocol_UDP:` should be `SITL:` and
  `AP_RCProtocol:`. Fixing them needs a reword, which needs a push grant.
- The cherry-picked Rise255 defaults still carry 54 names that do not
  exist on master.
- The Copter change is on by default. Maintainers may want it behind a
  `GUID_OPTIONS` bit. A script or companion landing a heli on direct thrust
  now has to bring the collective down before the land detector sees it.
