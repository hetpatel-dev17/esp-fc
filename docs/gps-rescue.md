# GPS Rescue (Foundation)

Status: **foundation / log-only** — computes and logs what a return-to-home
guidance state machine would do, but does not yet write to the flight
controller's mixer or motor output. See [Roadmap](#roadmap) below.

This fills the `GPS Navigation` item on the project's `README.md` Todo list.
It was designed and built incrementally against the actual firmware source
(not assumed from documentation), verified on real ESP32 hardware at each
stage, and bench- and field-tested for GPS acquisition and arming behavior
before any guidance logic was written.

## Motivation

ESP-FC had no return-to-home or GPS-triggered failsafe behavior at all —
GPS data was parsed only for telemetry (position/satellite count shown in
Betaflight Configurator's GPS tab). This adds the missing decision-making
layer, structured so it can be configured entirely from the existing
Betaflight Configurator UI, with no custom tooling required.

## Design goals

- **No new UI to build.** Enable/disable and trigger surfaces reuse
  mechanisms Configurator already speaks: the Modes tab (aux-switch
  assignment) and the Failsafe tab's procedure dropdown. Tunable parameters
  (return altitude, speed, descent rate, sanity thresholds) are exposed as
  ordinary CLI parameters.
- **Reuse proven control loops, not new ones.** The existing angle-mode
  attitude PID and the altitude-hold vertical PID are the intended actuation
  path for a future flight-enabled pass — this foundation is scoped to stop
  short of touching them, so the guidance logic can be validated against
  real GPS data before it has any authority over the aircraft.
- **PX4/ArduPilot-inspired phase structure**, adapted to what's actually
  available on this platform (see [Constraints](#constraints)): climb to a
  safe altitude first, then cruise home, decelerate and descend near home,
  with abort conditions modeled after the safety cases those projects
  handle explicitly (home-lock validity, GPS staleness, pilot override,
  distance sanity checks) rather than assuming the happy path.

## Architecture

```
Espfc::update()  (50Hz actuator-tier schedule)
  └─ Actuator::update()        // builds mode mask from switches + failsafe
       └─ canActivateMode(MODE_GPS_RESCUE)  // gated on gps.isHomeValid()
  └─ GpsRescue::update()       // NEW — log-only guidance state machine
       ├─ checkAborts()        // disarmed / home invalid / low sats /
       │                       // stale GPS / pilot override / too close /
       │                       // distance-not-decreasing sanity check
       └─ phase state machine: INIT → CLIMB → RETURN → DESCEND → HOVER
            (recomputed once per fresh GPS fix, not every control cycle —
             GPS runs at 5-25Hz vs. this schedule's 50Hz)
```

New/changed files:

| File | Role |
|---|---|
| `lib/Espfc/src/Control/GpsRescue.h/.cpp` | Guidance state machine (new) |
| `lib/Espfc/src/ModelState.h` | `GpsRescueState`, phase/abort enums |
| `lib/Espfc/src/ModelConfig.h` | `GpsRescueConfig`, `MODE_GPS_RESCUE`, `FailsafeProcedure` |
| `lib/Espfc/src/Espfc.h/.cpp` | Scheduler wiring |
| `lib/Espfc/src/Control/Actuator.cpp` | Mode activation gate + failsafe-triggered engage |
| `lib/Espfc/src/Connect/MspProcessor.cpp` | `MSP_BOXNAMES`/`MSP_BOXIDS`, `failsafe_procedure` round-trip |
| `lib/Espfc/src/Connect/Cli.cpp/.hpp` | `gps_rescue_*` params, `gps_rescue` status command |

## Configuring it

1. **Modes tab** — assign an aux switch to the new `GPSRESCUE` box.
2. **Failsafe tab** — select "GPS Rescue" as the failsafe procedure to have
   RX loss also engage the guidance logic (for logging visibility only —
   the existing hard-disarm-on-failsafe behavior is unchanged; arming is
   required for `GpsRescue` to run at all, so a real failsafe disarm
   preempts it in the same control-cycle).
3. **CLI tab** — tune `gps_rescue_return_alt`, `gps_rescue_return_speed`,
   `gps_rescue_climb_rate`, `gps_rescue_descend_rate`,
   `gps_rescue_descent_dist`, `gps_rescue_landing_alt`,
   `gps_rescue_min_start_dist`, `gps_rescue_min_sats`,
   `gps_rescue_max_angle`, `gps_rescue_stale_timeout`,
   `gps_rescue_stick_override`, `gps_rescue_allow_override`,
   `gps_rescue_sanity_checks`.
4. `set debug_mode GPS_RESCUE` surfaces live phase/target values in
   `debug[0..7]` (visible via `status`/blackbox), or run the `gps_rescue`
   CLI command for a one-shot human-readable dump.

## Constraints

Documented rather than hidden, since they shape what "flight-enabled" will
require:

- No EKF or horizontal position/velocity estimator exists on this platform.
  The guidance math uses raw GPS position/velocity (5-25Hz) directly,
  not a fused, higher-rate estimate the way PX4/ArduPilot do.
- Altitude reference is baro+accel only (`Control/Altitude.hpp`); GPS
  altitude is not fused in. "Return altitude" is relative to the rescue
  engage point, not a stable altitude-above-home datum.
- `GpsRescue::computeTargetLeanAngle()` is a placeholder P-controller,
  explicitly not tuned or validated — it exists to produce a bounded,
  observable number for this log-only phase, not to fly the aircraft.

## Roadmap

- [x] Mode/config/MSP/CLI foundation, Configurator-integrated
- [x] Guidance state machine (climb/return/descend/hover phases, abort
      handling), log-only
- [x] Bench-tested: phase transitions verified against real GPS movement
- [x] Field-tested: GPS 3D fix acquisition and GPS-gated arming confirmed
      on hardware
- [ ] Wire computed setpoints into the real attitude/altitude PID loops
      (separate, explicitly scoped follow-up — not started)
- [ ] Tuned position/velocity controller to replace the placeholder
- [ ] Real landing/disarm behavior at the end of DESCEND
