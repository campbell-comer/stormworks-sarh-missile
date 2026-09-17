# stormworks-sarh-missile

A semi-active radar homing missile system for Stormworks: Build and Rescue,
written entirely in microcontroller Lua. A ground radar array tracks the
target through a full extended Kalman filter and streams the solution to the
missile over a datalink; the missile flies proportional navigation on that
feed and hands the terminal aim point to its own two onboard radars.

Everything runs inside the game's constraints: one Lua script block per
microcontroller, 32 number + 32 bool channels per composite bus, 60 ticks per
second, and one tick of delay per logic block hop.

## System overview

```
GROUND RIG                                    MISSILE
8x radar ─ nr-radar-fe (x8)                   seeker radar 1 ─ seeker-slot-filter
              │ world contacts                seeker radar 2 ─ seeker-slot-filter
              v                                       │ range/bearing
        nr-tracker-coproc ── track table,             v
              │              hook/lock UI       sarh-guidance ── fins
              v                                       ^   │
          nr-tracker ── 9-state EKF                   │   └─ seeker reject
              │         + fixed-lag smoother          │
              └── datalink (position/velocity) ───────┘
                                                prox-fuse ── warhead
gantry-control ── rack aiming + fire            (independent)
nr-tracker-display ── touchscreen HUD
```

## The tracker (`src/tracker/`)

The in-game radar reports polar contacts with uniform noise: range error of
1% of distance, bearing error of 0.001 turns. Eight fixed radars stare at the
same sky with staggered scan phases (`nr-scan-stagger.lua`), each cleaned up
and converted to world coordinates by its own front end (`nr-radar-fe.lua`).

`nr-tracker.lua` fuses them:

- Per-radar sample bursts are condensed with a robust midhull (median-anchored
  trimmed midrange), which suppresses uniform quantisation noise better than
  averaging and comes with a known variance reduction that feeds straight into
  the measurement noise model.
- A 9-state EKF (position, velocity, acceleration per axis) runs on the
  condensed polar measurements. Process noise follows a Singer model whose
  level adapts on normalised innovation, so the filter is stiff on straight
  targets and compliant in turns.
- A fixed-lag RTS smoother re-runs the recent past every tick, and the
  published solution is propagated forward over the smoother lag plus the
  measured composite-bus delay, so the datalink carries an estimate of the
  target now rather than as it was when measured.
- A coordinated-turn detector estimates turn rate from the velocity history
  and publishes the implied centripetal acceleration.

`nr-tracker-coproc.lua` manages the multi-target picture (association,
merging, hook/lock selection, mount steering) and seeds the EKF.
`nr-tracker-display.lua` is the operator's touchscreen: camera-projected
target markers, lock control, and fault reporting.

## The missile (`src/missile/`)

`sarh-guidance.lua` is the flight computer. The guidance law is zero-effort-
miss proportional navigation in direction form: it splits missile and target
velocity into components tangential to the line of sight, commands the
cross-track velocity that nulls the miss, and spends the remaining speed
budget closing along the line of sight. It deliberately assumes nothing above
constant target velocity, because the game has no Doppler channel and any
acceleration estimate would be a second derivative of noisy positions.

Details that made it hit:

- **Terminal seeker fusion.** The datalink's position error is roughly
  constant in metres, so the angle it subtends grows as 1/range; a bearing
  sensor's error is constant in angle instead. Inside 200 m the two onboard
  radars take over the aim point. A metric veto (transverse and vertical
  disagreement against a fresh datalink aim) rejects radar phantoms, and a
  reject signal back to the seeker filters bans the offending contact.
- **Energy management.** Guidance authority is scheduled down with speed:
  hard pulls bleed speed, and lateral acceleration scales with fin times
  speed squared, so over-commanding enters an induced-drag spiral that costs
  more miss distance than it saves. Arriving fast beats turning hard.
- **Tick-rate hygiene.** The PID derivative is a two-tick difference (a
  one-tick difference has maximum gain at the Nyquist frequency and drove the
  fins into a limit cycle), and the bank angle is wrap-safely averaged for
  the same reason.
- Self-track and launch-point guards reject the failure modes where the
  ground tracker locks onto the outgoing missile or a return near the
  launcher.

`seeker-slot-filter.lua` gives each onboard radar object-persistent tracking
across the radar block's shuffling output slots. `prox-fuse.lua` detonates on
sphere entry, swept-segment intersection, or predicted entry, independent of
the flight computer. `gantry-control.lua` aims the launch rack and sequences
fire pulses.

## Conventions

- World frame: X east, Y north, Z up, metres. The physics sensor reports
  x east / y up / z north, so scripts remap it on read.
- Angles at IO boundaries are turns (1.0 = 360 degrees); trig is in radians.
- All rates are per tick internally (60 ticks per second); telemetry channels
  publish per second.
- Scripts use globals by design. The in-game limit is 8192 characters per
  script; deployment goes through an external minifier, so the sources stay
  readable and commented.

Each script's header documents its full I/O contract; the composite wiring
between microcontrollers follows those channel maps.

## Related

- [stormworks-laser-missile](https://github.com/campbell-comer/stormworks-laser-missile):
  the laser-guided sibling, with a 25-beam scanned lidar tracker.
- [stormworks-lua-ballistics](https://github.com/campbell-comer/stormworks-lua-ballistics):
  ballistic fire control for the gun side of the same fight.
- [microcontroller-to-lua](https://github.com/campbell-comer/microcontroller-to-lua):
  the toolkit used to study reference microcontrollers for this project.
