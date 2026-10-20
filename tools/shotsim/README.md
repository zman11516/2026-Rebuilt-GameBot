# shotsim: 5454 shot maps and shoot-on-the-move

`tools/shotsim` builds the shot maps the robot uses for the HUB and for feeding (passing), and has a web
simulator for looking at them. It is an independent re-implementation of the approach Team 4414 HighTide
described publicly for their "TideShot" tool (Chief Delphi posts, the "Behind the Bumpers" interview and the
TideShot deep-dive video), which itself builds on Team 1690 Orbit's 2022 software talk on shot selection.
4414's tool is private; none of its code is used here. Credit for the method goes to 4414 and 1690.

In short: simulate shots offline over a grid of (distance, radial velocity), pick the most robust shot at every
grid point, export polynomial fits plus the raw grid, and on the robot cancel the tangential part of the robot
velocity with 3D vectors.

Plain Node (18+) and browser ES modules, no dependencies and no build step. The web UI loads Plotly 2.35.2 from
jsDelivr; without it the charts are replaced by a note and every number still works.

## Quick start

```
node tools/shotsim/serve.mjs                      # web UI: http://localhost:5454/  (--port N)
node tools/shotsim/cli.mjs generate               # all maps + shooter_calibration.json (about 90 s)
node tools/shotsim/cli.mjs generate --only hub    # one config: hub | feed | netfeed
node tools/shotsim/cli.mjs calibrate [--tof] [--write] [--data tools/shotsim/data/tuned_hub.csv]
node tools/shotsim/cli.mjs point --config hub --d 3 --vr 0
node tools/shotsim/cli.mjs fixtures               # Java parity fixtures -> src/test/resources/shotmap/
node --test tools/shotsim/test/*.test.mjs
```

The UI needs `serve.mjs` (browsers will not `fetch` configs from `file://`). The server is read-only, serves
`tools/shotsim/` at `/` and the committed maps in `src/main/deploy/shotmaps/` at `/deploy/shotmaps/`.

## Files

| Path | What |
|---|---|
| `configs/mechanism.json` | 5454 hood, flywheel and launcher geometry and the two calibration knobs (single source of truth) |
| `configs/hub.json`, `feed.json`, `netfeed.json` | physics, target geometry, selection, grid, fit and tangential settings per map family |
| `data/tuned_hub.csv` | hand-tuned hub shots used by `calibrate` |
| `lib/physics.mjs` | RK4 ball flight with drag, Magnus lift and buoyancy |
| `lib/mechanism.mjs` | hood rotations <-> launch angle, rotor rps <-> exit speed, release point |
| `lib/targets.mjs` | hub and feed shot classification, feed obstacle profiles, crossing boxes |
| `lib/sweep.mjs`, `lib/select.mjs` | per-angle speed windows, maximin selection, Monte Carlo check |
| `lib/grid.mjs`, `lib/fit.mjs`, `lib/export.mjs` | grid solve, polynomial fits, map JSON (`shotsim5454/1`) |
| `lib/mapeval.mjs`, `lib/sotm.mjs` | reference runtime: map evaluation and the shoot-on-the-move solver (the Java code must match) |
| `lib/calibrate.mjs` | fits theta0 and exit speed per rps to the tuned table |
| `cli.mjs`, `serve.mjs`, `index.html`, `app.mjs`, `worker.mjs` | command line, static server, web UI |

## Web UI

| Tab | Shows |
|---|---|
| Single Sweep | trajectory fan for one (d, vr): lower (red) and upper (green) speed limit at every hood angle, chosen shot in blue, hub or feed obstacles; the valid region (angle vs speed) with the margin bars; optimal angle @ speed, margins, TOF, apex, hood rotations, rotor rps, score and Monte Carlo P(score) |
| Full Sweep | the whole grid in Web Workers (one per core less one), heatmaps of angle, speed, rps, TOF, weakest margin and tangential limit; grey cells have no scoring shot. Click a cell to open it in Single Sweep |
| Fit | per-output rms / max error for degree 2 to 5 and the recommended mode; fitted angle surface vs the simulated points |
| Moving Shot | a map (last sweep or a committed one) at (d, vr, vt) in POLY and GRID mode, the tangential compensation, the resulting hood and flywheel commands and validity checks, and the vector diagram |
| Calibrate | fit of theta0 and exit speed per rps to the tuned table, per-shot residuals and cost profile; can copy the result into the mechanism fields |
| Feed | top-down field with the feed obstacles; drag the robot to see which map applies (feed or net feed) and the resulting shot |
| Export | download the last sweep's map and `shooter_calibration.json` |

Edits in the config panel stay in the browser. The committed maps are produced by `cli.mjs generate` from the
files in `configs/`; a browser export of an unedited config matches them exactly except for `generator.configHash`.

## The model

### Ball flight

A point mass in the vertical plane of the shot: x along the line from the turret axis to the target, z up from
the floor. Forces are gravity, buoyancy, quadratic drag `-k Cd |v| v` with `k = rho A / (2 m)`, and Magnus lift
`k Cl |v|^2` perpendicular to v with `Cl = clamp(kL * S, +/- clMax)` and `S = omega r / |v|`. The backspin is
`omega = spinRatio * s / r`, set by the shooter exit speed `s`, not the field speed. RK4 at 4 ms.

The hood pivots on the flywheel axle, so the ball leaves tangent to the wheel: release =
`axle + exitRadius * (-sin theta, cos theta)`. The launch velocity is `(s cos theta + vr, s sin theta)`.
Radial velocity is simulated rather than subtracted because it changes the airspeed (drag and lift) for the
whole flight while the spin still follows the shooter speed.

### Hub shots

The ball centre must come down through the entry plane z = 1.8288 m within `halfWidth - r - windowInset` of the
hub centre (x = d), keep `r + lipClearance` away from the front face (d - 0.5969) and the deck up to the front
edge of the opening (d - 0.5296), and stay below `apexMax`. Each simulation returns two margins, "not short" and
"not long", so at a fixed angle the scoring speeds are one interval `[sLow, sHigh]` found by bisection.

### Feed shots

The ball centre must land (z = 0.075 m) within 0.75 m of the target. The obstacles along the shot line depend
on where the robot stands, so for each distance the generator samples launch points on the circle of radius d
around the target (inside the launch region, outside obstacles) and keeps the union of every obstacle's
footprint (grown by the ball radius) along the line: the ball centre must be at least `zTop + r + clearance`
there. Like 4414 there are two map families: `feed` covers lines that miss the hub body and net, `netfeed`
covers lines that cross the net (the ball has to clear 3.06 m). The robot picks one by testing the turret-to-
target segment against the `crossing.boxes` in the map.

### Shot selection (maximin)

For every angle the speed is put in the middle of `[sLow, sHigh]`. The speed margin is half the window; the
angle margins are how far the angle can drift at that speed before the shot stops scoring. Each margin is
divided by its expected error (`sigmaSpeed`, `sigmaAngleDeg`) and the shot with the largest weakest margin wins
(`score`, in sigmas; `speedBias` optionally penalises speed). The angle sweep extends `anglePadDeg` past the hood
travel so margins near the hood limits come from the physics. The UI's P(score) is a 256-shot Monte Carlo
cross-check with the same sigmas.

### Map export

Each grid point stores angle, speed, TOF and the four margins (plus vtMax, apex, score). The seven outputs are
fitted with a least-squares polynomial in normalised (d, vr) of total degree `fit.degree`; margin fits are
shifted down by their worst overshoot so they never overstate the margin. `recommendedMode` is `POLY` when the
angle fit is within 0.5 deg and the speed fit within 0.05 m/s everywhere, otherwise `GRID` (bilinear
interpolation of `rawGrid`; a cell with an invalid corner is invalid). Today the hub map is `GRID`: the hood
limit puts a kink in the angle surface that a low-order polynomial cannot follow. `rows` holds the valid distance
range per radial velocity (the "corner cuts") and `vtMax` the largest tangential speed each shot can cancel
(hood minimum angle, flywheel maximum, `tangential.absMax` and the bearing-rate rule).

### Shoot on the move (runtime)

Reference: `lib/sotm.mjs`; robot: `frc.robot.subsystems.shooter.shotmap.ShootOnMoveSolver`.

```
pose ahead by the latency:  P' = P.exp(Twist2d(vx t, vy t, omega t))
turret point and velocity:  p = P'.t + R(psi') o,   v = R(psi') (vx - omega o_y, vy + omega o_x)
line of sight:              u = (target - p) / d,  t_hat = (-u_y, u_x)   (left of u)
                            vr = v . u  (+ closing),  vt = v . t_hat  (+ left)
map at (d, vr):             theta, s, tof, margins;  H = s cos theta,  V = s sin theta
launch vector:              L = H u - vt t_hat + V z     (L + v = (H + vr) u + V z, the simulated shot)
turret yaw:                 atan2(L_y, L_x) - psi' - asin(axleLateral / d)
elevation:                  atan2(V, sqrt(H^2 + vt^2))
exit speed:                 sqrt(H^2 + vt^2 + V^2)
hood rotations:             (elevation - theta0Deg) / degPerRot
flywheel rotor rps:         speed / (exitSpeedPerRotorRps * exitSpeedScale)
valid:                      map valid, |vt| <= vtMax, elevation >= hood minimum, rps <= rotorRpsMax
```

Example (4414's): base vector (H, 0, V) = (3, 0, 6) with the robot moving 2 m/s to the left gives the shooter
vector (3, -2, 6): turret -33.7 deg, hood 63.4 -> 59.0 deg, exit speed 6.71 -> 7.00 m/s.

## Parameters and sources

| Parameter | Value | Source / status |
|---|---|---|
| Ball | 0.215 kg, 0.1501 m | Game Manual 5.10.1 (5.91 in, 0.448-0.500 lb); weigh ours (code comment says 226 g) |
| Air density | 1.204 kg/m^3 | 20 C, sea level; only rho*Cd is identifiable |
| Drag | Cd 0.55 | passive-hood preset (every open sim uses 0.47); drag and lift trade off, tune the mechanism knobs instead |
| Lift | Cl = min(0.8 S, 0.35), spinRatio 0.9 | single wheel against a fixed hood (CD thread 394940 kinematics); cross-checked against 6328, 6940, 5572 maps |
| Hub entry | 1.8288 m, half-width 0.5296 m, face 0.5969 m | Manual 5.4 and field drawings p.25 (41.7 in hexagon across flats -> inscribed circle; 47 in hub) |
| Hub margins | windowInset 0.02 m, lipClearance 0.04 m | assumed |
| Apex | 7.0 m (25 ft venue minimum); 3.5 m practice toggle | regional site requirements; 12 ft shop ceiling less ball and margin |
| Feed landing | z 0.075 m, radius 0.75 m | keeps 1.2 m from the guardrail (Tyler Handel's rebuilt-sim-2026 rule) |
| Feed targets | left (2.5, 6.0), right (2.5, 1.96) | `FieldConstants` pass targets, blue frame |
| Feed obstacles | trench top 1.0224 m, bump 0.1654 m, hub 1.8288 m, net plane x 5.4831 m z 1.2637-3.0571 m, tower 1.988 m | Manual + field drawings via rebuilt-sim-2026 and 6328 `FieldConstants`; **trench height VERIFY** (frc-fire-control uses 1.346 m pillars) |
| Selection | sigmaSpeed 0.2 m/s, sigmaAngle 1.0 deg, step 0.5 deg, pad 3 deg | about our +/-2 rps flywheel gate; 1 deg as in Aieda / 2084 / Tyler sims; refine with a 20-shot test |
| Hub grid | d 1.0-6.5 m step 0.1, vr -2..2 step 0.25 | far corner is 6.14 m; SOTM drive is throttled to 1.5 m/s |
| Feed grid | d 2-12 m step 0.25, vr -2..2 step 0.5 | about the reach at 96 rps |
| Tangential | abs 1.5 m/s, bearing rate 0.4 rad/s (hub) | 6328 `DriveCommands`; set null to disable |
| Latency | phaseDelay 0.03 s + mechLatency 0.02 s | 6328 / 1540 phase delay; 5962 mechanism latency; measure feed-to-exit on video |
| Hood | launch angle = 72.4 - 34.82 * rot, rot 0-0.6 | CAD: hood plate 25.711 deg at CANcoder 0.0 -> 46.601 deg at 0.6 (20.89 deg / 0.6 rot), raising the plate flattens the shot; theta0 (72.4) fitted from the tuned table |
| Flywheel | 0.1464 m/s per rotor rps, scale 1, 20-96 rps | 4 in Stealth 40A, single wheel, passive hood, 5.177 in gap (0.733 in compression); ratio fitted from the tuned table (gear ratio folded in); 96 rps = top of `PassLookUpTable` |
| Launcher | axle 0.4453 m high, 0.0994 m forward, 0.0243 m lateral; exit radius 0.11655 m | CAD: axle 17.531 in above the origin, 3.913 in forward and 0.956 in lateral of the turret axis (+ = left of the shot line, **VERIFY sign**); exit radius = 2.0 in wheel + 2.589 in (half the gap) |
| Turret offset | (-0.0762, 0.1397) m, robot frame | `Constants.TurretConstants.kTurretOffsetX/Y`, matches CAD within 1/8 in |

## Calibrating on the robot

Like 4414 there are two knobs: the hood angle offset (`hood.theta0Deg`) and the exit speed per rotor rps
(`flywheel.exitSpeedPerRotorRps`, trimmed by `exitSpeedScale`). Both live in `configs/mechanism.json`; after any
change run `cli.mjs generate`, which also rewrites `shooter_calibration.json`.

1. **Hood angle and exit speed from video.** Film the shot from the side at 240 fps with a scale in frame at hood
   0, 0.3 and 0.6 rotations and two or three flywheel speeds. The launch angle over the first ~0.1 s gives
   `theta0Deg` (and checks `degPerRot`); the ball travel over the first ~0.3 m gives the exit speed and so
   `exitSpeedPerRotorRps`. Until then both come from `cli.mjs calibrate` on the tuned table (`--write` stores them).
2. **Dispersion.** 20 shots from one spot and setting onto a taped floor grid, logging hood and flywheel error at
   the moment of feeding. The spreads set `selection.sigmaSpeed` and `sigmaAngleDeg`.
3. **Latency.** Count video frames from the feed command (logged timestamp) to the ball leaving: `sotm.mechLatency`.
4. **Static check** at 1.5, 2, 3, 4, 5 and 6 m, 10 balls each. All short or all long: adjust `exitSpeedScale`.
   An error that changes with distance (short close, long far or the reverse): adjust `theta0Deg`. Re-check.
5. **Moving checks.** Radial +/-1 m/s at 3 and 5 m, then tangential 1 m/s. Hits trailing the motion mean the
   latency is too small.
6. **Regenerate after hardware changes** (wheels, compression, belts, hood) and compare fresh and worn balls;
   4414 found a single ratio change re-tunes every distance.
7. **Verify** the trench height in `feed.json` / `netfeed.json` and the sign of `launcher.axleLateral`.

## How the maps reach the robot

```
configs/*.json ──cli.mjs generate──> src/main/deploy/shotmaps/{hub,feed_left,feed_right,netfeed_left,netfeed_right}.json
configs/mechanism.json ────────────> src/main/deploy/shooter_calibration.json
```

GradleRIO deploys `src/main/deploy/` with the robot code. On the robot `frc.robot.subsystems.shooter.shotmap.ShotMaps`
loads the five maps and the calibration once; `ShotMap` evaluates a map in its recommended mode,
`ShootOnMoveSolver` applies the tangential compensation and `ShooterCalibration` converts to hood rotations and
rotor rps. `lib/mapeval.mjs` and `lib/sotm.mjs` are the reference for that Java code: `cli.mjs fixtures` writes
`src/test/resources/shotmap/` and the Java tests must reproduce it, so re-run it after changing either file.
Commit the regenerated JSON together with the config change that produced it (`generator.configHash` records
which config a map came from).
