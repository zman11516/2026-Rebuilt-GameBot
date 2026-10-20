# Team 4414's shot system, and how 5454 replicates it

This page summarizes what Team 4414 (HighTide) has said publicly about their 2026 shooter software. It also maps each piece to our implementation (`tools/shotsim` offline, `frc.robot.subsystems.shooter.shotmap` on the robot) and lists the parameters we chose, with their sources.

How to use it:
- **Running the tools and calibrating:** see [`docs/ShotMappingGuide.md`](../ShotMappingGuide.md), section 5.
- **The design:** read sections 1 and 2.
- **Checking a number in a config:** use section 3.

## Sources

| Tag | Source |
|---|---|
| Binder | 4414's public 2026 technical binder, https://2026.team4414.com (software section and its screenshots of their "FRC Shot Simulator") |
| CD | Chief Delphi thread 519602 (4414 tech binder thread), https://www.chiefdelphi.com/t/team-4414-hightide-2026-tech-binder-ripcurrent/519602. Post numbers are given as CD #n. The key posts are #62 (tilt compensation), #107–#111 (localization) and #116 (shot system and calibration). |
| Video | The "TideShot" deep-dive video posted by a 4414 member in a public FRC community Discord thread in October 2026. It was posted in a series the team described as generated with Codex from their codebase, so it is probably codebase-derived. The team said it "grossly simplified" the step from simulated shots to robot code. |
| BtB | 4414's "Behind the Bumpers" interview |
| Discord | Answers 4414 members gave in a public FRC community Discord thread (robot-pics-2026 / 4414), May–October 2026 |

4414's simulator (shot.team4414.com) is password-protected. **We did not see or use its code.** Our version was written from the public descriptions above, plus open-source 2026 projects for the physical parameters: 6328, 5962 (frc-fire-control), 5572, 6940, 1540, Tyler Handel's rebuilt-sim-2026, Aieda's REBUILT Projectile Simulator, and the 2084 calculator.

Confidence labels used below:
- **stated:** 4414 said it.
- **shown:** read from their screenshots or video overlays.
- **inferred:** our reading of those sources.

---

## 1. What 4414 does

### 1.1 Offline: simulate, select, fit (stated)
- **Overall flow.** Before the match, simulate shots over a grid of **distance to the hub** × **robot radial velocity** (vr, positive = closing on the hub). Pick one robust shot per grid point and fit small polynomials to the picks. The robot only evaluates the fits. [Video, CD #116, Binder]
- **Why vr is a grid axis.** It is not simply subtracted, because the ball leaves with shooter velocity plus robot velocity. Radial motion changes the airspeed, so drag and lift change over the whole flight. Matching a field velocity with a different wheel speed also changes backspin. So every vr row is re-simulated and re-searched. [Video]
- **Physics.** The video states gravity, drag and spin-dependent Magnus lift. Rim collisions were called "trivial to add", but it is not confirmed whether they were enabled. [Video, CD #116]
- **Speed window per angle.** At each grid point the tool sweeps launch angles in about 1° steps. For each angle it finds the lowest and highest exit speed that still scores: the red (lower) and green (upper) limits in their plots. [Video, shown]
- **Shot counts.** A "single sweep" (one grid point) is about 29,000 simulated shots. Feeding is about 125,000, "because you have to go over the net". [BtB]
- **Selection.** Angle and speed margins are each normalized by the expected error, and the tool keeps the shot with the **best weakest margin** (maximin). The speed bias and the allowed hood range are configurable; a Discord answer suggests the bias trades accuracy against power, i.e. it favours lower speeds. [Video, CD #116, Discord]
  - Example readout: "Optimal 55.0° @ 8.20 m/s, speed −0.31/+0.31 m/s, angle −13.0°/+6.0°". [Binder, shown]
  - The chosen angle is often off-grid (52.9°, 56.1°), but the margin edges land on whole degrees. So the angle search is continuous or interpolated, and each margin edge is snapped to the last whole-degree angle that still scores. [shown, inferred]
  - Whether the rule is "axis maximin" or "largest inscribed ellipse" cannot be settled from the published numbers. The October video, probably made from the codebase, describes axis margins.
- **Fit outputs.** Seven fitted values per (d, vr): hood angle, exit speed, time of flight, and hood margin low/high and speed margin low/high. [Video]
  - The basis is quadratic in normalized d and vr: 1, d, r, d², d·r, r². The video says "this revision" uses quadratic fits, so the order has changed before.
  - The fit follows 1690's 2022 software talk. 1690 kept only shots that hit the goal centre; 4414 keeps both rim limits, which gives a valid region and a tolerance for each shot. [CD #116]
- **Validity export.** Distance and radial bounds, "corner cuts" (grid points with no scoring shot, such as close range while closing fast), a tangential-speed limit, and hood and speed limits. [Video, shown]
- **Fixed in the map.** Shot height is baked into the precompute; making it a runtime input would turn the n² grid into n³. A full solve already takes about 5–10 minutes on a laptop. [Discord]
- **Grid size.** The video's illustration grid (about 0.5–7 m, vr ±3 m/s in 0.5 steps) is an animation, not necessarily their configuration. A binder plot uses 1–6 m and ±2 m/s.

### 1.2 Runtime: cancel tangential velocity with 3D vectors (stated)
1. Split the turret's field velocity into radial vr and tangential vt.
2. Evaluate the fit at (d, vr), giving the forward launch speed H = s·cos θ and the upward speed V = s·sin θ.
3. In the target frame, the shooter vector is (H, −vt, V) and the robot adds (vr, +vt, 0), so the ball's field velocity is (H + vr, 0, V). That is exactly the radial shot that was simulated.
4. Convert to commands:
   - turret = atan2(y, x)
   - hood = atan2(z, √(x² + y²))
   - exit speed = |vector|, then converted to wheel speed

Worked example from the video: (3, 0, 6) with 2 m/s sideways becomes turret −33.7°, hood 63.4° → 59.0°, and speed 6.71 → 7.00 m/s.

Other runtime details:
- **Time of flight.** It is not used for a "virtual target" shift. It only turns a residual sideways-velocity error into drift. That drift check is disabled in their current hub code, and readiness is listed as turret angular tolerance. The video does not say whether flywheel and hood are also gated. [Video]
- **Tilt compensation.** The SOTM vector then goes through a tilt-compensation solver. It rotates the launch vector back through the chassis yaw, pitch and roll, and is applied above 2° of tilt unless a lockout is active. It is a closed form, tested against WPILib `Rotation3d`. A sign error came from skipping the gyro mount calibration. [CD #62, Discord]
- **The video simplifies.** 4414 cautioned that "getting sotm to be a 3d vector is a bit more complex" than the video shows. [Discord]

### 1.3 Calibration: two knobs (stated)
- **Knob 1, hood-angle offset.** Measured from slow-motion video. The error was nearly constant across angles, so it is a single constant.
- **Knob 2, the m/s → rps ratio.** Tuned until the balls go in. It was nearly linear for most shooter revisions; one revision needed a small lookup. As balls wear, a small change to this one ratio re-tunes every distance. [CD #116]
- **Feed speed matters.** Feed speed changes shot distance ("strong velocity control there was helpful"), and 4414's feeder is geared to the flywheel. When calibrating, hold the feed speeds at their match values. [Discord]
- **Hardware changes break the map.** For example, the map was tuned while a belt was slipping; once a second belt fixed the slip, far shots went 3–5% long. [Discord]

### 1.4 Feeding (stated, few details)
- The same simulate-and-select process was run for separate feeding tunes: a low shot optimized for low bounce, and one over the back of the net. [CD #116]
- Hub, feed and net-feed shots all pass through tilt compensation. [CD #62]
- Their simulator tabs include "Fixed Sweep" and "Net Fixed Sweep", which suggests fixed-hood feed variants. [shown, low confidence]
- 4414 gave no details on landing zones or how the target is chosen.

### 1.5 Context we did not replicate
- **Localization.** A turret-mounted Limelight 4 running MegaTag1 provides about 95% of localization, and vision corrects heading slowly. The turret is zeroed by eye before each match. [CD #107, BtB]
- **Architecture.** Fully state-machine based, with no WPILib commands. [Binder, BtB]
- **4414's own view.** They did not have the best shot or the best SOTM of the season. The system was built before the robot ran and was mostly set-and-forget; afterwards they only adjusted feedforwards and the m/s → rps ratio. [CD #116]

---

## 2. How our implementation maps to it

| 4414 | 5454 | Where |
|---|---|---|
| Drag + Magnus physics | 2D RK4 point mass with gravity, buoyancy, quadratic drag, and lift Cl = min(kL·S, clMax). Backspin is set by the **shooter** exit speed. | `tools/shotsim/lib/physics.mjs` |
| (d, vr) grid, vr re-simulated | Hub: 1.0–6.5 m in 0.1 m steps × vr ±2 m/s in 0.25 steps. Feed: 2–12 m in 0.25 m steps × vr ±2 m/s in 0.5 steps. | `configs/*.json`, `lib/grid.mjs` |
| Lower and upper speed limit per angle | Bisection on the "not short" and "not long" margins, at 0.5° angle steps. The sweep extends 3° past the hood travel so margins near the limits reflect the physics, not the mechanism. | `lib/sweep.mjs` |
| Hub scoring check | The ball centre must cross 1.8288 m descending inside the inscribed half-width − r − 0.02 m. It must stay r + 0.04 m clear of the front face and lip, and below the apex cap. | `lib/targets.mjs` |
| "Best weakest margin" | Axis maximin. At each angle the speed is the middle of the window; the angle margin is how far the angle can move at that speed and still score. Each margin is divided by its σ and the largest minimum wins. **Differences:** candidate angles sit on a 0.5° grid, and the margin edges are interpolated instead of snapped to whole degrees. | `lib/select.mjs` |
| 7 outputs, quadratic fit | Same 7 outputs and normalized quadratic basis. **Additions:** margin fits are shifted down by their worst overshoot so they never overstate the simulation, and the raw grid is exported too. `recommendedMode` is POLY only if the fit stays within 0.5° and 0.05 m/s; the current maps all recommend GRID. | `lib/fit.mjs`, `lib/export.mjs` |
| Bounds, corner cuts, tangential limit | Each vr row stores its valid distance range, which replaces corner cuts. Each grid point gets a vt limit from the hood minimum, the flywheel maximum, \|vt\| ≤ 1.5 m/s and (hub only) a 0.4 rad/s bearing rate. | `lib/grid.mjs`, `lib/mapeval.mjs` |
| 3D-vector SOTM | L = H·u − vt·t̂ + V·ẑ, converted to turret yaw, elevation and speed. **5454 additions:** look ahead by phase delay + mechanism latency with `Pose2d.exp`; turret-point velocity includes ω × r; a lateral release offset is corrected with asin(axleLateral / d). | `lib/sotm.mjs` → `ShootOnMoveSolver.java` |
| Readiness: turret tolerance | Feed only while all of these hold:<br>• the shot is valid<br>• flywheel within min(2 rps, ½ speed margin)<br>• hood within ½ angle margin<br>• turret within atan(0.2 m / d)<br>• turret target inside the soft limits<br>Re-checked every loop; feeding pauses (it does not leave SHOOT) when a gate fails. | `ShotOnTheMoveCommand.java` |
| Feed and net-feed tunes | Separate `feed_*` and `netfeed_*` maps per pass target. Launch bearings are sampled on the circle of radius d around the target, and the union of obstacle intervals is the constraint, so one map covers every launch point at that distance. On the robot, `netfeed` is used when the turret → target line crosses the hub-net footprint. **This is our design;** 4414 gave no details. | `lib/targets.mjs`, `TurretUtil.computeShotMapSolution` |
| Two calibration knobs | `hood.theta0Deg` (launch angle at CANcoder 0) and `flywheel.exitSpeedPerRotorRps`, fitted to our tuned table by `cli.mjs calibrate`. The live trims are `SOTM/HoodOffsetDeg` and `SOTM/ExitSpeedScale`. | `lib/calibrate.mjs`, `ShooterCalibration.java` |
| Tilt compensation, turret-camera localization, state machines, rim collisions | **Not done yet.** | — |

The JS files under `tools/shotsim/lib` are the reference. The Java classes are ports and are checked against fixtures generated by `cli.mjs fixtures`.

---

## 3. Parameters

Each row gives the value our configs use and its source. Section 3.6 lists the numbers that still need measuring on our robot.

### 3.1 Ball and air

| Parameter | Value | Basis | Status |
|---|---|---|---|
| Ball mass | 0.215 kg | Game Manual 5.10.1 (0.448–0.500 lb); AndyMark's 215 g production target. Our old code comment says 226 g. | Weigh ours (optional) |
| Ball diameter | 0.1501 m | Manual 5.10.1 (5.91 in) | Official |
| Air density | 1.204 kg/m³ | 20 °C at sea level (Aieda, 2084); 6328 uses 1.2041. Adjust for altitude. | Assumed |
| Buoyancy | On (0.097 m/s², about 1% of g) | A constant force, so Cd cannot absorb it. Aieda enables it by default. | Physics |
| Drag Cd | 0.55 (passive-hood preset) | Every open sim uses 0.47. 0.55 is our aero cross-check against open-source maps. Only ρ·Cd is identifiable, and it trades off against lift. Tune the two knobs, not Cd. | Assumed |
| Lift | Cl = min(0.8·S, 0.35), spin ratio 0.9, no spin decay | Single wheel against a fixed hood gives high backspin (kinematics from CD thread 394940). A powered hood roller would be about 0.25; zero spin is 0. | **Confirm shooter type** |

### 3.2 Hub (blue frame)

| Parameter | Value | Basis | Status |
|---|---|---|---|
| Hub centre | Ours (4.620, 4.040); drawings (4.6256, 4.0345) welded | Field drawings p.11. The two are 7.9 mm apart. | Official |
| Entry height | 1.8288 m (72 ± 0.5 in) | Manual 5.4; drawings p.25. Only the front edge is specified at 72 in. | Official |
| Opening half-width | 0.5296 m | 41.7 in across flats, inscribed circle (Manual Fig 5-7; 6328). The drawings' 41.932 in is the **outside** dimension, not the opening. | Official |
| Window inset | 0.02 m beyond the ball radius | No source; acceptance window \|x\| ≤ 0.4345 m | Assumed |
| Front face / lip | Face 0.5969 m from centre (47 in hub); clearance r + 0.04 m | Manual 5.4. The 0.04 margin mirrors Tyler's lip check. | Official / assumed margin |
| Apex cap | 7.0 m | 25 ft minimum venue ceiling (regional site requirements). A 3.5 m practice cap is **binding**: above about 57° at 6 m, 71.5° at 3 m and 80° at 1.5 m. Use it only for practice maps. | Official |

### 3.3 Feeding (blue frame)

| Parameter | Value | Basis | Status |
|---|---|---|---|
| Targets | L (2.50, 6.00), R (2.50, 1.96) | `FieldConstants`. These are not mirror images about the hub (+1.966 / −2.075 m). | Ours |
| Landing zone | Ball centre at z 0.075 m, within 0.75 m of the target | The 0.75 m radius keeps 1.2 m from the guardrail (Tyler's rule). | Assumed |
| Obstacles | Trench top 1.0224 m; bump 0.1654 m; hub body 1.8288 m; hub net plane at x 5.4831, y 3.2927–4.7763, z 1.2637–3.0571; tower 1.988 m; alliance wall 2.0015 m | Manual 5.x, field drawings; Tyler (net bottom from GE-26300); 6328 and Tyler (tower y from tag 31) | Official; **verify the trench height** (5962 uses 1.346 m pillars) |
| Obstacle clearance | r + 0.04 m | Over the trench the ball centre must be at or above 1.1375 m. A 4 m pass at 40° clears by only about 7 mm; 45° clears by 0.17 m; 50° by 0.35 m. | Assumed |
| Launch region | x 5.3–16.3, y 0.4–7.67 | Our neutral zone and opponent half | Ours |

### 3.4 Selection, grid and SOTM

| Parameter | Value | Basis | Status |
|---|---|---|---|
| σ speed / σ angle | 0.2 m/s / 1.0° | 4414's sidebar shows a speed σ of about 0.2. Our ±2 rps flywheel gate is about ±0.29 m/s. 1° is from 2084 (Aieda uses 0.5°). Ball hardness alone accounts for only about 0.05–0.08 m/s (1540). | **Refine with a 20-shot test** |
| Angle step / pad | 0.5° / 3° | Pad = headroom for tangential compensation, which lowers the elevation | Ours |
| Distance grid | Hub 1.0–6.5 m @ 0.1; feed 2–12 m @ 0.25 | Turret axis to target, 2D. 6.5 m covers our far corner (6.14 m). | Ours |
| Radial grid | ±2 m/s (hub @ 0.25, feed @ 0.5) | SOTM drive is throttled to 25% (1.5 m/s) | Re-grid if the throttle is raised |
| Tangential limit | \|vt\| ≤ 1.5 m/s; hub bearing rate ≤ 0.4 rad/s | 1.5 = SOTM throttle. The bearing-rate rule is from 6328's DriveCommands; passing has no limit, as at 6328. | Loosen after testing |
| Latency | 0.03 s phase delay + 0.02 s mechanism | 6328 / 1540 (30 ms) and 5962 (20 ms). Do not use hand-timed TOF tables as flight time: they run long against the model (6328 +0.18 s, 6940 +0.26–0.29 s, ours up to about +0.1 s). | **Measure the feed-to-exit time on video** |

### 3.5 Our mechanism (`tools/shotsim/configs/mechanism.json`)

| Parameter | Value | Basis | Status |
|---|---|---|---|
| Hood → launch angle | θ = 72.4° − 34.82°/rot × rot, travel 0–0.6 rot | Slope from CAD: the plate goes from 25.711° at 0 to 46.601° at 0.6 rot. θ at CANcoder 0 (knob 1) is fitted by `calibrate` to the tuned table. This replaces the earlier 70–71° guess. | **Confirm with an inclinometer or 240 fps video** |
| Exit speed per rotor rps | 0.1464 m/s per rps (knob 2) | Fitted by `calibrate`. The earlier prior was 0.1436 (η 0.45, 4 in wheel). A Kraken's kV cannot reveal the gear ratio. | Fitted |
| Rotor speed range | 20–96 rps | 96 is the highest speed used in `PassLookUpTable`. The low end does not bind. | Ours |
| Launcher | Axle height 0.4453 m, forward 0.0994 m, lateral 0.0243 m, exit radius 0.11655 m | CAD (the old 0.44 m was the turret transform height, not the release point) | **Verify the sign of the lateral offset** |
| Turret offset | (−0.0762, 0.1397) m | `Constants.TurretConstants`; matches CAD within 1/8 in | Measured |

### 3.6 Still to measure on our robot (by impact)
1. Hood angle at CANcoder 0 and at 0.6, and the true release point.
2. Shooter type and spin: hood material, compression, any roller. This sets the lift preset.
3. Exit speed per rps, from 240 fps video at 40, 60 and 80 rps.
4. Feed-to-exit latency.
5. Shot dispersion (σ speed, σ angle) and flywheel droop while feeding.
6. Trench height on the real field.
7. The practice ceiling, if maps are generated for practice.
