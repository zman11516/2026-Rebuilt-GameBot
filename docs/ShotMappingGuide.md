# Shotmapping Guide: Time of Flight (TOF) and Code Setup

This guide details the procedure for performing **Shotmapping** to collect distance, flywheel speed, hood position, and time of flight (TOF) values, and how to configure them in the robot's code.

> **Hood units:** every hood value in this guide and in the code (`"Hood Target Position"`, the lookup tables, `HoodSetPos`, `getHoodPos`) is the hood **CANcoder position in rotations** (0 to about 0.6), not degrees. 0 is the steepest shot; raising the value flattens it. The launch angle is about 72.4° − 34.82° × rotations (see `src/main/deploy/shooter_calibration.json`).

---

## 1. How the Shotmapping Command Works

The [`ShootMappingCommand`](file:///c:/Users/steve/FRC/2026-Rebuilt-GameBot/src/main/java/frc/robot/commands/ShootMappingCommand.java) allows manual control of the shooter speeds and hood positions directly from the SmartDashboard/Shuffleboard so you can find the sweet spot for any given distance.

### Code Bindings
In [`RobotContainer.java`](file:///c:/Users/steve/FRC/2026-Rebuilt-GameBot/src/main/java/frc/robot/RobotContainer.java#L311-L313):
- **Operator Controller**: Pressing and holding the **Start** button (`m_xBoxOperator.start()`) runs the `ShootMappingCommand`.

### SmartDashboard NetworkTable Inputs/Outputs
The command reads configuration values from, and writes feedback to, the SmartDashboard:
* **Inputs (Editable on Shuffleboard/SmartDashboard)**:
  * `"Target Speed"`: The target flywheel speed in Revolutions Per Second (RPS) (Default: `40`).
  * `"Hood Target Position"`: The hood target in CANcoder rotations, not degrees (Default: `0`).
  * `"Idle Speed"`: The idle speed in RPS when the shooter is not active (Default: `20`).
* **Outputs**:
  * `"Odom Distance"`: Calculated horizontal distance from the turret center to the Hub target center in meters.

---

## 2. Step-by-Step Shotmapping Procedure

Follow these steps to map the field and gather data points:

### Step A: Setup & Positioning
1. Position the robot at a measured distance from the Hub (e.g., 2.0 meters, 2.5 meters, 3.0 meters, etc.).
2. Turn on the robot and connect to the driver station.
3. Open **Shuffleboard** or **SmartDashboard**. Ensure the inputs `"Target Speed"` and `"Hood Target Position"` are visible on the dashboard. If they are not, you can manually add double values with these exact names.

### Step B: Configure the Shot Test
1. Set the `"Target Speed"` (RPS) and `"Hood Target Position"` (CANcoder rotations) on the dashboard to your starting test values.
2. Read the `"Odom Distance"` on the SmartDashboard (or measure the distance physically from the Hub center to the robot's turret pivot center).

### Step C: Capture the Shot and Time of Flight
1. Set up an external high-speed camera (e.g., a phone recording at 120 FPS or 240 FPS) with a clear view of both the shooter exit and the target (Hub).
2. Start the camera recording.
3. Press and hold the **Start button on the Operator Controller** to execute the shot.
   * *What the robot does*: The hood will actuate to `"Hood Target Position"`, the flywheel will spin up to `"Target Speed"`, and once at target speed, it will feed the ball from the hopper to the shooter.
4. Stop the camera recording.

### Step D: Calculate Time of Flight (TOF)
Because Time of Flight is measured externally:
1. Open the recorded video in an editing software or player that allows frame-by-frame scrubbing (like VLC, Avidemux, or similar).
2. Count the number of frames from the exact frame the ball leaves the shooter wheel to the exact frame the ball enters the target or makes contact with the Hub target.
3. Calculate the TOF in seconds:
   $$\text{TOF (seconds)} = \frac{\text{Number of Frames}}{\text{Frame Rate (FPS)}}$$
   * *Example*: If you recorded at 240 FPS and counted 204 frames:
     $$\text{TOF} = \frac{204}{240} = 0.85\text{ seconds}$$

---

## 3. Entering Shotmapping Data into Code

Once you have recorded a set of successful shot parameters, you need to add them to the Lookup Table in [`HubLookUpTable.java`](file:///c:/Users/steve/FRC/2026-Rebuilt-GameBot/src/main/java/frc/robot/subsystems/shooter/HubLookUpTable.java).

1. Open [`HubLookUpTable.java`](file:///c:/Users/steve/FRC/2026-Rebuilt-GameBot/src/main/java/frc/robot/subsystems/shooter/HubLookUpTable.java).
2. Navigate to the `initializeLookupTable()` method.
3. Use the `addEntry` method to add your data points:
   ```java
   // Syntax: addEntry(distanceMeters, shooterSpeedRPS, hoodCANcoderRotations, timeOfFlightSeconds);
   addEntry(2.52, 51.0, 0.1, 0.9);
   ```
4. Add entries sorted by distance. The lookup table will automatically perform linear interpolation (`lerp`) for any distance between the points you define.
5. Copy the same rows into `tools/shotsim/data/tuned_hub.csv` (`d,rps,hoodRot,tof`). The shot-map calibration in section 5 fits to them.

---

## 4. Verification of Shoot-On-The-Move (lookup-table path)

This is the old path. It still runs when `SOTM/UseShotMap` is off or the shot-map files did not load; section 5 covers the shot-map path that replaces it.

The [`ShotOnTheMoveCommand`](file:///c:/Users/steve/FRC/2026-Rebuilt-GameBot/src/main/java/frc/robot/commands/ShotOnTheMoveCommand.java) works by using the Time of Flight (TOF) data points you provide in `HubLookUpTable` to predict where the robot will be when the ball arrives at the target.

### How it operates:
1. **Inputs**: It retrieves the current robot pose and velocity from the drivetrain (`m_swerve`).
2. **Iterative Compensation**:
   * It calculates the field-relative velocity of the turret.
   * It runs a 5-iteration refinement loop to project the virtual turret position based on the ball's Time of Flight (TOF):
     $$\vec{p}_{\text{virtual}} = \vec{p}_{\text{turret}} + \vec{v}_{\text{turret}} \times \text{TOF}$$
   * It fetches the updated TOF for that virtual distance and repeats the loop.
3. **Aim & Speed**:
   * The shooter speed and hood angle are selected based on the virtual distance.
   * The turret is steered to face the target from the future position (so that the robot's velocity vector directs the ball into the hub).

---

## 5. Shot Maps and Shoot-On-The-Move (Team 4414 method)

The shot-map path replaces the hand-tuned table for shooting on the move. A simulator builds the map offline, and the robot only looks it up. Background and sources are in [`docs/shotsim/RESEARCH.md`](shotsim/RESEARCH.md).

### What runs where
* **Offline (`tools/shotsim`, Node 18+, no dependencies):** simulates every (distance, radial velocity) grid point with drag and Magnus lift, picks the most robust hood angle and exit speed (4414's "best weakest margin" rule), fits polynomials, and writes the maps:
  * `src/main/deploy/shotmaps/hub.json`
  * `feed_left.json` / `feed_right.json` for passes that miss the hub
  * `netfeed_left.json` / `netfeed_right.json` for passes over the hub net
  * `src/main/deploy/shooter_calibration.json` with the hood/flywheel conversions, launcher geometry and latency
* **On the robot (`frc.robot.subsystems.shooter.shotmap`):** `TurretUtil.computeShotMapSolution` looks up the map at the turret's distance and radial velocity. It then cancels the tangential velocity with 3D vectors, which sets the turret lead, hood and flywheel. `ShotOnTheMoveCommand` feeds only while every gate passes (shot valid, flywheel, hood and turret within half the map's scoring margin) and re-checks the gates every loop.

### Commands
```bash
node tools/shotsim/cli.mjs generate                 # all maps + shooter_calibration.json (about 1.5 min)
node tools/shotsim/cli.mjs generate --only hub      # one config: hub | feed | netfeed
node tools/shotsim/cli.mjs calibrate                # fit the two knobs to tools/shotsim/data/tuned_hub.csv
node tools/shotsim/cli.mjs calibrate --write        # ...and save them into configs/mechanism.json
node tools/shotsim/cli.mjs point --config hub --d 3 --vr 0   # inspect one grid point
node tools/shotsim/serve.mjs                        # web UI at http://localhost:5454
node --test tools/shotsim/test/*.test.mjs           # tests
```
Edit `tools/shotsim/configs/*.json`, never the generated files. After any change, run `generate`, then deploy.

### Dashboard keys (SmartDashboard)
Each key is published once with its default the first time a SOTM command starts. A changed value is picked up the next time the command starts.

| Key | Default | Meaning |
|---|---|---|
| `SOTM/UseShotMap` | true if the maps loaded | A/B switch: shot map (true) or the old lookup-table lead solution (false). Read every loop. |
| `SOTM/MapMode` | `AUTO` | `POLY` (fitted polynomials), `GRID` (interpolate the raw simulated grid), or anything else for the map's recommendation (GRID for every current map). Read every loop. POLY margins are shifted down to stay conservative. On the current hub map that leaves almost no hood tolerance, so the robot rarely feeds in POLY mode. |
| `SOTM/HoodOffsetDeg` | 0 | Knob 1: degrees added to every launch angle the hood produces. A negative value also lowers the hood's top angle; a shot that needs more than that top angle plus its angle margin reports `HOOD_ABOVE_MAX` and is not fed. |
| `SOTM/ExitSpeedScale` | `flywheel.exitSpeedScale` in the calibration file (1) | Knob 2: measured exit speed / modelled exit speed. Above 1 means the ball leaves faster than modelled, so the robot commands less rps. |
| `SOTM/LatencySec` | `phaseDelay + mechLatency` (0.05 s) | How far ahead the robot pose is predicted. Read every loop. |

Logs are under `Shooter/SOTM/Map/*` (d, vr, vt, elevation, exit speed, hood rotations, flywheel rps, tof, margins, reasons, and the launch vector as `LaunchLine3d`) and `Shooter/SOTM/Ready/*` (one flag per gate).

### Calibration procedure
4414 tunes only two knobs, and so do we. Keep kicker and hopper speeds the same as in a match while calibrating, because feed speed changes the shot.
1. **Measure the geometry once.** Confirm the hood angle at CANcoder 0 and at 0.6 rotations (inclinometer on the last surface the ball touches, or 240 fps side video). Also confirm the release height and the turret offset in `configs/mechanism.json`.
2. **Fit the knobs to tuned shots.** Shot-map the hub at 6 to 10 distances (sections 2 and 3). Put the rows in `tools/shotsim/data/tuned_hub.csv`, then run `calibrate`. It fits `hood.theta0Deg` (the launch angle at CANcoder 0) and `flywheel.exitSpeedPerRotorRps`. Check the per-row misses, then run `calibrate --write` (it also resets `exitSpeedScale` to 1, since the fit is the whole ratio) and `generate`, and deploy.
3. **Static check.** Shoot about 10 balls each at 2, 3, 4 and 5 m with `SOTM/UseShotMap` on, standing still.
   * Everything short or everything long: change `SOTM/ExitSpeedScale`. Start with 0.01 to 0.02 steps; going long means the ball is faster than modelled, so raise it.
   * Error that changes with distance: change `SOTM/HoodOffsetDeg`.
   * Write the final values back into `configs/mechanism.json` (`theta0Deg` and `exitSpeedScale`) and re-run `generate`, so the next deploy starts at zero trims.
4. **Moving check.** Drive toward and away from the hub at about 1 m/s, then sideways. Shots that trail the motion mean `SOTM/LatencySec` is too small. Check `Shooter/SOTM/Ready/*` to see which gate holds the feed.
5. **Passing.** Hold driver X or operator right trigger. The robot picks the nearest pass target and uses the net-feed map when the line to the target crosses the hub net.
6. **Redo steps 2 and 3 after any shooter hardware change** (wheel, compression, belts, hood).
