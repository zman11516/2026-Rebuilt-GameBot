// Ball-flight physics for the 2026 FUEL ball.
//
// 2D point mass in the vertical plane that contains the shot:
//   x = horizontal distance along the shot line (m), z = height above the floor (m).
// Forces: gravity, buoyancy, quadratic drag, and Magnus lift from a constant backspin.
// Spin is set by the SHOOTER exit speed (not the field speed), which is why robot
// radial velocity has to be simulated instead of just subtracted (see 4414's TideShot notes).

/**
 * Precompute the per-shot constants from a physics config block.
 * @param {object} p physics config (see configs/hub.json "physics")
 */
export function makeAero(p) {
  const r = p.ballDiameter / 2;
  const area = Math.PI * r * r;
  const volume = (4 / 3) * Math.PI * r * r * r;
  const kq = (p.airDensity * area) / (2 * p.ballMass); // a = kq * C * v^2
  return {
    r,
    g: p.gravity,
    buoyancy: p.buoyancy ? (p.airDensity * volume * p.gravity) / p.ballMass : 0,
    kDrag: kq * p.dragCoefficient,
    kq,
    kL: p.lift?.kL ?? 0,
    clMax: p.lift?.clMax ?? 0,
    spinRatio: p.lift?.spinRatio ?? 0,
    dt: p.integrator?.dt ?? 0.004,
    maxTime: p.integrator?.maxTime ?? 5,
  };
}

/** Backspin (rad/s) produced by a shooter exit speed s (m/s). */
export function spinFor(aero, shooterSpeed) {
  return aero.r > 0 ? (aero.spinRatio * shooterSpeed) / aero.r : 0;
}

/**
 * Integrate one shot with RK4 and report events to an observer.
 *
 * The observer is called after every step as observer(state) where state is a reused object
 * { t, x, z, vx, vz, px, pz, pvx, pvz, pt } (p* = previous step). Return true to stop early.
 *
 * @param {object} aero from makeAero
 * @param {number} x0 release x (m)
 * @param {number} z0 release z (m)
 * @param {number} vx0 initial field-frame horizontal velocity (m/s)
 * @param {number} vz0 initial vertical velocity (m/s)
 * @param {number} spin backspin (rad/s), positive = lift
 * @param {(s: object) => boolean} observer
 */
export function integrate(aero, x0, z0, vx0, vz0, spin, observer) {
  const { g, buoyancy, kDrag, kq, kL, clMax, r, dt, maxTime } = aero;
  const gz = -g + buoyancy;
  const wr = spin * r;

  // Acceleration, written inline for speed (this runs millions of times per map).
  let ax = 0;
  let az = 0;
  const accel = (vx, vz) => {
    const v = Math.sqrt(vx * vx + vz * vz);
    if (v < 1e-9) {
      ax = 0;
      az = gz;
      return;
    }
    let cl = kL * (wr / v);
    if (cl > clMax) cl = clMax;
    else if (cl < -clMax) cl = -clMax;
    const kl = kq * cl;
    // drag: -kDrag*v*(vx,vz); lift: kl*v*(-vz, vx)
    ax = -kDrag * v * vx - kl * v * vz;
    az = gz - kDrag * v * vz + kl * v * vx;
  };

  const s = { t: 0, x: x0, z: z0, vx: vx0, vz: vz0, pt: 0, px: x0, pz: z0, pvx: vx0, pvz: vz0 };
  let x = x0;
  let z = z0;
  let vx = vx0;
  let vz = vz0;
  let t = 0;
  const h2 = dt / 2;
  while (t < maxTime) {
    accel(vx, vz);
    const k1x = vx, k1z = vz, k1vx = ax, k1vz = az;
    accel(vx + k1vx * h2, vz + k1vz * h2);
    const k2x = vx + k1vx * h2, k2z = vz + k1vz * h2, k2vx = ax, k2vz = az;
    accel(vx + k2vx * h2, vz + k2vz * h2);
    const k3x = vx + k2vx * h2, k3z = vz + k2vz * h2, k3vx = ax, k3vz = az;
    accel(vx + k3vx * dt, vz + k3vz * dt);
    const k4x = vx + k3vx * dt, k4z = vz + k3vz * dt, k4vx = ax, k4vz = az;

    s.pt = t;
    s.px = x;
    s.pz = z;
    s.pvx = vx;
    s.pvz = vz;
    x += (dt / 6) * (k1x + 2 * k2x + 2 * k3x + k4x);
    z += (dt / 6) * (k1z + 2 * k2z + 2 * k3z + k4z);
    vx += (dt / 6) * (k1vx + 2 * k2vx + 2 * k3vx + k4vx);
    vz += (dt / 6) * (k1vz + 2 * k2vz + 2 * k3vz + k4vz);
    t += dt;
    s.t = t;
    s.x = x;
    s.z = z;
    s.vx = vx;
    s.vz = vz;
    if (observer(s) || z < -0.5) break;
  }
}

/** Sample a full trajectory (for plots). Stops when the ball reaches the floor. */
export function trajectory(aero, x0, z0, vx0, vz0, spin, { maxPoints = 2000, untilX = Infinity } = {}) {
  const pts = [{ t: 0, x: x0, z: z0 }];
  integrate(aero, x0, z0, vx0, vz0, spin, (s) => {
    if (pts.length < maxPoints) pts.push({ t: s.t, x: s.x, z: s.z });
    return s.z <= 0 || s.x > untilX;
  });
  return pts;
}
