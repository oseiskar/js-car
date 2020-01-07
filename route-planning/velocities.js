/* globals RouteHelpers */
function planVelocities(trackPoints, carProperties, {
  safetyMarginBank = 0.9,
  safetyMarginBrake = 0.9
} = {}) {
  const maxAcceleration = carProperties.staticFriction * carProperties.gravity * safetyMarginBank;
  const maxBrake = carProperties.maxThrust / carProperties.mass * safetyMarginBrake;

  const curvatureRadii = RouteHelpers.computeCurvature(trackPoints)
    .map(kappa => 1.0 / Math.max(1e-6, Math.abs(kappa)));

  const maxVelocities = curvatureRadii.map(R => Math.sqrt(maxAcceleration * R));

  const minVelIdx = MathHelpers.argMax(maxVelocities.map(x => -x));
  let maxVel = maxVelocities[minVelIdx];
  let idx = minVelIdx;
  do {
    const nextPoint = trackPoints[idx];
    idx--;
    if (idx < 0) idx += maxVelocities.length;
    const ds = MathHelpers.distance(nextPoint, trackPoints[idx]);
    const dt = ds / maxVel;
    const dv = Math.min(dt * maxBrake, maxVelocities[idx] - maxVel);
    const a = MathHelpers.limitAbs(dv / dt, maxBrake);

    if (true /*a > 0.0*/) {
      const R = curvatureRadii[idx];
      const maxCorneringVel = (maxAcceleration**2 - a**2)**0.25 * Math.sqrt(R);
      maxVel = Math.min(maxVel + dt, maxCorneringVel);
    } else {
      maxVel += dt;
    }
    maxVelocities[idx] = maxVel;
  } while(idx !== minVelIdx);

  return maxVelocities;
}
