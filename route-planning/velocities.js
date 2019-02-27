function planVelocities(trackPoints, carProperties, {
  minVelocity = 5.0,
  safetyMarginBank = 0.9,
  safetyMarginBrake = 0.9
} = {}) {
  const maxAcceleration = carProperties.staticFriction * carProperties.gravity * safetyMarginBank;
  const maxBrake = carProperties.maxThrust / carProperties.mass * safetyMarginBrake;

  const maxVelocities = RouteHelpers.computeInverseCurvature(trackPoints).map(invCurv => {
    const R = 1.0 / Math.max(1e-6, Math.abs(invCurv));
    return Math.max(minVelocity, Math.sqrt(maxAcceleration * R));
  });

  const minVelIdx = MathHelpers.argMax(maxVelocities.map(x => -x));
  let maxVel = maxVelocities[minVelIdx];
  let idx = minVelIdx;
  do {
    const nextPoint = trackPoints[idx];
    idx--;
    if (idx < 0) idx += maxVelocities.length;
    const ds = MathHelpers.distance(nextPoint, trackPoints[idx]);
    const dt = ds / maxVel;
    const dv = dt * maxBrake;
    maxVel = Math.min(maxVelocities[idx], maxVel + dv);
    maxVelocities[idx] = maxVel;
  } while(idx !== minVelIdx);

  return maxVelocities;
}
