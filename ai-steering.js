function pidSteering(trackPoints, {
  targetVelocity = 8.0,
  targetPointOffset = 4,
  restrictAngleMargin = 1.2,
  velocityPid = {
    p: 1.0
  },
  anglePid = {
    p: 10.0
  }
} = {}) {

  function pidControl({ p = 0.0, i = 0.0, d = 0.0 } = {}) {
    let prevX;
    let xIntegral = 0.0;
    return (x, dt) => {
      xIntegral += x*dt;
      let dx = 0.0;
      if (dt > 0.0 && prevX !== undefined) {
        dx = (x - prevX) / dt;
      }
      prevX = x;
      return x * p + xIntegral * i + dx * d;
    }
  }

  function computeMaxSafeAngle(car) {
    if (!restrictAngleMargin) return 1000;
    const { gravity, length, staticFriction, maxWheelAngle } = car.properties;
    const minR = math.dot(car.v, car.v) / (staticFriction * gravity) * restrictAngleMargin;
    return Math.min(maxWheelAngle, car.slip.back ? 1000 : Math.atan(length / minR));
  }

  const velocityControl = pidControl(velocityPid);
  const angleControl = pidControl(anglePid);

  const { norm, normalize, argMax, cross2d, distance } = MathHelpers;

  return (car, dt) => {
    const closestTrackPointIndex = argMax(
      trackPoints.map(p => -distance(p, car.pos)));

    const targetIndex = (closestTrackPointIndex + targetPointOffset) % trackPoints.length;
    const targetPoint = trackPoints[targetIndex];
    const targetDistance = norm(targetPoint, car.pos);
    const targetVec = normalize(math.subtract(targetPoint, car.pos));
    const angleDeviation = Math.asin(cross2d(targetVec, car.getForwardDir()));
    const maxAngle = computeMaxSafeAngle(car);
    const targetWheelAngle = MathHelpers.limitAbs(angleDeviation, maxAngle);

    // handle both constant and variable velocities
    let curTargetVelocity = targetVelocity.length ?
      targetVelocity[closestTrackPointIndex] :
      targetVelocity;

    const angleSaturated = Math.abs(targetWheelAngle) === Math.abs(maxAngle);
    if (angleSaturated) curTargetVelocity *= 0.5;

    return {
      throttle: car.slip.back ? 0.0 : velocityControl(curTargetVelocity - car.getSpeed(), dt),
      wheelTurnSpeed: angleControl(targetWheelAngle - car.wheelAngle, dt)
    };
  };
}

function computeInverseCurvature(points) {
  const { normalize, norm, cross2d } = MathHelpers;
  let prev = points[points.length-1];
  let idx = 0;
  return points.map(point => {
    const next = points[++idx % points.length];
    const vPrev = math.subtract(point, prev);
    const vNext = math.subtract(next, point);
    const dAng = Math.asin(cross2d(normalize(vPrev), normalize(vNext)));
    prev = point;
    return dAng / (norm(vPrev) + norm(vNext));
  });
}

function plannedVelocityPidSteering(trackPoints, carProperties, { minVelocity = 5.0 } = {}) {
  const SAFETY_MARGIN_BANK = 0.95;
  const SAFETY_MARGIN_BRAKE = 0.9;
  const maxAcceleration = carProperties.staticFriction * carProperties.gravity * SAFETY_MARGIN_BANK;
  const maxBrake = carProperties.maxThrust / carProperties.mass * SAFETY_MARGIN_BRAKE;

  const maxVelocities = computeInverseCurvature(trackPoints).map(invCurv => {
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

  const func = pidSteering(trackPoints, {
    targetVelocity: maxVelocities
  });

  func.visualization = {
    track: trackPoints,
    velocities: maxVelocities
  };

  return func;
}
