function pidSteering(trackPoints, {
  targetVelocity = 8.0,
  targetPointOffset = 5,
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

function computeRouteLength(points) {
  const { norm } = MathHelpers;
  let prev = points[points.length-1];
  let l = 0;
  points.map(point => {
    const v = math.subtract(point, prev);
    prev = point;
    l += MathHelpers.norm(v);
  });
  return l;
}

function rubberBandRoute(trackPoints, trackWidth, {
  iterations = 60,
  springConstant = 1.0,
  trackConstraintSpringConstant = 20.0,
  dt = 0.4
} = {}) {
  let points = [...trackPoints];

  const restLength = 0.0; //computeRouteLength(points) / points.length * 0.0;
  const { norm, normalize, distance, argMax } = MathHelpers;

  function springForce(vec) {
    const dir = normalize(vec);
    return math.multiply(dir, (restLength - norm(vec))**2 * springConstant);
  }

  for (let itr = 0; itr < iterations; ++itr) {
    let idx = 0;
    let prev = points[points.length-1];
    points = points.map(point => {
      const closestTrackPointIndex = argMax(
        trackPoints.map(p => -distance(p, point)));

      const trackVec = math.subtract(trackPoints[closestTrackPointIndex], point);
      const normal = normalize(trackVec);
      const trackDist = Math.max(0.0, norm(trackVec) - trackWidth);

      const next = points[++idx % points.length];

      const F = math.add(
        springForce(math.subtract(prev, point)),
        springForce(math.subtract(next, point)),
        math.multiply(trackVec, trackDist ** 2 * trackConstraintSpringConstant));

      //console.log(F);
      prev = point;
      return math.add(point, math.multiply(F, dt));
    });
  }

  return points;
}

function planVelocities(trackPoints, carProperties, {
  minVelocity = 5.0,
  safetyMarginBank = 0.95,
  safetyMarginBrake = 0.9
} = {}) {
  const maxAcceleration = carProperties.staticFriction * carProperties.gravity * safetyMarginBank;
  const maxBrake = carProperties.maxThrust / carProperties.mass * safetyMarginBrake;

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

  return maxVelocities;
}

function plannedVelocityPidSteering(trackPoints, carProperties, options = {}) {
  const maxVelocities = planVelocities(trackPoints, carProperties, options);
  const func = pidSteering(trackPoints, { targetVelocity: maxVelocities });

  func.visualization = {
    track: trackPoints,
    velocities: maxVelocities
  };

  return func;
}

function plannedVelocityRubberBandPidSteering(trackPoints, trackWidth, carProperties, options = { minVelocity: 1.0 }) {
  const route = rubberBandRoute(trackPoints, trackWidth);
  const maxVelocities = planVelocities(route, carProperties, options);
  const func = pidSteering(route, { targetVelocity: maxVelocities });

  func.visualization = {
    track: route,
    velocities: maxVelocities
  };

  return func;
}
