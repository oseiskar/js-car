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

function rubberBandRouteOptimizer(trackPoints, trackWidth, {
  iterations = 60,
  springConstant = 2.0,
  stepSize = 0.2,
  trackConstraintSpringConstant = 0.5,
} = {}) {
  let points = [...trackPoints];

  const restLength = 0.0;
  const { norm, normalize, distance, argMax } = MathHelpers;

  function springForce(vec) {
    const dir = normalize(vec);
    return math.multiply(dir, -(restLength - norm(vec)) * springConstant);
  }

  let itr = 0;
  let idx = 0;
  let newPoints;
  let prev;

  function work() {
    if (idx === 0) {
      prev = points[points.length-1];
      newPoints = [];
    }

    const point = points[idx];

    const closestTrackPointIndex = argMax(
      trackPoints.map(p => -distance(p, point)));

    const trackVec = math.subtract(trackPoints[closestTrackPointIndex], point);
    const normal = normalize(trackVec);
    const trackDist = Math.max(0.0, norm(trackVec) - trackWidth*0.7);

    const next = points[(idx+1) % points.length];

    const F = math.add(
      springForce(math.subtract(prev, point)),
      springForce(math.subtract(next, point)),
      math.multiply(trackVec, trackDist * trackConstraintSpringConstant));

    const dt = stepSize * (1.0 - itr/iterations);
    newPoints.push(math.add(point, math.multiply(F, dt)));

    prev = point;
    if (++idx === points.length) {
      console.log(`new route ${itr}`);
      itr++;
      points = newPoints;
      idx = 0;
      return true;
    }
    return false;
  }

  return () => {
    const ready = work() || work() || work();
    return {
      ready,
      final: itr >= iterations,
      route: points
    };
  };
}

function planVelocities(trackPoints, carProperties, {
  minVelocity = 5.0,
  safetyMarginBank = 0.9,
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

function plannedVelocityRubberBandPidSteering(trackPoints, trackWidth, carProperties, options = { minVelocity: 1.0 }) {
  const routeOptimizer = rubberBandRouteOptimizer(trackPoints, trackWidth*0.5 - carProperties.width);

  // a bit of a hack: allow modifying these in place elsewhere so that
  // steering always uses the latest version of the, possibly changed track
  // and velocities
  let route = [...trackPoints];
  let maxVelocities = trackPoints.map(() => 1);

  const steering = pidSteering(route, { targetVelocity: maxVelocities });

  let version = 0;
  let finished;
  const func = (car, dt) => {
    changed = false;
    if (!finished) {
      const curRoute = routeOptimizer();
      if (curRoute.ready) {
        version++;
        finished = curRoute.final;
        const curMaxVelocities = planVelocities(route, carProperties, options);
        for (let i = 0; i < route.length; ++i) {
          route[i] = curRoute.route[i];
          maxVelocities[i] = curMaxVelocities[i];
        }
      }
    }
    return steering(car, dt);
  }

  func.visualization = () => ({
    version, // to only render when changed
    route,
    velocities: maxVelocities
  });

  return func;
}
