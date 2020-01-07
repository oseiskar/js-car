// returns a function (carState, dt) => controls
function pidSteering(trackPoints, {
  targetVelocity = 8.0,
  targetPointOffset = 2,
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
    };
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
    const trackPointVector = math.subtract(targetPoint, car.pos);

    const trackDirection = normalize(math.subtract(
      trackPoints[(targetIndex + 1) % trackPoints.length],
      targetPoint));

    const targetDirection = normalize(math.add(
      trackDirection,
      trackPointVector
    ));

    const angleDeviation = Math.asin(cross2d(targetDirection, car.getForwardDir()));
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
