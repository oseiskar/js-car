function pidSteering(trackPoints, {
  targetVelocity = 3.5,
  targetPointOffset = 5,
  velocityPid = {
    p: 1.0
  },
  anglePid = {
    p: 2.0,
    d: 0.5
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

  const velocityControl = pidControl(velocityPid);
  const angleControl = pidControl(anglePid);

  const { norm, normalize, argMax, cross2d, distance } = MathHelpers;

  return (car, dt) => {
    const closestTrackPointIndex = argMax(
      trackPoints.map(p => -distance(p, car.pos)));

    // handle both constant and variable velocities
    const curTargetVelocity = (targetVelocity.length ?
      targetVelocity[closestTrackPointIndex] :
      targetVelocity) * (car.slip.back ? 0.5 : 1.0);

    const targetIndex = (closestTrackPointIndex + targetPointOffset) % trackPoints.length;
    const targetPoint = trackPoints[targetIndex];
    const targetDistance = norm(targetPoint, car.pos);
    const targetVec = normalize(math.subtract(targetPoint, car.pos));
    const angleDeviation = Math.asin(cross2d(targetVec, car.getForwardDir()));
    const targetWheelAngle = angleDeviation;

    return {
      throttle: velocityControl(curTargetVelocity - car.getSpeed(), dt),
      wheelTurnSpeed: angleControl(targetWheelAngle - car.wheelAngle, dt)
    };
  };
}

function simpleAISteering(trackPoints) {
  return pidSteering(trackPoints);
}
