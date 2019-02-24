function simpleAISteering(trackPoints) {
  const { norm, normalize, argMax, cross2d, distance } = MathHelpers;

  const N_FORWARD = 5;
  const TARGET_VELOCITY = 3.5;
  const VEL_P = 1.0;
  const ANGLE_P = 2.0;
  const ANGLE_D = 0.5;

  let prevAngleError;

  return (car, dt) => {
    const closestTrackPointIndex = argMax(
      trackPoints.map(p => -distance(p, car.pos)));

    const targetIndex = (closestTrackPointIndex + N_FORWARD) % trackPoints.length;
    const targetPoint = trackPoints[targetIndex];
    const targetDistance = norm(targetPoint, car.pos);
    const targetVec = normalize(math.subtract(targetPoint, car.pos));
    const fwd = car.getForwardDir();
    const angleDeviation = Math.asin(cross2d(targetVec, fwd));
    const targetWheelAngle = angleDeviation;

    const angleError = (targetWheelAngle - car.wheelAngle);

    let dAngleError = 0.0;
    if (prevAngleError !== undefined && dt > 0.0) {
      dAngleError = (angleError - prevAngleError) / dt;
    }
    prevAngleError = angleError;

    const targetVel = TARGET_VELOCITY * (car.slip.back ? 0.5 : 1.0);
    return {
      throttle: (targetVel - car.getSpeed()) * VEL_P,
      wheelTurnSpeed: (targetWheelAngle - car.wheelAngle) * ANGLE_P + dAngleError * ANGLE_D
    };
  };
}
