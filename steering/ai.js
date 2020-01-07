function plannedVelocityRubberBandPidSteering(trackPoints, trackWidth, carProperties, options = { minVelocity: 1.0 }) {
  const routeOptimizer = rubberBandRouteOptimizer(trackPoints, trackWidth*0.5 - carProperties.width);

  // a bit of a hack: allow modifying these in place elsewhere so that
  // steering always uses the latest version of the, possibly changed track
  // and velocities
  let route = [...trackPoints];
  let maxVelocities = trackPoints.map(() => 1);

  const steering = pidSteering(route, {
    targetVelocity: maxVelocities,
    anglePid: { p: 15 }
  });

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
  };

  func.visualization = () => ({
    version, // to only render when changed
    route,
    velocities: maxVelocities
  });

  return func;
}
