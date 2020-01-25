/* globals rubberBandRouteOptimizer, pidSteering, planVelocities */
function plannedVelocityRubberBandPidSteering(trackPoints, trackWidth, carProperties, options = { minVelocity: 1.0 }) {
  const routeOptimizer = new Worker('./route-planning/rubber-band.js');
  routeOptimizer.postMessage({
    trackPoints,
    options,
    carProperties,
    width: trackWidth*0.5 - carProperties.width
  });

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

  routeOptimizer.onmessage = function (e) {
    const { route: curRoute, maxVelocities: curMaxVelocities } = e.data;
    for (let i = 0; i < route.length; ++i) {
      route[i] = curRoute[i];
      maxVelocities[i] = curMaxVelocities[i];
    }
    version++;
  };

  steering.visualization = () => ({
    version, // to only render when changed
    route,
    velocities: maxVelocities
  });

  steering.stop = () => routeOptimizer.terminate();

  return steering;
}
