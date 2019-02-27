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
