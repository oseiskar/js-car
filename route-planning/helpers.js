const RouteHelpers = {
  computeInverseCurvature(points) {
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
  },

  computeRouteLength(points) {
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
};
