const MathHelpers = {
  cross2d(x, y) {
    return x[0]*y[1] - y[0]*x[1];
  },

  norm(vec) {
    return Math.sqrt(math.dot(vec, vec));
  },

  normalize(vec) {
    return math.multiply(vec, 1.0 / MathHelpers.norm(vec));
  },

  rot90cw(vec) {
    return [vec[1], -vec[0]];
  },

  rot90ccw(vec) {
    return [-vec[1], vec[0]];
  },

  argMax(array) { // array must not contain nulls / undefineds
    let maxX, bestI;
    for (let [i, x] of array.entries()) {
      if (i === 0 || x > maxX) {
        maxX = x;
        bestI = i;
      }
    };
    return bestI;
  },

  distance(x, y) {
    return MathHelpers.norm(math.subtract(x, y));
  },

  limitAbs(x, limit) {
    return Math.sign(x) * Math.min(Math.abs(x), limit);
  }
};
