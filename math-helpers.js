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
  }
};
