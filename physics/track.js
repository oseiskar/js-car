// generates a normally distributed random variable
function randnBoxMuller(rng) {
  let u = 0; let v = 0;
  while (u === 0) u = rng(); // Converting [0,1) to (0,1)
  while (v === 0) v = rng();
  return Math.sqrt(-2.0 * Math.log(u)) * Math.cos(2.0 * Math.PI * v);
}

class Track {
  constructor({ widthMeters = 600, heightMeters = 400 } = {}) {
      this.size = { width: widthMeters, height: heightMeters };
      this.cars = [];
      this.trackWidth = 2.0;
      this.points = this.generateTrackPoints();
  }

  addCar(steering, name) {
    const model = new Car();
    model.pos = this.points[0];
    this.cars.push({
      model,
      steering,
      name
    });
  }

  generateTrackPoints() {
    const N_TRACK_POINTS = 100;
    const TRACK_REL_RADIUS = 0.8;

    const fourierCoefficients = [];
    for (let i=0; i<10; ++i) {
      fourierCoefficients.push(randnBoxMuller(Math.random) * Math.exp(-i * 0.08)*0.1);
    }

    function restrict(c, limit) {
      return Math.max(-limit, Math.min(limit, c));
    }

    const points = [];
    for (let i=0; i < N_TRACK_POINTS; ++i) {
      const phi = i / N_TRACK_POINTS * Math.PI * 2;
      let r = 1.0;
      for (let j=0; j<fourierCoefficients.length; ++j) {
        const f = j % 2 === 0 ? Math.cos : Math.sin;
        const n = Math.ceil(j/2);
        r += f(n*phi) * fourierCoefficients[j];
      }
      r *= TRACK_REL_RADIUS;
      const rx = r * this.size.width * 0.5;
      const ry = r * this.size.height * 0.5;
      points.push([
        restrict(Math.cos(phi) * rx, this.size.width*0.5 - this.trackWidth/2.0),
        restrict(Math.sin(phi) * ry, this.size.height*0.5 - this.trackWidth/2.0)]);
    }

    return points;
  }

  move(dt) {
    this.cars.forEach(car => {
      car.model.move(dt, car.steering(car.model, dt));
      this.collisions(car.model, dt);
    });
  }

  collisions(car, dt) {
    const bnd = [
      this.size.width * 0.5,
      this.size.height * 0.5
    ];

    // how elastic are the wall collisions
    const coefficientOfRestitution = 0.2;

    car.getPolygon().forEach(p => {
      [0,1].forEach(coord => {
        [-1,1].forEach(side => {
          if (side*p[coord] > bnd[coord]) {
            const normal = [0,0];
            normal[coord] = -side;
            car.collideToWall(p, normal, coefficientOfRestitution);
          }
        });
      });
    });
  }
}
