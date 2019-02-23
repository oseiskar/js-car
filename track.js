class Track {
  constructor({ widthMeters = 600, heightMeters = 400 } = {}) {
      this.size = { width: widthMeters, height: heightMeters };
      this.car = new Car();
      this.car.pos = [0, 0];
  }

  move(dt, curControls) {
    this.car.move(dt, curControls);
    this.collisions(dt);
  }

  collisions(dt) {
    const bnd = [
      this.size.width * 0.5,
      this.size.height * 0.5
    ];

    // how elastic are the wall collisions
    const coefficientOfRestitution = 0.2;

    this.car.getPolygon().forEach(p => {
      [0,1].forEach(coord => {
        [-1,1].forEach(side => {
          if (side*p[coord] > bnd[coord]) {
            const normal = [0,0];
            normal[coord] = -side;
            this.car.collideToWall(p, normal, coefficientOfRestitution);
          }
        });
      });
    });
  }
};
