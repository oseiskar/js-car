class Track {
  constructor({ widthMeters = 600, heightMeters = 400 } = {}) {
      this.size = { width: widthMeters, height: heightMeters };
      this.car = new Car();
      this.car.pos = [0, 0];
  }

  move(dt, curControls) {
    this.car.move(dt, curControls);
  }
};
