
// debugging
const LOG_SOME = (() => {
  let counter = 0;
  return function() {
    if (counter++ < 1000) console.log(...arguments);
  };
})();

function Car() {
  this.properties = (() => {
    const dimScale = 0.2;
    const gravity = 9.81;
    const mass = 300.0 * dimScale**3; // kg
    const length = 3 * dimScale;
    const width = 2* dimScale;
    const minTurningRadius = length;

    // moment of inertia
    const MoI = 1/12 * (length * length + width * width) * mass;

    // inertia tensor (mass & MoI)
    const M = [
      [mass, 0, 0],
      [0, mass, 0],
      [0, 0, MoI]
    ];

    // Some of these are physical constants and some actually should
    // depend on the track (like friction). From the point of view of this
    // class, they are constants
    return {
      mass,
      MoI,
      length,
      width,
      gravity,
      M,
      Minv: math.inv(M),
      airResistance: mass * 0.3,
      staticFriction: 1.3,
      dynamicFriction: 0.4,
      wheelTurnSpeed: 2.0, // radians per second
      maxWheelAngle: Math.atan(length / minTurningRadius),
      maxThrust: mass * 0.7 * gravity * 0.5
    };
  })();

  this.pos = [0, 0];
  this.rot = Math.PI * 0.5;

  this.v = [0, this.properties.length];
  this.vrot = 0.0; //1.0;

  this.slip = {
    front: false,
    back: false
  };
  this.frontWheelAngles = [0, 0];
  this.wheelAngle = 0.0;

  const rotVec = MathHelpers.rot90ccw;
  const { cross2d, norm, normalize, rot90cw, limitAbs } = MathHelpers;

  this.move = (dt, controls) => {
    const c = this.properties; // c = "constants"

    if (dt == 0.0 || dt > 1.0) return;

    const v0 = [this.v[0], this.v[1], this.vrot];

    // local coordinates
    const fwd = this.getForwardDir();
    const right = rot90cw(fwd);

    const back = math.multiply(fwd, -c.length*0.5);
    const front = math.multiply(fwd, c.length*0.5);

    // steer
    const turnSpeed = limitAbs(controls.wheelTurnSpeed, c.wheelTurnSpeed);
    this.wheelAngle = limitAbs(this.wheelAngle + turnSpeed * dt, c.maxWheelAngle);

    let frontWheelAxis = right;
    this.frontWheelAngles = [0.0, 0.0];

    if (this.wheelAngle != 0.0) {
      const turningRadius = c.length / Math.tan(this.wheelAngle);
      const turningCenter = math.add(back, math.multiply(right, turningRadius));
      //console.log(wheelAngle, turningRadius);
      frontWheelAxis = normalize(math.subtract(turningCenter, front));

      // fake front wheel angles
      this.frontWheelAngles = [-1, 1].map(side => {
        const pos = math.add(front, math.multiply(right, side*c.width*0.5));
        const axis = normalize(math.subtract(turningCenter, pos));
        return Math.asin(math.dot(axis, fwd)) * Math.sign(turningRadius);
      });
    }

    const thrust = limitAbs(controls.throttle, 1.0) * c.maxThrust;

    // solve forces (no slip)
    const backWheelAxis = right;

    // external forces
    const resistance = math.multiply(this.v, -c.airResistance);
    const externalForces = resistance;

    const solveForcesNoSlip = () => {

      // TODO: this form does not strictly conserve energy!
      const KFperMdt = math.multiply(dt / c.mass, math.transpose([frontWheelAxis, backWheelAxis]));
      const kTauPerMdt = math.multiply(dt / c.MoI, [[cross2d(front, frontWheelAxis), cross2d(back, backWheelAxis)]]);
      const tauVFront = math.transpose([rotVec(front)]);
      const tauVBack = math.transpose([rotVec(back)]);

      const aFront = math.multiply(
        [frontWheelAxis],
        math.add(KFperMdt, math.multiply(tauVFront, kTauPerMdt)));
      const aBack = math.multiply(
        [backWheelAxis],
        math.add(KFperMdt, math.multiply(tauVBack, kTauPerMdt)));

      const thrustForce = math.multiply(thrust, fwd);
      const F0perMdt = math.multiply(dt / c.mass, math.add(externalForces, thrustForce));

      const bFront = -math.dot(frontWheelAxis, math.add(
        F0perMdt,
        this.v,
        math.multiply(rotVec(front), this.vrot))
      );
      const bBack = -math.dot(backWheelAxis, math.add(
        F0perMdt,
        this.v,
        math.multiply(rotVec(back), this.vrot))
      );

      const A = [aFront[0], aBack[0]];
      const b = [bFront, bBack];

      const forceCoeffs = math.lusolve(A, b);
      const forceFront = forceCoeffs[0][0];
      const forceBack = forceCoeffs[1][0];

      return [
        math.multiply(forceFront, frontWheelAxis),
        math.add(math.multiply(forceBack, backWheelAxis), thrustForce)
      ];
    };

    const solveForcesSemiSlip = (point, axis, externalF, externalT) => {
      const KFperMdt = math.multiply(dt / c.mass, math.transpose([axis]));
      const kTauPerMdt = math.multiply(dt / c.MoI, [[cross2d(point, axis)]]);
      const tauV = math.transpose([rotVec(point)]);
      const a = math.multiply([axis], math.add(KFperMdt, math.multiply(tauV, kTauPerMdt)));

      const F0perMdt = math.multiply(dt / c.mass, externalF);
      const bb = -math.dot(axis, math.add(
        F0perMdt,
        this.v,
        math.multiply(rotVec(point), this.vrot + dt * externalT / c.MoI))
      );

      // no need for lusolve here
      //const A = [a[0]];
      //const b = [bb];
      //const forceCoeffs = math.lusolve(A, b);
      const forceCoeff = bb / a[0][0];
      return math.multiply(forceCoeff, axis);
    };

    const solveForces = () => {
      const backFriction = this.slip.back ? c.dynamicFriction : c.staticFriction;
      const frontFriction = this.slip.front ? c.dynamicFriction : c.staticFriction;
      const maxForceBack = c.mass * c.gravity * backFriction / 2.0;
      const maxForceFront = c.mass * c.gravity * frontFriction / 2.0;

      let [forceFront, forceBack] = solveForcesNoSlip();
      this.slip.back = norm(forceBack) > maxForceBack;
      this.slip.front = norm(forceFront) > maxForceFront;

      if (this.slip.back || this.slip.front) {
        const maxForceSlip = c.mass * c.gravity * c.dynamicFriction / 2.0;

        const frontVel = math.add(this.v, math.multiply(rotVec(front), this.vrot));
        const backVel = math.add(this.v, math.multiply(rotVec(back), this.vrot));
        const backSlippy = math.multiply(normalize(forceBack), maxForceSlip);
        const frontSlippy = math.multiply(normalize(forceFront), maxForceSlip);

        forceFront = frontSlippy;
        forceBack = backSlippy;

        if (this.slip.back && !this.slip.front) {
          forceFront = solveForcesSemiSlip(front, frontWheelAxis, math.add(backSlippy, externalForces), cross2d(back, backSlippy));
          this.slip.front = norm(forceFront) > maxForceFront;
          if (this.slip.front) {
            forceFront = frontSlippy;
          }
        }
        else if (this.slip.front && !this.slip.back) {
          forceBack = solveForcesSemiSlip(back, backWheelAxis, math.add(frontSlippy, externalForces), cross2d(front, frontSlippy));
          this.slip.back = norm(forceBack) > maxForceBack;
          if (this.slip.back) {
            forceBack = backSlippy;
          }
        }
      }

      return [forceFront, forceBack];
    };

    const [forceFront, forceBack] = solveForces();

    const totalForce = math.add(externalForces, forceFront, forceBack);
    const totalTorque = cross2d(front, forceFront) + cross2d(back, forceBack);

    const deltaV = math.multiply(dt, math.multiply(c.Minv, [totalForce[0], totalForce[1], totalTorque]));

    // integrate position
    this.pos = math.add(this.pos, math.multiply(this.v, dt));
    this.rot += this.vrot * dt;

    const [vx, vy, vrot] = math.add(v0, deltaV);
    this.v = [vx, vy];
    this.vrot = vrot;
  };

  this.getPolygon = () => {
    const fwd = this.getForwardDir();
    const right = rot90cw(fwd);
    const back = math.multiply(fwd, -this.properties.length*0.5);
    const front = math.multiply(fwd, this.properties.length*0.5);
    const halfSide = math.multiply(right, this.properties.width*0.5);
    return [
      math.subtract(front, halfSide),
      math.add(front, halfSide),
      math.add(back, halfSide),
      math.subtract(back, halfSide)
    ].map(p => math.add(p, this.pos));
  };

  this.getSpeed = () => {
    return norm(this.v);
  };

  this.getForwardDir = () => {
    return [Math.cos(this.rot), Math.sin(this.rot)];
  };

  this.applyImpulse = (point, impulse) => {
    const r = math.subtract(point, this.pos);
    const angularImpulse = cross2d(r, impulse);
    const linearImpulse = impulse;

    const deltaV = math.multiply(this.properties.Minv, [linearImpulse[0], linearImpulse[1], angularImpulse]);
    const v1 = math.add([this.v[0], this.v[1], this.vrot], deltaV);
    this.v = [v1[0], v1[1]];
    this.vrot = v1[2];
  };

  this.collideToWall = (point, normal, restitution = 0.0) => {
    const relPoint = math.subtract(point, this.pos);
    const rv = rotVec(relPoint);
    const pointVel = math.add(this.v, math.multiply(rv, this.vrot));
    const vdotn = math.dot(normal, pointVel);
    if (vdotn > 0.0) return; // alredy detaching -> do nothing

    const { mass, MoI } = this.properties;
    const impulseInelastic = -vdotn / (1.0 / mass + 1 / MoI * cross2d(relPoint, normal) * math.dot(rv, normal));
    const impulse = impulseInelastic * (1.0 + restitution);

    this.applyImpulse(point, math.multiply(impulse, normal));
  };

  this.slowDown = (factor = 0.02) => {
      this.v = math.multiply(1.0 - factor, this.v);
  };
}
