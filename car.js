
// debugging
const LOG_SOME = (() => {
  let counter = 0;
  return function() {
    if (counter++ < 1000) console.log(...arguments);
  };
})();

function Car() {

  const dimScale = 0.2;
  //this.tScale = 3.0;

  this.dim = {
    length: 3 * dimScale,
    width: 2 * dimScale
  };

  this.mass = 300.0 * dimScale**3; // kg
  this.moi = 1/12 * (
    this.dim.length * this.dim.length +
    this.dim.width * this.dim.width) * this.mass;

  //this.cmBalance = -0.2;

  this.pos = [0, 0];
  this.rot = Math.PI * 0.5;

  this.v = [0, 5 * dimScale];
  this.vrot = 0.0; //1.0;

  this.slip = {
    front: false,
    back: false
  };

  let wheelAngle = 0.0;
  this.frontWheelAngles = [0, 0];

  const MIN_TURNING_RADIUS = this.dim.length;
  const MAX_WHEEL_ANGLE = Math.atan(this.dim.length / MIN_TURNING_RADIUS);
  const WHEEL_TURN_SPEED = 1.5; // radians / sec
  const AIR_RES_COEFF = this.mass * 0.3;
  const GRAVITY = 9.81;
  const STATIC_FRICTION = 0.99;
  const DYNAMIC_FRICTION = 0.2;

  const M = [
    [this.mass, 0, 0],
    [0, this.mass, 0],
    [0, 0, this.moi]
  ];

  const Minv = math.inv(M);

  function cross2d(x, y) {
    return x[0]*y[1] - y[0]*x[1];
  }

  function norm(vec) {
    return Math.sqrt(math.dot(vec, vec));
  }

  function normalize(vec) {
    return math.multiply(vec, 1.0 / norm(vec));
  }

  function rot90cw(vec) {
    return [vec[1], -vec[0]];
  }

  function rot90ccw(vec) {
    return [-vec[1], vec[0]];
  }

  const rotVec = rot90ccw;

  this.steer = (dt, curControls) => {
    // compute min cornering radius for this speed. This "auto-steering" is
    // just to make steering with arrow keys easier
    const SAFETY_MARGIN = 1.7;
    const minR = math.dot(this.v, this.v) / (STATIC_FRICTION * GRAVITY) * SAFETY_MARGIN;
    const maxAngle = Math.min(MAX_WHEEL_ANGLE, this.slip.back ? 1000 : Math.atan(this.dim.length / minR));
    //const sensitivity = Math.min(1.0 / (norm(this.v) / dimScale / 15.0), 1.0);
    //LOG_SOME(minR / this.dim.length);

    let targetWheelAngle = 0.0;
    if (curControls.left) {
      targetWheelAngle = -maxAngle;
    } else if (curControls.right) {
      targetWheelAngle = maxAngle;
    } else {
      targetWheelAngle = 0.0;
    }

    if (wheelAngle != targetWheelAngle) {
      const diff = targetWheelAngle - wheelAngle;
      //console.log(sensitivity);
      const maxDelta = dt * WHEEL_TURN_SPEED;
      wheelAngle += Math.sign(diff) * Math.min(Math.abs(diff), maxDelta);
    }

  };

  this.move = (dt, curControls) => {
    if (dt == 0.0 || dt > 1.0) return;
    //dt *= this.tScale;

    //console.log(curControls);

    const v0 = [this.v[0], this.v[1], this.vrot];

    //this.pos = math.add(this.pos, math.multiply(this.v, dt));
    //this.rot += this.vrot * dt;

    // local coordinates
    const fwd = [Math.cos(this.rot), Math.sin(this.rot)];
    const right = rot90cw(fwd);
    //LOG_SOME(right);

    const back = math.multiply(fwd, -this.dim.length*0.5);
    const front = math.multiply(fwd, this.dim.length*0.5);

    this.steer(dt, curControls);

    let frontWheelAxis = right;
    this.frontWheelAngles = [0.0, 0.0];

    if (wheelAngle != 0.0) {
      const turningRadius = this.dim.length / Math.tan(wheelAngle);
      const turningCenter = math.add(back, math.multiply(right, turningRadius));
      //console.log(wheelAngle, turningRadius);
      frontWheelAxis = normalize(math.subtract(turningCenter, front));

      // fake front wheel angles
      this.frontWheelAngles = [-1, 1].map(side => {
        const pos = math.add(front, math.multiply(right, side*this.dim.width*0.5));
        const axis = normalize(math.subtract(turningCenter, pos));
        return Math.asin(math.dot(axis, fwd)) * Math.sign(turningRadius);
      });

      /*LOG_SOME(wheelAngle);
      LOG_SOME(turningRadius);
      LOG_SOME(back);
      LOG_SOME(right);
      LOG_SOME(turningCenter);
      LOG_SOME(frontWheelAxis);
      LOG_SOME(this.pos);*/
    }

    let thrust = 0.0;
    const maxThrust = this.mass * 0.7 * GRAVITY * 0.5;
    if (curControls.throttle) {
      thrust = maxThrust;
    }
    else if (curControls.brake) {
      thrust = -maxThrust;
    }

    // solve velocity constraints

    // solve forces (no slip)
    const backWheelAxis = right;

    // external forces
    const resistance = math.multiply(this.v, -AIR_RES_COEFF);
    const externalForces = resistance;

    const solveForcesNoSlip = () => {

      //LOG_SOME({frontWheelAxis,backWheelAxis});

      // TODO: this form does not strictly conserve energy!
      const KFperMdt = math.multiply(dt / this.mass, math.transpose([frontWheelAxis, backWheelAxis]));
      const kTauPerMdt = math.multiply(dt / this.moi, [[cross2d(front, frontWheelAxis), cross2d(back, backWheelAxis)]]);
      const tauVFront = math.transpose([rotVec(front)]);
      const tauVBack = math.transpose([rotVec(back)]);

      //console.log([frontWheelAxis]);
      //console.log(math.add(KFperMdt, math.multiply(tauVFront, kTauPerMdt)));

      const aFront = math.multiply(
        [frontWheelAxis],
        math.add(KFperMdt, math.multiply(tauVFront, kTauPerMdt)));
      const aBack = math.multiply(
        [backWheelAxis],
        math.add(KFperMdt, math.multiply(tauVBack, kTauPerMdt)));

      const thrustForce = math.multiply(thrust, fwd);
      //LOG_SOME(this.v);
      //LOG_SOME(fwd);
      //LOG_SOME(thrustForce);
      const F0perMdt = math.multiply(dt / this.mass, math.add(externalForces, thrustForce));

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

      //console.log({dt, vrot: this.vrot});
      //LOG_SOME({A,b});

      const forceCoeffs = math.lusolve(A, b);
      //LOG_SOME(forceCoeffs);

      const forceFront = forceCoeffs[0][0];
      const forceBack = forceCoeffs[1][0];

      //console.log({forceFront, forceBack});

      return [
        math.multiply(forceFront, frontWheelAxis),
        math.add(math.multiply(forceBack, backWheelAxis), thrustForce)
      ];
    };

    const solveForcesSemiSlip = (point, axis, externalF, externalT) => {
      const KFperMdt = math.multiply(dt / this.mass, math.transpose([axis]));
      const kTauPerMdt = math.multiply(dt / this.moi, [[cross2d(point, axis)]]);
      const tauV = math.transpose([rotVec(point)]);
      const a = math.multiply([axis], math.add(KFperMdt, math.multiply(tauV, kTauPerMdt)));

      //LOG_SOME({point, axis, externalF, externalT});

      //const thrustForce = math.multiply(thrust, fwd);
      const F0perMdt = math.multiply(dt / this.mass, externalF);
      const bb = -math.dot(axis, math.add(
        F0perMdt,
        this.v,
        math.multiply(rotVec(point), this.vrot + dt * externalT / this.moi))
      );

      //console.log({dt, vrot: this.vrot});
      //console.log({A,b});

      // no need for lusolve here
      //const A = [a[0]];
      //const b = [bb];
      //const forceCoeffs = math.lusolve(A, b);
      const forceCoeff = bb / a[0][0];
      return math.multiply(forceCoeff, axis);
    };

    const solveForces = () => {
      const backFriction = this.slip.back ? DYNAMIC_FRICTION : STATIC_FRICTION;
      const frontFriction = this.slip.front ? DYNAMIC_FRICTION : STATIC_FRICTION;
      const maxForceBack = this.mass * GRAVITY * backFriction / 2.0;
      const maxForceFront = this.mass * GRAVITY * frontFriction / 2.0;

      let [forceFront, forceBack] = solveForcesNoSlip();
      //console.log(forceFront, forceBack);
      this.slip.back = norm(forceBack) > maxForceBack;
      this.slip.front = norm(forceFront) > maxForceFront;

      //console.log(norm(forceFront), maxForce);

      if (this.slip.back || this.slip.front) {

        //LOG_SOME(`maxForce: ${maxForceFront},${maxForceBack} ${norm(forceFront)},${norm(forceBack)}`);
        const maxForceSlip = this.mass * GRAVITY * DYNAMIC_FRICTION / 2.0;

        const frontVel = math.add(this.v, math.multiply(rotVec(front), this.vrot));
        const backVel = math.add(this.v, math.multiply(rotVec(back), this.vrot));
        //const backSlippy = math.multiply(normalize(frontVel), -maxForceSlip);
        //const frontSlippy = math.multiply(normalize(backVel), -maxForceSlip);
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
        //LOG_SOME(this.slip.front, this.slip.back);
      }

      return [forceFront, forceBack];
    };

    const [forceFront, forceBack] = solveForces();

    const totalForce = math.add(externalForces, forceFront, forceBack);
    const totalTorque = cross2d(front, forceFront) + cross2d(back, forceBack);

    //console.log({totalForce, totalTorque});
    //console.log(math.multiply(dt, math.multiply(Minv, [totalForce[0], totalForce[1], totalTorque])));

    const deltaV = math.multiply(dt, math.multiply(Minv, [totalForce[0], totalForce[1], totalTorque]));
    //const deltaV = vDiff;

    //console.log(deltaV);

    //console.log(`energy: ${E0}`);

    // integrate position
    this.pos = math.add(this.pos, math.multiply(this.v, dt));
    this.rot += this.vrot * dt;

    const [vx, vy, vrot] = math.add(v0, deltaV);
    this.v = [vx, vy];
    this.vrot = vrot;

    //console.log(vrot);
  };

  this.getPolygon = () => {
    const fwd = [Math.cos(this.rot), Math.sin(this.rot)];
    const right = rot90cw(fwd);
    const back = math.multiply(fwd, -this.dim.length*0.5);
    const front = math.multiply(fwd, this.dim.length*0.5);
    const halfSide = math.multiply(right, this.dim.width*0.5);
    return [
      math.subtract(front, halfSide),
      math.add(front, halfSide),
      math.add(back, halfSide),
      math.subtract(back, halfSide)
    ].map(p => math.add(p, this.pos));
  };

  this.applyImpulse = (point, impulse) => {
    const r = math.subtract(point, this.pos);
    const angularImpulse = cross2d(r, impulse);
    const linearImpulse = impulse;

    //console.log({totalForce, totalTorque});
    //console.log(math.multiply(dt, math.multiply(Minv, [totalForce[0], totalForce[1], totalTorque])));

    const deltaV = math.multiply(Minv, [linearImpulse[0], linearImpulse[1], angularImpulse]);
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

    const impulseInelastic = -vdotn / (1.0 / this.mass + 1 / this.moi * cross2d(relPoint, normal) * math.dot(rv, normal));
    const impulse = impulseInelastic * (1.0 + restitution);

    this.applyImpulse(point, math.multiply(impulse, normal));
  };
}
