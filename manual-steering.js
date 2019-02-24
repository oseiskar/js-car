
function manualSteering() {
  const keyControls = {};

  const keymap = {
    37: 'left',
    39: 'right',
    38: 'throttle',
    40: 'brake'
  };

  document.onkeydown = function(e) {
    if (keymap[e.keyCode]) {
      keyControls[keymap[e.keyCode]] = true;
    }
  };

  document.onkeyup = function(e) {
    if (keymap[e.keyCode]) {
      keyControls[keymap[e.keyCode]] = false;
    }
  };

  let timeSinceSteerChange = 0.0;
  let lastSteerDir = 0;

  // these are further limited by the car
  const MAX_WHEEL_ANGLE = Math.PI;
  const MAX_TURN_SPEED = 1000; // max target turn speed

  const EXTREME_SENSITVITY = 1.0 / 40.0;

  const STATIC_FRICTION = 1.3;
  const GRAVITY = 9.81;

  return (car, dt) => {

    // compute min cornering radius for this speed. This "auto-steering" is
    // just to make steering with arrow keys easier
    const SAFETY_MARGIN = 1.8 - Math.min(timeSinceSteerChange, 0.8) + Math.abs(car.vrot) * 0.1;
    const minR = math.dot(car.v, car.v) / (STATIC_FRICTION * GRAVITY) * SAFETY_MARGIN;
    const maxAngle = Math.min(MAX_WHEEL_ANGLE, car.slip.back ? 1000 : Math.atan(car.dim.length / minR));

    //const sensitivity = Math.min(1.0 / (norm(this.v) / dimScale / 15.0), 1.0);
    //LOG_SOME(minR / this.dim.length);

    const steerDir = keyControls.left ? -1 : (keyControls.right ? 1 : 0);
    if (lastSteerDir === steerDir) {
      timeSinceSteerChange += dt;
    } else {
      timeSinceSteerChange = 0;
    }
    lastSteerDir = steerDir;

    const targetWheelAngle = steerDir ? maxAngle * steerDir : 0;

    let turnSpeed = MAX_TURN_SPEED;
    if (steerDir) {
      const d = Math.abs(targetWheelAngle - car.wheelAngle);
      turnSpeed = Math.min(MAX_TURN_SPEED, d / EXTREME_SENSITVITY);
    }

    let turn = 0.0;
    if (car.wheelAngle != targetWheelAngle) {
      const diff = targetWheelAngle - car.wheelAngle;
      //console.log(sensitivity);
      const maxDelta = dt * turnSpeed;
      turn = Math.sign(diff) * Math.min(Math.abs(diff), maxDelta);
    }
    if (dt > 0) turn /= dt;

    return {
      throttle: keyControls.throttle ? 1.0 : (keyControls.brake ? -1.0 : 0.0),
      wheelTurnSpeed: turn
    };
  };
}
