
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

  function computeMaxSafeAngle(car) {
    const { gravity, length, staticFriction, maxWheelAngle } = car.properties;

    // compute min cornering radius for this speed. This "auto-steering" is
    // just to make steering with arrow keys easier
    const SAFETY_MARGIN = 1.8 - Math.min(timeSinceSteerChange, 0.8) + Math.abs(car.vrot) * 0.1;
    const minR = math.dot(car.v, car.v) / (staticFriction * gravity) * SAFETY_MARGIN;
    return Math.min(maxWheelAngle, car.slip.back ? 1000 : Math.atan(length / minR));
  }

  return (car, dt) => {
    const maxAngle = computeMaxSafeAngle(car);

    const steerDir = keyControls.left ? -1 : (keyControls.right ? 1 : 0);
    if (lastSteerDir === steerDir) {
      timeSinceSteerChange += dt;
    } else {
      timeSinceSteerChange = 0;
    }
    lastSteerDir = steerDir;

    const targetWheelAngle = steerDir ? maxAngle * steerDir : 0;

    const { wheelTurnSpeed, maxWheelAngle } = car.properties;
    let turnSpeed = wheelTurnSpeed;
    if (steerDir) {
      const EXTREME_SENSITVITY = 1.0 / 40.0;
      const d = Math.abs(targetWheelAngle - car.wheelAngle);
      turnSpeed = Math.min(maxWheelAngle, d / EXTREME_SENSITVITY);
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
