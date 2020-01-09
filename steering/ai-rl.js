/* globals PolicyNetwork, pidControl, tf */
function reinforcementLearningSteering(trackPoints, {
  targetPointOffset = 2
} = {}) {
  const { norm, normalize, argMax, distance } = MathHelpers;

  let agent;
  const velocityControl = pidControl({ p: 1.0 });
  let nTurn = 0;

  function initAgent(options) {
    const network = new PolicyNetwork(options);
    const trainer = new Worker('./steering/rl-worker.js');
    trainer.postMessage({ initialize: options });
    trainer.onmessage = function (msg) {
      tf.loadLayersModel({
        // load using a trivial tf-js LoadHandler interface
        load() { return Promise.resolve(msg.data.model); }
      }).then(model => {
        console.log(`new model weights!`);
        network.policyNet = model;
      });
    };

    return function (inputs, lastReward) {
      trainer.postMessage({ inputs, lastReward });
      return network.getAction(inputs);
    };
  }

  return (car, dt) => {
    const closestTrackPointIndex = argMax(
      trackPoints.map(p => -distance(p, car.pos)));

    const targetIndex = (closestTrackPointIndex + targetPointOffset) % trackPoints.length;
    const targetPoint = trackPoints[targetIndex];
    const trackPointVector = math.subtract(targetPoint, car.pos);
    const trackDistance = norm(trackPointVector);

    const trackDirection = normalize(math.subtract(
      trackPoints[(targetIndex + 1) % trackPoints.length],
      targetPoint));

    const targetDirection = normalize(math.add(
      trackDirection,
      trackPointVector
    ));

    const V_SCALE = 10.0;
    const fwd = car.getForwardDir();

    const inputs = [
      targetDirection[0],
      targetDirection[1],
      trackDistance,
      car.wheelAngle,
      fwd[0],
      fwd[1],
      car.vrot,
      //car.pos[0],
      //car.pos[1],
      //car.v[0] / V_SCALE,
      //car.v[1] / V_SCALE,
      //car.slip.front,
      //car.slip.back
    ];

    nTurn++;
    const texp = 1.0 - Math.exp(-nTurn*0.001);
    const wheelAnglePenalty = 10.0 * (1.0 - texp) + 0.1;
    const trackDistPenalty = 0.1 * texp;

    //const lastReward = norm(car.v);
    const lastReward = math.dot(targetDirection, fwd)*texp - Math.abs(car.wheelAngle)*wheelAnglePenalty - trackDistance*trackDistPenalty;

    if (!agent) agent = initAgent({ inputs: inputs.length, outputs: 1 });
    const action = agent(inputs, lastReward);

    //if (nTurn % 10 == 0) LOG_SOME({action, lastReward, wheelAnglePenalty, trackDistance, inputs});
    //const [ throttle, wheelTurnSpeed ] = action;

    // simple steering model
    const curTargetVelocity = texp;
    const throttle = car.slip.back ? 0.0 : velocityControl(curTargetVelocity - car.getSpeed(), dt);
    const wheelTurnSpeed = (action == 1 ? 1 : -1);

    return {
      throttle: throttle,
      wheelTurnSpeed: wheelTurnSpeed * car.properties.wheelTurnSpeed
    };
  };
}
