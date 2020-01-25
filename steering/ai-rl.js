/* globals PolicyNetwork, pidControl, tf */
function reinforcementLearningSteering(trackPoints, {
  targetPointOffset = 8,
  agent = null
} = {}) {
  const { norm, normalize, argMax, distance } = MathHelpers;

  const velocityControl = pidControl({ p: 1.0 });
  let nTurn = 0;
  let trainer;
  let version = 0;
  let visualizations;
  let offlineMode = !!agent;

  function initAgent({ networkOptions, trackData }) {
    const network = new PolicyNetwork(networkOptions);
    trainer = new Worker('./steering/rl-worker.js');
    trainer.postMessage({ initialize: { networkOptions, trackData, offline: true } });
    trainer.onmessage = function (msg) {
      version++;
      visualizations = msg.data.visualizations;
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

  const steering = (car, dt) => {
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
    const DIST_SCALE = car.properties.length * 10.0;
    const fwd = car.getForwardDir();
    const curVelocity = car.getSpeed();
    const relWheelAngle = car.wheelAngle / car.properties.maxWheelAngle;

    const inputs = [
      MathHelpers.cross2d(targetDirection, fwd),
      //targetDirection[0],
      //targetDirection[1],
      trackDistance / DIST_SCALE,
      relWheelAngle,
      //curVelocity,
      //fwd[0],
      //fwd[1],
      //car.vrot,
      //car.pos[0],
      //car.pos[1],
      //car.v[0] / V_SCALE,
      //car.v[1] / V_SCALE,
      //car.slip.front,
      //car.slip.back
    ];

    //console.log(inputs);

    nTurn++;

    //const lastReward = -Math.abs(relWheelAngle);
    const dirDot = math.dot(targetDirection, fwd);
    let lastReward = dirDot * curVelocity;
    lastReward -= Math.abs(relWheelAngle);
    if (car.collided) lastReward -= 10;
    if (dirDot < 0) lastReward -= 2;
    /*lastReward -= trackDistance * 0.1;
    //lastReward -= car.collidedTurns * 5;
    if (dirDot < 0) lastReward -= 2;*/

    if (!agent) agent = initAgent({
      networkOptions: {
        inputs: inputs.length,
        outputs: 1
      },
      trackData: {
        trackPoints,
        car: {
          pos: car.pos
        }
      }
    });
    const action = agent(inputs, lastReward);

    //if (nTurn % 10 == 0) LOG_SOME({action, lastReward, wheelAnglePenalty, trackDistance, inputs});
    //const [ throttle, wheelTurnSpeed ] = action;

    // simple steering model
    const curTargetVelocity = 3.0;
    const throttle = car.slip.back ? 0.0 : velocityControl(curTargetVelocity - curVelocity, dt);
    const wheelTurnSpeed = (action == 1 ? 1 : -1);

    return {
      throttle: throttle,
      wheelTurnSpeed: wheelTurnSpeed * car.properties.wheelTurnSpeed
    };
  };

  steering.stop = () => {
    if (trainer) trainer.terminate();
  };

  steering.visualization = () => ({
    version, // to only render when changed
    traces: visualizations
  });

  return steering;
}
