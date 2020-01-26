/* globals PolicyNetwork, pidControl, tf */
function reinforcementLearningSteering(trackPoints, trackWidth, {
  agent = null,
} = {}) {
  const { norm, normalize, argMax, distance } = MathHelpers;

  const velocityControl = pidControl({ p: 1.0 });
  const angleControl = pidControl({ p: 1.0 });
  let nTurn = 0;
  let trainer;
  let version = 0;
  let visualizations;
  let currentCheckpointIndex = 0;
  //const online = !agent;

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
      const deterministic = false;
      return network.getAction(inputs, deterministic);
    };
  }

  const steering = (car, dt) => {
    const closestTrackPointIndex = argMax(
      trackPoints.map(p => -distance(p, car.pos)));

    function getTrackVectorsAt(offset) {
      const targetIndex = (closestTrackPointIndex + offset) % trackPoints.length;
      const targetPoint = trackPoints[targetIndex];
      const trackPointVector = math.subtract(targetPoint, car.pos);
      const trackDistance = norm(trackPointVector);
      const trackDirection = normalize(math.subtract(
        trackPoints[(targetIndex + 1) % trackPoints.length],
        targetPoint));
      //const targetDirection = normalize(trackPointVector);

      return { trackDistance, trackDirection, trackPointVector };
    }

    const nearPoint = getTrackVectorsAt(2);
    const V_SCALE = 10.0;
    const DIST_SCALE = car.properties.length * 10.0;
    const fwd = car.getForwardDir();
    const curVelocity = car.getSpeed();
    const relWheelAngle = car.wheelAngle / car.properties.maxWheelAngle;
    const anySlip = car.slip.front || car.slip.back;

    const inputs = [
      MathHelpers.cross2d(nearPoint.trackPointVector, nearPoint.trackDirection) / DIST_SCALE,
      MathHelpers.cross2d(nearPoint.trackDirection, fwd),
      //MathHelpers.cross2d(getTrackVectorsAt(10).trackDirection, fwd),
      MathHelpers.cross2d(getTrackVectorsAt(30).trackDirection, fwd),
      curVelocity / V_SCALE,
      relWheelAngle,
      anySlip ? 1 : 0,
      //car.pos[0] / DIST_SCALE,
      //car.pos[1] / DIST_SCALE,
      //MathHelpers.cross2d(targetDirection, fwd),
      //targetDirection[0],
      //targetDirection[1],
      //trackDistance / DIST_SCALE,
      //relWheelAngle,
      //curVelocity,
      //fwd[0],
      //fwd[1],
      //car.vrot,
    ];

    //console.log(inputs);

    nTurn++;

    let reward = 0;
    if (currentCheckpointIndex === null)
      currentCheckpointIndex = closestTrackPointIndex;

    if (!car.offTrack) {
      // main reward is the covered distance (on track) computed with this
      // checkpoint mechanism
      for (let j=0; j<10; ++j) {
        if (closestTrackPointIndex == (currentCheckpointIndex + j) % trackPoints.length) {
          currentCheckpointIndex = closestTrackPointIndex + 1;
          reward += 100;
          break;
        }
      }
    }

    //reward -= Math.pow(Math.abs(relWheelAngle), 2.0);
    //reward -= Math.abs(car.pos[0] - trackPoints[0][0]);
    const dirDot = math.dot(nearPoint.trackDirection, car.v);
    reward -= Math.abs(relWheelAngle) * 0.2;
    if (car.collided) reward -= 30;
    if (car.offTrack) reward -= 2;
    if (dirDot > 0) reward += dirDot;
    if (dirDot < 0) reward -= 10;
    if (math.dot(car.v, fwd) < 0) reward -= 10; // no reverse
    if (car.slip.front || car.slip.back) reward -= 4;

    if (!agent) agent = initAgent({
      networkOptions: {
        inputs: inputs.length,
        outputs: 1
      },
      trackData: {
        trackPoints,
        trackWidth,
        car: {
          pos: car.pos
        }
      }
    });
    const actions = agent(inputs, reward);
    //if (!online) console.log({actions});
    //if (online && nTurn % 100 == 0) console.log(actions);
    //if (!online && nTurn == 2) console.log({offline: actions});

    return {
      throttle: actions[0],
      wheelTurnSpeed: actions[1]
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
