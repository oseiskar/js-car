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

    //const lastReward = -Math.abs(relWheelAngle);
    const dirDot = math.dot(nearPoint.trackDirection, fwd);
    reward -= Math.abs(relWheelAngle) * 0.2;
    if (car.collided) reward -= 30;
    if (car.offTrack) reward -= 0.5 * Math.pow(nearPoint.trackDistance / (trackWidth * 0.5), 2.0);
    if (dirDot > 0) reward += 1;
    if (dirDot < 0) reward -= 10;
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
    const action = agent(inputs, reward);

    const targetWheelAngle = ((action % 2) === 1 ? -1 : 1) * car.properties.maxWheelAngle;
    let wheelTurnSpeed = angleControl(targetWheelAngle - car.wheelAngle, dt);

    // "slow" or "fast"
    let targetVelocity = (action < 2) ? 10.0 : 1.0;
    if (action < 2) {
      if (!anySlip) wheelTurnSpeed *= 0.3;
      targetVelocity = 10.0;
    } else {
      targetVelocity = 1.0;
    }
    return {
      throttle: car.slip.back ? 0 : velocityControl(targetVelocity, dt),
      wheelTurnSpeed
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
