 /* globals PolicyNetwork, Track, reinforcementLearningSteering, pidSteering, tf */
 importScripts(
  './policy-network.js',
  './ai-rl.js',
  './pid.js',
  '../math-helpers.js',
  '../physics/car.js',
  '../physics/track.js',
  '//cdnjs.cloudflare.com/ajax/libs/mathjs/5.4.2/math.min.js',
  '//cdn.jsdelivr.net/npm/@tensorflow/tfjs@1.5.1/dist/tf.min.js');

function stride(n, includeEnd = true) {
  let i = 0;
  return (array) => array.filter(() => {
    if (includeEnd && i === array.length - 1) return true;
    return (i++ % n) === 0;
  });
}

function SupervisedTrainer(saveCallback, { networkOptions, trackData }) {
  const policyNetwork = new PolicyNetwork(networkOptions);
  const { trackPoints, trackWidth, trackSize } = trackData;
  const referenceSteering = pidSteering(trackPoints);
  const track = new Track({ trackPoints });
  let lastOnlineState;

  function collectPidEpisode(length) {
    const { trackPoints, car } = trackData;
    const networkInputs = [];
    const referenceOutputs = [];
    const episodeVisualization = [];

    function buildSteering() {
      const collector = (inputs, reward) => {
        networkInputs.push(inputs);
        return [0, 0];
      };

      const nnSteering = reinforcementLearningSteering(trackPoints, trackWidth, trackSize, { agent: collector });

      function wrapper(car, dt) {
        episodeVisualization.push([car.pos[0], car.pos[1], 0]);
        const ret = referenceSteering(car, dt);
        const { throttle, wheelTurnSpeed } = ret;
        referenceOutputs.push([throttle, wheelTurnSpeed]);
        nnSteering(car, dt);
        return ret;
      }

      return wrapper;
    }

    track.cars = [];
    track.addCar(buildSteering());
    const mycar = track.cars[0].model;
    if (lastOnlineState) {
      Object.entries(lastOnlineState).forEach(([key, value]) => {
        mycar[key] = value;
      });
      lastOnlineState = null;
    } else {
      const margin = 0.95;
      const rnd = () => (Math.random() - 0.5) * 2.0;
      mycar.pos = [
          0.5 * rnd() * trackSize.width * margin,
          0.5 * rnd() * trackSize.height * margin
      ];

      mycar.rot = Math.random() * Math.PI * 2.0;
      mycar.v = math.multiply(mycar.getForwardDir(), Math.random() * 15);
    }

    for (let i = 0; i < length; ++i) track.move();
    return { networkInputs, referenceOutputs, episodeVisualization };
  }

  function save(model, visus) {
    return model.save({
      save(data) {
        saveCallback({
          model: data,
          visualizations: {
            current: visus,
            best: visus
          }
        });
      }
    });
  }

  const optimizer = tf.train.adam(policyNetwork.learningRate);

  let epochIdx = 0;

  let allInput = [];
  let allOutput = [];

  function trainRound() {
    if (epochIdx > 50) {
      return;
    }

    console.log(`epoch ${epochIdx}`);
    allInput = stride(2, false)(allInput);
    allOutput = stride(2, false)(allOutput);

    const visualizationTracks = [];
    const EPISODES_PER_ROUND = 10;
    for (let i = 0; i < EPISODES_PER_ROUND; ++i) {
      const { networkInputs, referenceOutputs, episodeVisualization } = collectPidEpisode(400);
      console.log(" - round collected");
      const TRAIN_STRIDE = 3;
      for (let i = 0; i < networkInputs.length; i += TRAIN_STRIDE) {
        allInput.push(networkInputs[i]);
        allOutput.push(referenceOutputs[i]);
      }
      visualizationTracks.push(episodeVisualization);
    }

    //const [inShuffled, outShuffled] = shuffleTwoArrays(allInput, allOutput);

    const model = policyNetwork.policyNet;
      //console.log({allInput, allOutput});

    const ds = tf.data.zip({
      xs: tf.data.array(allInput),
      ys: tf.data.array(allOutput)
    }).shuffle(allInput.length).batch(32);

    ds.forEachAsync(({ xs, ys }) => {
      return optimizer.minimize(() => {
        //console.log('ss', {xs, ys});
        const predYs = model.predict(xs);
        const loss = tf.losses.meanSquaredError(ys, predYs);
        //loss.data().then(l => console.log('Loss', l));
        return loss;
      });
    }).then(() => {
      console.log('epoch finished');
      save(model, visualizationTracks.slice(0));
      setTimeout(trainRound, 10);
      epochIdx++;
    });
  }

  this.setLastOnlineState = function (state) { lastOnlineState = state; };

  trainRound();

}

function OfflineTrainer(saveCallback, { networkOptions, trackData }) {
  const policyNetwork = new PolicyNetwork(networkOptions);

  this.visualTraces = [];
  console.log("offline training");

  let nEpisodes = 0;
  let lastReward = 0;
  let bestTotal = 0;
  function buildSteering(trackPoints, episodeTrack) {
    const agent = (inputs, reward) => {
      lastReward = reward;
      const { actions } = policyNetwork.push(inputs, reward);
      return actions;
    };

    const actualSteering = reinforcementLearningSteering(trackPoints, trackData.trackWidth, trackData.trackSize, { agent });
    function wrapper(car, dt) {
      const ret = actualSteering(car, dt);
      episodeTrack.push([car.pos[0], car.pos[1], lastReward]);
      return ret;
    }

    return wrapper;
  }

  let visualizationTracks = [], bestVisualizationTracks = [];
  let bestModelData;
  const track = new Track({ trackPoints: trackData.trackPoints });
  let episodeTrack = [];
  let lastTotalReward = 0;
  let badStreak = 0;
  let lastOnlineState = { pos: [0, 0] };

  this.setLastOnlineState = function (state) { lastOnlineState = state; };

  function trainEpisode() {
    nEpisodes++;
    const { trackPoints, car } = trackData;

    let totalReward = 0;
    const offTrackEvery = bestTotal > 0 ? 3 : 10;
    const startOffroad = (nEpisodes % offTrackEvery) == offTrackEvery - 1;

    track.cars = [];
    const steering = buildSteering(trackPoints, episodeTrack);
    track.addCar(steering);
    if (startOffroad) {
      Object.entries(lastOnlineState).forEach(([key, value]) => {
        track.cars[0].model[key] = value;
      });
    }
    policyNetwork.rollingAverageRewardNormalization = !startOffroad;

    episodeTrack.length = 0; // clear

    const maxSteps = 1000;
    let negativeRewardStreak = 0;
    let i;
    let endBonus = 0;
    for (i = 0; i < maxSteps; ++i) {
      track.move();
      totalReward += lastReward;

      if (lastReward < 0 && !startOffroad) {
        negativeRewardStreak++;
      } else {
        negativeRewardStreak = 0;
      }

      if (negativeRewardStreak > i/3) {
        break; // fail early if going badly
      }
      if (track.cars[0].model.collidedTurns > 50) {
        endBonus -= 10000;
        break;
      }
    }

    // reset on failure
    if (i < maxSteps) endBonus -= (maxSteps - i) * 10;
    const badPlace = i < 200;

    totalReward += endBonus;

    visualizationTracks.push(episodeTrack.slice(0));
    policyNetwork.endEpisode(endBonus);

    const delta = lastTotalReward - totalReward;
    console.log(`Episode ${nEpisodes}, ${i}/${maxSteps} steps, reward ${totalReward} (delta ${delta})`);
    lastTotalReward = totalReward;

    if (!startOffroad) {
      if (badPlace) {
        badStreak++;
      } else {
        badStreak = 0;
      }
    }

    if (!startOffroad && Math.abs(delta) < 1e-7 || badStreak > 10) {
      badStreak = 0;
      console.warn('training in a bad place, restoring best known model');
      tf.loadLayersModel({
        // load using a trivial tf-js LoadHandler interface
        load() { return Promise.resolve(bestModelData); }
      }).then(model => {
        policyNetwork.policyNet = model;
        if (bestTotal > 0) {
          policyNetwork.decreaseLearningRate(0.9);
          console.log(`decreased learning rate to ${policyNetwork.learningRate}`);
        }
      });
    }

    const wasBest = !startOffroad && (totalReward > bestTotal || !bestModelData);
    const curVisualizationTracks = visualizationTracks;
    visualizationTracks = [];
    policyNetwork.policyNet.save({
      save(data) {
        if (wasBest) {
          console.log(` *** new best model with reward ${totalReward} ***`);
          bestTotal = totalReward;
          bestModelData = data;
          bestVisualizationTracks = curVisualizationTracks;
        }
        saveCallback({
          model: bestModelData,
          visualizations: {
            current: curVisualizationTracks,
            best: bestVisualizationTracks
          }
        });
      }
    });

    setTimeout(trainEpisode, 10);
  }

  setTimeout(trainEpisode, 1);
}


let trainer;
onmessage = function (e) {
  const { initialize, car } = e.data;
  if (initialize) {
    trainer = new SupervisedTrainer(postMessage, initialize);
  } else if (car) {
    trainer.setLastOnlineState(car);
  }

};
