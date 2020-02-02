 /* globals PolicyNetwork, Track, reinforcementLearningSteering, tf */
 importScripts(
  './policy-network.js',
  './ai-rl.js',
  './pid.js',
  '../math-helpers.js',
  '../physics/car.js',
  '../physics/track.js',
  '//cdnjs.cloudflare.com/ajax/libs/mathjs/5.4.2/math.min.js',
  '//cdn.jsdelivr.net/npm/@tensorflow/tfjs@1.5.1/dist/tf.min.js');

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

    const actualSteering = reinforcementLearningSteering(trackPoints, trackData.trackWidth, { agent });
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
    // console.log(initData);
    trainer = new OfflineTrainer(postMessage, initialize);
  } else if (car) {
    trainer.setLastOnlineState(car);
  }

};
