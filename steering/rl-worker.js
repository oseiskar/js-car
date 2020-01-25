 /* globals PolicyNetwork, Track, reinforcementLearningSteering */
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
  function buildSteering(trackPoints, episodeTrack) {
    const agent = (inputs, reward) => {
      lastReward = reward;
      const { action } = policyNetwork.push(inputs, reward);
      return action;
    };

    const actualSteering = reinforcementLearningSteering(trackPoints, { agent });
    function wrapper(car, dt) {
      const ret = actualSteering(car, dt);
      episodeTrack.push([car.pos[0], car.pos[1], lastReward]);
      return ret;
    }

    return wrapper;
  }

  let visualizationTracks = [];
  function trainEpisode() {
    nEpisodes++;
    console.log(`Episode ${nEpisodes}`);
    const { trackPoints, car } = trackData;
    const episodeTrack = [];
    const track = new Track({ trackPoints });
    track.addCar(buildSteering(trackPoints, episodeTrack));

    const nSteps = 1000 + nEpisodes * 10;
    for (let i = 0; i < nSteps; ++i) {
      track.move();
    }

    visualizationTracks.push(episodeTrack);

    policyNetwork.policyNet.save({
      save(data) {
        saveCallback({ model: data, visualizations: visualizationTracks });
        visualizationTracks = [];
      }
    });

    policyNetwork.endEpisode();

    setTimeout(trainEpisode, 10);
  }

  setTimeout(trainEpisode, 1);
}


let trainer;
onmessage = function (e) {
  const initData = e.data.initialize;
  if (initData) {
    // console.log(initData);
    trainer = new OfflineTrainer(postMessage, initData);
  }
};
