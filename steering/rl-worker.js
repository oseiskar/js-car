 /* globals tf, PolicyNetwork */
importScripts('./rl-agent.js', '//cdn.jsdelivr.net/npm/@tensorflow/tfjs@1.5.1/dist/tf.min.js');

//let policyNetwork;
onmessage = function (e) {
  if (e.data.initialize) {
    policyNetwork = new PolicyNetwork(e.data.initialize);
  }
  else {
    const { inputs, lastReward } = e.data;
    if (policyNetwork.push(inputs, lastReward)) {
      // trained
      policyNetwork.policyNet.save({
        save(data) {
          postMessage({ model: data });
        }
      });
    }

    // TODO: check if trained again and post message
  }
};
