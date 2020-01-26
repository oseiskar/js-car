/**
 * @license
 * Copyright 2018 Google LLC. All Rights Reserved.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * =============================================================================
 */
// based on
// https://github.com/tensorflow/tfjs-examples/blob/b789b5e3cca2eb4d2314b6c4129f3588333131c3/cart-pole/index.js
// see Git commit history for modifications by oseiskar

 /* globals tf */

class PolicyNetwork {
  /**
   * Create the underlying model of this policy network.
   */
  constructor({ inputs, outputs = 1 } = {}) {
    const hiddenLayerSizes = [6, 4];
    this.learningRate = 0.07;
    this.discountRate = 0.998;
    this.nOutputs = 4;

    this.policyNet = tf.sequential();
    hiddenLayerSizes.forEach((hiddenLayerSize, i) => {
      this.policyNet.add(tf.layers.dense({
        units: hiddenLayerSize,
        activation: 'elu',
        // `inputShape` is required only for the first layer.
        inputShape: i === 0 ? [inputs] : undefined
      }));
    });
    // Logits for each possible action
    this.policyNet.add(tf.layers.dense({units: this.nOutputs}));

    this.allGradients = [];
    this.allRewards = [];

    this.curGradients = [];
    this.curRewards = [];

    this.optimizer = tf.train.adam(this.learningRate);
  }
  /**
   * Create the underlying model of this policy network.
   *
   * @param {number[]} inputVector Observed state
   * @param {number} lastReward Latest reward
   * @returns chosen action
   */
  push(inputVector, lastReward) {

    const batchEvery = 200;
    this.lastInputVector = inputVector;

    // For every step of the game, remember gradients of the policy
    // network's weights with respect to the probability of the action
    // choice that lead to the reward.
    const gradients = tf.tidy(() => {
      const inputTensor = tf.tensor2d([inputVector]);
      return this.getGradientsAndSaveActions(inputTensor).grads;
    });

    this.pushGradients(this.curGradients, gradients);
    const action = this.currentActions_[0];

    this.curRewards.push(lastReward);

    let trained = false;
    if (this.curRewards.length >= batchEvery) {
      //console.log("saving batch");

      this.pushGradients(this.allGradients, this.curGradients);
      this.allRewards.push(this.curRewards);

      this.curRewards = [];
      this.curGradients = [];
    }
    return { action, trained };
  }

  decreaseLearningRate(factor = 0.7) {
    this.learningRate *= factor;
    this.optimizer = tf.train.adam(this.learningRate);
  }

  endEpisode(endBonus = 0) {
    //console.log("saving batch");
    if (this.lastInputVector) {
      this.push(this.lastInputVector, endBonus);
    }

    this.pushGradients(this.allGradients, this.curGradients);
    this.allRewards.push(this.curRewards);

    this.curRewards = [];
    this.curGradients = [];

    console.log("training, episode end");
    tf.tidy(() => {
      const normalizedRewards =
          discountAndNormalizeRewards(this.allRewards, this.discountRate);
      this.optimizer.applyGradients(
          scaleAndAverageGradients(this.allGradients, normalizedRewards));
    });
    tf.dispose(this.allGradients);
    this.allGradients = [];
    this.allRewards = [];
  }

  getGradientsAndSaveActions(inputTensor) {
    const f = () => tf.tidy(() => {
      const [logits, actions] = this.getLogitsAndActions(inputTensor);
      this.currentActions_ = actions.dataSync();

      const labels = tf.oneHot(this.currentActions_, this.nOutputs);
      // see https://github.com/simoninithomas/Deep_reinforcement_learning_Course/blob/master/Policy%20Gradients/Doom/Doom%20REINFORCE%20Monte%20Carlo%20Policy%20gradients.ipynb
      return tf.losses.softmaxCrossEntropy(labels, logits).asScalar();
    });
    return tf.variableGrads(f);
  }

  getCurrentActions() {
    return this.currentActions_;
  }

  /**
   * Get policy-network logits and the action based on state-tensor inputs.
   *
   * @param {tf.Tensor} inputs A tf.Tensor instance of shape `[batchSize, nInputs]`.
   * @returns {[tf.Tensor, tf.Tensor]}
   *   1. The logits tensor, of shape `[batchSize, 1]`.
   *   2. The actions tensor, of shape `[batchSize, 1]`.
   */
  getLogitsAndActions(inputs) {
    return tf.tidy(() => {
      const logits = this.policyNet.predict(inputs);
      const action_probs = tf.softmax(logits);
      const actions = tf.multinomial(action_probs, 1, null, true);
      return [logits, actions];
    });
  }

  /**
   * Get actions based on a state-tensor input.
   *
   * @param {tf.Tensor} inputs A tf.Tensor instance of shape `[batchSize, nInputs]`.
   * @param {Float32Array} inputs The actions for the inputs, with length
   *   `batchSize`.
   */
  getActions(inputs) {
    return this.getLogitsAndActions(inputs)[1].dataSync();
  }

  getAction(input) {
    return tf.tidy(() => {
      return this.getActions(tf.tensor2d([input]))[0];
    });
  }

  /**
   * Push a new dictionary of gradients into records.
   *
   * @param {{[varName: string]: tf.Tensor[]}} record The record of variable
   *   gradient: a map from variable name to the Array of gradient values for
   *   the variable.
   * @param {{[varName: string]: tf.Tensor}} gradients The new gradients to push
   *   into `record`: a map from variable name to the gradient Tensor.
   */
  pushGradients(record, gradients) {
    for (const key in gradients) {
      if (key in record) {
        record[key].push(gradients[key]);
      } else {
        record[key] = [gradients[key]];
      }
    }
  }
}

/**
 * Discount the reward values.
 *
 * @param {number[]} rewards The reward values to be discounted.
 * @param {number} discountRate Discount rate: a number between 0 and 1, e.g.,
 *   0.95.
 * @returns {tf.Tensor} The discounted reward values as a 1D tf.Tensor.
 */
function discountRewards(rewards, discountRate) {
  const discountedBuffer = tf.buffer([rewards.length]);
  let prev = 0;
  for (let i = rewards.length - 1; i >= 0; --i) {
    const current = discountRate * prev + rewards[i];
    discountedBuffer.set(current, i);
    prev = current;
  }
  return discountedBuffer.toTensor();
}

/**
 * Discount and normalize reward values.
 *
 * This function performs two steps:
 *
 * 1. Discounts the reward values using `discountRate`.
 * 2. Normalize the reward values with the global reward mean and standard
 *    deviation.
 *
 * @param {number[][]} rewardSequences Sequences of reward values.
 * @param {number} discountRate Discount rate: a number between 0 and 1, e.g.,
 *   0.95.
 * @returns {tf.Tensor[]} The discounted and normalize reward values as an
 *   Array of tf.Tensor.
 */
function discountAndNormalizeRewards(rewardSequences, discountRate) {
  return tf.tidy(() => {
    const discounted = [];
    for (const sequence of rewardSequences) {
      discounted.push(discountRewards(sequence, discountRate));
    }
    // Compute the overall mean and stddev.
    const concatenated = tf.concat(discounted);
    const mean = tf.mean(concatenated);
    const std = tf.sqrt(tf.mean(tf.square(concatenated.sub(mean))));
    // Normalize the reward sequences using the mean and std.
    const normalized = discounted.map(rs => rs.sub(mean).div(std));
    return normalized;
  });
}

/**
 * Scale the gradient values using normalized reward values and compute average.
 *
 * The gradient values are scaled by the normalized reward values. Then they
 * are averaged across all games and all steps.
 *
 * @param {{[varName: string]: tf.Tensor[][]}} allGradients A map from variable
 *   name to all the gradient values for the variable across all games and all
 *   steps.
 * @param {tf.Tensor[]} normalizedRewards An Array of normalized reward values
 *   for all the games. Each element of the Array is a 1D tf.Tensor of which
 *   the length equals the number of steps in the game.
 * @returns {{[varName: string]: tf.Tensor}} Scaled and averaged gradients
 *   for the variables.
 */
function scaleAndAverageGradients(allGradients, normalizedRewards) {
  return tf.tidy(() => {
    const gradients = {};
    for (const varName in allGradients) {
      gradients[varName] = tf.tidy(() => {
        // Stack gradients together.
        const varGradients = allGradients[varName].map(
            varGameGradients => tf.stack(varGameGradients));
        // Expand dimensions of reward tensors to prepare for multiplication
        // with broadcasting.
        const expandedDims = [];
        for (let i = 0; i < varGradients[0].rank - 1; ++i) {
          expandedDims.push(1);
        }
        const reshapedNormalizedRewards = normalizedRewards.map(
            rs => rs.reshape(rs.shape.concat(expandedDims)));
        for (let g = 0; g < varGradients.length; ++g) {
          // This mul() call uses broadcasting.
          varGradients[g] = varGradients[g].mul(reshapedNormalizedRewards[g]);
        }
        // Concatenate the scaled gradients together, then average them across
        // all the steps of all the games.
        return tf.mean(tf.concat(varGradients, 0), 0);
      });
    }
    return gradients;
  });
}