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
    const hiddenLayerSizes = [6, 6, 6];
    this.learningRate = 0.02;
    this.discountRate = 0.99;
    this.nOutputs = 2;
    this.gaussianStdev = 0.3;
    this.rollingAverageRewardNormalization = true;

    this.policyNet = tf.sequential();
    hiddenLayerSizes.forEach((hiddenLayerSize, i) => {
      this.policyNet.add(tf.layers.dense({
        units: hiddenLayerSize,
        //activation: 'sigmoid',
        activation: 'elu',
        // `inputShape` is required only for the first layer.
        inputShape: i === 0 ? [inputs] : undefined
      }));
    });
    // Predicts the means of the Gaussian policy
    this.policyNet.add(tf.layers.dense({units: this.nOutputs }));

    this.allGradients = [];
    this.allRewards = [];

    this.curGradients = [];
    this.curRewards = [];
    this.nTurn = 0;

    this.optimizer = tf.train.adam(this.learningRate);
    this.nEpisodes = 0;
  }

  actionsAsArray(actions) {
    // convert FloatArray to regular array
    const arr = [];
    for (let i = 0; i < this.nOutputs; ++i) arr.push(actions[i]);
    return arr;
  }

  /**
   * Create the underlying model of this policy network.
   *
   * @param {number[]} inputVector Observed state
   * @param {number} lastReward Latest reward
   * @returns chosen action
   */
  push(inputVector, lastReward) {
    // For every step of the game, remember gradients of the policy
    // network's weights with respect to the probability of the action
    // choice that lead to the reward.
    const gradients = tf.tidy(() => {
      const inputTensor = tf.tensor2d([inputVector]);
      return this.getGradientsAndSaveActions(inputTensor).grads;
    });

    const actions = this.actionsAsArray(this.currentActions_);
    this.pushGradients(this.curGradients, gradients);
    if (this.nTurn > 0) {
      this.curRewards.push(lastReward);
    }

    //this.actionHistory = this.actionHistory || [];
    //this.actionHistory.push(actions);

    this.nTurn++;

    const trained = false;
    return { actions, trained };
  }

  decreaseLearningRate(factor = 0.7) {
    this.learningRate *= factor;
    this.optimizer = tf.train.adam(this.learningRate);
  }

  endEpisode(endBonus = 0) {
    this.curRewards.push(endBonus);

    this.nEpisodes++;

    this.pushGradients(this.allGradients, this.curGradients);
    this.allRewards.push(this.curRewards);

    this.curRewards = [];
    this.curGradients = [];

    console.log(`training, episode ${this.nEpisodes} end`);
    //console.log(this.actionHistory.map(a => a[0]));
    //console.log(this.actionHistory.map(a => a[0]).reduce((a,b) => a + b) / this.actionHistory.length);
    this.typicalRewards = tf.tidy(() => {
      const { normalized, typicalRewards } =
          discountAndNormalizeRewards(this.allRewards, this.discountRate, this.typicalRewards);
      //console.log({reward: normalized[0].dataSync(), typical: typicalRewards.dataSync() });
      //console.log({reward: normalized[0].dataSync() });
      this.optimizer.applyGradients(
          scaleAndAverageGradients(this.allGradients, normalized));
      return typicalRewards;
    });

    tf.dispose(this.allGradients);
    this.allGradients = [];
    this.allRewards = [];
    //this.actionHistory = [];
    this.nTurn = 0;
  }

  getGradientsAndSaveActions(inputTensor) {
    const f = () => tf.tidy(() => {
      const [means, actions] = this.getMeansAndActions(inputTensor);
      this.currentActions_ = actions.dataSync();
      const coeff = this.nOutputs * Math.pow(this.gaussianStdev, -2.0);
      //return tf.mul(tf.losses.meanSquaredError(means, actions), coeff).asScalar();
      return tf.sum(tf.mul(coeff, tf.squaredDifference(this.currentActions_, means))).asScalar();
    });
    return tf.variableGrads(f);
  }

  getCurrentActions() {
    return this.currentActions_;
  }

  getMeansAndActions(inputs) {
    return tf.tidy(() => {
      const means = this.policyNet.predict(inputs);
      const actions = tf.add(means, tf.mul(tf.randomNormal(means.shape), this.gaussianStdev));
      return [means, actions];
    });
  }

  /**
   * Get actions based on a state-tensor input.
   * If deterministic, skips the stochastic part in the policy and returns
   * the mean
   */
  getActions(inputs, deterministic = true) {
    return this.getMeansAndActions(inputs)[deterministic ? 0 : 1].dataSync();
  }

  getAction(input, deterministic = true) {
    return tf.tidy(() => {
      return this.actionsAsArray(this.getActions(tf.tensor2d([input])));
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
function discountAndNormalizeRewards(rewardSequences, discountRate, typicalRewards = null, bias = 0) {
  return tf.tidy(() => {
    const discounted = [];
    let normalized;
    if (this.rollingAverageRewardNormalization) {
      let mean;
      for (const sequence of rewardSequences) {
        const cur = tf.add(discountRewards(sequence, discountRate), bias);
        discounted.push(cur);
        if (mean) {
          mean = tf.add(mean, cur);
        } else {
          mean = cur;
        }
      }
      mean = tf.mul(mean, 1.0/rewardSequences.length);

      // Normalize the reward sequences using the mean and std.
      let normalized = discounted;
      if (typicalRewards) {
        normalized = discounted.map(rs => rs.sub(typicalRewards));
        const alpha = 0.1;
        typicalRewards = tf.add(tf.mul(typicalRewards, 1.0 - alpha), tf.mul(mean, alpha));
      } else {
        typicalRewards = mean;
      }

      const max = tf.maximum(1e-6, tf.max(tf.abs(normalized[0])).asScalar());
      normalized = normalized.map(n => n.div(max));
    } else {
      for (const sequence of rewardSequences) {
        discounted.push(discountRewards(sequence, discountRate));
      }
      // Compute the overall mean and stddev.
      const concatenated = tf.concat(discounted);
      const mean = tf.mean(concatenated);
      const std = tf.sqrt(tf.mean(tf.square(concatenated.sub(mean))));

      // Normalize the reward sequences using the mean and std.
      normalized = discounted.map(rs => rs.sub(mean).div(std));
    }
    return { normalized, typicalRewards };
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
