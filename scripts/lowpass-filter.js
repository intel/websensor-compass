// @license
// Copyright (c) 2018 Intel Corporation. All rights reserved.

// @ts-check
export class LowPassFilterData {
  constructor(reading, bias) {
    Object.assign(this, { x: reading.x, y: reading.y, z: reading.z });
    this.bias = bias;
  }

  update(reading) {
    this.x = this.x * this.bias + reading.x * (1 - this.bias);
    this.y = this.y * this.bias + reading.y * (1 - this.bias);
    this.z = this.z * this.bias + reading.z * (1 - this.bias);
  }
};