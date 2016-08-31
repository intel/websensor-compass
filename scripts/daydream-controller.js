/**
 * MIT License
 *
 * Copyright (c) 2017 Mr.doob
 *               2017 Intel Corporation, all rights reserved.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// @ts-check
import * as mat4 from "../node_modules/gl-matrix/src/gl-matrix/mat4.js";
import * as vec3 from "../node_modules/gl-matrix/src/gl-matrix/vec3.js";

const createSensor = (state, key) => new class {
  constructor() { this.onreading = null; }
  start() {}
  stop() {}
  get timestamp() { return state.timestamp / 10; }
  get x() { return state[key].x; }
  get y() { return state[key].y; }
  get z() { return state[key].z; }
  populateMatrix(mat) {
    // axis-angle stored as: unit vector * angle.

    let axis = vec3.create();
    let angle = Math.sqrt(this.x ** 2 + this.y ** 2 + this.z ** 2);

    if (angle > 0 ) {
      vec3.set(axis, this.z, - this.x, - this.y);
      vec3.scale(axis, axis, 1 / angle);

      mat4.fromRotation(mat, angle, axis);
    } else {
      mat = mat4.create();
    }
  }
}

export class DaydreamController {
  constructor() {
    const state = {
      timestamp: 0,
      sequence: -1,
      accelerometer: {},
      gyroscope: {},
      orientation: {}
    };

    const orientation = createSensor(state, "orientation");
    this.AbsoluteOrientationSensor = function() {
      return orientation;
    }

    const accelerometer = createSensor(state, "accelerometer");
    this.Accelerometer = function() {
      return accelerometer;
    }

    const gyroscope = createSensor(state, "gyroscope");
    this.Gyroscope = function() {
      return gyroscope;
    }

    const connect = async () => {
      const device = await navigator.bluetooth.requestDevice({
          filters: [ { name: 'Daydream controller' } ],
          optionalServices: [ 0xfe55 ]
      });

      const server = await device.gatt.connect();
      const service = await server.getPrimaryService(0xfe55);
      const characteristic = await service.getCharacteristic('00000001-1000-1000-8000-00805f9b34fb');

      characteristic.addEventListener('characteristicvaluechanged', oncharacteristicvaluechanged);
      characteristic.startNotifications();
    }

    // We have 12 bit values plus sign (last bit), so extend to 32.
    const signext = value => (value << 19) >> 19;

    const oncharacteristicvaluechanged = (event) => {
      const data = event.target.value;

      const timestamp = ((data.getUint8(0) & 0xFF) << 1 | (data.getUint8(1) & 0x80) >> 7);
      const sequence = (data.getUint8(1) & 0x7C) >> 2;

      // Same sequence, drop.
      if (sequence === state.sequence) return;

      state.sequence = sequence;
      state.timestamp += timestamp;

      // http://stackoverflow.com/questions/40730809/use-daydream-controller-on-hololens-or-outside-daydream/40753551#40753551

      state.isClickDown = (data.getUint8(18) & 0x1) > 0;
      state.isAppDown = (data.getUint8(18) & 0x4) > 0;
      state.isHomeDown = (data.getUint8(18) & 0x2) > 0;
      state.isVolumePlusDown = (data.getUint8(18) & 0x10) > 0;
      state.isVolumeMinusDown = (data.getUint8(18) & 0x8) > 0;

      let orientationData = [
        (data.getUint8(1) & 0x03) << 11 | (data.getUint8(2) & 0xFF) << 3 | (data.getUint8(3) & 0x80) >> 5,
        (data.getUint8(3) & 0x1F) << 8 | (data.getUint8(4) & 0xFF),
        (data.getUint8(5) & 0xFF) << 5 | (data.getUint8(6) & 0xF8) >> 3
      ]
      .map(signext)
      .map(value => value * 2 * Math.PI / 4095.0 /* 12 bit precision (2 ** 12) - 1 */);

      state.orientation.x = orientationData[0];
      state.orientation.y = orientationData[1];
      state.orientation.z = orientationData[2];

      let accelerometerData = [
        (data.getUint8(6) & 0x07) << 10 | (data.getUint8(7) & 0xFF) << 2 | (data.getUint8(8) & 0xC0) >> 6,
        (data.getUint8(8) & 0x3F) << 7 | (data.getUint8(9) & 0xFE) >>> 1,
        (data.getUint8(9) & 0x01) << 12 | (data.getUint8(10) & 0xFF) << 4 | (data.getUint8(11) & 0xF0) >> 4
      ]
      .map(signext)
      .map(value => value * 8 * 9.81 /* gravity on earth */ / 4095.0);

      state.accelerometer.x = accelerometerData[0];
      state.accelerometer.y = accelerometerData[1];
      state.accelerometer.z = accelerometerData[2];

      let gyroscopeData = [
        ((data.getUint8(11) & 0x0F) << 9 | (data.getUint8(12) & 0xFF) << 1 | (data.getUint8(13) & 0x80) >> 7),
        ((data.getUint8(13) & 0x7F) << 6 | (data.getUint8(14) & 0xFC) >> 2),
        ((data.getUint8(14) & 0x03) << 11 | (data.getUint8(15) & 0xFF) << 3 | (data.getUint8(16) & 0xE0) >> 5)
      ]
      .map(signext)
      .map(value => value * 2048 / 180 * Math.PI / 4095.0);

      state.gyroscope.x = gyroscopeData[0];
      state.gyroscope.y = gyroscopeData[1];
      state.gyroscope.z = gyroscopeData[2];

      state.xTouch = ((data.getUint8(16) & 0x1F) << 3 | (data.getUint8(17) & 0xE0) >> 5) / 255.0;
      state.yTouch = ((data.getUint8(17) & 0x1F) << 3 | (data.getUint8(18) & 0xE0) >> 5) / 255.0;

      if (orientation.onreading) orientation.onreading();
      if (accelerometer.onreading) accelerometer.onreading();
      if (gyroscope.onreading) gyroscope.onreading();
    }

    Object.assign(this, { connect });
  }
}