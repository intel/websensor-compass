// @license
// Copyright (c) 2018 Intel Corporation. All rights reserved.

// @ts-check
import * as mat4 from "../node_modules/gl-matrix/src/gl-matrix/mat4.js";
import * as vec3 from "../node_modules/gl-matrix/src/gl-matrix/vec3.js";

const createSensor = (state, key) => new class {
  constructor() { this.onreading = null; }
  start() {}
  stop() {}
  get timestamp() { return state.timestamp; }
  get x() { return state[key].x; }
  get y() { return state[key].y; }
  get z() { return state[key].z; }
  get quaternion() { return state[key].quaternion; }
  populateMatrix(mat) {
    mat4.fromQuat(mat, this.quaternion);
    mat4.rotate(mat, mat, Math.PI, [0, 0, 1]); 
  }
}

export class Thingy52Controller {
  constructor() {
    const state = {
      orientation: {}
    };

    const orientation = createSensor(state, "orientation");
    this.AbsoluteOrientationSensor = function() {
      return orientation;
    }


    const connect = async () => {
      const device = await navigator.bluetooth.requestDevice({
          filters: [{ services: ['ef680100-9b35-4933-9b10-52ffa9740042'] }],
          optionalServices: [
            "ef680200-9b35-4933-9b10-52ffa9740042", 
            "ef680300-9b35-4933-9b10-52ffa9740042",
            "ef680400-9b35-4933-9b10-52ffa9740042",
            "ef680500-9b35-4933-9b10-52ffa9740042"
          ]
      });

      const server = await device.gatt.connect();
      const service = await server.getPrimaryService('ef680400-9b35-4933-9b10-52ffa9740042');
      const quaternionCharacteristic = await service.getCharacteristic('ef680404-9b35-4933-9b10-52ffa9740042');

      quaternionCharacteristic.addEventListener('characteristicvaluechanged', onquaternionchanged);
      quaternionCharacteristic.startNotifications();
    }

    const onquaternionchanged = event => {
      const value = event.target.value;

      let w = value.getInt32(0, true) / (1 << 30);
      let x = value.getInt32(4, true) / (1 << 30);
      let y = value.getInt32(8, true) / (1 << 30);
      let z = value.getInt32(12, true)/ (1 << 30);
 
      let norm = Math.sqrt(x ** 2 + y ** 2 + z ** 2 + w ** 2);

      const quaternion = [y / norm, x / norm, - z / norm, - w / norm];

      state.orientation.quaternion = quaternion;

      if (orientation.onreading) orientation.onreading();
    }

    Object.assign(this, { connect });
  }
}