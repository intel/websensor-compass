// @license
// Copyright (c) 2018 Intel Corporation. All rights reserved.

// @ts-check
export class MotionController {
  constructor() {
    const state = {
      timestamp: 0,
      sequence: -1
    };

    state.accelerometerData = [0, 0, 0];
    state.gyroscopeData =[0, 0, 0];


    this.accelerometer = new class Accelerometer {
      get timestamp() { return state.timestamp; }
      get x() { return state.accelerometerData[0]; }
      get y() { return state.accelerometerData[1]; }
      get z() { return state.accelerometerData[2]; }
      constructor() { this.onreading = null; }
    }

    this.gyroscope = new class Gyroscope {
      get timestamp() { return state.timestamp; }
      get x() { return state.gyroscopeData[0]; }
      get y() { return state.gyroscopeData[1]; }
      get z() { return state.gyroscopeData[2]; }
      constructor() { this.onreading = null; }
    }

    // May throw
    const connect = async () => {
      const device = await navigator.bluetooth.requestDevice({
          filters: [
            { name: 'Zephyr' },
            { name: 'Arduino101' },
            { name: 'Intel Curie' }
          ],
          optionalServices: [0xFC00]
      });

      const server = await device.gatt.connect();
      const service = await server.getPrimaryService(0xFC00);
      const characteristic = await service.getCharacteristic(0xFC0A);

      characteristic.addEventListener('characteristicvaluechanged', oncharacteristicvaluechanged);
      characteristic.startNotifications();
    }

    const oncharacteristicvaluechanged = (event) => {
      const data = event.target.value;

      state.timestamp = performance.now();

      const multi = 2 ** (32 - 14); // 14 bit precision.

      const isAccel = data.getUint8(0);
      const values = [
        (data.getUint32(1) << 1) / multi,
        (data.getUint32(5) << 1) / multi,
        (data.getUint32(9) << 1) / multi
      ];

      if (isAccel) {
        state.accelerometerData = values;
      } else {
        state.gyroscopeData = values;
      }

      if (this.accelerometer.onreading) this.accelerometer.onreading();
      if (this.gyroscope.onreading) this.gyroscope.onreading();
    }

    Object.assign(this, { connect });
  }
}
