// @license
// Copyright (c) 2018 Intel Corporation. All rights reserved.

// @ts-check
export class GeolocationSensor {
  constructor(options) {
    this.options = options;
    this.watchId = null;
    this.onreading = null;
  }

  start() {
    this.watchId = navigator.geolocation.watchPosition(pos => {
      const coords = pos.coords;

      this.latitude = coords.latitude;
      this.longitude = coords.longitude;
      this.altitude = coords.altitude;
      this.speed = coords.speed;
      this.accuracy = coords.accuracy;
      this.altitudeAccuracy = coords.altitudeAccuracy;

      this.timestamp = pos.timestamp;

      if (this.onreading) this.onreading();
    })

  }

  stop() {
    if (this.watchId) {
      navigator.geolocation.clearWatch(this.watchId);
    }
  }
}