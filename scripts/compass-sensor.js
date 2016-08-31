// @license
// Copyright (c) 2018 Intel Corporation. All rights reserved.

// @ts-check
import { GeolocationSensor } from "./geolocation-sensor.js";
import { GeomagneticField } from "./geomagnetic-field.js";

import * as euler from "./euler.js";
import * as mat4 from "../node_modules/gl-matrix/src/gl-matrix/mat4.js";
import * as vec3 from "../node_modules/gl-matrix/src/gl-matrix/vec3.js";

export class CompassSensor {
  constructor() {
    this.alpha = 0;
    this.beta = 0;
    this.gamma = 0;

    const geolocation = new GeolocationSensor({ enableHighAccuracy: true });
    let geofield = null;
    geolocation.onreading = function() {
      let millis = (new Date()).getTime();
      geofield = new GeomagneticField(this.latitude, this.longitude, this.altitude, millis);
    };
    geolocation.start();

    function update(gravity, geomagnetic) {
      // The gravity vector points towards the earths core when mostly stationary.
      // The magnetic vector points to the north, but not necessarily horizontally
      // with the ground.

      const ground = vec3.normalize(vec3.create(), vec3.fromValues(gravity.x, gravity.y, gravity.z));
      const bField = vec3.normalize(vec3.create(), vec3.fromValues(geomagnetic.x, geomagnetic.y, geomagnetic.z));

      // The cross product between the gravity and magnetic
      // vector will point east on horizontal plane.
      const east = vec3.normalize(vec3.create(), vec3.cross(vec3.create(), ground, bField));

      // The cross product gravity vector and the east vector
      // will point north on horizontal plane.
      const north = vec3.normalize(vec3.create(), vec3.cross(vec3.create(), east, ground));

      const orientation = mat4.set(mat4.create(),
        north[0], north[1], north[2], 0,
        east[0], east[1], east[2], 0,
        ground[0], ground[1], ground[2], 0,
        0, 0, 0, 1
      );

      mat4.rotateZ(orientation, orientation, (geofield) ? geofield.declinationRad : 0);

      // FIXME: Some bug somewhere!
      mat4.rotateZ(orientation, orientation, - Math.PI / 2);

      const angles = euler.fromMat4(euler.create(), orientation);

      this.alpha = angles[0];
      this.beta = angles[1];
      this.gamma = angles[2];

      if (this.onreading) this.onreading();
    }

    Object.assign(this, { update });
  }
}