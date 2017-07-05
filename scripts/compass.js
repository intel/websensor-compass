/*
 * Marine Compass
 * http://github.com/richtr/Marine-Compass
 *
 * Copyright (c) 2012-2014, Rich Tibbett
 *               2016-2017, Kenneth Christiansen
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
  * Based on sample code from the Marine Compass for Android project:
  *
  * Author:    Pierre Hébert
  * License:   Apache License, Version 2.0 (see LICENSE file)
  * URLs:      http://www.pierrox.net/cmsms/applications/marine-compass.html
  */

class ZephyrController {
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

    const connect = async () => {
      try {
        const device = await navigator.bluetooth.requestDevice({
            filters: [
              { name: 'Zephyr' },
              { name: 'Arduino101' },
              { name: 'Geoff101' },
              { name: 'Geoff' },
              { name: 'Intel Curie' }
            ],
            optionalServices: [0xFC00]
        });

        const server = await device.gatt.connect();
        const service = await server.getPrimaryService(0xFC00);
        const characteristic = await service.getCharacteristic(0xFC0A);

        characteristic.addEventListener('characteristicvaluechanged', oncharacteristicvaluechanged);
        characteristic.startNotifications();
      } catch(err) {
        console.log(err);
      }
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

      //if (!isAccel) console.log(values);

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

/**
 * DayDreamController:
 * @author mrdoob / http://mrdoob.com/ and Kenneth Christiansen
 */

class DaydreamController {
  constructor() {
    const state = {
      timestamp: 0,
      sequence: -1
    };

    this.orientation = new class OrientationSensor {
      get timestamp() { return state.timestamp / 10; }
      get x() { return state.orientationData[0]; }
      get y() { return state.orientationData[1]; }
      get z() { return state.orientationData[2]; }
      constructor() { this.onreading = null; }
    }

    this.accelerometer = new class Accelerometer {
      get timestamp() { return state.timestamp / 10; }
      get x() { return state.accelerometerData[0]; }
      get y() { return state.accelerometerData[1]; }
      get z() { return state.accelerometerData[2]; }
      constructor() { this.onreading = null; }
    }

    this.gyroscope = new class Gyroscope {
      get timestamp() { return state.timestamp / 10; }
      get x() { return state.gyroscopeData[0]; }
      get y() { return state.gyroscopeData[1]; }
      get z() { return state.gyroscopeData[2]; }
      constructor() { this.onreading = null; }
    }

    const connect = async () => {
      try {
        const device = await navigator.bluetooth.requestDevice({
            filters: [ { name: 'Daydream controller' } ],
            optionalServices: [ '0000fe55-0000-1000-8000-00805f9b34fb' ]
        });

        const server = await device.gatt.connect();
        const service = await server.getPrimaryService('0000fe55-0000-1000-8000-00805f9b34fb');
        const characteristic = await service.getCharacteristic('00000001-1000-1000-8000-00805f9b34fb');

        characteristic.addEventListener('characteristicvaluechanged', oncharacteristicvaluechanged);
        characteristic.startNotifications();
      } catch(err) {
        console.log(err);
      }
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

      state.orientationData = [
        (data.getUint8(1) & 0x03) << 11 | (data.getUint8(2) & 0xFF) << 3 | (data.getUint8(3) & 0x80) >> 5,
        (data.getUint8(3) & 0x1F) << 8 | (data.getUint8(4) & 0xFF),
        (data.getUint8(5) & 0xFF) << 5 | (data.getUint8(6) & 0xF8) >> 3
      ]
      .map(signext)
      .map(value => value * 2 * Math.PI / 4095.0 /* 12 bit precision (2 ** 12) - 1 */);

      state.accelerometerData = [
        (data.getUint8(6) & 0x07) << 10 | (data.getUint8(7) & 0xFF) << 2 | (data.getUint8(8) & 0xC0) >> 6,
        (data.getUint8(8) & 0x3F) << 7 | (data.getUint8(9) & 0xFE) >>> 1,
        (data.getUint8(9) & 0x01) << 12 | (data.getUint8(10) & 0xFF) << 4 | (data.getUint8(11) & 0xF0) >> 4
      ]
      .map(signext)
      .map(value => value * 8 * 9.81 /* gravity on earth */ / 4095.0);

      state.gyroscopeData = [
        ((data.getUint8(11) & 0x0F) << 9 | (data.getUint8(12) & 0xFF) << 1 | (data.getUint8(13) & 0x80) >> 7),
        ((data.getUint8(13) & 0x7F) << 6 | (data.getUint8(14) & 0xFC) >> 2),
        ((data.getUint8(14) & 0x03) << 11 | (data.getUint8(15) & 0xFF) << 3 | (data.getUint8(16) & 0xE0) >> 5)
      ]
      .map(signext)
      .map(value => value * 2048 / 180 * Math.PI / 4095.0);

      state.xTouch = ((data.getUint8(16) & 0x1F) << 3 | (data.getUint8(17) & 0xE0) >> 5) / 255.0;
      state.yTouch = ((data.getUint8(17) & 0x1F) << 3 | (data.getUint8(18) & 0xE0) >> 5) / 255.0;

      if (this.orientation.onreading) this.orientation.onreading();
      if (this.accelerometer.onreading) this.accelerometer.onreading();
      if (this.gyroscope.onreading) this.gyroscope.onreading();
    }

    Object.assign(this, { connect });
  }
}

var degToRad = Math.PI / 180;
var radToDeg = 180 / Math.PI;

class HighPassFilterData {
  constructor(reading, cutoffFrequency) {
    Object.assign(this, { x: reading.x, y: reading.y, z: reading.z });
    this.cutoff = cutoffFrequency;
    this.timestamp = reading.timestamp;
  }

  update(reading) {
    let dt = reading.timestamp - this.timestamp / 1000;
    this.timestamp = reading.timestamp;

    for (let i of ["x", "y", "z"]) {
      let alpha = this.cutoff / (this.cutoff + dt);
      this[i] = this[i] + alpha * (reading[i] - this[i]);
    }
  }
};

class LowPassFilterData {
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

class GeolocationSensor {
  constructor(options) {
    this.options = options;
    this.watchId = null;
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

class CompassSensor {
  constructor() {
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

/**
 * Utility class to compute a table of Gauss-normalized associated Legendre
 * functions P_n^m(cos(theta))
 */
class LegendreTable {
  /**
   * @param maxN
   *            The maximum n- and m-values to support
   * @param thetaRad
   *            Returned functions will be Gauss-normalized
   *            P_n^m(cos(thetaRad)), with thetaRad in radians.
   */
  constructor(maxN, thetaRad) {
    // These are the Gauss-normalized associated Legendre functions -- that
    // is, they are normal Legendre functions multiplied by
    // (n-m)!/(2n-1)!! (where (2n-1)!! = 1*3*5*...*2n-1)
    this.mP = [];

    // Derivative of mP, with respect to theta.
    this.mPDeriv = [];

    // Compute the table of Gauss-normalized associated Legendre
    // functions using standard recursion relations. Also compute the
    // table of derivatives using the derivative of the recursion
    // relations.
    const cos = Math.cos(thetaRad);
    const sin = Math.sin(thetaRad);

    this.mP = Array(maxN + 1);
    this.mPDeriv = Array(maxN + 1);

    this.mP[0] = [ 1.0 ];
    this.mPDeriv[0] = [ 0.0 ];

    for (let n = 1; n <= maxN; n++) {
      this.mP[n] = new Array(n + 1);
      this.mPDeriv[n] = new Array(n + 1);

      for (let m = 0; m <= n; m++) {
        if (n == m) {
          this.mP[n][m] = sin * this.mP[n - 1][m - 1];
          this.mPDeriv[n][m] = cos * this.mP[n - 1][m - 1] + sin * this.mPDeriv[n - 1][m - 1];
        } else if (n == 1 || m == n - 1) {
          this.mP[n][m] = cos * this.mP[n - 1][m];
          this.mPDeriv[n][m] = -sin * this.mP[n - 1][m] + cos * this.mPDeriv[n - 1][m];
        } else {
          const k = ((n - 1) * (n - 1) - m * m) / ((2 * n - 1) * (2 * n - 3));
          this.mP[n][m] = cos * this.mP[n - 1][m] - k * this.mP[n - 2][m];
          this.mPDeriv[n][m] = -sin * this.mP[n - 1][m] + cos * this.mPDeriv[n - 1][m] - k * this.mPDeriv[n - 2][m];
        }
      }
    }
  }
}

// field = new GeomagneticField(55.731381, 12.481383, 10, (new Date().getTime());

/**
 * Copyright (C) 2009 The Android Open Source Project
 *
 * Estimates magnetic field at a given point on
 * Earth, and in particular, to compute the magnetic declination from true
 * north.
 *
 * <p>This uses the World Magnetic Model produced by the United States National
 * Geospatial-Intelligence Agency.  More details about the model can be found at
 * <a href="http://www.ngdc.noaa.gov/geomag/WMM/DoDWMM.shtml">http://www.ngdc.noaa.gov/geomag/WMM/DoDWMM.shtml</a>.
 * This class currently uses WMM-2010 which is valid until 2015, but should
 * produce acceptable results for several years after that. Future versions of
 * Android may use a newer version of the model.
 */
class GeomagneticField {
 /**
   * Estimate the magnetic field at a given point and time.
   *
   * @param gdLatitudeDeg
   *            Latitude in WGS84 geodetic coordinates -- positive is east.
   * @param gdLongitudeDeg
   *            Longitude in WGS84 geodetic coordinates -- positive is north.
   * @param altitudeMeters
   *            Altitude in WGS84 geodetic coordinates, in meters.
   * @param timeMillis
   *            Time at which to evaluate the declination, in milliseconds
   *            since January 1, 1970. (approximate is fine -- the declination
   *            changes very slowly).
   */
  constructor(gdLatitudeDeg, gdLongitudeDeg, altitudeMeters, timeMillis) {
    // The magnetic field at a given point, in nonoteslas in geodetic
    // coordinates.
    this.mX;
    this.mY;
    this.mZ;

    // Geocentric coordinates -- set by computeGeocentricCoordinates.
    this.mGcLatitudeRad = null;
    this.mGcLongitudeRad = null;
    this.mGcRadiusKm = null;

    const EARTH_REFERENCE_RADIUS_KM = 6371.2;

    // These coefficients and the formulae used below are from:
    // NOAA Technical Report: The US/UK World Magnetic Model for 2010-2015
    const G_COEFF = [
      [ 0.0 ],
      [ -29496.6, -1586.3 ],
      [ -2396.6, 3026.1, 1668.6 ],
      [ 1340.1, -2326.2, 1231.9, 634.0 ],
      [ 912.6, 808.9, 166.7, -357.1, 89.4 ],
      [ -230.9, 357.2, 200.3, -141.1, -163.0, -7.8 ],
      [ 72.8, 68.6, 76.0, -141.4, -22.8, 13.2, -77.9 ],
      [ 80.5, -75.1, -4.7, 45.3, 13.9, 10.4, 1.7, 4.9 ],
      [ 24.4, 8.1, -14.5, -5.6, -19.3, 11.5, 10.9, -14.1, -3.7 ],
      [ 5.4, 9.4, 3.4, -5.2, 3.1, -12.4, -0.7, 8.4, -8.5, -10.1 ],
      [ -2.0, -6.3, 0.9, -1.1, -0.2, 2.5, -0.3, 2.2, 3.1, -1.0, -2.8 ],
      [ 3.0, -1.5, -2.1, 1.7, -0.5, 0.5, -0.8, 0.4, 1.8, 0.1, 0.7, 3.8 ],
      [ -2.2, -0.2, 0.3, 1.0, -0.6, 0.9, -0.1, 0.5, -0.4, -0.4, 0.2, -0.8, 0.0 ]
    ];

    const H_COEFF = [
        [ 0.0 ],
        [ 0.0, 4944.4 ],
        [ 0.0, -2707.7, -576.1 ],
        [ 0.0, -160.2, 251.9, -536.6 ],
        [ 0.0, 286.4, -211.2, 164.3, -309.1 ],
        [ 0.0, 44.6, 188.9, -118.2, 0.0, 100.9 ],
        [ 0.0, -20.8, 44.1, 61.5, -66.3, 3.1, 55.0 ],
        [ 0.0, -57.9, -21.1, 6.5, 24.9, 7.0, -27.7, -3.3 ],
        [ 0.0, 11.0, -20.0, 11.9, -17.4, 16.7, 7.0, -10.8, 1.7 ],
        [ 0.0, -20.5, 11.5, 12.8, -7.2, -7.4, 8.0, 2.1, -6.1, 7.0 ],
        [ 0.0, 2.8, -0.1, 4.7, 4.4, -7.2, -1.0, -3.9, -2.0, -2.0, -8.3 ],
        [ 0.0, 0.2, 1.7, -0.6, -1.8, 0.9, -0.4, -2.5, -1.3, -2.1, -1.9, -1.8 ],
        [ 0.0, -0.9, 0.3, 2.1, -2.5, 0.5, 0.6, 0.0, 0.1, 0.3, -0.9, -0.2, 0.9 ]
    ];

    const DELTA_G = [
        [ 0.0 ],
        [ 11.6, 16.5 ],
        [ -12.1, -4.4, 1.9 ],
        [ 0.4, -4.1, -2.9, -7.7 ],
        [ -1.8, 2.3, -8.7, 4.6, -2.1 ],
        [ -1.0, 0.6, -1.8, -1.0, 0.9, 1.0 ],
        [ -0.2, -0.2, -0.1, 2.0, -1.7, -0.3, 1.7 ],
        [ 0.1, -0.1, -0.6, 1.3, 0.4, 0.3, -0.7, 0.6 ],
        [ -0.1, 0.1, -0.6, 0.2, -0.2, 0.3, 0.3, -0.6, 0.2 ],
        [ 0.0, -0.1, 0.0, 0.3, -0.4, -0.3, 0.1, -0.1, -0.4, -0.2 ],
        [ 0.0, 0.0, -0.1, 0.2, 0.0, -0.1, -0.2, 0.0, -0.1, -0.2, -0.2 ],
        [ 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.1, 0.0 ],
        [ 0.0, 0.0, 0.1, 0.1, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.1, 0.1 ]
    ];

    const DELTA_H = [
        [ 0.0 ],
        [ 0.0, -25.9 ],
        [ 0.0, -22.5, -11.8 ],
        [ 0.0, 7.3, -3.9, -2.6 ],
        [ 0.0, 1.1, 2.7, 3.9, -0.8 ],
        [ 0.0, 0.4, 1.8, 1.2, 4.0, -0.6 ],
        [ 0.0, -0.2, -2.1, -0.4, -0.6, 0.5, 0.9 ],
        [ 0.0, 0.7, 0.3, -0.1, -0.1, -0.8, -0.3, 0.3 ],
        [ 0.0, -0.1, 0.2, 0.4, 0.4, 0.1, -0.1, 0.4, 0.3 ],
        [ 0.0, 0.0, -0.2, 0.0, -0.1, 0.1, 0.0, -0.2, 0.3, 0.2 ],
        [ 0.0, 0.1, -0.1, 0.0, -0.1, -0.1, 0.0, -0.1, -0.2, 0.0, -0.1 ],
        [ 0.0, 0.0, 0.1, 0.0, 0.1, 0.0, 0.1, 0.0, -0.1, -0.1, 0.0, -0.1 ],
        [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
    ];

    const BASE_TIME = (new Date(2010, 1, 1)).getTime();

    const MAX_N = G_COEFF.length; // Maximum degree of the coefficients.

    // The ratio between the Gauss-normalized associated Legendre functions and
    // the Schmid quasi-normalized ones. Compute these once staticly since they
    // don't depend on input variables at all.
    const SCHMIDT_QUASI_NORM_FACTORS = GeomagneticField.computeSchmidtQuasiNormFactors(MAX_N);

    // We don't handle the north and south poles correctly -- pretend that
    // we're not quite at them to avoid crashing.
    gdLatitudeDeg = Math.min(90.0 - 1e-5, Math.max(-90.0 + 1e-5, gdLatitudeDeg));

    this.computeGeocentricCoordinates(gdLatitudeDeg, gdLongitudeDeg, altitudeMeters);

    // Note: LegendreTable computes associated Legendre functions for
    // cos(theta).  We want the associated Legendre functions for
    // sin(latitude), which is the same as cos(PI/2 - latitude), except the
    // derivate will be negated.
    const legendre = new LegendreTable(MAX_N - 1, Math.PI / 2.0 - this.mGcLatitudeRad);

    // Compute a table of (EARTH_REFERENCE_RADIUS_KM / radius)^n for i in
    // 0..MAX_N-2 (this is much faster than calling Math.pow MAX_N+1 times).
    let relativeRadiusPower = new Array(MAX_N + 2);
    relativeRadiusPower[0] = 1.0;
    relativeRadiusPower[1] = EARTH_REFERENCE_RADIUS_KM / this.mGcRadiusKm;

    for (let i = 2; i < relativeRadiusPower.length; ++i) {
      relativeRadiusPower[i] = relativeRadiusPower[i - 1] * relativeRadiusPower[1];
    }

    // Compute tables of sin(lon * m) and cos(lon * m) for m = 0..MAX_N --
    // this is much faster than calling Math.sin and Math.com MAX_N+1 times.
    let sinMLon = new Array(MAX_N);
    let cosMLon = new Array(MAX_N);
    sinMLon[0] = 0.0;
    cosMLon[0] = 1.0;
    sinMLon[1] = Math.sin(this.mGcLongitudeRad);
    cosMLon[1] = Math.cos(this.mGcLongitudeRad);

    for (let m = 2; m < MAX_N; ++m) {
      // Standard expansions for sin((m-x)*theta + x*theta) and
      // cos((m-x)*theta + x*theta).
      let x = m >> 1;
      sinMLon[m] = sinMLon[m-x] * cosMLon[x] + cosMLon[m-x] * sinMLon[x];
      cosMLon[m] = cosMLon[m-x] * cosMLon[x] - sinMLon[m-x] * sinMLon[x];
    }

    let inverseCosLatitude = 1.0 / Math.cos(this.mGcLatitudeRad);
    let yearsSinceBase = (timeMillis - BASE_TIME) / (365 * 24 * 60 * 60 * 1000);

    // We now compute the magnetic field strength given the geocentric
    // location. The magnetic field is the derivative of the potential
    // function defined by the model. See NOAA Technical Report: The US/UK
    // World Magnetic Model for 2010-2015 for the derivation.
    let gcX = 0.0;  // Geocentric northwards component.
    let gcY = 0.0;  // Geocentric eastwards component.
    let gcZ = 0.0;  // Geocentric downwards component.

    for (let n = 1; n < MAX_N; n++) {
      for (let m = 0; m <= n; m++) {
        // Adjust the coefficients for the current date.
        let g = G_COEFF[n][m] + yearsSinceBase * DELTA_G[n][m];
        let h = H_COEFF[n][m] + yearsSinceBase * DELTA_H[n][m];
        // Negative derivative with respect to latitude, divided by
        // radius.  This looks like the negation of the version in the
        // NOAA Techincal report because that report used
        // P_n^m(sin(theta)) and we use P_n^m(cos(90 - theta)), so the
        // derivative with respect to theta is negated.
        gcX += relativeRadiusPower[n+2]
            * (g * cosMLon[m] + h * sinMLon[m])
            * legendre.mPDeriv[n][m]
            * SCHMIDT_QUASI_NORM_FACTORS[n][m];
        // Negative derivative with respect to longitude, divided by
        // radius.
        gcY += relativeRadiusPower[n+2] * m
            * (g * sinMLon[m] - h * cosMLon[m])
            * legendre.mP[n][m]
            * SCHMIDT_QUASI_NORM_FACTORS[n][m]
            * inverseCosLatitude;
        // Negative derivative with respect to radius.
        gcZ -= (n + 1) * relativeRadiusPower[n+2]
            * (g * cosMLon[m] + h * sinMLon[m])
            * legendre.mP[n][m]
            * SCHMIDT_QUASI_NORM_FACTORS[n][m];
      }
    }

    // Convert back to geodetic coordinates.  This is basically just a
    // rotation around the Y-axis by the difference in latitudes between the
    // geocentric frame and the geodetic frame.
    let latDiffRad = (gdLatitudeDeg * Math.PI / 180) - this.mGcLatitudeRad;
    this.mX = gcX * Math.cos(latDiffRad) + gcZ * Math.sin(latDiffRad);
    this.mY = gcY;
    this.mZ = - gcX * Math.sin(latDiffRad) + gcZ * Math.cos(latDiffRad);
  }

  /**
   * @return The X (northward) component of the magnetic field in nanoteslas.
   */
  get x() { return this.mX; }

  /**
   * @return The Y (eastward) component of the magnetic field in nanoteslas.
   */
  get y() { return this.mY; }

  /**
   * @return The Z (downward) component of the magnetic field in nanoteslas.
   */
  get z() { return this.mZ; }

  /**
   * @return The declination of the horizontal component of the magnetic
   *         field from true north, in radians (i.e. positive means the
   *         magnetic field is rotated east that much from true north).
   */
  get declinationRad() { return Math.atan2(this.mY, this.mX); }

  /**
   * @return The inclination of the magnetic field in radians -- positive
   *         means the magnetic field is rotated downwards.
   */
  get inclinationRad() { return Math.atan2(this.mZ, this.horizontalStrength); }

  /**
   * @return  Horizontal component of the field strength in nonoteslas.
   */
  get horizontalStrength() { return Math.hypot(this.mX, this.mY); }

  /**
   * @return  Total field strength in nanoteslas.
   */
  get fieldStrength() { return Math.sqrt(this.mX ** 2 + this.mY ** 2 + this.mZ ** 2); }

  /**
   * @param gdLatitudeDeg
   *            Latitude in WGS84 geodetic coordinates.
   * @param gdLongitudeDeg
   *            Longitude in WGS84 geodetic coordinates.
   * @param altitudeMeters
   *            Altitude above sea level in WGS84 geodetic coordinates.
   * @return Geocentric latitude (i.e. angle between closest point on the
   *         equator and this point, at the center of the earth.
   */
  computeGeocentricCoordinates(gdLatitudeDeg, gdLongitudeDeg, altitudeMeters) {
    // Constants from WGS84 (the coordinate system used by GPS)
    const EARTH_SEMI_MAJOR_AXIS_KM = 6378.137;
    const EARTH_SEMI_MINOR_AXIS_KM = 6356.7523142;

    let altitudeKm = altitudeMeters / 1000.0;
    let a2 = EARTH_SEMI_MAJOR_AXIS_KM * EARTH_SEMI_MAJOR_AXIS_KM;
    let b2 = EARTH_SEMI_MINOR_AXIS_KM * EARTH_SEMI_MINOR_AXIS_KM;
    let gdLatRad = gdLatitudeDeg * Math.PI / 180;
    let clat = Math.cos(gdLatRad);
    let slat = Math.sin(gdLatRad);
    let tlat = slat / clat;
    let latRad = Math.sqrt(a2 * clat * clat + b2 * slat * slat);

    this.mGcLatitudeRad = Math.atan(tlat * (latRad * altitudeKm + b2) / (latRad * altitudeKm + a2));
    this.mGcLongitudeRad = gdLongitudeDeg * Math.PI / 180;

    let radSq = altitudeKm * altitudeKm
        + 2 * altitudeKm * Math.sqrt(a2 * clat * clat + b2 * slat * slat)
        + (a2 * a2 * clat * clat + b2 * b2 * slat * slat)
        / (a2 * clat * clat + b2 * slat * slat);

    this.mGcRadiusKm = Math.sqrt(radSq);
  }

  /**
   * Compute the ration between the Gauss-normalized associated Legendre
   * functions and the Schmidt quasi-normalized version. This is equivalent to
   * sqrt((m==0?1:2)*(n-m)!/(n+m!))*(2n-1)!!/(n-m)!
   */
  static computeSchmidtQuasiNormFactors(maxN) {
    let schmidtQuasiNorm = new Array(maxN + 1);

    schmidtQuasiNorm[0] = [ 1.0 ];

    for (let n = 1; n <= maxN; n++) {
      schmidtQuasiNorm[n] = new Array(n + 1);
      schmidtQuasiNorm[n][0] = schmidtQuasiNorm[n - 1][0] * (2 * n - 1) / n;

      for (let m = 1; m <= n; m++) {
        schmidtQuasiNorm[n][m] = schmidtQuasiNorm[n][m - 1] * Math.sqrt((n - m + 1) * (m == 1 ? 2 : 1) / (n + m));
      }
    }
    return schmidtQuasiNorm;
  }
};


class KalmanFilter {
  constructor() {
    this.Q_angle = 0.01;
    this.Q_gyro = 0.0003;
    this.R_angle = 0.01;

    this.reset();
  }

  reset() {
    this.angle = 0.0;
    this.bias = 0;

    this.P00 = 0;
    this.P01 = 0;
    this.P10 = 0;
    this.P11 = 0;
  }

  filter(accAngle, gyroRate, dt) {
    this.angle += dt * (gyroRate - this.bias);

    this.P00 += -dt * (this.P10 + this.P01) + this.Q_angle * dt;
    this.P01 += -dt * this.P11;
    this.P10 += -dt * this.P11;
    this.P11 += + this.Q_gyro * dt;

    let axis = accAngle - this.angle;
    let S = this.P00 + this.R_angle;
    let K0 = this.P00 / S;
    let K1 = this.P10 / S;

    this.angle += K0 * axis;
    this.bias  += K1 * axis;

    this.P00 -= K0 * this.P00;
    this.P01 -= K0 * this.P01;
    this.P10 -= K1 * this.P00;
    this.P11 -= K1 * this.P01;

    return this.angle;
  }
};

const euler = {};

euler.create = function() {
    var out = new glMatrix.ARRAY_TYPE(3);
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    return out;
};

euler.clone = function(a) {
    var out = new glMatrix.ARRAY_TYPE(3);
    out[0] = a[0];
    out[1] = a[1];
    out[2] = a[2];
    return out;
};

euler.toMat4 = function(out, a) {
  a = a || {};

  const z = (a[0] || 0) * degToRad;
  const x = (a[1] || 0) * degToRad;
  const y = (a[2] || 0) * degToRad;

  const cX = Math.cos(x);
  const cY = Math.cos(y);
  const cZ = Math.cos(z);
  const sX = Math.sin(x);
  const sY = Math.sin(y);
  const sZ = Math.sin(z);

  mat4.set(out,
    cZ * cY - sZ * sX * sY, - cX * sZ, cY * sZ * sX + cZ * sY, 0,
    cY * sZ + cZ * sX * sY, cZ * cX, sZ * sY - cZ * cY * sX, 0,
    - cX * sY, sX, cX * cY, 0,
    0, 0, 0, 1
  );

  return out;
};

euler.fromMat4 = function(out, a) {
  if (a[10] > 0) { // cos(beta) > 0
    out[0] = Math.atan2(-a[1], a[5]);
    out[1]  = Math.asin(a[9]); // beta (-pi/2, pi/2)
    out[2] = Math.atan2(-a[8], a[10]); // gamma (-pi/2, pi/2)
  }
  else if (a[10] < 0) {  // cos(beta) < 0
    out[0] = Math.atan2(a[1], -a[5]);
    out[1]  = -Math.asin(a[9]);
    out[1]  += (out[1] >= 0) ? -Math.PI : Math.PI; // beta [-pi,-pi/2) U (pi/2,pi)
    out[2] = Math.atan2(a[8], -a[10]); // gamma (-pi/2, pi/2)
  }
  else { // a[10] (m33) == 0
    if (a[8] > 0) {  // cos(gamma) == 0, cos(beta) > 0
      out[0] = Math.atan2(-a[1], a[5]);
      out[1]  = Math.asin(a[9]); // beta [-pi/2, pi/2]
      out[2] = - (Math.PI / 2); // gamma = -pi/2
    }
    else if (a[8] < 0) { // cos(gamma) == 0, cos(beta) < 0
      out[0] = Math.atan2(a[1], -a[5]);
      out[1]  = -Math.asin(a[9]);
      out[1]  += (out[1] >= 0) ? - Math.PI : Math.PI; // beta [-pi,-pi/2) U (pi/2,pi)
      out[2] = - (Math.PI / 2); // gamma = -pi/2
    }
    else { // a[8] (m31) == 0, cos(beta) == 0
      // Gimbal lock discontinuity
      out[0] = Math.atan2(a[4], a[0]);
      out[1]  = (a[9] > 0) ? (Math.PI / 2) : - (Math.PI / 2); // beta = +-pi/2
      out[3] = 0; // gamma = 0
    }
  }

  // alpha is in [-pi, pi], make sure it is in [0, 2*pi).
  if (out[0] < 0) {
    out[0] += 2 * Math.PI; // alpha [0, 2*pi)
  }

  out[0] *= radToDeg;
  out[1] *= radToDeg;
  out[2] *= radToDeg;

  return out;
};


!(function(window, undefined) {
  var document = window.document;

  function toRad(val) {
      return val * Math.PI / 180;
  }
  function toDeg(val) {
      return val * 180 / Math.PI;
  }

  function isPowerOf2(value) {
    return (value & (value - 1)) == 0;
  };

  var compassVertexSource = [
  "attribute vec3 aVertexPosition;",
  "attribute vec2 aTextureCoord;",
  "",
  "uniform mat4 uMVMatrix;",
  "uniform mat4 uPMatrix;",
  "",
  "varying mediump vec2 vTextureCoord;",
  "",
  "void main(void) {",
  "  gl_Position = uPMatrix * uMVMatrix * vec4(aVertexPosition, 1.0);",
  "  vTextureCoord = aTextureCoord;",
  "}"
  ].join("\n");

  var compassFragmentSource = [
  "precision mediump float;",
  "",
  "varying vec2 vTextureCoord;",
  "",
  "uniform sampler2D uSampler;",
  "",
  "void main(void) {",
  "  vec4 textureColor = texture2D(uSampler, vTextureCoord);",
  "  gl_FragColor = textureColor;",
  "}"
  ].join("\n");

  var create3DContext = function(canvas, opt_attribs) {
    var names = ["webgl", "experimental-webgl", "webkit-3d", "moz-webgl"];
    var context = null;
    for (var ii = 0; ii < names.length; ii++) {
      try {
        context = canvas.getContext(names[ii], opt_attribs);
      } catch(e) {}
      if (context) {
        break;
      }
    }
    return context;
  };


  customElements.define('sensor-compass', class extends HTMLElement {
    constructor() {
      super();

      const template = document.querySelector('#sensor-compass');
      const clone = document.importNode(template.content, true);

      const shadowRoot = this.attachShadow({ mode: 'open' });
      shadowRoot.appendChild(clone);

      this.canvasEl = shadowRoot.querySelector('#glCanvas');
      this.headingEl = shadowRoot.querySelector('#headingReading');
      this.titleEl = shadowRoot.querySelector('#title');

      this.canvasEl.setAttribute('width', window.innerWidth);
      this.canvasEl.setAttribute('height', window.innerHeight);

      this.kalmanY = new KalmanFilter();
      this.kalmanX = new KalmanFilter();
      this.kalmanZ = new KalmanFilter();

      this.alpha = 0.0;
      this.beta = 0.0;
      this.gamma = 0.0;
      this.methodUsed = null;	//Variable to track which method of determining orientation we are using
      this.orientationMat = mat4.create();	//orientation matrix that will be populated by the sensor when using AbsoluteOrientationSensor, not used in other cases yet
      this.sensors = {};

      this.external = null;

      try {
        this.gl = create3DContext(this.canvasEl);
        if (!this.gl) {
          console.error('Unable to initialize WebGL. Your browser may not support it', 'http://get.webgl.org');
        } else {
          this.gl.viewportWidth = this.canvasEl.getAttribute('width');
          this.gl.viewportHeight = this.canvasEl.getAttribute('height');

          this._createViewport();
          this.start();
          this.render();
        }
      }
      catch(e) {
        console.error(e);
      }
    }

    set backend(value) {
      this.external = null;

      switch(value) {
        case 'zephyr':
          this.external = new ZephyrController();
          break;
        case 'daydream':
          this.external = new DaydreamController();
          break;
      }

      this.onRouteChanged();

      if (this.external) this.external.connect();
    }

    setTitle(value) {
      this.mCompassRenderer.setCompassFilter(value);
    }

    onRouteChanged() {
      let params = new URLSearchParams(new URL(window.location.href).search.slice(1));
      let filter = params.get("filter");

      for (let sensor of Object.values(this.sensors)) {
        if (!sensor) { continue; }
        sensor.onreading = null;

        // Not working properly:
        // if (sensor.state == "activating" || sensor.state == "active") {
        //   sensor.stop();
        // }
      }

      try {
        this.sensors.Accelerometer = null;
        this.sensors.Accelerometer = (this.external) ? this.external.accelerometer : new Accelerometer({ frequency: 50, includeGravity: true });
        this.sensors.Accelerometer.onerror = err => {
          this.sensors.Accelerometer = null;
          console.log(`Accelerometer ${err.error}`)
        };
      } catch(err) { }

      try {
        this.sensors.Gyroscope = null;
        this.sensors.Gyroscope = (this.external) ? this.external.gyroscope : new Gyroscope({ frequency: 50 });
        this.sensors.Gyroscope.onerror = err => {
          this.sensors.Gyroscope = null;
          console.log(`Gyroscope ${err.error}`)
        };
      } catch(err) { }

      try {
        this.sensors.Magnetometer = null;
        this.sensors.Magnetometer = new Magnetometer({ frequency: 50 });
        this.sensors.Magnetometer.onerror = err => {
          this.sensors.Magnetometer = null;
          console.log(`Magnetometer ${err.error}`)
        };
      } catch(err) { }

      try {
        this.sensors.AmbientLightSensor = null;
        this.sensors.AmbientLightSensor = new AmbientLightSensor({ frequency: 50 });
        this.sensors.AmbientLightSensor.onerror = err => {
          this.sensors.AmbientLightSensor = null;
          console.log(`AmbientLightSensor ${err.error}`)
        };
      } catch(err) { }

      try {
        this.sensors.AbsoluteOrientationSensor = null;
        this.sensors.AbsoluteOrientationSensor = new AbsoluteOrientationSensor({ frequency: 50 });
        this.sensors.AbsoluteOrientationSensor.onerror = err => {
          this.sensors.AbsoluteOrientationSensor = null;
          console.log(`AbsoluteOrientationSensor ${err.error}`)
        };
      } catch(err) { }

      this.alpha = 0.0;
      this.beta = 0.0;
      this.gamma = 0.0;

      switch (filter) {
        case "o":
          this.startAbsoluteOrientationDemo();
          break;
        case "l":
          this.startAmbientLightDemo();
          break;
        case "m":
          this.startMagnetometerDemo();
          break;
        case "k":
          this.startGyroscopeDemo(0);
          break;
        case "a":
        case "0":
          this.startAccelerometerDemo();
          break;
        case "g":
        case "1":
          this.startGyroscopeDemo(1);
          break;
        case "c":
          let num = parseFloat(filter);
          if (Number.isNaN(num))
            num = 0.98;
          let weight = Math.min(1, Math.max(0, num));
          this.startGyroscopeDemo(weight);
          break;
        default:
          this.startMagnetometerDemo();
      }
    }

    _startSensors(...requiredSensors) {
      for (let sensor of requiredSensors) {
        if (!this.sensors[sensor]) {
          return false;
        }
      }
      for (let sensor of requiredSensors) {
        if (this.sensors[sensor].activated == false) {
          this.sensors[sensor].start();
        }
      }
      return true;
    }

    startAmbientLightDemo() {
      this.setTitle("Ambient Light");
      this.methodUsed = "AmbientLight";
      if (!this._startSensors("AmbientLightSensor")) {
        console.error('Ambient light demo requires an ambient light sensor');
        return false;
      }

      function remap(value, inRangeStart, inRangeEnd, outRangeStart, outRangeEnd) {
        return outRangeStart + (outRangeEnd - outRangeStart) * ((value - inRangeStart) / (inRangeEnd - inRangeStart));
      };

      this.sensors.AmbientLightSensor.onreading = event => {
        let value = Math.min(Math.max(remap(this.sensors.AmbientLightSensor.illuminance, 0, 100, 0, 100), 0), 100);
        this.canvasEl.style = `filter: grayscale(${value}%)`;
      }

      return true;
    }

    startAbsoluteOrientationDemo() {
      this.setTitle("AbsoluteOrientation");
      this.methodUsed = "AbsoluteOrientation";
      if (!this._startSensors("AbsoluteOrientationSensor")) {
        console.error('AbsoluteOrientationSensor demo requires an accelerometer, magnetometer and gyroscope sensors');
        return false;
      }
      this.sensors.AbsoluteOrientationSensor.onreading = event => {
        this.sensors.AbsoluteOrientationSensor.populateMatrix(this.orientationMat);
      };

      return true;
    }

    startAccelerometerDemo() {
      this.setTitle("Accelerometer");
      this.methodUsed = "Accelerometer";
      if (!this._startSensors("Accelerometer")) {
        console.error('Accelerometer demo requires an accelerometer sensor');
        return false;
      }

      this.sensors.Accelerometer.onreading = event => {
        let xAccel = this.sensors.Accelerometer.y;
        let yAccel = this.sensors.Accelerometer.x;
        let zAccel = this.sensors.Accelerometer.z;

        // Treat the acceleration vector as an orientation vector by normalizing it.
        // Keep in mind that the if the device is flipped, the vector will just be
        // pointing in the other direction, so we have no way to know from the
        // accelerometer data what way the device is oriented.
        let norm = Math.sqrt(xAccel ** 2 + yAccel ** 2 + zAccel ** 2);

        // As we only can cover half of the spectrum we multiply the unit vector
        // with 90 so that it coveres the -90 to 90 degrees (180 degrees in total).
        this.beta = (xAccel / norm) * 90;
        this.gamma = (yAccel / norm) * - 90;
        this.alpha = 0;
      };

      return true;
    }

    start() {
      this.onRouteChanged();
    }

    startGyroscopeDemo(weight = 1) {
      if (weight == 1) {
        this.setTitle("Gyroscope");
        this.methodUsed = "Gyroscope";
        if (!this._startSensors("Gyroscope")) {
          console.error('The Gyroscope demo requires a gyroscope sensor');
          return false;
        }
      }

      else if (weight <= 0) {
        this.setTitle("Kalman filter");
        this.methodUsed = "KalmanFilter";
        if (!this._startSensors("Gyroscope", "Accelerometer")) {
          console.error('The Kalman filter demo requires both gyroscope and accelerometer sensors');
          return false;
        }
      }

      else if (weight > 0 && weight < 1) {
        this.setTitle(`Complementary (${weight}) filter`);
        this.methodUsed = "ComplementaryFilter";
        if (!this._startSensors("Gyroscope", "Accelerometer")) {
          console.error('The complementary filter demo requires both gyroscope and accelerometer sensors');
          return false;
        }
      }

      else {
        throw new Error;
      }

      this.sensors.Gyroscope.onreading = event => {
        let xAccel = 0;
        let yAccel = 0;
        let zAccel = 0;

        if (weight != 1) {
          xAccel = this.sensors.Accelerometer.y;
          yAccel = this.sensors.Accelerometer.x;
          zAccel = this.sensors.Accelerometer.z;

          // Treat the acceleration vector as an orientation vector by normalizing it.
          // Keep in mind that the if the device is flipped, the vector will just be
          // pointing in the other direction, so we have no way to know from the
          // accelerometer data what way the device is oriented.
          let norm = Math.sqrt(xAccel ** 2 + yAccel** 2 + zAccel ** 2);

          // As we only can cover half of the spectrum we multiply the unit vector
         // with 90 so that it coveres the -90 to 90 degrees (180 degrees in total).
          xAccel = (xAccel / norm) * 90;
          yAccel = (yAccel / norm) * - 90;
          zAccel = 0;
        }

        //console.log(this.sensors.Gyroscope);

        let xGyro = this.sensors.Gyroscope.x * 180 / Math.PI;
        let yGyro = this.sensors.Gyroscope.y * 180 / Math.PI;
        let zGyro = this.sensors.Gyroscope.z * 180 / Math.PI;

        let dt = 0;
        if (this.timestamp) {
          dt = (this.sensors.Gyroscope.timestamp - this.timestamp) / 1000;
        }
        this.timestamp = this.sensors.Gyroscope.timestamp;

        // Kalmar filter
        if (weight <= 0) {
          this.alpha = this.kalmanZ.filter(zAccel, zGyro, dt);
          this.beta = this.kalmanX.filter(xAccel, xGyro, dt);
          this.gamma = this.kalmanY.filter(yAccel, yGyro, dt);
        }

        // Complementary filter
        else {
          // E.g.: Current angle = 98% * (current angle + gyro rotation rate) + (2% * Accelerometer angle)
          this.alpha = (this.alpha + zGyro * dt);
          this.beta = weight * (this.beta + xGyro * dt) + (1.0 - weight) * xAccel;
          this.gamma = weight * (this.gamma + yGyro * dt) + (1.0 - weight) * yAccel;
        }
      };

      return true;
    }

    startMagnetometerDemo() {
      this.setTitle("Magnetometer");
      this.methodUsed = "Magnetometer";
      if (!this._startSensors("Magnetometer", "Accelerometer")) {
        console.error('Magnetometer demo requires magnetometer and accelerometer sensors');
        return false;
      }

      // Isolate gravity with low pass filter.
      const gravity = new LowPassFilterData(this.sensors.Accelerometer, 0.8);
      const compass = new CompassSensor();

      this.sensors.Magnetometer.onreading = event => {
        gravity.update(this.sensors.Accelerometer);
        compass.update(gravity, this.sensors.Magnetometer);

        this.alpha = compass.alpha;
        this.beta = compass.beta;
        this.gamma = compass.gamma;
      };
    }

    _createViewport() {
      // Create rotation matrix object (calculated per canvas draw)
      this.rotationMatrix = mat4.create();
      this.rotationMatrix = mat4.identity(mat4.create());

      // Create screen transform matrix (calculated once)
      this.screenMatrix = mat4.identity(mat4.create());

      var inv = toRad(180);

      this.screenMatrix[0] =   Math.cos(inv);
      this.screenMatrix[1] =   Math.sin(inv);
      this.screenMatrix[4] = - Math.sin(inv);
      this.screenMatrix[5] =   Math.cos(inv);

      // Create world transformation matrix (calculated once)
      this.worldMatrix = mat4.identity(mat4.create());

      var up = toRad(90);

      this.worldMatrix[5]  =   Math.cos(up);
      this.worldMatrix[6]  =   Math.sin(up);
      this.worldMatrix[9]  = - Math.sin(up);
      this.worldMatrix[10] =   Math.cos(up);

      // CompassRenderer manages 3D objects and gl surface life cycle
      this.mCompassRenderer = new CompassRenderer(this);

      // Catch window resize event
      window.addEventListener('orientationchange', _ => {
        window.setTimeout(() => {
          this.gl.viewportWidth = this.canvasEl.width = window.innerWidth;
          this.gl.viewportHeight = this.canvasEl.height = window.innerHeight;

          // Rescale webgl viewport
          this.gl.viewport(0, 0, this.gl.viewportWidth, this.gl.viewportHeight);

          // Recalculate perspective
          this.mCompassRenderer.pMatrix = mat4.identity(mat4.create());
          mat4.perspective(this.mCompassRenderer.pMatrix, 45, this.gl.viewportWidth / this.gl.viewportHeight, 1, 100);
          this.gl.uniformMatrix4fv(
            this.gl.getUniformLocation(this.mCompassRenderer.shaderProgram, "uPMatrix"),
            false, this.mCompassRenderer.pMatrix
          );
        }, 200);
      }, true);

      this.gl.clearDepth(500);
      this.gl.viewport(0, 0, this.gl.viewportWidth, this.gl.viewportHeight);
    }

    checkGLError() {
      var error = this.gl.getError();
      if (error != this.gl.NO_ERROR && error != this.gl.CONTEXT_LOST_WEBGL) {
        var str = "GL Error: " + error;
        console.error(str);
        throw str;
      }
    }

    calculateRotationMatrix() {
      this.rotationMatrix = mat4.identity(mat4.create());
      let compassOrientation = null;

      // Apply screen orientation.
      mat4.rotateZ(this.rotationMatrix, this.rotationMatrix, (window.screen.orientation.angle || 0) * degToRad);

      // Apply compass orientation.
      /*
       * When using method AbsoluteOrientation, use rotation matrix directly. Otherwise use angles.
      */
      switch(this.methodUsed) {
        case "AbsoluteOrientation":	//In this case, use the new backend
          compassOrientation = this.orientationMat;
          break;
        default:
          compassOrientation = euler.toMat4(mat4.create(), [this.alpha, this.beta, this.gamma]);
          break;
      }
      mat4.multiply(this.rotationMatrix, this.rotationMatrix, compassOrientation);

      const q = mat4.getRotation(quat.create(), this.rotationMatrix);
      const axis = quat.getAxisAngle(vec3.fromValues(0, 0, 1), q) * radToDeg;

      // Invert compass heading.
      mat4.multiply(this.rotationMatrix, this.rotationMatrix, this.screenMatrix);

      // Apply world orientation (heads-up display).
      mat4.multiply(this.rotationMatrix, this.rotationMatrix, this.worldMatrix);

      this.mCompassRenderer.setRotationMatrix(this.rotationMatrix);
      this.mCompassRenderer.setCompassHeading(Math.floor(axis));
    }

    render() {
      // Update orientation buffer
      this.calculateRotationMatrix();

      // Draw frame
      this.mCompassRenderer.draw();

      // Re-render at next key frame
      // see: http://stackoverflow.com/questions/6065169/requestanimationframe-with-this-keyword
      window.requestAnimationFrame(this.render.bind(this));
    }
  });

  // +++ COMPASSRENDERER +++
  class CompassRenderer {
    constructor(compass) {
      this.compass = compass;
      this.gl = this.compass.gl;

      this.pMatrix = mat4.create();
      this.mvMatrix = mat4.create();
      this.rotationMatrix = mat4.create();

      this.heading = 0;
      this.lastCompassHeading = 0;

      this.init();

      this.mTurntable = new Turntable(this);
    }

    loadShader(type, shaderSrc) {
      var shader = this.gl.createShader(type);
      // Load the shader source
      this.gl.shaderSource(shader, shaderSrc);
      // Compile the shader
      this.gl.compileShader(shader);
      // Check the compile status
      if (!this.gl.getShaderParameter(shader, this.gl.COMPILE_STATUS) &&
        !this.gl.isContextLost()) {
          var infoLog = this.gl.getShaderInfoLog(shader);
          console.error("Error compiling shader:\n" + infoLog);
          this.gl.deleteShader(shader);
          return null;
      }
      return shader;
    }

    init() {
      // initialize
      var vertexShader = this.loadShader(this.gl.VERTEX_SHADER, compassVertexSource);
      var fragmentShader = this.loadShader(this.gl.FRAGMENT_SHADER, compassFragmentSource);

      this.shaderProgram = this.gl.createProgram();
      this.gl.attachShader(this.shaderProgram, vertexShader);
      this.gl.attachShader(this.shaderProgram, fragmentShader);

      this.gl.linkProgram(this.shaderProgram);

      // Check the link status
      var linked = this.gl.getProgramParameter(this.shaderProgram, this.gl.LINK_STATUS);
      if (!linked && !this.gl.isContextLost()) {
        var infoLog = this.gl.getProgramInfoLog(this.shaderProgram);
        console.error("Error linking program:\n" + infoLog);
        this.gl.deleteProgram(this.shaderProgram);
        return;
      }

      this.gl.useProgram(this.shaderProgram);

      this.gl.textureCoordAttribute = this.gl.getAttribLocation(this.shaderProgram, "aTextureCoord");
      this.gl.vertexPositionAttribute = this.gl.getAttribLocation(this.shaderProgram, "aVertexPosition");

      this.gl.enableVertexAttribArray(this.gl.textureCoordAttribute);
      this.gl.enableVertexAttribArray(this.gl.vertexPositionAttribute);

      this.gl.enable(this.gl.DEPTH_TEST);

      // Set some uniform variables for the shaders
      this.shaderProgram.mvMatrixUniform = this.gl.getUniformLocation(this.shaderProgram, "uMVMatrix");
      this.shaderProgram.shaderUniform = this.gl.getUniformLocation(this.shaderProgram, "uSampler");

      // Calculate perspective
      this.pMatrix = mat4.identity(mat4.create());
      mat4.perspective(this.pMatrix, 45, this.gl.viewportWidth / this.gl.viewportHeight, 1, 100);
      this.gl.uniformMatrix4fv(this.gl.getUniformLocation(this.shaderProgram, "uPMatrix"), false, this.pMatrix);
    }

    setMatrixUniforms() {
      this.gl.uniformMatrix4fv(this.shaderProgram.mvMatrixUniform, false, this.mvMatrix);
    }

    setRotationMatrix(matrix) {
      this.rotationMatrix = matrix;
    }

    setCompassHeading(heading) {
      this.heading = heading;
    }

    setCompassFilter(filter) {
      this.filter = filter;
    }

    draw() {
      // Clear the canvas
      this.gl.clear(this.gl.COLOR_BUFFER_BIT | this.gl.DEPTH_BUFFER_BIT);

      // Reset move matrix
      this.mvMatrix = mat4.identity(mat4.create());
      mat4.translate(this.mvMatrix, this.mvMatrix, [ 0, 0, -4 ]);

      // Apply calculated device rotation matrix
      mat4.multiply(this.mvMatrix, this.mvMatrix, this.rotationMatrix);

      this.setMatrixUniforms();

      // Display compass heading
      var thisCompassHeading = this.heading;
      if (this.lastCompassHeading !== thisCompassHeading) {
        this.lastCompassHeading = thisCompassHeading;
        this.compass.headingEl.textContent = this.lastCompassHeading;
      }
      this.compass.titleEl.textContent = this.filter;

      this.mTurntable.draw();
    }
  };

  // +++ TURNTABLE +++
  class Turntable {
    constructor(compassrenderer) {
      this.compassrenderer = compassrenderer;
      this.gl = this.compassrenderer.compass.gl;

      this.DETAIL_X = [15, 25, 30];
      this.DETAIL_Y = [3, 6, 6];
      this.RING_HEIGHT = [2, 3, 3];

      this.TEXTURE_RING = 0;
      this.TEXTURE_DIAL = 1;
      this.TEXTURE_CAP = 2;

      this.CARDINAL_POINTS = ["N", "W", "S", "E"];

      this.mDetailsLevel = 1;
      this.mReversedRing = true;

      this.needsInitializing = true;
      this.ready = false;
    }

    buildObjects() {
      this.buildRingObject();
      this.buildCapObject();
      this.buildDialObject();
    }

    buildRingObject() {
      // build vertices
      var dx = this.DETAIL_X[this.mDetailsLevel];
      var dy = this.DETAIL_Y[this.mDetailsLevel];
      var rh = this.RING_HEIGHT[this.mDetailsLevel];

      var vertices = new Array(((dx + 1) * (rh + 1)) * 3);
      var normals = new Array(((dx + 1) * (rh + 1)) * 3);

      var n = 0;

      for (var i = 0; i <= dx; i++) {
        for (var j = 0; j <= rh; j++) {
          var a = i * 2 * Math.PI / dx;
          var b = j * Math.PI / (dy * 2);

          var x = Math.sin(a) * Math.cos(b);
          var y = -Math.sin(b);
          var z = Math.cos(a) * Math.cos(b);

          vertices[n] = x;
          vertices[n + 1] = y;
          vertices[n + 2] = z;
          normals[n] = vertices[n];
          normals[n + 1] = vertices[n + 1];
          normals[n + 2] = vertices[n + 2];

          n += 3;
        }
      }

      // build textures coordinates
      var texCoords = new Array((dx + 1) * (rh + 1) * 2);
      n = 0;
      for (var i = 0; i <= dx; i++) {
        for (var j = 0; j <= rh; j++) {
          texCoords[n++] = i / dx;
          texCoords[n++] = j / rh;
        }
      }

      // build indices
      var indices = new Array(dx * rh * 3 * 2);
      n = 0;
      for (var i = 0; i < dx; i++) {
        for (var j = 0; j < rh; j++) {
          var p0 = ((rh + 1) * i + j);
          indices[n++] = p0;
          indices[n++] = (p0 + rh + 1);
          indices[n++] = (p0 + 1);

          indices[n++] = (p0 + rh + 1);
          indices[n++] = (p0 + rh + 2);
          indices[n++] = (p0 + 1);
        }
      }

      // Bind buffers to WebGL renderer
      this.mRingVertexBufferGL = this.gl.createBuffer();
      this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.mRingVertexBufferGL);
      this.gl.bufferData(this.gl.ARRAY_BUFFER, new Float32Array(vertices), this.gl.STATIC_DRAW);
      this.mRingVertexBufferGL.itemSize = 3;
      this.mRingVertexBufferGL.numItems = vertices.length / 3;

      this.mRingNormalBufferGL = this.gl.createBuffer();
      this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.mRingNormalBufferGL);
      this.gl.bufferData(this.gl.ARRAY_BUFFER, new Float32Array(normals), this.gl.STATIC_DRAW);
      this.mRingNormalBufferGL.itemSize = 3;
      this.mRingNormalBufferGL.numItems = normals.length / 3;

      this.mRingTexCoordBufferGL = this.gl.createBuffer();
      this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.mRingTexCoordBufferGL);
      this.gl.bufferData(this.gl.ARRAY_BUFFER, new Float32Array(texCoords), this.gl.STATIC_DRAW);
      this.mRingTexCoordBufferGL.itemSize = 2;
      this.mRingTexCoordBufferGL.numItems = texCoords.length / 2;

      this.mRingIndexBufferGL = this.gl.createBuffer();
      this.gl.bindBuffer(this.gl.ELEMENT_ARRAY_BUFFER, this.mRingIndexBufferGL);
      this.gl.bufferData(this.gl.ELEMENT_ARRAY_BUFFER, new Uint16Array(indices), this.gl.STREAM_DRAW);
      this.mRingIndexBufferGL.itemSize = 1;
      this.mRingIndexBufferGL.numItems = indices.length;
    }

    buildCapObject() {
      var dx = this.DETAIL_X[this.mDetailsLevel];
      var dy = this.DETAIL_Y[this.mDetailsLevel];
      var rh = this.RING_HEIGHT[this.mDetailsLevel];

      var h = dy - rh;

      // build vertices
      var vertices = new Array(((dx + 1) * (h + 1)) * 3);

      var n = 0;
      for (var i = 0; i <= dx; i++) {
        for (var j = rh; j <= dy; j++) {
          var a = i * 2 * Math.PI / dx;
          var b = j * Math.PI / (dy * 2);

          var x = Math.sin(a) * Math.cos(b);
          var y = -Math.sin(b);
          var z = Math.cos(a) * Math.cos(b);

          vertices[n++] = x;
          vertices[n++] = y;
          vertices[n++] = z;
        }
      }

      // build indices
      var indices = new Array(dx * h * 3 * 2);
      n = 0;
      for (var i = 0; i < dx; i++) {
        for (var j = 0; j < h; j++) {
          var p0 = ((h + 1) * i + j);
          indices[n++] = p0;
          indices[n++] = (p0 + h + 1);
          indices[n++] = (p0 + 1);

          indices[n++] = (p0 + h + 1);
          indices[n++] = (p0 + h + 2);
          indices[n++] = (p0 + 1);
        }
      }

      // Create texture coordinates for the 1x1 dummy texture,
      // all of which points to the [0, 0] texel.
      let texCoords = [];
      for (let i = 0; i < vertices.length; i++) {
        texCoords.push(0);
      }

      this.mCapVertexBufferGL = this.gl.createBuffer();
      this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.mCapVertexBufferGL);
      this.gl.bufferData(this.gl.ARRAY_BUFFER, new Float32Array(vertices), this.gl.STATIC_DRAW);
      this.mCapVertexBufferGL.itemSize = 3;
      this.mCapVertexBufferGL.numItems = vertices.length / 3;

      this.mCapTexCoordBufferGL = this.gl.createBuffer();
      this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.mCapTexCoordBufferGL);
      this.gl.bufferData(this.gl.ARRAY_BUFFER, new Float32Array(texCoords), this.gl.STATIC_DRAW);
      this.mCapTexCoordBufferGL.itemSize = 2;
      this.mCapTexCoordBufferGL.numItems = texCoords.length / 2;

      this.mCapIndexBufferGL = this.gl.createBuffer();
      this.gl.bindBuffer(this.gl.ELEMENT_ARRAY_BUFFER, this.mCapIndexBufferGL);
      this.gl.bufferData(this.gl.ELEMENT_ARRAY_BUFFER, new Uint16Array(indices), this.gl.STREAM_DRAW);
      this.mCapIndexBufferGL.itemSize = 1;
      this.mCapIndexBufferGL.numItems = indices.length;
    }

    buildDialObject() {
      var dx = this.DETAIL_X[this.mDetailsLevel];

      var vertices = new Array((dx + 2) * 3);
      var normals = new Array((dx + 2) * 3);

      var n = 0;
      // center of the dial
      vertices[n] = 0;
      vertices[n + 1] = 0;
      vertices[n + 2] = 0;
      normals[n] = 0;
      normals[n + 1] = 1;
      normals[n + 2] = 0;
      n += 3;
      for (var i = 0; i <= dx; i++) {
        var a = i * 2 * Math.PI / dx;

        var x = Math.sin(a);
        var z = Math.cos(a);

        vertices[n] = x;
        vertices[n + 1] = 0;
        vertices[n + 2] = z;
        normals[n] = 0;
        normals[n + 1] = 1;
        normals[n + 2] = 0;
        n += 3;
      }

      // build textures coordinates
      var texCoords = new Array((dx + 2) * 2);
      n = 0;
      texCoords[n++] = 0.5;
      texCoords[n++] = 0.5;
      for (var i = 0; i <= dx; i++) {
        var a = i * 2 * Math.PI / dx;

        var x = (Math.sin(a) + 1) / 2;
        var z = (Math.cos(a) + 1) / 2;

        texCoords[n++] = x;
        texCoords[n++] = z;
      }

      // build indices
      var indices = new Array(dx + 2);
      n = 0;
      for (var i = 0; i <= (dx + 1); i++) {
        indices[n++] = i;
      }

      this.mDialVertexBufferGL = this.gl.createBuffer();
      this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.mDialVertexBufferGL);
      this.gl.bufferData(this.gl.ARRAY_BUFFER, new Float32Array(vertices), this.gl.STATIC_DRAW);
      this.mDialVertexBufferGL.itemSize = 3;
      this.mDialVertexBufferGL.numItems = vertices.length / 3;

      this.mDialNormalBufferGL = this.gl.createBuffer();
      this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.mDialNormalBufferGL);
      this.gl.bufferData(this.gl.ARRAY_BUFFER, new Float32Array(normals), this.gl.STATIC_DRAW);
      this.mDialNormalBufferGL.itemSize = 3;
      this.mDialNormalBufferGL.numItems = normals.length / 3;

      this.mDialTexCoordBufferGL = this.gl.createBuffer();
      this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.mDialTexCoordBufferGL);
      this.gl.bufferData(this.gl.ARRAY_BUFFER, new Float32Array(texCoords), this.gl.STATIC_DRAW);
      this.mDialTexCoordBufferGL.itemSize = 2;
      this.mDialTexCoordBufferGL.numItems = texCoords.length / 2;

      this.mDialIndexBufferGL = this.gl.createBuffer();
      this.gl.bindBuffer(this.gl.ELEMENT_ARRAY_BUFFER, this.mDialIndexBufferGL);
      this.gl.bufferData(this.gl.ELEMENT_ARRAY_BUFFER, new Uint16Array(indices), this.gl.STREAM_DRAW);
      this.mDialIndexBufferGL.itemSize = 1;
      this.mDialIndexBufferGL.numItems = indices.length;
    }

    buildImageTexture(entry, canvas) {
      return new Promise(resolve => {
        const image = document.createElement('img');
        image.onload = _ => {
          this.mTextures[entry] = this.gl.createTexture();

          this.gl.bindTexture(this.gl.TEXTURE_2D, this.mTextures[entry]);
          this.gl.texImage2D(this.gl.TEXTURE_2D, 0, this.gl.RGBA, this.gl.RGBA, this.gl.UNSIGNED_BYTE, image);

          this.gl.texParameteri(this.gl.TEXTURE_2D, this.gl.TEXTURE_WRAP_S, this.gl.CLAMP_TO_EDGE);
          this.gl.texParameteri(this.gl.TEXTURE_2D, this.gl.TEXTURE_WRAP_T, this.gl.CLAMP_TO_EDGE);
          // see: http://stackoverflow.com/a/19748905
          if (isPowerOf2(image.width) && isPowerOf2(image.height)) {
            // the dimensions are power of 2 so generate mips and turn on
            // tri-linear filtering.
            this.gl.generateMipmap(this.gl.TEXTURE_2D);
            this.gl.texParameteri(this.gl.TEXTURE_2D, this.gl.TEXTURE_MIN_FILTER, this.gl.LINEAR_MIPMAP_LINEAR);
          } else {
            // at least one of the dimensions is not a power of 2 so set the filtering
            // so WebGL will render it.
            this.gl.texParameteri(this.gl.TEXTURE_2D, this.gl.TEXTURE_MIN_FILTER, this.gl.LINEAR);
          }

          this.gl.bindTexture(this.gl.TEXTURE_2D, null);

          resolve(image);
        };

        image.src = canvas.toDataURL();
      });
    }

    async buildTextures() {
      this.mTextures = new Array(3);

      await Promise.all([
        this.buildRingTexture(),
        this.buildDialTexture(),
        this.buildCapTexture()
      ]);

      this.ready = true;
    }

    buildCapTexture() {
      this.mTextures[this.TEXTURE_CAP] = this.gl.createTexture();

      this.gl.bindTexture(this.gl.TEXTURE_2D, this.mTextures[this.TEXTURE_CAP]);
      this.gl.texImage2D(this.gl.TEXTURE_2D, 0, this.gl.RGBA, 1, 1, 0, this.gl.RGBA, this.gl.UNSIGNED_BYTE,
        new Uint8Array([0, 0, 0, 255])); // initialize as black 1x1 texture

      this.gl.bindTexture(this.gl.TEXTURE_2D, null);
      return Promise.resolve();
    }

    buildRingTexture() {
      var length = 512;
      var height = 64;

      var canvas = document.createElement('canvas');
      canvas.setAttribute('width', length);
      canvas.setAttribute('height', height);
      //document.body.appendChild(canvas); // debugging
      var context = canvas.getContext('2d');

      context.fillStyle = '#000000';
      context.fillRect(0, 0, length, height);

      // draw medium graduations in white
      context.strokeStyle = '#fff';
      context.lineWidth = 1;

      for (var d = 0; d < 360; d += 10) {
        var pos = d * length / 360;

        context.beginPath();
        context.moveTo(pos, 0);
        context.lineTo(pos, 20);
        context.closePath();

        context.stroke();
      }

      // draw major graduations in red
      context.strokeStyle = '#FF0000';
      context.lineWidth = 2;

      for (var d = 0; d < 360; d += 90) {
        var pos = d * length / 360;

        context.beginPath();
        context.moveTo(pos, 0);
        context.lineTo(pos, 30);
        context.closePath();

        context.stroke();
      }

      context.textAlign = "center";

      // draw minor graduations text
      context.font = 'bold 9px sans-serif';

      context.fillStyle = '#fff';
      context.textAlign = 'center';

      for (var d = 0; d < 360; d += 30) {
        // do not draw 0/90/180/270
        var pos = d * length / 360;
        var angle = this.mReversedRing ? (360 + 180 - d) % 360: 360 - d;
        if (d % 90 != 0) context.fillText(angle, pos, 30);
      }

      // draw N/O/S/E
      // hack : go till 360, so that "N" is printed at both end of the texture...
      context.font = 'bold 20px sans-serif';

      context.fillStyle = '#fff';
      context.textAlign = 'center';

      for (var d = 0; d <= 360; d += 90) {
        var pos = d * length / 360;
        if (this.mReversedRing) {
          context.fillText(this.CARDINAL_POINTS[((d + 180) / 90) % 4], pos, 50);
        } else {
          context.fillText(this.CARDINAL_POINTS[(d / 90) % 4], pos, 50);
        }
      }

      var gradient = context.createLinearGradient(0, 5, 0, 0);
      gradient.addColorStop(0.5, "#FF0000");
      gradient.addColorStop(0.5, "#FFF");
      context.fillStyle = gradient;

      context.fillRect(0, 0, length, 5);

      return this.buildImageTexture(this.TEXTURE_RING, canvas);
    }

    buildDialTexture() {
      var radius = 256;

      var canvas = document.createElement('canvas');
      canvas.setAttribute('width', radius * 2);
      canvas.setAttribute('height', radius * 2);
      //document.body.appendChild(canvas); // debugging
      var context = canvas.getContext('2d');

      context.fillStyle = '#000000';
      context.fillRect(0, 0, radius * 2, radius * 2);

      // outer shaded ring
      context.strokeStyle = '#666';
      //context.fillStyle = '#000';
      context.lineWidth = 4;

      context.beginPath();
      context.arc(radius, radius, radius - 10, 0, 2 * Math.PI);
      context.stroke();
      //context.fill();
      context.closePath();

      // build the inner decoration, using two symmetrical paths
      context.save();
      for (var i = 0; i < 4; i++) {
        context.translate(radius, radius);
        context.rotate(i * Math.PI / 2);
        context.translate( - radius, -radius);

        context.fillStyle = '#666';
        context.beginPath();
        context.moveTo(radius, radius / 2);
        context.lineTo(radius + 20, radius - 20);
        context.lineTo(radius, radius);
        context.closePath();
        context.fill();

        context.fillStyle = '#FFFFFF';
        context.beginPath();
        context.moveTo(radius, radius / 2);
        context.lineTo(radius - 20, radius - 20);
        context.lineTo(radius, radius);
        context.closePath();
        context.fill();
      }
      context.restore();

      // draw medium graduations in white
      context.strokeStyle = '#FFFFFF';
      context.lineWidth = 2;
      for (var i = 0; i < 360; i += 10) {
        context.save();
        context.translate(radius, radius);
        context.rotate(i * Math.PI / 180);
        context.translate( - radius, -radius);
        context.beginPath();
        context.moveTo(radius, radius * 2);
        context.lineTo(radius, 1.75 * radius);
        context.stroke();
        //context.closePath();
        context.restore();
      }

      // draw major graduations in red
      context.strokeStyle = '#FF0000';
      context.lineWidth = 3;
      for (var i = 0; i < 360; i += 90) {
        context.save();
        context.translate(radius, radius);
        context.rotate(i * Math.PI / 180);
        context.translate( - radius, -radius);
        context.beginPath();
        context.moveTo(radius, radius * 2);
        context.lineTo(radius, 1.70 * radius);
        context.stroke();
        //context.closePath();
        context.restore();
      }

      // medium graduation texts
      context.font = 'bold 24px sans-serif';
      context.fillStyle = '#fff';
      context.textAlign = 'center';
      for (var i = 0; i < 360; i += 30) {
        if ((i % 90) != 0) {
          var a = -i * 2 * Math.PI / 360;
          var x = Math.sin(a) * 0.7 * radius + radius;
          var y = Math.cos(a) * 0.7 * radius + radius;

          context.save();
          context.translate(x, y);
          context.rotate(i * Math.PI / 180);
          context.translate( - x, -y);

          context.fillText(i, x, y);

          context.restore();
        }
      }

      // draw N/O/S/E
      context.font = 'bold 38px sans-serif';
      context.fillStyle = '#FF0000';
      context.textAlign = 'center';
      for (var i = 0; i < 360; i += 90) {
        var a = i * 2 * Math.PI / 360;
        var x = Math.sin(a) * 0.65 * radius + radius;
        var y = Math.cos(a) * 0.65 * radius + radius;

        context.save();
        context.translate(x, y);
        context.rotate( - i * Math.PI / 180);
        context.translate( - x, -y);

        context.fillText(this.CARDINAL_POINTS[i / 90], x, y);

        context.restore();
      }

      return this.buildImageTexture(this.TEXTURE_DIAL, canvas);
    }

    draw() {
      if (this.needsInitializing) {
        this.needsInitializing = false;
        this.buildObjects();
        this.buildTextures();
      }

      if (!this.ready) {
        return;
      }

      var dx = this.DETAIL_X[this.mDetailsLevel];
      var dy = this.DETAIL_Y[this.mDetailsLevel];
      var rh = this.RING_HEIGHT[this.mDetailsLevel];

      // Enable texture for the ring and dial objects
      this.gl.enableVertexAttribArray(this.gl.textureCoordAttribute);

      // Draw Ring Object
      this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.mRingVertexBufferGL);
      this.gl.vertexAttribPointer(this.gl.vertexPositionAttribute, this.mRingVertexBufferGL.itemSize, this.gl.FLOAT, false, 0, 0);

      this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.mRingTexCoordBufferGL);
      this.gl.vertexAttribPointer(this.gl.textureCoordAttribute, this.mRingTexCoordBufferGL.itemSize, this.gl.FLOAT, false, 0, 0);

      this.gl.bindTexture(this.gl.TEXTURE_2D, this.mTextures[this.TEXTURE_RING]);
      this.gl.bindBuffer(this.gl.ELEMENT_ARRAY_BUFFER, this.mRingIndexBufferGL);

      this.gl.drawElements(this.gl.TRIANGLES, dx * rh * 6, this.gl.UNSIGNED_SHORT, 0);

      // Draw Dial Object
      this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.mDialVertexBufferGL);
      this.gl.vertexAttribPointer(this.gl.vertexPositionAttribute, this.mDialVertexBufferGL.itemSize, this.gl.FLOAT, false, 0, 0);

      this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.mDialTexCoordBufferGL);
      this.gl.vertexAttribPointer(this.gl.textureCoordAttribute, this.mDialTexCoordBufferGL.itemSize, this.gl.FLOAT, false, 0, 0);

      this.gl.bindTexture(this.gl.TEXTURE_2D, this.mTextures[this.TEXTURE_DIAL]);
      this.gl.bindBuffer(this.gl.ELEMENT_ARRAY_BUFFER, this.mDialIndexBufferGL);

      this.gl.drawElements(this.gl.TRIANGLE_FAN, dx + 2, this.gl.UNSIGNED_SHORT, 0);


      // Draw Cap Object
      this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.mCapVertexBufferGL);
      this.gl.vertexAttribPointer(this.gl.vertexPositionAttribute, this.mCapVertexBufferGL.itemSize, this.gl.FLOAT, false, 0, 0);

      this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.mCapTexCoordBufferGL);
      this.gl.vertexAttribPointer(this.gl.textureCoordAttribute, this.mCapTexCoordBufferGL.itemSize, this.gl.FLOAT, false, 0, 0);

      this.gl.bindTexture(this.gl.TEXTURE_2D, this.mTextures[this.TEXTURE_CAP]);
      this.gl.bindBuffer(this.gl.ELEMENT_ARRAY_BUFFER, this.mCapIndexBufferGL);

      this.gl.drawElements(this.gl.TRIANGLES, dx * (dy - rh) * 6, this.gl.UNSIGNED_SHORT, 0);

      this.compassrenderer.compass.checkGLError();
    }
  };
})(window);
