/*
 * Marine Compass
 * http://github.com/richtr/Marine-Compass
 *
 * Copyright (c) 2012-2014, Rich Tibbett
 *               2016-2017, Intel Corporation, all rights reserved
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
  * License:   Apache License, Version 2.0 (see LICENSE.md file)
  * URLs:      http://www.pierrox.net/cmsms/applications/marine-compass.html
  */

// @ts-check
import { LitElement, html } from '../scripts/lit-element.js';

import * as glMatrix from "../node_modules/gl-matrix/src/gl-matrix/common.js";
import * as mat4 from "../node_modules/gl-matrix/src/gl-matrix/mat4.js";
import * as quat from "../node_modules/gl-matrix/src/gl-matrix/quat.js";
import * as vec3 from "../node_modules/gl-matrix/src/gl-matrix/vec3.js";
import * as euler from "./euler.js";

import { MotionController } from "./motion-controller-zephyr.js";
import { DaydreamController } from "./daydream-controller.js";
import { Thingy52Controller } from "./thingy52-controller.js";

import { LowPassFilterData } from "./lowpass-filter.js";
import { KalmanFilter } from "./kalman-filter.js";

import { CompassSensor } from "./compass-sensor.js";

function toRad(val) {
  return val * Math.PI / 180;
}
function toDeg(val) {
  return val * 180 / Math.PI;
}

function isPowerOf2(value) {
  return (value & (value - 1)) == 0;
};

const report = (err) => {
  if (err.target) {
    console.error(`${err.target.constructor.name}.${err.error}`);
  } else {
    console.error(`${err}`);
  }
}

let _Gyroscope = window['Gyroscope'];
let _Accelerometer = window['Accelerometer'];
let _AbsoluteOrientationSensor = window['AbsoluteOrientationSensor'];

const compassVertexSource = `
  attribute vec3 aVertexPosition;
  attribute vec2 aTextureCoord;

  uniform mat4 uMVMatrix;
  uniform mat4 uPMatrix;

  varying mediump vec2 vTextureCoord;

  void main(void) {
    gl_Position = uPMatrix * uMVMatrix * vec4(aVertexPosition, 1.0);
    vTextureCoord = aTextureCoord;
  }`;

const compassFragmentSource = `
  precision mediump float;
  
  varying vec2 vTextureCoord;
  
  uniform sampler2D uSampler;
  
  void main(void) {
    vec4 textureColor = texture2D(uSampler, vTextureCoord);
    gl_FragColor = textureColor;
  }`;

class SensorCompass extends LitElement {

  static get properties() {
    return {
      title: {
        type: String,
        value: "Marine Compass"
      }
    }
  }

  render() {
    return html`
      <style>
        div {
          display: block;
          text-align: center;
          padding: 10px 0 0;
          height: 40px;
          color: #fff;
          font-size: 1.8em;
          font-weight: normal;
        }
        canvas {
          position: absolute;
          pointer-events: none;
          top: 0;
        }
      </style>
      <div>
        <span>${this.heading}</span>&deg;
      </div>
      <div>${this.title}</div>
      <canvas id="canvas"></canvas>
    `;
  }

  async connectedCallback() {
    super.connectedCallback();

    await this.invalidate();
  
    this.$("canvas").width = window.innerWidth;
    this.$("canvas").height = window.innerHeight;

    this.kalmanY = new KalmanFilter();
    this.kalmanX = new KalmanFilter();
    this.kalmanZ = new KalmanFilter();

    this.alpha = 0.0;
    this.beta = 0.0;
    this.gamma = 0.0;

    this.sensors = {};

    try {
      this.gl = this.$("canvas").getContext("webgl2");
      if (!this.gl) {
        console.error('Unable to initialize WebGL. Your browser may not support it', 'http://get.webgl.org');
      } else {
        this.gl.viewportWidth = this.$("canvas").width;
        this.gl.viewportHeight = this.$("canvas").height;

        this._createViewport();
        this.start();  
      }
    }
    catch(e) {
      console.error(e);
    }

    void function renderCompass() {
      this.calculateRotationMatrix();
      this.mCompassRenderer.draw();

      window.requestAnimationFrame(renderCompass.bind(this));
    }.bind(this)()
  }

  async requestBTController(value) {
    this.controller = null;
    _Gyroscope = window['Gyroscope'];
    _Accelerometer = window['Accelerometer'];
    _AbsoluteOrientationSensor = window['AbsoluteOrientationSensor'];

    if (value == "zephyr") {
      this.controller = new MotionController();
      try {
        await this.controller.connect();
        _Gyroscope = this.controller.gyroscope;
        _Accelerometer = this.controller.accelerometer;
      } catch(err) {}
    }

    if (value == "daydream") {
      this.controller = new DaydreamController();
      try {
        await this.controller.connect();
        _Gyroscope = this.controller.Gyroscope;
        _Accelerometer = this.controller.Accelerometer;
        _AbsoluteOrientationSensor = this.controller.AbsoluteOrientationSensor;
      } catch(err) {}
    }

    if (value == "thingy52") {
      this.controller = new Thingy52Controller();
      try {
        await this.controller.connect();
        _AbsoluteOrientationSensor = this.controller.AbsoluteOrientationSensor;
      } catch(err) {}
    }

    this.onRouteChanged();
  }

  start() {
    this.onRouteChanged();
  }

  onRouteChanged() {
    let params = new URLSearchParams(new URL(window.location.href).search.slice(1));
    let filter = params.get("filter");

    if (this.session) {
      for (let sensor of this.session.sensors) {
        sensor.onreading = null;
        sensor.stop();
      }
    }
    this.session = { sensors: []};

    this.alpha = 0.0;
    this.beta = 0.0;
    this.gamma = 0.0;

    try {
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
          this.startKalmanFilterDemo();
          break;
        case "a":
        case "0":
          this.startAccelerometerDemo();
          break;
        case "g":
        case "1":
          this.startGyroscopeDemo();
          break;
        case "c":
          let num = parseFloat(filter);
          if (Number.isNaN(num))
            num = 0.98;
          let weight = Math.min(1, Math.max(0, num));
          this.startComplementaryFilterDemo(weight);
          break;
        default:
          this.startMagnetometerDemo();
      }
    } catch (err) {
      report(err);
    }
  }

  createOnALSReadingFn(ambientLightSensor) {
    function remap(value, inRangeStart, inRangeEnd, outRangeStart, outRangeEnd) {
      return outRangeStart + (outRangeEnd - outRangeStart) * ((value - inRangeStart) / (inRangeEnd - inRangeStart));
    };

    return event => {
      let value = Math.min(Math.max(remap(ambientLightSensor.illuminance, 0, 100, 0, 100), 0), 100);
      this.$("canvas").style = `filter: grayscale(${value}%)`;
    }
  }

  createOnAccelerometerReadingFn(accelerometer) {
    return event => {
      let xAccel = accelerometer.y;
      let yAccel = accelerometer.x;
      let zAccel = accelerometer.z;

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
  }

  createOnGyroscopeReadingFn(gyroscope, accelerometer, weight) {
    return event => {
      let xAccel = 0;
      let yAccel = 0;
      let zAccel = 0;

      if (weight != 1) {
        xAccel = accelerometer.y;
        yAccel = accelerometer.x;
        zAccel = accelerometer.z;

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

      let xGyro = gyroscope.x * 180 / Math.PI;
      let yGyro = gyroscope.y * 180 / Math.PI;
      let zGyro = gyroscope.z * 180 / Math.PI;

      let dt = 0;
      if (this.timestamp) {
        dt = (gyroscope.timestamp - this.timestamp) / 1000;
      }
      this.timestamp = gyroscope.timestamp;

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
  }

  createOnAbsoluteOrientationSensorReadingFn(absOrientationSensor) {
    return event => {
      let orientation = mat4.create();
      absOrientationSensor.populateMatrix(orientation);

      const angles = euler.fromMat4(euler.create(), orientation);

      this.alpha = angles[0];
      this.beta = angles[1];
      this.gamma = angles[2];
    };
  }

  createOnMagnetometerSensorReadingFn(magnetometer, accelerometer) {
    // Isolate gravity with low pass filter.
    const gravity = new LowPassFilterData(accelerometer, 0.8);
    const compass = new CompassSensor();

    return event => {
      gravity.update(accelerometer);
      compass.update(gravity, magnetometer);

      this.alpha = compass.alpha;
      this.beta = compass.beta;
      this.gamma = compass.gamma;
    };
  }

  startAmbientLightDemo() {
    this.title = "Ambient Light";

    let ambientLightSensor = new AmbientLightSensor({ frequency: 10 });
    ambientLightSensor.onreading = this.createOnALSReadingFn(ambientLightSensor);
    ambientLightSensor.onerror = err => report(err);
    this.session.sensors.push(ambientLightSensor);

    ambientLightSensor.start();
  }

  startAccelerometerDemo() {
    this.title = "Accelerometer";

    let accelerometer = new _Accelerometer({ frequency: 50 });
    accelerometer.onreading = this.createOnAccelerometerReadingFn(accelerometer);
    accelerometer.onerror = err => report(err);
    this.session.sensors.push(accelerometer);

    accelerometer.start();
  }

  startGyroscopeDemo() {
    this.title = "Gyroscope";

    let gyroscope = new _Gyroscope({ frequency: 50 });
    gyroscope.onreading = this.createOnGyroscopeReadingFn(gyroscope, null, 1);
    gyroscope.onerror = err => report(err);
    this.session.sensors.push(gyroscope);

    gyroscope.start();
  }

  startKalmanFilterDemo() {
    this.title = "Kalman filter";

    let accelerometer = new _Accelerometer({ frequency: 50 });
    accelerometer.onerror = err => report(err);
    this.session.sensors.push(accelerometer);

    let gyroscope = new _Gyroscope({ frequency: 50 });
    gyroscope.onreading = this.createOnGyroscopeReadingFn(gyroscope, accelerometer, 0);
    gyroscope.onerror = err => report(err);
    this.session.sensors.push(gyroscope);

    accelerometer.start();
    gyroscope.start();
  }

  startComplementaryFilterDemo(weight) {
    this.title = `Complementary (${weight}) filter`;

    let accelerometer = new _Accelerometer({ frequency: 50 });
    accelerometer.onerror = err => report(err);
    this.session.sensors.push(accelerometer);

    let gyroscope = new _Gyroscope({ frequency: 50 });
    gyroscope.onreading = this.createOnGyroscopeReadingFn(gyroscope, accelerometer, weight);
    gyroscope.onerror = err => report(err);
    this.session.sensors.push(gyroscope);

    accelerometer.start();
    gyroscope.start();
  }

  startAbsoluteOrientationDemo() {
    this.title = "Absolute Orientation";

    let absOrientationSensor = new _AbsoluteOrientationSensor({ frequency: 50 });
    absOrientationSensor.onreading = this.createOnAbsoluteOrientationSensorReadingFn(absOrientationSensor);
    absOrientationSensor.onerror = err => report(err);
    this.session.sensors.push(absOrientationSensor);

    absOrientationSensor.start();
  }

  startMagnetometerDemo() {
    this.title = "Magnetometer";

    let accelerometer = new Accelerometer({ frequency: 20 });
    accelerometer.onerror = err => report(err);
    this.session.sensors.push(accelerometer);

    let magnetometer = new Magnetometer({ frequency: 10 });
    magnetometer.onreading = this.createOnMagnetometerSensorReadingFn(magnetometer, accelerometer);
    magnetometer.onerror = err => report(err);
    this.session.sensors.push(magnetometer);

    accelerometer.start();
    magnetometer.start();
  }

  _createViewport() {
    // Create rotation matrix object (calculated per canvas draw)
    this.rotationMatrix = mat4.create();
    this.rotationMatrix = mat4.identity(mat4.create());

    // Create screen transform matrix (calculated once)
    this.screenMatrix = mat4.identity(mat4.create());

    const inv = toRad(180);

    this.screenMatrix[0] =   Math.cos(inv);
    this.screenMatrix[1] =   Math.sin(inv);
    this.screenMatrix[4] = - Math.sin(inv);
    this.screenMatrix[5] =   Math.cos(inv);

    // Create world transformation matrix (calculated once)
    this.worldMatrix = mat4.identity(mat4.create());

    const up = toRad(90);

    this.worldMatrix[5]  =   Math.cos(up);
    this.worldMatrix[6]  =   Math.sin(up);
    this.worldMatrix[9]  = - Math.sin(up);
    this.worldMatrix[10] =   Math.cos(up);

    // CompassRenderer manages 3D objects and gl surface life cycle
    this.mCompassRenderer = new CompassRenderer(this);

    // Catch window resize event
    window.addEventListener('orientationchange', _ => {
      window.setTimeout(() => {
        this.gl.viewportWidth = this.$("canvas").width = window.innerWidth;
        this.gl.viewportHeight = this.$("canvas").height = window.innerHeight;

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
    const error = this.gl.getError();
    if (error != this.gl.NO_ERROR && error != this.gl.CONTEXT_LOST_WEBGL) {
      const str = `GL Error ${error}`;
      console.error(str);
      throw str;
    }
  }

  calculateRotationMatrix() {
    this.rotationMatrix = mat4.identity(mat4.create());

    // Apply screen orientation.
    mat4.rotateZ(this.rotationMatrix, this.rotationMatrix, toRad(window.screen.orientation.angle || 0));

    // Apply compass orientation.
    const compassOrientation = euler.toMat4(mat4.create(), [this.alpha, this.beta, this.gamma]);
    mat4.multiply(this.rotationMatrix, this.rotationMatrix, compassOrientation);

    const q = mat4.getRotation(quat.create(), this.rotationMatrix);
    const axis = toDeg(quat.getAxisAngle(vec3.fromValues(0, 0, 1), q));

    // Invert compass heading.
    mat4.multiply(this.rotationMatrix, this.rotationMatrix, this.screenMatrix);

    // Apply world orientation (heads-up display).
    mat4.multiply(this.rotationMatrix, this.rotationMatrix, this.worldMatrix);

    this.mCompassRenderer.setRotationMatrix(this.rotationMatrix);

    this.heading = Math.floor(axis);
  }
}

customElements.define('sensor-compass', SensorCompass.withProperties());

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
    let shader = this.gl.createShader(type);
    // Load the shader source
    this.gl.shaderSource(shader, shaderSrc);
    // Compile the shader
    this.gl.compileShader(shader);
    // Check the compile status
    if (!this.gl.getShaderParameter(shader, this.gl.COMPILE_STATUS) &&
      !this.gl.isContextLost()) {
        const infoLog = this.gl.getShaderInfoLog(shader);
        console.error(`Error compiling shader:\n${infoLog}`);
        this.gl.deleteShader(shader);
        return null;
    }
    return shader;
  }

  init() {
    // initialize
    let vertexShader = this.loadShader(this.gl.VERTEX_SHADER, compassVertexSource);
    let fragmentShader = this.loadShader(this.gl.FRAGMENT_SHADER, compassFragmentSource);

    this.shaderProgram = this.gl.createProgram();
    this.gl.attachShader(this.shaderProgram, vertexShader);
    this.gl.attachShader(this.shaderProgram, fragmentShader);

    this.gl.linkProgram(this.shaderProgram);

    // Check the link status
    const linked = this.gl.getProgramParameter(this.shaderProgram, this.gl.LINK_STATUS);
    if (!linked && !this.gl.isContextLost()) {
      const infoLog = this.gl.getProgramInfoLog(this.shaderProgram);
      console.error(`Error linking program:\n${infoLog}`);
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

  draw() {
    // Clear the canvas
    this.gl.clear(this.gl.COLOR_BUFFER_BIT | this.gl.DEPTH_BUFFER_BIT);

    // Reset move matrix
    this.mvMatrix = mat4.identity(mat4.create());
    mat4.translate(this.mvMatrix, this.mvMatrix, [ 0, 0, -4 ]);

    // Apply calculated device rotation matrix
    mat4.multiply(this.mvMatrix, this.mvMatrix, this.rotationMatrix);

    this.setMatrixUniforms();

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
    const dx = this.DETAIL_X[this.mDetailsLevel];
    const dy = this.DETAIL_Y[this.mDetailsLevel];
    const rh = this.RING_HEIGHT[this.mDetailsLevel];

    const vertices = new Array(((dx + 1) * (rh + 1)) * 3);
    const normals = new Array(((dx + 1) * (rh + 1)) * 3);

    let n = 0;

    for (let i = 0; i <= dx; i++) {
      for (let j = 0; j <= rh; j++) {
        const a = i * 2 * Math.PI / dx;
        const b = j * Math.PI / (dy * 2);

        const x = Math.sin(a) * Math.cos(b);
        const y = -Math.sin(b);
        const z = Math.cos(a) * Math.cos(b);

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
    const texCoords = new Array((dx + 1) * (rh + 1) * 2);
    n = 0;
    for (let i = 0; i <= dx; i++) {
      for (let j = 0; j <= rh; j++) {
        texCoords[n++] = i / dx;
        texCoords[n++] = j / rh;
      }
    }

    // build indices
    const indices = new Array(dx * rh * 3 * 2);
    n = 0;
    for (let i = 0; i < dx; i++) {
      for (let j = 0; j < rh; j++) {
        const p0 = ((rh + 1) * i + j);
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
    const dx = this.DETAIL_X[this.mDetailsLevel];
    const dy = this.DETAIL_Y[this.mDetailsLevel];
    const rh = this.RING_HEIGHT[this.mDetailsLevel];

    const h = dy - rh;

    // build vertices
    const vertices = new Array(((dx + 1) * (h + 1)) * 3);

    let n = 0;
    for (let i = 0; i <= dx; i++) {
      for (let j = rh; j <= dy; j++) {
        const a = i * 2 * Math.PI / dx;
        const b = j * Math.PI / (dy * 2);

        const x = Math.sin(a) * Math.cos(b);
        const y = -Math.sin(b);
        const z = Math.cos(a) * Math.cos(b);

        vertices[n++] = x;
        vertices[n++] = y;
        vertices[n++] = z;
      }
    }

    // build indices
    const indices = new Array(dx * h * 3 * 2);
    n = 0;
    for (let i = 0; i < dx; i++) {
      for (let j = 0; j < h; j++) {
        const p0 = ((h + 1) * i + j);
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
    const dx = this.DETAIL_X[this.mDetailsLevel];

    const vertices = new Array((dx + 2) * 3);
    const normals = new Array((dx + 2) * 3);

    let n = 0;
    // center of the dial
    vertices[n] = 0;
    vertices[n + 1] = 0;
    vertices[n + 2] = 0;
    normals[n] = 0;
    normals[n + 1] = 1;
    normals[n + 2] = 0;
    n += 3;
    for (let i = 0; i <= dx; i++) {
      const a = i * 2 * Math.PI / dx;

      const x = Math.sin(a);
      const z = Math.cos(a);

      vertices[n] = x;
      vertices[n + 1] = 0;
      vertices[n + 2] = z;
      normals[n] = 0;
      normals[n + 1] = 1;
      normals[n + 2] = 0;
      n += 3;
    }

    // build textures coordinates
    const texCoords = new Array((dx + 2) * 2);
    n = 0;
    texCoords[n++] = 0.5;
    texCoords[n++] = 0.5;
    for (let i = 0; i <= dx; i++) {
      const a = i * 2 * Math.PI / dx;

      const x = (Math.sin(a) + 1) / 2;
      const z = (Math.cos(a) + 1) / 2;

      texCoords[n++] = x;
      texCoords[n++] = z;
    }

    // build indices
    const indices = new Array(dx + 2);
    n = 0;
    for (let i = 0; i <= (dx + 1); i++) {
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

      image.src = canvas.toDataURL("image/webp");
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
    const length = 512;
    const height = 64;

    const canvas = document.createElement('canvas');
    canvas.setAttribute('width', length.toString());
    canvas.setAttribute('height', height.toString());
    //document.body.appendChild(canvas); // debugging
    const context = canvas.getContext('2d');

    context.fillStyle = '#000000';
    context.fillRect(0, 0, length, height);

    // draw medium graduations in white
    context.strokeStyle = '#fff';
    context.lineWidth = 1;

    for (let d = 0; d < 360; d += 10) {
      const pos = d * length / 360;

      context.beginPath();
      context.moveTo(pos, 0);
      context.lineTo(pos, 20);
      context.closePath();

      context.stroke();
    }

    // draw major graduations in red
    context.strokeStyle = '#FF0000';
    context.lineWidth = 2;

    for (let d = 0; d < 360; d += 90) {
      const pos = d * length / 360;

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

    for (let d = 0; d < 360; d += 30) {
      // do not draw 0/90/180/270
      const pos = d * length / 360;
      const angle = this.mReversedRing ? (360 + 180 - d) % 360: 360 - d;
      if (d % 90 != 0) context.fillText(angle.toString(), pos, 30);
    }

    // draw N/O/S/E
    // hack : go till 360, so that "N" is printed at both end of the texture...
    context.font = 'bold 20px sans-serif';

    context.fillStyle = '#fff';
    context.textAlign = 'center';

    for (let d = 0; d <= 360; d += 90) {
      const pos = d * length / 360;
      if (this.mReversedRing) {
        context.fillText(this.CARDINAL_POINTS[((d + 180) / 90) % 4], pos, 50);
      } else {
        context.fillText(this.CARDINAL_POINTS[(d / 90) % 4], pos, 50);
      }
    }

    const gradient = context.createLinearGradient(0, 5, 0, 0);
    gradient.addColorStop(0.5, "#FF0000");
    gradient.addColorStop(0.5, "#FFF");
    context.fillStyle = gradient;

    context.fillRect(0, 0, length, 5);

    return this.buildImageTexture(this.TEXTURE_RING, canvas);
  }

  buildDialTexture() {
    const radius = 256;
    const diameter = radius * 2;

    const canvas = document.createElement('canvas');
    canvas.setAttribute('width', diameter.toString());
    canvas.setAttribute('height', diameter.toString());

    const context = canvas.getContext('2d');

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
    for (let i = 0; i < 4; i++) {
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
    for (let i = 0; i < 360; i += 10) {
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
    for (let i = 0; i < 360; i += 90) {
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
    for (let i = 0; i < 360; i += 30) {
      if ((i % 90) != 0) {
        const a = -i * 2 * Math.PI / 360;
        const x = Math.sin(a) * 0.7 * radius + radius;
        const y = Math.cos(a) * 0.7 * radius + radius;

        context.save();
        context.translate(x, y);
        context.rotate(i * Math.PI / 180);
        context.translate( - x, -y);

        context.fillText(i.toString(), x, y);

        context.restore();
      }
    }

    // draw N/O/S/E
    context.font = 'bold 38px sans-serif';
    context.fillStyle = '#FF0000';
    context.textAlign = 'center';
    for (let i = 0; i < 360; i += 90) {
      const a = i * 2 * Math.PI / 360;
      const x = Math.sin(a) * 0.65 * radius + radius;
      const y = Math.cos(a) * 0.65 * radius + radius;

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

    const dx = this.DETAIL_X[this.mDetailsLevel];
    const dy = this.DETAIL_Y[this.mDetailsLevel];
    const rh = this.RING_HEIGHT[this.mDetailsLevel];

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