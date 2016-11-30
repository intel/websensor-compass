/*
 * Marine Compass
 * http://github.com/richtr/Marine-Compass
 *
 * Copyright (c) 2012-2014, Rich Tibbett
 *               2016, Kenneth Christiansen
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

var degToRad = Math.PI / 180;
var radToDeg = 180 / Math.PI;

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


class RotationMatrix extends DOMMatrix {
  constructor(...args) {
    if (args.length == 16) {
      super(...args);
      return;
    }

    super();

    if (args.length == 9) {
      this.m11 = args[0];
      this.m12 = args[1];
      this.m13 = args[2];

      this.m21 = args[3];
      this.m22 = args[4];
      this.m23 = args[5];

      this.m31 = args[6];
      this.m32 = args[7];
      this.m33 = args[8];
    }
  }

  set(m11, m12, m13, m21, m22, m23, m31, m32, m33) {
    this.m11 = m11 || 1;
    this.m12 = m12 || 0;
    this.m13 = m13 || 0;
    this.m21 = m21 || 0;
    this.m22 = m22 || 1;
    this.m23 = m23 || 0;
    this.m31 = m31 || 0;
    this.m32 = m32 || 0;
    this.m33 = m33 || 1;
  }

  static fromRotationMatrix(other) {
    let matrix = new RotationMatrix();
    matrix.m11 = other.m11;
    matrix.m12 = other.m12;
    matrix.m13 = other.m13;
    matrix.m21 = other.m21;
    matrix.m22 = other.m22;
    matrix.m23 = other.m23;
    matrix.m31 = other.m31;
    matrix.m32 = other.m32;
    matrix.m33 = other.m33;

    return matrix;
  }

  fromSensorData(gravity, geomagnetic) {
    let Ax = gravity.x;
    let Ay = gravity.y;
    let Az = gravity.z;

    let Ex = geomagnetic.x;
    let Ey = geomagnetic.y;
    let Ez = geomagnetic.z;

    let Hx = Ey*Az - Ez*Ay;
    let Hy = Ez*Ax - Ex*Az;
    let Hz = Ex*Ay - Ey*Ax;

    let normH = Math.sqrt(Hx*Hx + Hy*Hy + Hz*Hz);

    let invH = 1.0 / normH;
    Hx *= invH;
    Hy *= invH;
    Hz *= invH;

    let invA = 1.0 / Math.sqrt(Ax*Ax + Ay*Ay + Az*Az);
    Ax *= invA;
    Ay *= invA;
    Az *= invA;

    let Mx = Ay*Hz - Az*Hy;
    let My = Az*Hx - Ax*Hz;
    let Mz = Ax*Hy - Ay*Hx;

    this.m11 = Hx;
    this.m12 = Hy;
    this.m13 = Hz;

    this.m21 = Mx;
    this.m22 = My;
    this.m23 = Mz;

    this.m31 = Ax;
    this.m32 = Ay;
    this.m33 = Az;
  }

  static fromEuler(euler) {
    euler = euler || {};

    let z = (euler.alpha || 0) * degToRad;
    let x = (euler.beta || 0) * degToRad;
    let y = (euler.gamma || 0) * degToRad;

    let cX = Math.cos(x);
    let cY = Math.cos(y);
    let cZ = Math.cos(z);
    let sX = Math.sin(x);
    let sY = Math.sin(y);
    let sZ = Math.sin(z);

    //
    // ZXY-ordered rotation matrix construction.
    //
    let matrix = new RotationMatrix();

    matrix.m11 = cZ * cY - sZ * sX * sY;
    matrix.m12 = - cX * sZ;
    matrix.m13 = cY * sZ * sX + cZ * sY;

    matrix.m21 = cY * sZ + cZ * sX * sY;
    matrix.m22 = cZ * cX;
    matrix.m23 = sZ * sY - cZ * cY * sX;

    matrix.m31 = - cX * sY;
    matrix.m32 = sX;
    matrix.m33 = cX * cY;

    return matrix.normalizeSelf();
  }

  static fromQuaternion(q) {
    let sqw, sqx, sqy, sqz;

    sqw = q.w * q.w;
    sqx = q.x * q.x;
    sqy = q.y * q.y;
    sqz = q.z * q.z;

    let matrix = new RotationMatrix();

    matrix.m11 = sqw + sqx - sqy - sqz;
    matrix.m12 = 2 * (q.x * q.y - q.w * q.z);
    matrix.m13 = 2 * (q.x * q.z + q.w * q.y);

    matrix.m21 = 2 * (q.x * q.y + q.w * q.z);
    matrix.m22 = sqw - sqx + sqy - sqz;
    matrix.m23 = 2 * (q.y * q.z - q.w * q.x);

    matrix.m31 = 2 * (q.x * q.z - q.w * q.y);
    matrix.m32 = 2 * (q.y * q.z + q.w * q.x);
    matrix.m33 = sqw - sqx - sqy + sqz;

    return matrix;
  }

  normalizeSelf() {
    // Calculate matrix determinant
    let determinant = 
        this.m11 * this.m22 * this.m33 
      - this.m11 * this.m23 * this.m32 
      - this.m12 * this.m21 * this.m33
      + this.m12 * this.m23 * this.m31 
      + this.m13 * this.m21 * this.m32 
      - this.m13 * this.m22 * this.m31;

    console.log(determinant);

    // Normalize matrix values
    this.m11 = this.m11 / determinant;
    this.m12 = this.m12 / determinant;
    this.m13 = this.m13 / determinant;
    this.m21 = this.m21 / determinant;
    this.m22 = this.m22 / determinant;
    this.m23 = this.m23 / determinant;
    this.m31 = this.m31 / determinant;
    this.m32 = this.m32 / determinant;
    this.m33 = this.m33 / determinant;

    return this;
  }

  normalize() {
    return RotationMatrix.fromRotationMatrix(this).normalizeSelf();
  }
};

class Euler{
  constructor(alpha, beta, gamma) {
    this.alpha = alpha;
    this.beta = beta;
    this.gamma = gamma;
  }

  static fromEuler(other) {
    let euler = new Euler();
    euler.alpha = other.alpha;
    euler.beta  = other.beta;
    euler.gamma = other.gamma;
  }

  static fromRotationMatrix(matrix) {
    let _alpha, _beta, _gamma;

    if (matrix.m33 > 0) { // cos(beta) > 0
      _alpha = Math.atan2(-matrix.m12, matrix.m22);
      _beta  = Math.asin(matrix.m32); // beta (-pi/2, pi/2)
      _gamma = Math.atan2(-matrix.m31, matrix.m33); // gamma (-pi/2, pi/2)
    } else if (matrix.m33 < 0) {  // cos(beta) < 0
      _alpha = Math.atan2(matrix.m12, -matrix.m22);
      _beta  = -Math.asin(matrix.m32);
      _beta  += (_beta >= 0) ? - Math.PI : Math.PI; // beta [-pi,-pi/2) U (pi/2,pi)
      _gamma = Math.atan2(matrix.m31, -matrix.m33); // gamma (-pi/2, pi/2)
    } else { // matrix.m33 == 0
      if (matrix.m31 > 0) {  // cos(gamma) == 0, cos(beta) > 0
        _alpha = Math.atan2(-matrix.m12, matrix.m22);
        _beta  = Math.asin(matrix.m32); // beta [-pi/2, pi/2]
        _gamma = - (Math.PI / 2); // gamma = -pi/2
      } else if (matrix.m31 < 0) { // cos(gamma) == 0, cos(beta) < 0
        _alpha = Math.atan2(matrix.m12, -matrix.m22);
        _beta  = -Math.asin(matrix.m32);
        _beta  += (_beta >= 0) ? - Math.PI : Math.PI; // beta [-pi,-pi/2) U (pi/2,pi)
        _gamma = - (Math.PI / 2); // gamma = -pi/2
      } else { // matrix.m31 == 0, cos(beta) == 0
        // gimbal lock discontinuity
        _alpha = Math.atan2(matrix.m21, matrix.m11);
        _beta  = (matrix.m32 > 0) ? (Math.PI / 2) : - (Math.PI / 2); // beta = +-pi/2
        _gamma = 0; // gamma = 0
      }
    }

    // alpha is in [-pi, pi], make sure it is in [0, 2*pi).
    if (_alpha < 0) {
      _alpha += 2 * Math.PI; // alpha [0, 2*pi)
    }

    // Convert to degrees
    _alpha *= radToDeg;
    _beta  *= radToDeg;
    _gamma *= radToDeg;

    // apply derived euler angles to current object
    return new Euler(_alpha, _beta, _gamma);
  }

  static fromQuaternion(q) {
    let _alpha, _beta, _gamma;

    var sqw = q.w * q.w;
    var sqx = q.x * q.x;
    var sqy = q.y * q.y;
    var sqz = q.z * q.z;

    var unitLength = sqw + sqx + sqy + sqz; // Normalised == 1, otherwise correction divisor.
    var wxyz = q.w * q.x + q.y * q.z;
    var epsilon = 1e-6; // rounding factor

    if (wxyz > (0.5 - epsilon) * unitLength) {
      _alpha = 2 * Math.atan2(q.y, q.w);
      _beta = Math.PI / 2;
      _gamma = 0;
    } else if (wxyz < (-0.5 + epsilon) * unitLength) {
      _alpha = -2 * Math.atan2(q.y, q.w);
      _beta = -Math.PI / 2;
      _gamma = 0;
    } else {
      var aX = sqw - sqx + sqy - sqz;
      var aY = 2 * (q.w * q.z - q.x * q.y);

      var gX = sqw - sqx - sqy + sqz;
      var gY = 2 * (q.w * q.y - q.x * q.z);

      if (gX > 0) {
        _alpha = Math.atan2(aY, aX);
        _beta  = Math.asin(2 * wxyz / unitLength);
        _gamma = Math.atan2(gY, gX);
      } else {
        _alpha = Math.atan2(-aY, -aX);
        _beta  = -Math.asin(2 * wxyz / unitLength);
        _beta  += _beta < 0 ? Math.PI : - Math.PI;
        _gamma = Math.atan2(-gY, -gX);
      }
    }

    // alpha is in [-pi, pi], make sure it is in [0, 2*pi).
    if (_alpha < 0) {
      _alpha += 2 * Math.PI; // alpha [0, 2*pi)
    }

    // Convert to degrees
    _alpha *= radToDeg;
    _beta  *= radToDeg;
    _gamma *= radToDeg;

   // apply derived euler angles to current object
    return new Euler(_alpha, _beta, _gamma);
  }

  rotateAxisAngle(x, y, z, angle) {
    let matrix = RotationMatrix.fromEuler(this);
    matrix.rotateAxisAngleSelf(x, y, z, angle);

    return Euler.fromRotationMatrix(matrix);
  }

  rotateAxisAngleSelf(x, y, z, angle) {
    let other = this.rotateAxisAngle(x, y, z, angle);
    this.alpha = other.alpha;
    this.beta = other.beta;
    this.gamma = other.gamma;

    return this;
  }
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

  // +++ COMPASS +++
  class Compass {
    constructor(canvasElement, headingElement, titleElement) {
      if (!canvasElement) {
        canvasElement = document.createElement('canvas');
        canvasElement.setAttribute('width', window.innerWidth);
        canvasElement.setAttribute('height', window.innerHeight);
        document.body.appendChild(canvasElement);
      }

      this.canvasElement = canvasElement;
      this.headingElement = headingElement || document.createElement('div');
      this.titleElement = titleElement;

      this.wGyro = -1;

      try {
        this.gl = create3DContext(canvasElement);
        if (!this.gl) {
          this.output('Unable to initialize WebGL. Your browser may not support it', 'http://get.webgl.org');
        } else {
          this.gl.viewportWidth = canvasElement.getAttribute('width');
          this.gl.viewportHeight = canvasElement.getAttribute('height');

          this.start();
          this.render();
        }
      }
      catch(e) {
        this.output(e);
      }
    }

    _processOptions() {
      let params = new URLSearchParams(new URL(window.location.href).search.slice(1));
      let filter = params.get("filter");

      try {
        this.accel = new Accelerometer({ frequency: 50, includesGravity: true });
        this.gyros = new Gyroscope({ frequency: 50 });
        this.magnet = new Magnetometer({ frequency: 50 });
        this.light = new AmbientLightSensor({ frequency: 50 });
      } catch (err) {
        this.output('Unable to initialize sensors: ' + err , 'https://www.w3.org/TR/generic-sensor/');
        return;
      }

      this.light.start();
      switch (filter) {
        case "m":
          this.filter = "Magnetometer";
          this.driver = this.magnet;
          this.accel.start();
          this.magnet.start();
          break;
        case "k":
          this.wGyro = -1;
          this.filter = "Kalman";
          this.driver = this.gyros;
          this.accel.start();
          this.gyros.start();
          break;
        case "a":
        case "0":
          this.wGyro = 0;
          this.filter = "Accelerometer";
          this.driver = this.accel;
          this.accel.start();
          break;
        case "g":
        case "1":
          this.wGyro = 1;
          this.filter = "Gyroscope";
          this.driver = this.gyros;
          this.gyros.start();
          break;
        case "c":
        default:
          let num = parseFloat(filter);
          if (Number.isNaN(num))
            num = 0.98;
          this.wGyro = Math.min(1, Math.max(0, num))
          this.filter = `Complementary (${this.wGyro})`;
          this.driver = this.gyros;
          this.accel.start();
          this.gyros.start();
      }
    }

    _createViewport() {
      // Create rotation matrix object (calculated per canvas draw)
      this.rotationMatrix = mat4.create();
      mat4.identity(this.rotationMatrix);

      // Create screen transform matrix (calculated once)
      this.screenMatrix = mat4.create();
      mat4.identity(this.screenMatrix);

      var inv = toRad(180);

      this.screenMatrix[0] =   Math.cos(inv);
      this.screenMatrix[1] =   Math.sin(inv);
      this.screenMatrix[4] = - Math.sin(inv);
      this.screenMatrix[5] =   Math.cos(inv);

      // Create world transformation matrix (calculated once)
      this.worldMatrix = mat4.create();
      mat4.identity(this.worldMatrix);

      var up = toRad(90);

      this.worldMatrix[5]  =   Math.cos(up);
      this.worldMatrix[6]  =   Math.sin(up);
      this.worldMatrix[9]  = - Math.sin(up);
      this.worldMatrix[10] =   Math.cos(up);

      // CompassRenderer manages 3D objects and gl surface life cycle
      this.mCompassRenderer = new CompassRenderer(this);
      this.mCompassRenderer.setCompassFilter(this.filter);

      // Catch window resize event
      window.addEventListener('orientationchange', _ => {
        window.setTimeout(() => {
          this.gl.viewportWidth = this.canvasElement.width = window.innerWidth;
          this.gl.viewportHeight = this.canvasElement.height = window.innerHeight;

          // Rescale webgl viewport
          this.gl.viewport(0, 0, this.gl.viewportWidth, this.gl.viewportHeight);

          // Recalculate perspective
          mat4.identity(this.mCompassRenderer.pMatrix);
          mat4.perspective(45, this.gl.viewportWidth / this.gl.viewportHeight, 1, 100, this.mCompassRenderer.pMatrix);
          this.gl.uniformMatrix4fv(
            this.gl.getUniformLocation(this.mCompassRenderer.shaderProgram, "uPMatrix"),
            false, this.mCompassRenderer.pMatrix
          );
        }, 200);
      }, true);

      this.gl.clearDepth(500);
      this.gl.viewport(0, 0, this.gl.viewportWidth, this.gl.viewportHeight);
    }

    start() {
      this._processOptions();
      this._createViewport();

      let kalmanY = new KalmanFilter();
      let kalmanX = new KalmanFilter();
      let kalmanZ = new KalmanFilter();

      this.alpha = 0.0;
      this.beta = 0.0;
      this.gamma = 0.0;

      let canvas = document.querySelector("canvas");

      function remap(value, inRangeStart, inRangeEnd, outRangeStart, outRangeEnd) {
        return outRangeStart + (outRangeEnd - outRangeStart) * ((value - inRangeStart) / (inRangeEnd - inRangeStart));
      };

      this.light.onchange = event => {
        let value = Math.min(Math.max(remap(this.light.reading.illuminance, 0, 100, 0, 100), 0), 100);
        canvas.style = `filter: grayscale(${value}%)`;
        console.log(`filter: grayscale(${value}%)`);
      }

      this.accel.onchange = event => {
        if (this.driver != this.accel)
          return;

        let xAccel = this.accel.reading.y / 9.81;
        let yAccel = this.accel.reading.x / 9.81;
        let zAccel = this.accel.reading.z / 9.81;

        let norm = Math.sqrt(Math.pow(xAccel, 2) + Math.pow(yAccel, 2) + Math.pow(zAccel, 2));
        this.beta = (xAccel / norm) * 180 / 2;
        this.gamma = (yAccel / norm) * -180 / 2;
        this.alpha = (zAccel / norm);
      };

      this.gyros.onchange = event => {
        if (this.driver != this.gyros)
          return;

        let xAccel = 0;
        let yAccel = 0;
        let zAccel = 0;

        if (this.wGyro != 1) {
          xAccel = this.accel.reading.y / 9.81;
          yAccel = this.accel.reading.x / 9.81;
          zAccel = this.accel.reading.z / 9.81;

          let norm = Math.sqrt(Math.pow(xAccel, 2) + Math.pow(yAccel, 2) + Math.pow(zAccel, 2));
          xAccel = (xAccel / norm) * 90;
          yAccel = (yAccel / norm) * -90;
          zAccel = (zAccel / norm);
        }

        let xGyro = this.gyros.reading.x * 180 / Math.PI;
        let yGyro = this.gyros.reading.y * 180 / Math.PI;
        let zGyro = this.gyros.reading.z * 180 / Math.PI;

        let dt = 0;
        if (this.timestamp) {
          dt = (this.gyros.reading.timeStamp - this.timestamp) / 1000;
        }
        this.timestamp = this.gyros.reading.timeStamp;

        // Kalmar filter
        if (this.wGyro < 0) {
          this.alpha = kalmanZ.filter(zAccel, zGyro, dt);
          this.beta = kalmanX.filter(xAccel, xGyro, dt);
          this.gamma = kalmanY.filter(yAccel, yGyro, dt);
        }

        // Complementary filter
        else {
          // E.g.: Current angle = 98% * (current angle + gyro rotation rate) + (2% * Accelerometer angle)
          this.alpha = this.wGyro * (this.alpha + zGyro * dt) + (1.0 - this.wGyro) * zAccel;
          this.beta = this.wGyro * (this.beta + xGyro * dt) + (1.0 - this.wGyro) * xAccel;
          this.gamma = this.wGyro * (this.gamma + yGyro * dt) + (1.0 - this.wGyro) * yAccel;
        }
      };

      this.magnet.onchange = event => {
        if (this.driver != this.magnet)
          return;

        let rotationMatrix = RotationMatrix.fromSensorData(this.accel.reading, this.magnet.reading);
        let euler = Euler.fromRotationMatrix(rotationMatrix);

        this.alpha = euler.alpha;
      };
    }

    output(str, link) {
      console.error(str);

      // Display error to user
      var outputContainer = document.createElement('div');
      outputContainer.setAttribute('class', 'output_err');
      outputContainer.appendChild(document.createTextNode(str + ". "));

      if (link) {
        var output_link = document.createElement('a');
        output_link.href = link;
        output_link.textContent = link;
        outputContainer.appendChild(output_link);
      }

      document.body.appendChild(outputContainer);
    }

    checkGLError() {
      var error = this.gl.getError();
      if (error != this.gl.NO_ERROR && error != this.gl.CONTEXT_LOST_WEBGL) {
        var str = "GL Error: " + error;
        this.output(str);
        throw str;
      }
    }

    calculateRotationMatrix() {
      var orientationMatrix = RotationMatrix.fromEuler(new Euler(this.alpha, this.beta, this.gamma));

      let screenOrientationAngle = (window.screen.orientation.angle || 0) * degToRad;
      orientationMatrix.rotateAxisAngleSelf(0, 0, 1, -screenOrientationAngle);

      // Copy 3x3 RotationMatrix values to 4x4 gl-matrix mat4
      this.rotationMatrix = orientationMatrix.toFloat32Array();

      // Invert compass heading
      mat4.multiply(this.rotationMatrix, this.screenMatrix);

      // Apply world orientation (heads-up display)
      mat4.multiply(this.rotationMatrix, this.worldMatrix);

      this.mCompassRenderer.setRotationMatrix(this.rotationMatrix);

      var euler = Euler.fromRotationMatrix(orientationMatrix);
      let value = 360 - euler.alpha;
      value = Math.floor(value < 360 ? value : value % 360);
      this.mCompassRenderer.setCompassHeading(value);
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
  };

  window.Compass = Compass;

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
          this.compass.output("Error compiling shader:\n" + infoLog);
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
        this.compass.output("Error linking program:\n" + infoLog);
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
      mat4.identity(this.pMatrix);
      mat4.perspective(45, this.gl.viewportWidth / this.gl.viewportHeight, 1, 100, this.pMatrix);
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
      mat4.identity(this.mvMatrix);
      mat4.translate(this.mvMatrix, [ 0, 0, -4 ]);

      // Apply calculated device rotation matrix
      mat4.multiply(this.mvMatrix, this.rotationMatrix);

      this.setMatrixUniforms();

      // Display compass heading
      var thisCompassHeading = this.heading;
      if (this.lastCompassHeading !== thisCompassHeading) {
        this.lastCompassHeading = thisCompassHeading;
        this.compass.headingElement.textContent = this.lastCompassHeading;
      }
      this.compass.titleElement.textContent = this.filter;

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

      this.CARDINAL_POINTS = ["N", "W", "S", "E"];

      this.mDetailsLevel = 1;
      this.mReversedRing = true;

      this.mNeedObjectsUpdate = true;
      this.mNeedTextureUpdate = true;
    }

    buildObjects() {
      this.buildRingObject();
      this.buildCapObject();
      this.buildDialObject();

      this.mNeedObjectsUpdate = false;
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

      this.mCapVertexBufferGL = this.gl.createBuffer();
      this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.mCapVertexBufferGL);
      this.gl.bufferData(this.gl.ARRAY_BUFFER, new Float32Array(vertices), this.gl.STATIC_DRAW);
      this.mCapVertexBufferGL.itemSize = 3;
      this.mCapVertexBufferGL.numItems = vertices.length / 3;

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

    buildTextures() {
      this.mTextures = new Array(2);

      this.mTextures[this.TEXTURE_RING] = this.gl.createTexture();
      this.mTextures[this.TEXTURE_DIAL] = this.gl.createTexture();

      // Initialize textures with empty 1x1 texture while real textures are loaded
      // see: http://stackoverflow.com/a/19748905
      for(var i = 0, l = this.mTextures.length; i < l; i++) {
        this.gl.bindTexture(this.gl.TEXTURE_2D, this.mTextures[i]);
        this.gl.texImage2D(this.gl.TEXTURE_2D, 0, this.gl.RGBA, 1, 1, 0, this.gl.RGBA, this.gl.UNSIGNED_BYTE,
          new Uint8Array([0, 0, 0, 255])); // initialize as black 1x1 texture
        this.gl.bindTexture(this.gl.TEXTURE_2D, null);
      }

      this.buildRingTexture();
      this.buildDialTexture();

      this.mNeedTextureUpdate = false;
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

      // ***
      var image = document.createElement('img');
      image.onload = _ => {
        this.mTextures[this.TEXTURE_RING] = this.gl.createTexture();

        this.gl.bindTexture(this.gl.TEXTURE_2D, this.mTextures[this.TEXTURE_RING]);
        this.gl.texImage2D(this.gl.TEXTURE_2D, 0, this.gl.RGBA, this.gl.RGBA, this.gl.UNSIGNED_BYTE, image);

        // see: http://stackoverflow.com/a/19748905
        if (isPowerOf2(image.width) && isPowerOf2(image.height)) {
          // the dimensions are power of 2 so generate mips and turn on
          // tri-linear filtering.
          this.gl.generateMipmap(this.gl.TEXTURE_2D);
          this.gl.texParameteri(this.gl.TEXTURE_2D, this.gl.TEXTURE_MIN_FILTER, this.gl.LINEAR_MIPMAP_LINEAR);
        } else {
          // at least one of the dimensions is not a power of 2 so set the filtering
          // so WebGL will render it.
          this.gl.texParameteri(this.gl.TEXTURE_2D, this.gl.TEXTURE_WRAP_S, this.gl.CLAMP_TO_EDGE);
          this.gl.texParameteri(this.gl.TEXTURE_2D, this.gl.TEXTURE_WRAP_T, this.gl.CLAMP_TO_EDGE);
          this.gl.texParameteri(this.gl.TEXTURE_2D, this.gl.TEXTURE_MIN_FILTER, this.gl.LINEAR);
        }

        this.gl.bindTexture(this.gl.TEXTURE_2D, null);
        this.compassrenderer.compass.checkGLError();
      };
      image.src = canvas.toDataURL();
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

      // ***
      var image = document.createElement('img');
      image.onload = _ => {
        this.mTextures[this.TEXTURE_DIAL] = this.gl.createTexture();

        this.gl.bindTexture(this.gl.TEXTURE_2D, this.mTextures[this.TEXTURE_DIAL]);
        this.gl.texImage2D(this.gl.TEXTURE_2D, 0, this.gl.RGBA, this.gl.RGBA, this.gl.UNSIGNED_BYTE, image);

        // see: http://stackoverflow.com/a/19748905
        if (isPowerOf2(image.width) && isPowerOf2(image.height)) {
          // the dimensions are power of 2 so generate mips and turn on
          // tri-linear filtering.
          this.gl.generateMipmap(this.gl.TEXTURE_2D);
          this.gl.texParameteri(this.gl.TEXTURE_2D, this.gl.TEXTURE_MIN_FILTER, this.gl.LINEAR_MIPMAP_LINEAR);
        } else {
          // at least one of the dimensions is not a power of 2 so set the filtering
          // so WebGL will render it.
          this.gl.texParameteri(this.gl.TEXTURE_2D, this.gl.TEXTURE_WRAP_S, this.gl.CLAMP_TO_EDGE);
          this.gl.texParameteri(this.gl.TEXTURE_2D, this.gl.TEXTURE_WRAP_T, this.gl.CLAMP_TO_EDGE);
          this.gl.texParameteri(this.gl.TEXTURE_2D, this.gl.TEXTURE_MIN_FILTER, this.gl.LINEAR);
        }

        this.gl.bindTexture(this.gl.TEXTURE_2D, null);
        this.compassrenderer.compass.checkGLError();
      };
      image.src = canvas.toDataURL();
    }

    draw() {
      // rebuild objects or textures if needed
      if (this.mNeedObjectsUpdate) {
        this.buildObjects();
      }

      if (this.mNeedTextureUpdate) {
        this.buildTextures();
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

      //this.gl.activeTexture(this.gl.TEXTURE0);
      this.gl.bindTexture(this.gl.TEXTURE_2D, this.mTextures[this.TEXTURE_RING]);
      //this.gl.uniform1i(this.compassrenderer.shaderProgram.shaderUniform, 0);
      this.gl.bindBuffer(this.gl.ELEMENT_ARRAY_BUFFER, this.mRingIndexBufferGL);

      this.gl.drawElements(this.gl.TRIANGLES, dx * rh * 6, this.gl.UNSIGNED_SHORT, 0);

      // Draw Dial Object
      this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.mDialVertexBufferGL);
      this.gl.vertexAttribPointer(this.gl.vertexPositionAttribute, this.mDialVertexBufferGL.itemSize, this.gl.FLOAT, false, 0, 0);

      this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.mDialTexCoordBufferGL);
      this.gl.vertexAttribPointer(this.gl.textureCoordAttribute, this.mDialTexCoordBufferGL.itemSize, this.gl.FLOAT, false, 0, 0);

      //this.gl.activeTexture(this.gl.TEXTURE1);
      this.gl.bindTexture(this.gl.TEXTURE_2D, this.mTextures[this.TEXTURE_DIAL]);
      //this.gl.uniform1i(this.compassrenderer.shaderProgram.shaderUniform, 1);
      this.gl.bindBuffer(this.gl.ELEMENT_ARRAY_BUFFER, this.mDialIndexBufferGL);

      this.gl.drawElements(this.gl.TRIANGLE_FAN, dx + 2, this.gl.UNSIGNED_SHORT, 0);

      // Disable texture for cap object
      this.gl.disableVertexAttribArray(this.gl.textureCoordAttribute);

      // Draw Cap Object
      this.gl.bindBuffer(this.gl.ARRAY_BUFFER, this.mCapVertexBufferGL);
      this.gl.vertexAttribPointer(this.gl.vertexPositionAttribute, this.mCapVertexBufferGL.itemSize, this.gl.FLOAT, false, 0, 0);

      this.gl.bindTexture(this.gl.TEXTURE_2D, null);

      this.gl.bindBuffer(this.gl.ELEMENT_ARRAY_BUFFER, this.mCapIndexBufferGL);

      this.gl.drawElements(this.gl.TRIANGLES, dx * (dy - rh) * 6, this.gl.UNSIGNED_SHORT, 0);

      this.compassrenderer.compass.checkGLError();
    }
  };
})(window);
