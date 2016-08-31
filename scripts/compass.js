/*
 * Marine Compass
 * http://github.com/richtr/Marine-Compass
 *
 * Copyright (c) 2012-2014, Rich Tibbett
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

var M_PI   = Math.PI;
var M_PI_2 = M_PI / 2;
var M_2_PI = 2 * M_PI;

var degToRad = M_PI / 180;
var radToDeg = 180 / M_PI;

class RotationMatrix {
  constructor(m11, m12, m13, m21, m22, m23, m31, m32, m33) {
	var outMatrix;

	this.elements = new Float32Array(9);
	this.set(m11, m12, m13, m21, m22, m23, m31, m32, m33);
  }

  set(m11, m12, m13, m21, m22, m23, m31, m32, m33) {
	this.elements[ 0 ] = m11 || 1;
	this.elements[ 1 ] = m12 || 0;
	this.elements[ 2 ] = m13 || 0;
	this.elements[ 3 ] = m21 || 0;
	this.elements[ 4 ] = m22 || 1;
	this.elements[ 5 ] = m23 || 0;
	this.elements[ 6 ] = m31 || 0;
	this.elements[ 7 ] = m32 || 0;
	this.elements[ 8 ] = m33 || 1;
  }

  copy(matrix) {
	this.elements[ 0 ] = matrix.elements[ 0 ];
	this.elements[ 1 ] = matrix.elements[ 1 ];
	this.elements[ 2 ] = matrix.elements[ 2 ];
	this.elements[ 3 ] = matrix.elements[ 3 ];
	this.elements[ 4 ] = matrix.elements[ 4 ];
	this.elements[ 5 ] = matrix.elements[ 5 ];
	this.elements[ 6 ] = matrix.elements[ 6 ];
	this.elements[ 7 ] = matrix.elements[ 7 ];
	this.elements[ 8 ] = matrix.elements[ 8 ];
  }

  identity() {
    this.set(1, 0, 0,
			 0, 1, 0,
			 0, 0, 1);

	return this;
  }

  setFromEuler(euler) {
	let _x, _y, _z;
	let cX, cY, cZ, sX, sY, sZ;

	euler = euler || {};

	_z = ( euler.alpha || 0 ) * degToRad;
	_x = ( euler.beta || 0 ) * degToRad;
	_y = ( euler.gamma || 0 ) * degToRad;

	cX = Math.cos( _x );
	cY = Math.cos( _y );
	cZ = Math.cos( _z );
	sX = Math.sin( _x );
	sY = Math.sin( _y );
	sZ = Math.sin( _z );

	//
	// ZXY-ordered rotation matrix construction.
	//

	this.set(
	  cZ * cY - sZ * sX * sY, // 1,1
	  - cX * sZ,              // 1,2
	  cY * sZ * sX + cZ * sY, // 1,3

	  cY * sZ + cZ * sX * sY, // 2,1
	  cZ * cX,                // 2,2
	  sZ * sY - cZ * cY * sX, // 2,3

	  - cX * sY,              // 3,1
	  sX,                     // 3,2
	  cX * cY                 // 3,3
	);

	this.normalize();
	return this;
  }

  setFromQuaternion(q) {
    let sqw, sqx, sqy, sqz;

    sqw = q.w * q.w;
	sqx = q.x * q.x;
	sqy = q.y * q.y;
	sqz = q.z * q.z;

	this.set(
  	  sqw + sqx - sqy - sqz,       // 1,1
	  2 * (q.x * q.y - q.w * q.z), // 1,2
	  2 * (q.x * q.z + q.w * q.y), // 1,3

	  2 * (q.x * q.y + q.w * q.z), // 2,1
	  sqw - sqx + sqy - sqz,       // 2,2
	  2 * (q.y * q.z - q.w * q.x), // 2,3

	  2 * (q.x * q.z - q.w * q.y), // 3,1
	  2 * (q.y * q.z + q.w * q.x), // 3,2
	  sqw - sqx - sqy + sqz        // 3,3
	);

	return this;
  }

  static multiplyMatrices(a, b) {
	let matrix = new RotationMatrix();

	let aE, bE;

	aE = a.elements;
	bE = b.elements;

	matrix.set(
	  aE[0] * bE[0] + aE[1] * bE[3] + aE[2] * bE[6],
	  aE[0] * bE[1] + aE[1] * bE[4] + aE[2] * bE[7],
	  aE[0] * bE[2] + aE[1] * bE[5] + aE[2] * bE[8],

	  aE[3] * bE[0] + aE[4] * bE[3] + aE[5] * bE[6],
  	  aE[3] * bE[1] + aE[4] * bE[4] + aE[5] * bE[7],
	  aE[3] * bE[2] + aE[4] * bE[5] + aE[5] * bE[8],

	  aE[6] * bE[0] + aE[7] * bE[3] + aE[8] * bE[6],
	  aE[6] * bE[1] + aE[7] * bE[4] + aE[8] * bE[7],
	  aE[6] * bE[2] + aE[7] * bE[5] + aE[8] * bE[8]
	);

	return matrix;
  }

  static normalize(matrix) {
	let R = matrix.elements;

	// Calculate matrix determinant
	let determinant = R[0] * R[4] * R[8] - R[0] * R[5] * R[7] - R[1] * R[3] * R[8] + R[1] * R[5] * R[6] + R[2] * R[3] * R[7] - R[2] * R[4] * R[6];

	// Normalize matrix values
	R[0] /= determinant;
	R[1] /= determinant;
	R[2] /= determinant;
	R[3] /= determinant;
	R[4] /= determinant;
	R[5] /= determinant;
	R[6] /= determinant;
	R[7] /= determinant;
	R[8] /= determinant;

	matrix.elements = R;

	return matrix;
  }

  static rotateByAxisAngle(targetRotationMatrix, axis, angle) {
	let outputMatrix = new RotationMatrix();
	let transformMatrix = new RotationMatrix();

	let sA, cA;
	let validAxis = false;

	transformMatrix.identity(); // reset transform matrix

	validAxis = false;

	sA = Math.sin( angle );
	cA = Math.cos( angle );

	if ( axis[ 0 ] === 1 && axis[ 1 ] === 0 && axis[ 2 ] === 0 ) { // x
	  validAxis = true;

	  transformMatrix.elements[4] = cA;
	  transformMatrix.elements[5] = -sA;
	  transformMatrix.elements[7] = sA;
	  transformMatrix.elements[8] = cA;
	} else if ( axis[ 1 ] === 1 && axis[ 0 ] === 0 && axis[ 2 ] === 0 ) { // y
      validAxis = true;

	  transformMatrix.elements[0] = cA;
	  transformMatrix.elements[2] = sA;
	  transformMatrix.elements[6] = -sA;
	  transformMatrix.elements[8] = cA;
	} else if ( axis[ 2 ] === 1 && axis[ 0 ] === 0 && axis[ 1 ] === 0 ) { // z
	  validAxis = true;

	  transformMatrix.elements[0] = cA;
	  transformMatrix.elements[1] = -sA;
	  transformMatrix.elements[3] = sA;
	  transformMatrix.elements[4] = cA;
    }

	if ( validAxis ) {
	  outputMatrix = this.multiplyMatrices( targetRotationMatrix, transformMatrix );
      outputMatrix = this.normalize( outputMatrix );
    } else {
      outputMatrix = targetRotationMatrix;
    }

    return outputMatrix;
  }

  multiply(m) {
	let outMatrix = this.multiplyMatrices( this, m );
	this.copy( outMatrix );

	return this;
  }

  rotateX(angle) {
	let outMatrix = RotationMatrix.rotateByAxisAngle( this, [ 1, 0, 0 ], angle );
	this.copy( outMatrix );

	return this;
  };

  rotateY(angle) {
    let outMatrix = RotationMatrix.rotateByAxisAngle( this, [ 0, 1, 0 ], angle );
	this.copy( outMatrix );

	return this;
  }

  rotateZ(angle) {
	let outMatrix = RotationMatrix.rotateByAxisAngle( this, [ 0, 0, 1 ], angle );
	this.copy( outMatrix );

	return this;
  }

  normalize() {
	return RotationMatrix.normalize( this );
  }
};



class Euler{
  constructor(alpha, beta, gamma) {
	this.set( alpha, beta, gamma );
  }

  set(alpha, beta, gamma) {
	this.alpha = alpha || 0;
	this.beta  = beta  || 0;
	this.gamma = gamma || 0;
  }

  copy(inEuler) {
	this.alpha = inEuler.alpha;
	this.beta  = inEuler.beta;
	this.gamma = inEuler.gamma;
  }

  setFromRotationMatrix(matrix) {
    let R, _alpha, _beta, _gamma;

	R = matrix.elements;

	if (R[8] > 0) { // cos(beta) > 0

		_alpha = Math.atan2(-R[1], R[4]);
		_beta  = Math.asin(R[7]); // beta (-pi/2, pi/2)
		_gamma = Math.atan2(-R[6], R[8]); // gamma (-pi/2, pi/2)

	} else if (R[8] < 0) {  // cos(beta) < 0

		_alpha = Math.atan2(R[1], -R[4]);
		_beta  = -Math.asin(R[7]);
		_beta  += (_beta >= 0) ? - M_PI : M_PI; // beta [-pi,-pi/2) U (pi/2,pi)
		_gamma = Math.atan2(R[6], -R[8]); // gamma (-pi/2, pi/2)

	} else { // R[8] == 0
	  if (R[6] > 0) {  // cos(gamma) == 0, cos(beta) > 0
			_alpha = Math.atan2(-R[1], R[4]);
			_beta  = Math.asin(R[7]); // beta [-pi/2, pi/2]
			_gamma = - M_PI_2; // gamma = -pi/2
	  } else if (R[6] < 0) { // cos(gamma) == 0, cos(beta) < 0
			_alpha = Math.atan2(R[1], -R[4]);
			_beta  = -Math.asin(R[7]);
			_beta  += (_beta >= 0) ? - M_PI : M_PI; // beta [-pi,-pi/2) U (pi/2,pi)
			_gamma = - M_PI_2; // gamma = -pi/2
	  } else { // R[6] == 0, cos(beta) == 0
			// gimbal lock discontinuity
			_alpha = Math.atan2(R[3], R[0]);
			_beta  = (R[7] > 0) ? M_PI_2 : - M_PI_2; // beta = +-pi/2
			_gamma = 0; // gamma = 0
	  }
	}

	// alpha is in [-pi, pi], make sure it is in [0, 2*pi).
	if (_alpha < 0) {
	  _alpha += M_2_PI; // alpha [0, 2*pi)
	}

	// Convert to degrees
	_alpha *= radToDeg;
	_beta  *= radToDeg;
	_gamma *= radToDeg;

	// apply derived euler angles to current object
	this.set(_alpha, _beta, _gamma);
  }

  setFromQuaternion(q) {
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
				_beta = M_PI_2;
				_gamma = 0;

			} else if (wxyz < (-0.5 + epsilon) * unitLength) {

				_alpha = -2 * Math.atan2(q.y, q.w);
				_beta = -M_PI_2;
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
					_beta  += _beta < 0 ? M_PI : - M_PI;
					_gamma = Math.atan2(-gY, -gX);

				}

			}

			// alpha is in [-pi, pi], make sure it is in [0, 2*pi).
			if (_alpha < 0) {
				_alpha += M_2_PI; // alpha [0, 2*pi)
			}

			// Convert to degrees
			_alpha *= radToDeg;
			_beta  *= radToDeg;
			_gamma *= radToDeg;

			// apply derived euler angles to current object
	this.set( _alpha, _beta, _gamma );
  }

  rotateX(angle) {
	Euler.rotateByAxisAngle( this, [ 1, 0, 0 ], angle );
	return this;
  }

  rotateY(angle) {
    Euler.rotateByAxisAngle( this, [ 0, 1, 0 ], angle );
	return this;
  }

  rotateZ(angle) {
	Euler.rotateByAxisAngle( this, [ 0, 0, 1 ], angle );
	return this;
  }

  static rotateByAxisAngle(targetEuler, axis, angle) {
    let _matrix = new RotationMatrix();
	let outEuler;

	_matrix.setFromEuler( targetEuler );
	_matrix = RotationMatrix.rotateByAxisAngle( _matrix, axis, angle );

	targetEuler.setFromRotationMatrix( _matrix );

	return targetEuler;
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

    var HALF_PI = Math.PI / 2,
        TWO_PI = Math.PI * 2;

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
    window.Compass = function(canvasElement, headingElement) {

        if (!canvasElement) {
            canvasElement = document.createElement('canvas');
            canvasElement.setAttribute('width', window.innerWidth);
            canvasElement.setAttribute('height', window.innerHeight);
            document.body.appendChild(canvasElement);
        }

        this.canvasElement = canvasElement;
        this.headingElement = headingElement || document.createElement('div');

        try {
            this.gl = create3DContext(canvasElement);
            if(!this.gl) {
              this.output('Unable to initialize WebGL. Your browser may not support it', 'http://get.webgl.org');
            } else {
              this.gl.viewportWidth = canvasElement.getAttribute('width');
              this.gl.viewportHeight = canvasElement.getAttribute('height');

              this.init();

              this.render();
            }
        }
        catch(e) {
          console.log(e);
          this.output('Unable to initialize WebGL. Your browser may not support it', 'http://get.webgl.org');
        }

    };

    window.Compass.prototype = {

        constructor: window.Compass,

        init: function() {

            var self = this;

            self.accel = new AccelerometerSensor({ frequency: 60, includesGravity: true });
            self.gyros = new GyroscopeSensor({ frequency: 60 });
            self.accel.start();
            self.gyros.start();

            let xGyro = this.gyros.reading.rotationRateX;      
            let yGyro = this.gyros.reading.rotationRateY;
            let zGyro = this.gyros.reading.rotationRateZ;

            let timestamp = this.gyros.reading.timeStamp;

            self.gyros.onchange = event => {
              // http://www.instructables.com/id/Accelerometer-Gyro-Tutorial/step3/Combining-the-Accelerometer-and-Gyro/
              let gyroTrust = 0.98;

              let xAccel = this.accel.reading.accelerationX;
              let yAccel = this.accel.reading.accelerationY;
              let zAccel = this.accel.reading.accelerationZ;

              let dt = (this.gyros.reading.timeStamp - timestamp) / 10000;
              timestamp = this.gyros.reading.timeStamp;

              xGyro = xGyro + this.gyros.reading.rotationRateX * dt;
              yGyro = yGyro + this.gyros.reading.rotationRateY * dt;
              zGyro = zGyro + this.gyros.reading.rotationRateZ * dt;

              self.alpha = zGyro * gyroTrust + zAccel * (1 - gyroTrust);
              self.beta = xGyro * gyroTrust + xAccel * (1 - gyroTrust);
              self.gamma = yGyro * gyroTrust + yAccel * (1 - gyroTrust);
            }

            // Create rotation matrix object (calculated per canvas draw)
            this.rotationMatrix = mat4.create();
            mat4.identity(this.rotationMatrix);

            // Create screen transform matrix (calculated once)
            this.screenMatrix = mat4.create();
            mat4.identity(this.screenMatrix);

            var inv = toRad(180);

            this.screenMatrix[0] =   Math.cos( inv );
            this.screenMatrix[1] =   Math.sin( inv );
            this.screenMatrix[4] = - Math.sin( inv );
            this.screenMatrix[5] =   Math.cos( inv );

            // Create world transformation matrix (calculated once)
            this.worldMatrix = mat4.create();
            mat4.identity(this.worldMatrix);

            var up = toRad(90);

            this.worldMatrix[5]  =   Math.cos( up );
            this.worldMatrix[6]  =   Math.sin( up );
            this.worldMatrix[9]  = - Math.sin( up );
            this.worldMatrix[10] =   Math.cos( up );

            // CompassRenderer manages 3D objects and gl surface life cycle
            this.mCompassRenderer = new CompassRenderer(this);

            // Catch window resize event
            window.addEventListener('orientationchange', function() {

              window.setTimeout(function() {

                self.gl.viewportWidth = self.canvasElement.width = window.innerWidth;
                self.gl.viewportHeight = self.canvasElement.height = window.innerHeight;

                // Rescale webgl viewport
                self.gl.viewport(0, 0, self.gl.viewportWidth, self.gl.viewportHeight);

                // Recalculate perspective

                mat4.identity(self.mCompassRenderer.pMatrix);
                mat4.perspective(45, self.gl.viewportWidth / self.gl.viewportHeight, 1, 100, self.mCompassRenderer.pMatrix);
                self.gl.uniformMatrix4fv(
                self.gl.getUniformLocation(self.mCompassRenderer.shaderProgram, "uPMatrix"),
                false, self.mCompassRenderer.pMatrix
                );

              }, 200);

            }, true);

            this.gl.clearDepth(500);

            this.gl.viewport(0, 0, this.gl.viewportWidth, this.gl.viewportHeight);

        },

        output: function(str, link) {
          // Display error to developer
          console.error(str);

          // Display error to user
          var outputContainer = document.createElement('div');
          outputContainer.setAttribute('class', 'output_err');
          outputContainer.appendChild(document.createTextNode(str + ". "));

          if(link) {
            var output_link = document.createElement('a');
            output_link.href = link;
            output_link.textContent = link;
            outputContainer.appendChild(output_link);
          }

          document.body.appendChild(outputContainer);
        },

        checkGLError: function() {
            var error = this.gl.getError();
            if (error != this.gl.NO_ERROR && error != this.gl.CONTEXT_LOST_WEBGL) {
                var str = "GL Error: " + error;
                this.output(str);
                throw str;
            }
        },

    	calculateRotationMatrix: function() {           
            let euler = new Euler();
            euler.set(
			  this.alpha,
			  this.beta,
			  this.gamma
			);

            let orientationMatrix = new RotationMatrix();
			orientationMatrix.setFromEuler(euler);

            // Copy 3x3 RotationMatrix values to 4x4 gl-matrix mat4
            this.rotationMatrix[0] = orientationMatrix.elements[0];
            this.rotationMatrix[1] = orientationMatrix.elements[1];
            this.rotationMatrix[2] = orientationMatrix.elements[2];
            this.rotationMatrix[4] = orientationMatrix.elements[3];
            this.rotationMatrix[5] = orientationMatrix.elements[4];
            this.rotationMatrix[6] = orientationMatrix.elements[5];
            this.rotationMatrix[8] = orientationMatrix.elements[6];
            this.rotationMatrix[9] = orientationMatrix.elements[7];
            this.rotationMatrix[10] = orientationMatrix.elements[8];

            // Invert compass heading
            mat4.multiply(this.rotationMatrix, this.screenMatrix);

            // Apply world orientation (heads-up display)
            mat4.multiply(this.rotationMatrix, this.worldMatrix);

            this.mCompassRenderer.setRotationMatrix(this.rotationMatrix);

            euler.setFromRotationMatrix(orientationMatrix);
            this.mCompassRenderer.setCompassHeading(360 - euler.alpha);

    	},

        render: function() {
            // Update orientation buffer
            this.calculateRotationMatrix();

            // Draw frame
            this.mCompassRenderer.draw();

            // Re-render at next key frame
            // see: http://stackoverflow.com/questions/6065169/requestanimationframe-with-this-keyword
            window.requestAnimationFrame(this.render.bind(this));
        }

    };

    // +++ COMPASSRENDERER +++
    var CompassRenderer = function(compass) {

        this.compass = compass;
        this.gl = this.compass.gl;

        this.pMatrix = mat4.create();
        this.mvMatrix = mat4.create();
        this.rotationMatrix = mat4.create();

        this.heading = 0;

        this.init();

        this.mTurntable = new Turntable(this);

    };

    CompassRenderer.prototype = {
        constructor: CompassRenderer,

        loadShader: function(type, shaderSrc) {
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
        },

        init: function() {
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

        },

        setMatrixUniforms: function() {
            this.gl.uniformMatrix4fv(this.shaderProgram.mvMatrixUniform, false, this.mvMatrix);
        },

        setRotationMatrix: function(matrix) {
            this.rotationMatrix = matrix;
        },

        setCompassHeading: function(heading) {
            this.heading = heading < 360 ? heading : heading % 360;
        },

        lastCompassHeading: 0,

        draw: function() {
            // Clear the canvas
            this.gl.clear(this.gl.COLOR_BUFFER_BIT | this.gl.DEPTH_BUFFER_BIT);

            // Reset move matrix
            mat4.identity(this.mvMatrix);
            mat4.translate(this.mvMatrix, [ 0, 0, -4 ]);

            // Apply calculated device rotation matrix
            mat4.multiply(this.mvMatrix, this.rotationMatrix);

            this.setMatrixUniforms();

            // Display compass heading
            var thisCompassHeading = Math.floor(this.heading);
            if(this.lastCompassHeading !== thisCompassHeading) {
              this.compass.headingElement.textContent = thisCompassHeading;
              this.lastCompassHeading = thisCompassHeading;
            }

            // ***
            this.mTurntable.draw();
        }

    };

    // +++ TURNTABLE +++
    var Turntable = function(compassrenderer) {

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

    };

    Turntable.prototype = {
        constructor: Turntable,

        buildObjects: function() {
            this.buildRingObject();
            this.buildCapObject();
            this.buildDialObject();

            this.mNeedObjectsUpdate = false;
        },

        buildRingObject: function() {

            // build vertices
            var dx = this.DETAIL_X[this.mDetailsLevel];
            var dy = this.DETAIL_Y[this.mDetailsLevel];
            var rh = this.RING_HEIGHT[this.mDetailsLevel];

            var vertices = new Array(((dx + 1) * (rh + 1)) * 3);
            var normals = new Array(((dx + 1) * (rh + 1)) * 3);

            var n = 0;

            for (var i = 0; i <= dx; i++) {
                for (var j = 0; j <= rh; j++) {
                    var a = i * TWO_PI / dx;
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

        },

        buildCapObject: function() {

            var dx = this.DETAIL_X[this.mDetailsLevel];
            var dy = this.DETAIL_Y[this.mDetailsLevel];
            var rh = this.RING_HEIGHT[this.mDetailsLevel];

            var h = dy - rh;

            // build vertices
            var vertices = new Array(((dx + 1) * (h + 1)) * 3);

            var n = 0;
            for (var i = 0; i <= dx; i++) {
                for (var j = rh; j <= dy; j++) {
                    var a = i * TWO_PI / dx;
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

        },

        buildDialObject: function() {

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
                var a = i * TWO_PI / dx;

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
                var a = i * TWO_PI / dx;

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
        },

        buildTextures: function() {
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
        },

        buildRingTexture: function() {

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
            var self = this;
            image.onload = function() {

                self.mTextures[self.TEXTURE_RING] = self.gl.createTexture();

                self.gl.bindTexture(self.gl.TEXTURE_2D, self.mTextures[self.TEXTURE_RING]);

                self.gl.texImage2D(self.gl.TEXTURE_2D, 0, self.gl.RGBA, self.gl.RGBA, self.gl.UNSIGNED_BYTE, image);

                // see: http://stackoverflow.com/a/19748905
                if (isPowerOf2(image.width) && isPowerOf2(image.height)) {
                  // the dimensions are power of 2 so generate mips and turn on
                  // tri-linear filtering.
                  self.gl.generateMipmap(self.gl.TEXTURE_2D);
                  self.gl.texParameteri(self.gl.TEXTURE_2D, self.gl.TEXTURE_MIN_FILTER, self.gl.LINEAR_MIPMAP_LINEAR);
                } else {
                  // at least one of the dimensions is not a power of 2 so set the filtering
                  // so WebGL will render it.
                  self.gl.texParameteri(self.gl.TEXTURE_2D, self.gl.TEXTURE_WRAP_S, self.gl.CLAMP_TO_EDGE);
                  self.gl.texParameteri(self.gl.TEXTURE_2D, self.gl.TEXTURE_WRAP_T, self.gl.CLAMP_TO_EDGE);
                  self.gl.texParameteri(self.gl.TEXTURE_2D, self.gl.TEXTURE_MIN_FILTER, self.gl.LINEAR);
                }

                self.gl.bindTexture(self.gl.TEXTURE_2D, null);

                self.compassrenderer.compass.checkGLError();

            };
            image.src = canvas.toDataURL();

        },

        buildDialTexture: function() {

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
            context.arc(radius, radius, radius - 10, 0, TWO_PI);
            context.stroke();
            //context.fill();
            context.closePath();

            // build the inner decoration, using two symmetrical paths
            context.save();
            for (var i = 0; i < 4; i++) {

                context.translate(radius, radius);
                context.rotate(i * HALF_PI);
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
                    var a = -i * TWO_PI / 360;
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
                var a = i * TWO_PI / 360;
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
            var self = this;
            image.onload = function() {

              self.mTextures[self.TEXTURE_DIAL] = self.gl.createTexture();

              self.gl.bindTexture(self.gl.TEXTURE_2D, self.mTextures[self.TEXTURE_DIAL]);

              self.gl.texImage2D(self.gl.TEXTURE_2D, 0, self.gl.RGBA, self.gl.RGBA, self.gl.UNSIGNED_BYTE, image);

              // see: http://stackoverflow.com/a/19748905
              if (isPowerOf2(image.width) && isPowerOf2(image.height)) {
                // the dimensions are power of 2 so generate mips and turn on
                // tri-linear filtering.
                self.gl.generateMipmap(self.gl.TEXTURE_2D);
                self.gl.texParameteri(self.gl.TEXTURE_2D, self.gl.TEXTURE_MIN_FILTER, self.gl.LINEAR_MIPMAP_LINEAR);
              } else {
                // at least one of the dimensions is not a power of 2 so set the filtering
                // so WebGL will render it.
                self.gl.texParameteri(self.gl.TEXTURE_2D, self.gl.TEXTURE_WRAP_S, self.gl.CLAMP_TO_EDGE);
                self.gl.texParameteri(self.gl.TEXTURE_2D, self.gl.TEXTURE_WRAP_T, self.gl.CLAMP_TO_EDGE);
                self.gl.texParameteri(self.gl.TEXTURE_2D, self.gl.TEXTURE_MIN_FILTER, self.gl.LINEAR);
              }

              self.gl.bindTexture(self.gl.TEXTURE_2D, null);

              self.compassrenderer.compass.checkGLError();

            };
            image.src = canvas.toDataURL();

        },

        draw: function() {

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
