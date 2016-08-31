// @license
// Copyright (c) 2018 Intel Corporation. All rights reserved.

// @ts-check
import * as glMatrix from "../node_modules/gl-matrix/src/gl-matrix/common.js";
import * as mat4 from "../node_modules/gl-matrix/src/gl-matrix/mat4.js";

const degToRad = Math.PI / 180;
const radToDeg = 180 / Math.PI;

export function create() {
  let out = new glMatrix.ARRAY_TYPE(3);
  out[0] = 0;
  out[1] = 0;
  out[2] = 0;
  return out;
};

export function clone(a) {
  const out = new glMatrix.ARRAY_TYPE(3);
  out[0] = a[0];
  out[1] = a[1];
  out[2] = a[2];
  return out;
};

export function toMat4(out, a) {
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

export function fromMat4(out, a) {
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