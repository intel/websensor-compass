/*
 * Copyright (C) 2009 The Android Open Source Project
 *               2017, 2018 Intel Corporation. All rights reserved.
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

/**
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

export class GeomagneticField {
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