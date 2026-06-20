package org.gogpsproject.positioning;

import org.ejml.simple.SimpleMatrix;
import org.gogpsproject.Constants;
import org.gogpsproject.GoGPS;
import org.gogpsproject.GoGPS.WeightingStrategy;
import org.gogpsproject.RtkLibConstants;

//import com.google.maps.ElevationApi;
//import com.google.maps.GeoApiContext;
//import com.google.maps.model.ElevationResult;
//import com.google.maps.model.LatLng;

public abstract class Core {

  GoGPS goGPS;
  RoverPosition rover;
  MasterPosition master;
  Satellites sats;
  
  /** Covariance matrix of the position estimation error */
  SimpleMatrix positionCovariance; 

//static GeoApiContext context;
//static GeoApiContext getContext(){
//  if( context == null )
//    context = new GeoApiContext().setApiKey("...Add your key here");
//  return context;
//}
  public Core( GoGPS goGPS) {
    this.goGPS  = goGPS;
    this.rover  = goGPS.getRoverPos();
    this.master = goGPS.getMasterPos();
    this.sats   = goGPS.getSats();
  }

  /**
   * @param elevation
   * @param snr
   * @return weight computed according to the variable "goGPS.weights"
   */
  double computeWeight(double elevation, float snr) {

    double weight = 1;
    float Sa = Constants.SNR_a;
    float SA = Constants.SNR_A;
    float S0 = Constants.SNR_0;
    float S1 = Constants.SNR_1;

    if (Float.isNaN(snr) && (goGPS.getWeights() == WeightingStrategy.SIGNAL_TO_NOISE_RATIO ||
        goGPS.getWeights() == WeightingStrategy.COMBINED_ELEVATION_SNR)) {
      if(goGPS.isDebug()) System.out.println("SNR not available: forcing satellite elevation-based weights...");
      goGPS.setWeights(WeightingStrategy.SAT_ELEVATION);
    }

    switch (goGPS.getWeights()) {

      // Weight based on satellite elevation
      case SAT_ELEVATION:
        weight = 1 / Math.pow(Math.sin(elevation * Math.PI / 180), 2);
        break;

      // Weight based on signal-to-noise ratio
      case SIGNAL_TO_NOISE_RATIO:
        if (snr >= S1) {
          weight = 1;
        } else {
          weight = Math.pow(10, -(snr - S1) / Sa)
              * ((SA / Math.pow(10, -(S0 - S1) / Sa) - 1) / (S0 - S1)
                  * (snr - S1) + 1);
        }
        break;

      // Weight based on combined elevation and signal-to-noise ratio
      case COMBINED_ELEVATION_SNR:
        if (snr >= S1) {
          weight = 1;
        } else {
          double weightEl = 1 / Math.pow(Math.sin(elevation * Math.PI / 180), 2);
          double weightSnr = Math.pow(10, -(snr - S1) / Sa)
              * ((SA / Math.pow(10, -(S0 - S1) / Sa) - 1) / (S0 - S1) * (snr - S1) + 1);
          weight = weightEl * weightSnr;
        }
        break;

      // Same weight for all observations or default
      case EQUAL:
      default:
        weight = 1;
    }

    return weight;
  }

  /**
   * Compute measurement error variance (RTKLIB-aligned varerr).
   * Reference: RTKLIB rtkpos.c varerr()
   *
   * RTKLIB formula:
   *   fact = eratio[f-nf] * EFACT_sys   (eratio defaults to 1.0)
   *   a = fact * err[1],  b = fact * err[2]
   *   c = err[3] * bl / 1E4,  d = CLIGHT * sclkstab * dt
   *   var = 2.0 * (ionoopt==IFLC?3:1) * (a² + b²/sin²(el) + c²) + d²
   *
   * Note: fact is squared inside the expression (a = fact*err, so a² = fact²*err²).
   * This is important for non-GPS systems (e.g., GLO fact=1.5 → fact²=2.25).
   *
   * @param el      elevation angle (radians)
   * @param isPhase true for carrier-phase, false for pseudorange
   * @param satType satellite system type ('G','R','E','C','J')
   * @param freq    frequency index (0:L1,1:L2,2:L5)
   * @return measurement error variance (m²)
   */
  public static double varerr(double el, boolean isPhase, char satType, int freq) {
    return varerr(el, isPhase, satType, freq, 0.0, 0.0);
  }

  /**
   * Compute measurement error variance with baseline length and time-diff terms (RTKLIB-aligned).
   * Reference: RTKLIB rtkpos.c varerr()
   *
   * @param el      elevation angle (radians)
   * @param isPhase true for carrier-phase, false for pseudorange
   * @param satType satellite system type ('G','R','E','C','J')
   * @param freq    frequency index (0:L1,1:L2,2:L5)
   * @param bl      baseline length (m), 0 for unknown
   * @param dt      time difference between epochs (s), 0 for unknown
   * @return measurement error variance (m²)
   */
  public static double varerr(double el, boolean isPhase, char satType, int freq, double bl, double dt) {
    double sinel = Math.sin(el);
    if (sinel <= 0.0) sinel = 0.01;

    double a, b;
    if (isPhase) {
      a = 0.003; // carrier-phase error factor err[1] (m), RTKLIB default
      b = 0.003; // carrier-phase error factor err[2] (m), RTKLIB default
    } else {
      a = 0.3; // code error factor err[1] (m), RTKLIB default
      b = 0.3; // code error factor err[2] (m), RTKLIB default
    }

    // System-dependent error factor (RTKLIB: EFACT_GPS=1.0, EFACT_GLO=1.5, EFACT_SBS=3.0)
    double fact = RtkLibConstants.errorFactorForSatType(satType);

    // Frequency-dependent error ratio (RTKLIB eratio[f-nf], default 1.0 for all frequencies)
    // TODO: make eratio configurable per frequency (e.g., L5 lower noise)
    // For now, all frequencies use the same ratio (1.0), matching RTKLIB defaults.
    // double eratio = (freq >= 1) ? goGPS.getErario(freq) : 1.0;
    // if (eratio <= 0) eratio = 1.0;
    // fact *= eratio;

    // RTKLIB: a = fact * err[1], b = fact * err[2]
    // Then var = 2.0 * (a² + b²/sin² + c²) + d²
    // This means fact is squared: fact² * err[1]²
    a = fact * a;
    b = fact * b;

    // Baseline-dependent error term: c = err[3] * bl / 1E4 (RTKLIB default err[3]=0)
    // c is NOT multiplied by fact (RTKLIB: c = opt->err[3]*bl/1E4, no fact)
    double errBaseline = 0.0;
    double c = errBaseline * bl / 1E4;

    // Clock stability error term: d = CLIGHT * sclkstab * dt
    double sclkstab = 5e-12; // RTKLIB default sclkstab
    double d = RtkLibConstants.CLIGHT * sclkstab * dt;

    // IFLC (iono-free LC) factor: 3.0 when using iono-free combination
    // TODO: pass ionoopt to enable IFLC noise amplification
    // double iflcFactor = (ionoopt == IONOOPT_IFLC) ? 3.0 : 1.0;

    return 2.0 * (a * a + b * b / (sinel * sinel) + c * c) + d * d;
  }
  
  void updateDops( SimpleMatrix A ){
    // Compute covariance matrix from A matrix [ECEF reference system]
    SimpleMatrix covXYZ = A.transpose().mult(A).invert().extractMatrix(0, 3, 0, 3);

    // Allocate and build rotation matrix
    SimpleMatrix R = Coordinates.rotationMatrix(rover);
 
    /** Covariance matrix obtained from matrix A (satellite geometry) [local coordinates] */
    // Propagate covariance from global system to local system
    SimpleMatrix covENU = R.mult(covXYZ).mult(R.transpose());

     //Compute DOP values
    rover.pDop = Math.sqrt(covXYZ.get(0, 0) + covXYZ.get(1, 1) + covXYZ.get(2, 2));
    rover.hDop = Math.sqrt(covENU.get(0, 0) + covENU.get(1, 1));
    rover.vDop = Math.sqrt(covENU.get(2, 2));
  }
}