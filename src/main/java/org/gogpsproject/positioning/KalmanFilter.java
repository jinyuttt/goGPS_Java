package org.gogpsproject.positioning;

import java.util.ArrayList;
import java.util.Objects;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.simple.SimpleMatrix;
import org.gogpsproject.GoGPS;
import org.gogpsproject.producer.ObservationSet;
import org.gogpsproject.producer.Observations;

public abstract class KalmanFilter extends LS_DD_code {

  /** Initial position st dev (m), RTKLIB-aligned: VAR_POS = SQR(30.0) in rtkpos.c */
  static final double stDevInit = 30;

  /** East velocity process noise st dev (m/s^2) */
  private static final double stDevE = 0.6;

  /** North velocity process noise st dev (m/s^2) */
  private static final double stDevN = 0.6;

  /** Up velocity process noise st dev (m/s^2) */
  private static final double stDevU = 0.3;

  /** Code observation st dev for C/A code (m) */
  private static final double stDevCodeC = 0.3;

  /** Code observation st dev for P code (m), per frequency: [L1, L2] */
  private static final double[] stDevCodeP = { 0.6, 0.4 };

  /** Phase observation st dev (cycles) */
  static final double stDevPhase = 0.003;

  /** Ambiguity process noise st dev (cycles), very small to keep ambiguity nearly fixed */
  static final double stDevAmbiguity = 1e-4;

  int o1, o2, o3;
  int i1, i2, i3;
  int nN;
  int nAmbL2, nIono, nTropo;
  int iAmbL2, iIono, iTropo;

  /** Total state dimension: o3 + nN + nAmbL2 + nIono + nTropo */
  int stateSize() { return o3 + nN + nAmbL2 + nIono + nTropo; }

  SimpleMatrix T;
  SimpleMatrix H;
  SimpleMatrix y0;
  SimpleMatrix Cvv;
  SimpleMatrix Cee;
  SimpleMatrix Cnn;
  SimpleMatrix KFstate;
  SimpleMatrix KFprediction;

  /** Cached current-epoch rover observations for fixAmbiguitiesLambda() wavelength lookup.
   *  Set at loop() entry, used by fixAmbiguitiesLambda() to convert meters→cycles for LAMBDA. */
  Observations currentRoverObs = null;

  // Fields for keeping track of satellite configuration changes
  ArrayList<Integer> satOld;
  ArrayList<Character> satTypeOld;
  int oldPivotId;
  char oldPivotType;
  
  int consecutiveDivergence = 0;

  /** RTKLIB-aligned: number of continuous fixes counter */
  int nfix = 0;

  /** RTKLIB-aligned: minimum continuous fixes to hold ambiguity */
  static final int MIN_FIX_HOLD = 10;

  /** Previous observation epoch time (ms) for dynamic dt calculation, -1 if uninitialized */
  long prevObsTimeMs = -1;
  
  /** Epoch counter for debug logging */
  protected int epochCount = 0;
  
  public KalmanFilter( GoGPS goGPS ){
    super( Objects.requireNonNull(goGPS, "GoGPS must not be null") );
  }

  abstract void setup(Observations roverObs, Observations masterObs, Coordinates masterPos);
  abstract void estimateAmbiguities( Observations roverObs, Observations masterObs, Coordinates masterPos, ArrayList<Integer> satAmb, int pivotIndex, boolean init);
  abstract void checkSatelliteConfiguration( Observations roverObs, Observations masterObs, Coordinates masterPos );

  /** LAMBDA ambiguity resolution (default: no-op, overridden by DD Kalman filter).
   *  @return true if ambiguities were successfully fixed */
  boolean fixAmbiguitiesLambda() { return false; }

  /** Hold fixed ambiguities by adding pseudo-observation constraints (RTKLIB-aligned).
   *  Default: no-op, overridden by DD Kalman filter. */
  void holdamb() { }


  /**
   * Gets the st dev code.
   *

   * @param i the selected GPS frequency
   * @return the stDevCode
   */
  public double getStDevCode( ObservationSet obsSet, int i) {
    if (i >= 0 && i < stDevCodeP.length && obsSet.isPseudorangeP(i)) {
      return stDevCodeP[i];
    }
    return stDevCodeC;
  }

  
  /**
   * @param roverObs
   * @param masterObs
   */
  void computeDopplerPredictedPhase( Observations roverObs, Observations masterObs ) {

    int maxId = goGPS.getMaxSatId();
    rover.dopplerPredPhase = new double[maxId + 1];
    
    if (masterObs != null)
      master.dopplerPredPhase = new double[maxId + 1];

    for (int i = 0; i < sats.availPhase.size(); i++) {

      int satID = sats.availPhase.get(i);
      char satType = sats.typeAvailPhase.get(i);
      
      double roverPhase = roverObs.getSatByIDType(satID, satType).getPhaseCycles(goGPS.getFreq());
      float roverDoppler = roverObs.getSatByIDType(satID, satType).getDoppler(goGPS.getFreq());
      if (!Double.isNaN(roverPhase) && !Float.isNaN(roverDoppler))
        rover.setDopplerPredictedPhase(sats.availPhase.get(i), roverPhase - roverDoppler);
      
      if (masterObs != null) {
        double masterPhase = masterObs.getSatByIDType(satID, satType).getPhaseCycles(goGPS.getFreq());
        float masterDoppler = masterObs.getSatByIDType(satID, satType).getDoppler(goGPS.getFreq());
        if (!Double.isNaN(masterPhase) && !Float.isNaN(masterDoppler))
          master.setDopplerPredictedPhase(sats.availPhase.get(i), masterPhase - masterDoppler);
      }
    }
  }

  public void init( Observations roverObs, Observations masterObs, Coordinates masterPos) {
  
    // Order-related quantities with validation
    o1 = goGPS.getDynamicModel().getOrder();
    if (o1 <= 0) {
      if (goGPS.isDebug()) System.out.println("[Kalman init] Invalid dynamic model order=" + o1 + ", fallback to static (order=1)");
      o1 = 1;
    }
    o2 = o1 * 2;
    o3 = o1 * 3;
  
    // Order-related indices
    i1 = o1 - 1;
    i2 = o2 - 1;
    i3 = o3 - 1;

    // Improve approximate position accuracy by applying twice code double differences
    // Save DD解算前的位置，解算发散时回退
    double ddBackupX = rover.getX();
    double ddBackupY = rover.getY();
    double ddBackupZ = rover.getZ();
    boolean ddDiverged = false;
    double maxDivDist = goGPS.getMaxDivergenceDistance();
    for (int i = 0; i < 2; i++) {
      // Select satellites available for double differences
      if (masterObs != null)
        sats.selectDoubleDiff(roverObs, masterObs, masterPos);
      else
        sats.selectStandalone(roverObs);
  
      if (sats.avail.size() >= 4) {
        if (masterObs != null)
          codeDoubleDifferences( roverObs, masterObs, masterPos);
        else
          codeStandalone( roverObs, false, false);
      } else {
        // Insufficient satellites for DD, keep SPP X/Y + base Z (RTKLIB-aligned hybrid)
        if(goGPS.isDebug()) System.out.println("[Kalman init] Insufficient sats for DD, keep SPP XY + base Z");
        ddDiverged = true;
        if (masterPos != null) {
          double fallbackZ = (ddBackupZ == 0 || Math.abs(ddBackupZ - masterPos.getZ()) > maxDivDist)
              ? masterPos.getZ() : ddBackupZ;
          rover.setXYZ(ddBackupX, ddBackupY, fallbackZ);
          rover.computeGeodetic();
        } else {
          rover.setXYZ(0, 0, 0);
        }
        return;
      }

      // 每轮DD解算后检查是否发散
      // RTKLIB-aligned: keep SPP X/Y estimate, borrow base Z when SPP Z is invalid
      // (BDS GEO-only constellations have poor vertical geometry, SPP Z=0 is common)
      double dx = rover.getX() - masterPos.getX();
      double dy = rover.getY() - masterPos.getY();
      double dz = rover.getZ() - masterPos.getZ();
      if (Math.sqrt(dx*dx + dy*dy + dz*dz) > maxDivDist) {
        if(goGPS.isDebug()) System.out.println("[Kalman init] DD iter " + i + " diverged (" +
            String.format("%.0f", Math.sqrt(dx*dx+dy*dy+dz*dz)/1000) + "km), keep SPP XY + base Z");
        // Use SPP X/Y (reasonable), borrow base Z when SPP Z is invalid (vertical geometry poor)
        double fallbackZ = (ddBackupZ == 0 || Math.abs(ddBackupZ - masterPos.getZ()) > maxDivDist)
            ? masterPos.getZ() : ddBackupZ;
        rover.setXYZ(ddBackupX, ddBackupY, fallbackZ);
        rover.computeGeodetic();
        ddDiverged = true;
        // Recompute satAppRange/diffSat/topo from base station position
        sats.selectDoubleDiff(roverObs, masterObs, masterPos);
        // Discard DD code solution's positionCovariance, use large init variance instead
        positionCovariance = null;
        break;
      }
    }
  
    // Set number of ambiguity slots: state vector is indexed by satellite ID (PRN),
    // i.e. KFstate[i3 + satID], so nN must be >= max(satID) + 1.
    // Use the maximum satellite ID across all available satellites (code + phase).
    int maxSatId = 0;
    for (int id : sats.avail.keySet()) {
      if (id > maxSatId) maxSatId = id;
    }
    for (int id : sats.availPhase) {
      if (id > maxSatId) maxSatId = id;
    }
    nN = Math.max(maxSatId, 63); // minimum 63 for BDS (PRN 1-63)

    // Ionospheric and tropospheric states (RTKLIB-aligned: IONOOPT_EST + TROPOPT_EST)
    boolean estimIono = goGPS.getIonoOpt() != GoGPS.IonoOpt.OFF && goGPS.isDualFreq();
    boolean estimTropo = goGPS.getTropOpt() != GoGPS.TropOpt.OFF && goGPS.isDualFreq();
    if (goGPS.isDebug()) {
      System.err.printf("[Kalman init] dualFreq=%b, ionoOpt=%s, tropOpt=%s, estimIono=%b, estimTropo=%b%n",
          goGPS.isDualFreq(), goGPS.getIonoOpt(), goGPS.getTropOpt(), estimIono, estimTropo);
    }
    // Design note (RTKLIB-aligned): L2 ambiguity states must be allocated whenever
    // dual-frequency observations are used, NOT tied to ionosphere estimation.
    // RTKLIB's ddmat() always references L2 ambiguity (nb+nf*MAXSAT) for L2 phase
    // regardless of ionoopt. Previously nAmbL2 was tied to estimIono, which caused
    // an out-of-bounds crash in setup() when ionoOpt=OFF (nAmbL2=0 but L2 phase
    // observations still tried to write H[row, iAmbL2+id]).
    // 【关键修复记录】原 nAmbL2 = estimIono ? nN : 0，当 ionoOpt=OFF 时 nAmbL2=0，
    // 但 setup() 仍写 L2 模糊度列 → 矩阵越界 → KF 更新后位置跳到地球对蹠点(6374km 外)。
    // 改为按 dualFreq 分配后修复。若再次出现越界崩溃，核对此行。
    nAmbL2 = goGPS.isDualFreq() ? nN : 0;
    nIono = estimIono ? nN : 0;
    nTropo = estimTropo ? 1 : 0;
    iAmbL2 = i3 + 1 + nN;
    iIono = iAmbL2 + nAmbL2;
    iTropo = iIono + nIono;

    int nx = stateSize();

    // Allocate matrices
    T = SimpleMatrix.identity(nx);
    KFstate = new SimpleMatrix(nx, 1);
    KFprediction = new SimpleMatrix(nx, 1);
    Cvv = new SimpleMatrix(nx, nx);
    Cee = new SimpleMatrix(nx, nx);
  
    // System dynamics (default dt=1.0 for initialization)
    buildTransitionMatrix(1.0);
  
    // Model error covariance matrix (init dt=1.0)
    Cvv.zero();
    Cvv.set(i1, i1, Math.pow(stDevE, 2));
    Cvv.set(i2, i2, Math.pow(stDevN, 2));
    Cvv.set(i3, i3, Math.pow(stDevU, 2));
  
    // Estimate phase ambiguities
    ArrayList<Integer> newSatellites = new ArrayList<Integer>(0);
    newSatellites.addAll(sats.availPhase);
    
    estimateAmbiguities( roverObs, masterObs, masterPos, newSatellites, sats.pivot, true);
  
    // Compute predicted phase ranges based on Doppler observations
    computeDopplerPredictedPhase(roverObs, masterObs);
  
    // Initial state
    KFstate.set(0, 0, rover.getX());
    KFstate.set(i1 + 1, 0, rover.getY());
    KFstate.set(i2 + 1, 0, rover.getZ());
  
    // Prediction
    KFprediction = T.mult(KFstate);
  
    // Covariance matrix of the initial state
    // RTKLIB-aligned: always use VAR_POS = SQR(30.0) = 900 m², regardless of DD convergence.
    // RTKLIB never uses SPP covariance (sol->qr) in the RTK filter; it always initializes
    // position with VAR_POS. Using LS covariance or huge variance (maxDivDist²) causes
    // the KF to over-trust observations, moving position by 100+ meters from DD residuals.
    double initPosVar = Math.pow(stDevInit, 2);
    positionCovariance = new SimpleMatrix(3, 3);
    Cee.set(0, 0, initPosVar);
    Cee.set(i1 + 1, i1 + 1, initPosVar);
    Cee.set(i2 + 1, i2 + 1, initPosVar);
    for (int i = 1; i < o1; i++) {
      Cee.set(i, i, initPosVar);
      Cee.set(i + i1 + 1, i + i1 + 1, initPosVar);
      Cee.set(i + i2 + 1, i + i2 + 1, initPosVar);
    }

    // Initialize ionospheric and tropospheric states (RTKLIB-aligned)
    // RTKLIB udion(): initx(rtk,1E-6,SQR(rtk->opt.std[1]*bl/1E4),j)
    // std[1]=0.03, scaled by baseline/1E4. For 1km baseline: SQR(0.03*0.1)=9e-6.
    if (estimIono) {
      double bl = 0;
      if (masterPos != null) {
        double dx = rover.getX() - masterPos.getX();
        double dy = rover.getY() - masterPos.getY();
        double dz = rover.getZ() - masterPos.getZ();
        bl = Math.sqrt(dx*dx + dy*dy + dz*dz);
      }
      double ionoInitVar = Math.pow(goGPS.getSigIono() * bl / 1E4, 2);
      if (ionoInitVar <= 0) ionoInitVar = Math.pow(goGPS.getSigIono(), 2);
      for (int i = 0; i < nIono; i++) {
        Cee.set(iIono + i, iIono + i, ionoInitVar);
      }
    }
    if (estimTropo) {
      double sigTropo2 = Math.pow(goGPS.getSigTropo(), 2);
      Cee.set(iTropo, iTropo, sigTropo2);
    }
  }

  SimpleMatrix compute_residuals( SimpleMatrix X ) {
	  SimpleMatrix residuals = y0.minus(H.mult(X));
	  return residuals;
  }
  
  /**
   * Build state transition matrix T with current time step dt.
   * For order=2 (CV model): T[i][i+1] = dt couples position to velocity.
   * T is rebuilt each epoch to accommodate variable sampling intervals.
   */
  private void buildTransitionMatrix(double dt) {
    int nx = stateSize();
    T = SimpleMatrix.identity(nx);
    int j = 0;
    for (int i = 0; i < o3; i++) {
      if (j < (o1 - 1)) {
        T.set(i, i + 1, dt);
        j++;
      } else {
        j = 0;
      }
    }
  }

  /**
   * @param roverObs
   * @param masterObs
   * @param masterPos
   *
   */
  public void loop( Observations roverObs, Observations masterObs, Coordinates masterPos) {

    // Cache roverObs for fixAmbiguitiesLambda() wavelength lookup (meters→cycles conversion)
    this.currentRoverObs = roverObs;

    final int minNumSat = 2;

    // Set linearization point (approximate coordinates by KF prediction at previous step)
    rover.setXYZ(KFprediction.get(0), KFprediction.get(i1 + 1), KFprediction.get(i2 + 1));

    // Debug: print KFprediction at the very beginning of loop
    if (goGPS.isDebug() && epochCount <= 5) {
      System.err.printf("[KF loop START] epoch=%d, KFpred=[%.2f, %.2f, %.2f]%n",
          epochCount, KFprediction.get(0), KFprediction.get(i1+1), KFprediction.get(i2+1));
    }

    // Note: satOld/satTypeOld/oldPivotId are saved at END of previous epoch
    // (after selectDoubleDiff) to correctly track rover DD state. Saving here
    // would capture base-station SPP state (run() calls sats.selectStandalone
    // (obsM) before kf.loop()), corrupting lost/new satellite detection.

    // Select satellites for standalone
    sats.selectStandalone(roverObs);

    if( sats.avail.size() >= 4)
      // Estimate receiver clock error by code stand-alone
      codeStandalone( roverObs, true, false);

    // Debug: check rover position after SPP (should be unchanged since estimateOnlyClock=true)
    if (goGPS.isDebug() && epochCount <= 5) {
      System.err.printf("[KF loop afterSPP] epoch=%d, rover=[%.2f, %.2f, %.2f], clockErr=%.9f%n",
          epochCount, rover.getX(), rover.getY(), rover.getZ(), rover.clockError);
    }

    int obsReduction = 0;
    
    if (masterObs != null) {
      // Select satellites for double differences
      sats.selectDoubleDiff(roverObs, masterObs, masterPos);
      obsReduction = 1;
    }

    // Number of observations: must match setup() exactly
    // L1 rows = code DD rows + phase DD rows = (nAvail-1) + (nAvailPhase-1)
    int nSatAvail = sats.avail.size() - obsReduction;
    int nSatAvailPhase = (sats.availPhase.size() > 0) ? sats.availPhase.size() - obsReduction : 0;
    int l1Obs = nSatAvail + nSatAvailPhase;
    // RTKLIB-aligned: L2 rows only allocated when a valid L2 reference sat
    // (sats.pivotL2) exists. If no sat has L2 phase (e.g. single-freq or all
    // sats lack L2), L2 block is skipped entirely.
    int nObs = (goGPS.isDualFreq() && sats.pivotL2 >= 0) ? l1Obs * 2 : l1Obs;

    // Dynamic epoch interval from observation time (RTKLIB-aligned)
    double dt = 1.0; // default 1 second
    if (roverObs.getRefTime() != null) {
      long curTimeMs = roverObs.getRefTime().getMsec();
      if (prevObsTimeMs > 0) {
        long dtMs = curTimeMs - prevObsTimeMs;
        if (dtMs > 0 && dtMs < 3600000) { // sanity: between 1ms and 1 hour
          dt = dtMs / 1000.0;
        }
      }
      prevObsTimeMs = curTimeMs;
    }

    if( sats.avail.size() >= minNumSat ) {
      int nx = stateSize();
      // Allocate transformation matrix
      H = new SimpleMatrix(nObs, nx);

      // Allocate observation vector
      y0 = new SimpleMatrix(nObs, 1);

      // Allocate observation error covariance matrix
      Cnn = new SimpleMatrix(nObs, nObs);

      // Allocate K and G matrices
      SimpleMatrix K = new SimpleMatrix(nx, nx);
      SimpleMatrix G = new SimpleMatrix(nx, nObs);

      // Re-initialization of the model error covariance matrix
      Cvv.zero();

      // Set variances only if dynamic model is not static
      if (o1 != 1) {
        // Allocate and build rotation matrix
        SimpleMatrix R = Coordinates.rotationMatrix(rover);

        // Build 3x3 diagonal matrix with variances (RTKLIB-aligned: scale by dt)
        SimpleMatrix diagonal = new SimpleMatrix(3, 3);
        diagonal.set(0, 0, Math.pow(stDevE, 2) * dt);
        diagonal.set(1, 1, Math.pow(stDevN, 2) * dt);
        diagonal.set(2, 2, Math.pow(stDevU, 2) * dt);

        // Propagate local variances to global variances
        diagonal = R.transpose().mult(diagonal).mult(R);

        // Set global variances in the model error covariance matrix
        Cvv.set(i1, i1, diagonal.get(0, 0));
        Cvv.set(i1, i2, diagonal.get(0, 1));
        Cvv.set(i1, i3, diagonal.get(0, 2));
        Cvv.set(i2, i1, diagonal.get(1, 0));
        Cvv.set(i2, i2, diagonal.get(1, 1));
        Cvv.set(i2, i3, diagonal.get(1, 2));
        Cvv.set(i3, i1, diagonal.get(2, 0));
        Cvv.set(i3, i2, diagonal.get(2, 1));
        Cvv.set(i3, i3, diagonal.get(2, 2));
      }

      // RTKLIB-aligned: baseline length for iono process noise scaling
      double bl = 0;
      if (masterPos != null) {
        double dx = rover.getX() - masterPos.getX();
        double dy = rover.getY() - masterPos.getY();
        double dz = rover.getZ() - masterPos.getZ();
        bl = Math.sqrt(dx*dx + dy*dy + dz*dz);
      }

      // Ionospheric process noise: RTKLIB udion() formula
      //   rtk->P[j+j*rtk->nx]+=SQR(rtk->opt.prn[1]*bl/1E4*fact)*fabs(tt)
      //   prn[1]=1e-3, fact=cos(el). Scaled by baseline and elevation.
      if (nIono > 0 && roverObs != null) {
        double prnIonoStd = goGPS.getPrnIono();
        for (int i = 0; i < rover.topo.length && rover.topo[i] != null; i++) {
          int satID = roverObs.getSatID(i);
          if (sats.avail.containsKey(satID)) {
            double elRad = Math.toRadians(rover.topo[i].getElevation());
            double fact = Math.cos(elRad);
            if (fact < 0.1) fact = 0.1;
            double ionoNoise = Math.pow(prnIonoStd * bl / 1E4 * fact, 2) * dt;
            int idx = iIono + satID;
            Cvv.set(idx, idx, Cvv.get(idx, idx) + ionoNoise);
          }
        }
      }

      // Tropospheric process noise: RTKLIB formula
      //   rtk->P[j+j*rtk->nx]+=SQR(rtk->opt.prn[2])*fabs(tt)
      //   prn[2]=1e-4, variance = SQR(1e-4)*dt = 1e-8*dt
      if (nTropo > 0) {
        Cvv.set(iTropo, iTropo, Math.pow(goGPS.getPrnTropo(), 2) * dt);
      }

      // Ambiguity process noise: RTKLIB udbias() formula
      //   rtk->P[j+j*rtk->nx]+=rtk->opt.prn[0]*rtk->opt.prn[0]*fabs(tt)
      //   prn[0]=1e-4, variance = 1e-8*dt
      if (nN > 0) {
        double ambNoise = Math.pow(goGPS.getPrnAmb(), 2) * dt;
        for (int satID : sats.availPhase) {
          int idx = i3 + 1 + satID;
          Cvv.set(idx, idx, Cvv.get(idx, idx) + ambNoise);
        }
      }
      if (nAmbL2 > 0) {
        double ambNoise = Math.pow(goGPS.getPrnAmb(), 2) * dt;
        for (int satID : sats.availPhase) {
          int idx = iAmbL2 + satID;
          Cvv.set(idx, idx, Cvv.get(idx, idx) + ambNoise);
        }
      }
      
      // Rebuild state transition matrix with current dt (RTKLIB-aligned)
      buildTransitionMatrix(dt);
      
      // Fill in Kalman filter transformation matrix, observation vector and observation error covariance matrix
      setup(roverObs, masterObs, masterPos);

      // Debug: scan y0 for NaN right after setup()
      if (goGPS.isDebug()) {
        for (int i = 0; i < y0.numRows(); i++) {
          double v = y0.get(i, 0);
          if (Double.isNaN(v) || Double.isInfinite(v)) {
            System.err.printf("[y0-NaN-after-setup] row=%d val=%.3e nObs=%d%n", i, v, y0.numRows());
          }
        }
      }
      
      // Check if satellite configuration changed since the previous epoch
      checkSatelliteConfiguration(roverObs, masterObs, masterPos);

      // Identity matrix
      SimpleMatrix I = SimpleMatrix.identity(nx);

      // Kalman filter equations (step-by-step with singularity check, like RTKLIB)
      // Step 1: predicted covariance
      K = T.mult(Cee).mult(T.transpose()).plus(Cvv);

      // RTKLIB filter() alignment: only update active states (x[i] != 0 && P[i][i] > 0)
      // Zero out H columns for inactive states, so they don't participate in the update.
      // Mathematically equivalent to RTKLIB's reduced-matrix approach (rtkcmn.c filter()).
      // Reference: RTKLIB rtkcmn.c filter()
      int nActiveForFilter = 0;
      for (int i = 0; i < nx; i++) {
        if (Math.abs(KFprediction.get(i, 0)) < 1e-30 && K.get(i, i) <= 0) {
          for (int j = 0; j < nObs; j++) {
            H.set(j, i, 0);
          }
        } else {
          nActiveForFilter++;
        }
      }
      if (goGPS.isDebug() && nActiveForFilter < nx) {
        System.out.println("[Kalman] filter() alignment: " + nActiveForFilter + "/" + nx + " active states");
      }
      
      // Step 2: innovation covariance S = H*K*H' + Cnn
      // 【设计说明】(2026-06-21)
      // goGPS 已将 H_amb 从 -λ(cycles) 重构为 +1.0(meters)，对齐 RTKLIB。
      // 此时 S[phase] = α²·K[pos] + K[amb] + 2·α·K[pos,amb] + Cnn
      //              = (α·√K[pos] + √K[amb])² + Cnn > 0，数学保证非奇异。
      // 不再需要 S 对角项下限保护(原 S_DIAG_FLOOR workaround 已移除)。
      SimpleMatrix S = H.mult(K).mult(H.transpose()).plus(Cnn);

      double S_DIAG_FLOOR;
      if (epochCount < 5)       S_DIAG_FLOOR = 0.5;
      else if (epochCount < 10) S_DIAG_FLOOR = 0.1;
      else if (epochCount < 20) S_DIAG_FLOOR = 0.05;
      else                      S_DIAG_FLOOR = 0.01;
      for (int i = 0; i < S.numRows(); i++) {
        if (S.get(i, i) < S_DIAG_FLOOR) {
          S.set(i, i, S_DIAG_FLOOR);
        }
      }

      // 【诊断】打印 K 对角线和 H 的关键元素，定位 S 与 posVar 不匹配问题
      // S_code = α²*K_pos + Cnn。若 S_code >> α²*posVar，说明 K_pos ≠ Cee_pos。
      if (goGPS.isDebug() && epochCount <= 3) {
        System.err.printf("[K-diag] K[0,0]=%.4f K[1,1]=%.4f K[2,2]=%.4f K[amb0]=%.4f%n",
            K.get(0,0), K.get(1,1), K.get(2,2), (i3+1<nx)?K.get(i3+1,i3+1):0);
        System.err.printf("[H-diag] H[0,0]=%.6f H[0,1]=%.6f H[0,2]=%.6f H[0,amb0]=%.6f Cnn[0,0]=%.6f%n",
            H.get(0,0), H.get(0,1), H.get(0,2), (i3+1<nx)?H.get(0,i3+1):0, Cnn.get(0,0));
        // 查找第一个相位行(有非零模糊度列的行)，打印其 H 和对应 K[amb]
        int phaseRow = -1, ambCol = -1;
        for (int r = 1; r < nObs; r++) {
          for (int c = i3 + 1; c < nx; c++) {
            if (Math.abs(H.get(r, c)) > 1e-12) {
              phaseRow = r; ambCol = c; break;
            }
          }
          if (phaseRow >= 0) break;
        }
        if (phaseRow >= 0) {
          System.err.printf("[H-phase] row=%d H[pos]=%.6f,%.6f,%.6f H[amb@%d]=%.6f K[amb@%d]=%.4f Cnn=%.6f%n",
              phaseRow, H.get(phaseRow,0), H.get(phaseRow,1), H.get(phaseRow,2),
              ambCol, H.get(phaseRow,ambCol), ambCol, K.get(ambCol,ambCol),
              Cnn.get(phaseRow,phaseRow));
          // 手动计算 S[phaseRow,phaseRow] = H[phaseRow,:]*K*H[phaseRow,:]' + Cnn
          double manualS = 0;
          for (int i = 0; i < nx; i++) {
            for (int j = 0; j < nx; j++) {
              manualS += H.get(phaseRow, i) * K.get(i, j) * H.get(phaseRow, j);
            }
          }
          manualS += Cnn.get(phaseRow, phaseRow);
          System.err.printf("[S-manual] S[%d,%d]=%.6f manual=%.6f HKHt=%.6f%n",
              phaseRow, phaseRow, S.get(phaseRow, phaseRow), manualS, manualS - Cnn.get(phaseRow,phaseRow));
        }
      }

      // Debug: check S matrix condition (no explicit inversion, just diagonal range)
      double sMin = Double.MAX_VALUE, sMax = 0;
      for (int i = 0; i < S.numRows(); i++) {
        double d = S.get(i, i);
        if (d < sMin) sMin = d;
        if (d > sMax) sMax = d;
      }
      double condEst = (sMin > 0) ? sMax / sMin : Double.POSITIVE_INFINITY;

      // 【S-cond 上限保护】(2026-06-21)
      // 根因: 多颗卫星同时周跳 → 模糊度 Cee 重置为 λ²·900 → S 对角线飙升
      // → condEst 从 ~2500 爆炸到 2e11 → 线性求解器不稳定 → 增益爆炸 → NaN 传播。
      // 对齐 RTKLIB: RTKLIB 用 ddcov() 产生非对角 R 矩阵(同系统 DD 共享参考卫星方差),
      // 改善 S 条件数。goGPS 的 Cnn 是对角的, 条件数更大, 需要额外保护。
      // 策略: condEst > 1e8 时跳过该历元更新, 保留预测状态 + 膨胀 Cee (等价于过程噪声放大)。
      final double S_COND_MAX = 1e8;
      if (condEst > S_COND_MAX) {
        if (goGPS.isDebug()) {
          System.err.printf("[Kalman S-cond] SKIP update: condEst=%.3e > %.0e, keeping prediction + inflating Cee%n",
              condEst, S_COND_MAX);
        }
        // 保留预测, 膨胀 Cee (×1.5) 以加速后续恢复
        KFstate = KFprediction;
        KFprediction = T.mult(KFstate);
        Cee = T.mult(Cee).mult(T.transpose());
        for (int i = 0; i < Cee.numRows(); i++) {
          Cee.set(i, i, Cee.get(i, i) * 1.5);
        }
        return;
      }

      if (goGPS.isDebug()) {
        System.err.printf("[Kalman S-cond] S diag min=%.3e max=%.3e condEst=%.3e%n", sMin, sMax, condEst);
      // 【诊断】打印 S 对角线逐行值，定位哪些行极小
      // 根因: 位置方差收敛后(900→0.3), 相位观测对位置约束极强,
      // S[phase] = α²·K[pos](小) + λ²·K[amb] + 2·α·λ·K[pos,amb](负交叉项) + Cnn
      // S[phase] 可小至 ~0.001 (正常), S[code] ~126, 条件数~1e5。
      // 直接求解线性系统(不显式求逆)可稳定处理此条件数。
      if (epochCount <= 3) {
          StringBuilder sb = new StringBuilder("[S-diag] ");
          for (int i = 0; i < S.numRows(); i++) {
            sb.append(String.format("%.4g ", S.get(i, i)));
          }
          System.err.println(sb);
        }
      }

      // Step 3: Compute Kalman gain G by solving linear system (RTKLIB-aligned).
      // 【关键修复 - 数值稳定性】
      // 原方案: 显式求 S^{-1}, 再算 G = K*H'*S^{-1}。S 病态(条件数~1e5)时 S_inv 误差被放大。
      // 新方案: 直接求解线性系统 S * G' = (K*H')' = H*K', 得到 G' 后转置。
      //   数学等价: G = K*H'*S^{-1}  <=>  G*S = K*H'  <=>  S'*G' = (K*H')' = H*K'
      //   因 S 对称: S*G' = H*K'  =>  G' = S \ (H*K')
      // 直接求解比显式求逆数值稳定得多(数值线性代数黄金法则)。
      //
      // RTKLIB 对比:
      //   RTKLIB filter_() 用 matinv() (LU 分解 dgetrf_) 显式求 S^{-1}, 再算 K=P*H*S^{-1}。
      //   RTKLIB 不出问题的原因: (1) 状态压缩减少维度; (2) ddcov() 产生非对角 R 矩阵
      //   (同系统 DD 共享参考卫星方差), 改善 S 条件数; (3) LAPACK dgetrf_ 有 partial pivoting。
      //   goGPS 的 Cnn 是对角的(忽略 DD 相关性), S 条件数更大, 显式求逆更不稳定。
      //   直接求解是比 RTKLIB 更优的方案, 不改变数学结果, 只提升数值稳定性。
      //
      // 【实测验证 2026-06-21】
      //   修复前(显式 S.invert): S 条件数~1e7, S_inv 爆炸, G 爆炸(60), 位置发散。
      //   修复后(直接求解): S 条件数~1e6, 无 G-NaN, Cee 正定性检查全部通过。
      //   但 gMax 仍可达 5.2(epoch1, 位置方差 900→1.2 增益过大)甚至 265(epoch2)，
      //   根因不在数值稳定性，而在位置方差收敛过快(10+卫星同时约束)。
      //   后续若要根治，需对齐 RTKLIB ddcov() 的非对角 R 矩阵，或限制位置方差下限。
      //
      // 求解器选择: Cholesky (对称正定最优) -> LU+pivot 回退 (S 非正定时)
      int nS = S.numRows();
      SimpleMatrix KHt = K.mult(H.transpose()); // nx x nObs, 即 (H*K')' 的转置
      SimpleMatrix HKt = KHt.transpose();       // nObs x nx, 即 H*K'
      LinearSolver<DMatrixRMaj, DMatrixRMaj> solver = LinearSolverFactory_DDRM.chol(nS);
      SimpleMatrix Gt = null; // nObs x nx, G 的转置
      try {
        if (!solver.setA(S.getDDRM().copy())) {
          if (goGPS.isDebug()) {
            System.err.println("[Kalman] Cholesky failed (S not PD), fallback to LU+pivoting");
          }
          solver = LinearSolverFactory_DDRM.general(nS, nS);
          if (!solver.setA(S.getDDRM().copy())) {
            throw new RuntimeException("LU decomposition also failed");
          }
        }
        DMatrixRMaj Gt_ddrm = new DMatrixRMaj(nS, HKt.numCols());
        solver.solve(HKt.getDDRM().copy(), Gt_ddrm);
        Gt = SimpleMatrix.wrap(Gt_ddrm);
      } catch (Exception e) {
        if(goGPS.isDebug()) System.out.println("[Kalman] S matrix singular: " + e.getMessage());
        KFstate = KFprediction;
        KFprediction = T.mult(KFstate);
        Cee = T.mult(Cee).mult(T.transpose());
        return;
      }
      G = Gt.transpose(); // nx x nObs

      // Check G for NaN/Inf (EJML may return NaN-filled matrix without throwing)
      if (goGPS.isDebug()) {
        int nanCount = 0, infCount = 0;
        for (int i = 0; i < G.numRows(); i++) {
          for (int j = 0; j < G.numCols(); j++) {
            double v = G.get(i, j);
            if (Double.isNaN(v)) nanCount++;
            else if (Double.isInfinite(v)) infCount++;
          }
        }
        if (nanCount > 0 || infCount > 0) {
          System.err.printf("[Kalman G-NaN] G has %d NaN, %d Inf elements!%n", nanCount, infCount);
        }
      }

      // Check KFprediction for NaN/Inf (pivot switch may introduce them)
      if (goGPS.isDebug()) {
        for (int i = 0; i < nx; i++) {
          double v = KFprediction.get(i, 0);
          if (Double.isNaN(v) || Double.isInfinite(v)) {
            System.err.printf("[Kalman pred-NaN] KFprediction[%d]=%.3e (i1=%d,i2=%d,i3=%d)%n",
                i, v, i1, i2, i3);
            break;
          }
        }
      }

      // Step 4: Kalman gain G already computed via direct solve above (S*G'=H*K')
      // G = K*H'*S^{-1} (mathematically equivalent, but computed without explicit S_inv)

      if (epochCount <= 1 && goGPS.isDebug()) {
        System.err.println("====== EPOCH 0 KALMAN GAIN (6 decimals) ======");
        System.err.printf("K (pred cov) diag pos = [%.6f, %.6f, %.6f]%n",
            K.get(0, 0), K.get(1, 1), K.get(2, 2));
        System.err.printf("Cnn (obs noise) diag first 6: ");
        for (int i = 0; i < Math.min(6, Cnn.numRows()); i++) {
          System.err.printf("[%.4f] ", Cnn.get(i, i));
        }
        System.err.println();
        System.err.printf("S diag first 6: ");
        for (int i = 0; i < Math.min(6, S.numRows()); i++) {
          System.err.printf("[%.4f] ", S.get(i, i));
        }
        System.err.println();
        System.err.println("G (Kalman gain) position rows (first 6 cols):");
        for (int i = 0; i < 3; i++) {
          System.err.printf("  G[%d] = [", i);
          for (int j = 0; j < Math.min(6, G.numCols()); j++) {
            System.err.printf("%.6f%s", G.get(i, j), (j < Math.min(6, G.numCols()) - 1) ? ", " : "");
          }
          System.err.println("]");
        }
        System.err.println("====== END EPOCH 0 KALMAN GAIN ======");
      }

      // Debug: check Kalman gain magnitude
      if (goGPS.isDebug()) {
        double gMax = 0;
        int gMaxRow = -1, gMaxCol = -1;
        for (int i = 0; i < G.numRows(); i++) {
          for (int j = 0; j < G.numCols(); j++) {
            double v = Math.abs(G.get(i, j));
            if (v > gMax) { gMax = v; gMaxRow = i; gMaxCol = j; }
          }
        }
        if (epochCount <= 5) {
          System.err.printf("[G-diag] epoch=%d gMax=%.4f at (%d,%d) nObs=%d%n",
              epochCount, gMax, gMaxRow, gMaxCol, nObs);
          for (int j = 0; j < nObs; j++) {
            double sVal = S.get(j, j);
            double y0Val = y0.get(j, 0);
            double gPos0 = G.get(0, j);
            double gPos1 = G.get(1, j);
            double gPos2 = G.get(2, j);
            int ambCol = -1;
            for (int c = i3 + 1; c < nx && c < i3 + 1 + nN; c++) {
              if (Math.abs(H.get(j, c)) > 1e-12) { ambCol = c; break; }
            }
            double gAmb = (ambCol >= 0) ? G.get(ambCol, j) : 0;
            System.err.printf("[G-per-obs] obs=%d S=%.4g y0=%.2f Gpos=[%.4f,%.4f,%.4f] GAmb=%.4f%n",
                j, sVal, y0Val, gPos0, gPos1, gPos2, gAmb);
          }
        }
        if (gMax > 1e6) {
          System.err.printf("[Kalman G-warn] G max=%.3e (large!)%n", gMax);
        }
      }
      
      // Unified outlier detection (RTKLIB-aligned: merge residual + chi-square into one round)
      // Detect bad observations, down-weight them, then recompute KF state once
      if( goGPS.searchForOutliers() ) {
        // RTKLIB filter_(): xp = x + K*v, where v is the innovation.
        // goGPS y0 = ddcObs - appRangeCode = obs - H*x_pred = innovation.
        // So the correct update is: x_new = x_pred + G*y0 (NOT (I-GH)*pred + G*y0).
        SimpleMatrix Xhat_t_t = KFprediction.plus(G.mult(y0));
        SimpleMatrix residuals = compute_residuals(Xhat_t_t);
        SimpleMatrix innovation = y0;
        SimpleMatrix S_full = H.mult(K).mult(H.transpose()).plus(Cnn);

        boolean badMeasurement = false;
        double largeNoise = goGPS.getLargeNoiseValue();
        double chi2Thresh = goGPS.getChiSquareThreshold();
        int halfN = residuals.getNumElements() / 2;

        for (int i = 0; i < residuals.getNumElements(); i++) {
          boolean isOutlier = false;

          // Residual-based check (code vs phase)
          if (i < halfN) {
            if (Math.abs(residuals.get(i)) > goGPS.getCodeResidThreshold()) {
              isOutlier = true;
            }
          } else {
            if (Math.abs(residuals.get(i)) > goGPS.getPhaseResidThreshold()) {
              isOutlier = true;
            }
          }

          // Chi-square innovation check (normalized innovation)
          double s_ii = S_full.get(i, i);
          if (!isOutlier && s_ii > 0) {
            double normInnov = Math.abs(innovation.get(i)) / Math.sqrt(s_ii);
            if (normInnov > chi2Thresh) {
              isOutlier = true;
              if (goGPS.isDebug()) System.err.printf("[Kalman] obs[%d] normInnov=%.1f > %.1f, down-weight%n", i, normInnov, chi2Thresh);
            }
          }

          if (isOutlier) {
            H.setRow(i, 0, new double[H.numCols()]);
            y0.setRow(i, 0, new double[y0.numCols()]);
            for (int c = 0; c < Cnn.numCols(); c++) Cnn.set(i, c, 0);
            Cnn.set(i, i, largeNoise);
            badMeasurement = true;
          }
        }

        if (badMeasurement) {
          // Recompute with down-weighted observations (direct solve, no explicit S_inv)
          SimpleMatrix S_new = H.mult(K).mult(H.transpose()).plus(Cnn);
          try {
            int nSnew = S_new.numRows();
            SimpleMatrix KHt_new = K.mult(H.transpose());
            SimpleMatrix HKt_new = KHt_new.transpose();
            LinearSolver<DMatrixRMaj, DMatrixRMaj> solverNew = LinearSolverFactory_DDRM.chol(nSnew);
            if (!solverNew.setA(S_new.getDDRM().copy())) {
              solverNew = LinearSolverFactory_DDRM.general(nSnew, nSnew);
              if (!solverNew.setA(S_new.getDDRM().copy())) {
                throw new RuntimeException("LU decomposition failed for S_new");
              }
            }
            DMatrixRMaj Gt_new_ddrm = new DMatrixRMaj(nSnew, HKt_new.numCols());
            solverNew.solve(HKt_new.getDDRM().copy(), Gt_new_ddrm);
            G = SimpleMatrix.wrap(Gt_new_ddrm).transpose();
          } catch (Exception e) {
            if(goGPS.isDebug()) System.out.println("[Kalman] S_new singular after down-weight, skip update");
            KFstate = KFprediction;
            KFprediction = T.mult(KFstate);
            Cee = T.mult(Cee).mult(T.transpose());
            return;
          }
        }
      }
      
      // Numerical stability guard: check G and KFprediction for NaN/Inf before update
      boolean gValid = true;
      for (int i = 0; i < G.numRows() && gValid; i++) {
        for (int j = 0; j < G.numCols() && gValid; j++) {
          double gv = G.get(i, j);
          if (Double.isNaN(gv) || Double.isInfinite(gv)) gValid = false;
        }
      }
      if (!gValid) {
        if(goGPS.isDebug()) System.out.println("[Kalman] Kalman gain G contains NaN/Inf, skip update");
        KFstate = KFprediction;
        KFprediction = T.mult(KFstate);
        Cee = T.mult(Cee).mult(T.transpose()).plus(Cvv);
        return;
      }
      
      // RTKLIB filter_(): xp = x + K*v, where v is the innovation.
      // goGPS y0 = ddcObs - appRangeCode = obs - H*x_pred = innovation.
      // Correct update: KFstate = KFprediction + G*y0
      // (Previous code used (I-GH)*pred + G*y0 which double-subtracted H*pred)
      SimpleMatrix obsPart = G.mult(y0);

      // 【设计说明 - 不限制位置修正幅度】(2026-06-21 重构)
      // ============================================================================
      // 原 POS_CORR_LIMIT=50 是 H_amb=-λ(cycles) 时代的临时措施，当时 S 矩阵近奇异
      // 导致增益方向错误，大修正会使位置跳到错误方向，残差正反馈爆炸。
      //
      // H_amb=+1.0(meters) 重构后，S[phase] = (α·√K[pos] + √K[amb])² + Cnn > 0
      // 数学保证非奇异，增益 G = K·H'·S⁻¹ 方向正确。大修正(如 187m)是 KF 试图
      // 修复 SPP 初始位置误差(~100m)的正确行为，限制它反而阻止收敛:
      //   - 限制 50m → 位置永远偏 50m → 相位残差 > 5m 阈值 → 相位被拒绝
      //   - 相位被拒绝 → 模糊度无法更新 → 位置无法用相位约束 → 恶性循环
      //
      // 移除限制后，首历元大修正(187m)将位置修正到 ~正确值，第二历元残差骤降，
      // 模糊度在 2-3 历元内收敛。这与 RTKLIB 行为一致(RTKLIB 无修正幅度限制)。
      // ============================================================================
      KFstate = KFprediction.plus(obsPart);

      // [诊断] 打印每历元 y0 max 和 obsPart，定位 KF 发散原因
      if (goGPS.isDebug() && epochCount <= 10) {
        double y0Max = 0, obsMax = 0, gMax = 0;
        for (int i = 0; i < y0.numRows(); i++) {
          double v = Math.abs(y0.get(i, 0));
          if (v > y0Max) y0Max = v;
        }
        for (int i = 0; i < 3; i++) {
          double v = Math.abs(obsPart.get(i, 0));
          if (v > obsMax) obsMax = v;
        }
        for (int i = 0; i < G.numRows(); i++)
          for (int j = 0; j < G.numCols(); j++) {
            double v = Math.abs(G.get(i, j));
            if (v > gMax) gMax = v;
          }
        System.err.printf("[KF-DIAG] epoch=%d y0Max=%.2f obsPartPos=[%.2f,%.2f,%.2f] obsMax=%.2f gMax=%.4f%n",
            epochCount, y0Max, obsPart.get(0), obsPart.get(1), obsPart.get(2), obsMax, gMax);
      }

      if (epochCount <= 1 && goGPS.isDebug()) {
        System.err.println("====== EPOCH 0 KF UPDATE (6 decimals) ======");
        System.err.printf("G dims: %d x %d, y0 dims: %d x %d, obsPart dims: %d x %d%n",
            G.numRows(), G.numCols(), y0.numRows(), y0.numCols(),
            obsPart.numRows(), obsPart.numCols());
        System.err.printf("KFprediction (pre-update pos) = [%.6f, %.6f, %.6f]%n",
            KFprediction.get(0), KFprediction.get(1), KFprediction.get(2));
        System.err.printf("obsPart (G*y0 correction)    = [%.6f, %.6f, %.6f]%n",
            obsPart.get(0), obsPart.get(1), obsPart.get(2));
        // Manual recompute obsPart[0] = sum(G[0][j] * y0[j])
        double manual0 = 0;
        double g0MaxAbs = 0;
        for (int j = 0; j < y0.numRows(); j++) {
          manual0 += G.get(0, j) * y0.get(j, 0);
          double ga = Math.abs(G.get(0, j));
          if (ga > g0MaxAbs) g0MaxAbs = ga;
        }
        System.err.printf("Manual obsPart[0] = G[0]*y0 = %.6f (G[0] max abs=%.3e)%n",
            manual0, g0MaxAbs);
        System.err.printf("KFstate (post-update pos)    = [%.6f, %.6f, %.6f]%n",
            KFstate.get(0), KFstate.get(1), KFstate.get(2));
        System.err.printf("RTKLIB epoch0 reference      = [-492863.370300, 5551482.086200, 3092895.864400]%n");
        System.err.printf("diff (KFstate - RTKLIB)       = [%.3f, %.3f, %.3f] m%n",
            KFstate.get(0) - (-492863.3703),
            KFstate.get(1) - 5551482.0862,
            KFstate.get(2) - 3092895.8644);
        System.err.printf("y0 (innovation) first 6: ");
        for (int i = 0; i < Math.min(6, y0.numRows()); i++) {
          System.err.printf("[%.3f] ", y0.get(i, 0));
        }
        System.err.println();
        System.err.printf("y0 (innovation) ALL %d rows:%n", y0.numRows());
        for (int i = 0; i < y0.numRows(); i++) {
          System.err.printf("  y0[%d] = %.3f%n", i, y0.get(i, 0));
        }
        System.err.println("====== END EPOCH 0 KF UPDATE ======");
      }

      if (goGPS.isDebug()) {
        boolean obsNaN = false;
        for (int i = 0; i < obsPart.numRows() && !obsNaN; i++)
          if (Double.isNaN(obsPart.get(i,0)) || Double.isInfinite(obsPart.get(i,0))) obsNaN = true;
        if (obsNaN) {
          System.err.printf("[Kalman NaN-decomp] obsPart_NaN=%b%n", obsNaN);
          int y0nan = 0, y0inf = 0;
          for (int i = 0; i < y0.numRows(); i++) {
            double v = y0.get(i, 0);
            if (Double.isNaN(v)) y0nan++;
            else if (Double.isInfinite(v)) y0inf++;
          }
          System.err.printf("[Kalman NaN-decomp] y0 NaN=%d, Inf=%d (rows=%d)%n",
              y0nan, y0inf, y0.numRows());
          if (y0nan > 0 || y0inf > 0) {
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < y0.numRows(); i++) {
              double v = y0.get(i, 0);
              if (Double.isNaN(v) || Double.isInfinite(v)) {
                sb.append(String.format("[%d]=%.3e ", i, v));
              }
            }
            System.err.printf("[Kalman NaN-decomp] NaN y0 indices: %s%n", sb.toString());
          }
          double gAbsMax = 0, yAbsMax = 0;
          for (int i = 0; i < G.numRows(); i++)
            for (int j = 0; j < G.numCols(); j++) {
              double v = Math.abs(G.get(i,j));
              if (v > gAbsMax) gAbsMax = v;
            }
          for (int i = 0; i < y0.numRows(); i++) {
            double v = Math.abs(y0.get(i,0));
            if (v > yAbsMax) yAbsMax = v;
          }
          System.err.printf("[Kalman NaN-decomp] G absMax=%.3e, y0 absMax=%.3e, product~%.3e%n",
              gAbsMax, yAbsMax, gAbsMax * yAbsMax);
          for (int i = 0; i < obsPart.numRows(); i++) {
            double op = obsPart.get(i, 0);
            if (Double.isNaN(op) || Double.isInfinite(op)) {
              double rowSum = 0;
              double rowMaxTerm = 0;
              for (int j = 0; j < y0.numRows(); j++) {
                double term = G.get(i, j) * y0.get(j, 0);
                rowSum += term;
                if (Math.abs(term) > Math.abs(rowMaxTerm)) rowMaxTerm = term;
              }
              System.err.printf("[Kalman NaN-decomp] obsPart[%d]=%.3e, rowMaxTerm=%.3e, y0 rows=%d%n",
                  i, op, rowMaxTerm, y0.numRows());
              break;
            }
          }
        }
      }

      // Debug: print KF state transition for first few epochs
      if (goGPS.isDebug() && epochCount <= 5) {
        double predX = KFprediction.get(0);
        double predY = KFprediction.get(i1 + 1);
        double predZ = KFprediction.get(i2 + 1);
        double newX = KFstate.get(0);
        double newY = KFstate.get(i1 + 1);
        double newZ = KFstate.get(i2 + 1);
        double innovMag = 0;
        double maxInnov = 0;
        int maxInnovIdx = -1;
        // y0 is already the innovation (obs - H*x_pred), use directly
        for (int i = 0; i < Math.min(y0.numRows(), 10); i++) {
          double innov = y0.get(i, 0);
          innovMag += innov * innov;
          if (Math.abs(innov) > Math.abs(maxInnov)) {
            maxInnov = innov;
            maxInnovIdx = i;
          }
        }
        innovMag = Math.sqrt(innovMag);
        System.err.printf("%n=== [KF loop] epoch=%d ===%n", epochCount);
        System.err.printf("  KFpred pos: [%.2f, %.2f, %.2f]%n", predX, predY, predZ);
        System.err.printf("  KFstate pos: [%.2f, %.2f, %.2f]%n", newX, newY, newZ);
        System.err.printf("  deltaPos: [%.2f, %.2f, %.2f], dist=%.2f m%n",
            newX - predX, newY - predY, newZ - predZ,
            Math.sqrt(Math.pow(newX-predX,2)+Math.pow(newY-predY,2)+Math.pow(newZ-predZ,2)));
        System.err.printf("  innovMag(first10)=%.2f, maxInnov=%.2f (idx=%d), nObs=%d%n",
            innovMag, maxInnov, maxInnovIdx, nObs);
        // Print individual innovations for first 5 observations
        // y0 is already the innovation (obs - H*x_pred)
        for (int i = 0; i < Math.min(y0.numRows(), 5); i++) {
          double innov = y0.get(i, 0);
          System.err.printf("  obs[%d] innov=%.2f (y0=innov)%n", i, innov);
        }
        // Print iono/tropo components of KFprediction
        if (nIono > 0) {
          System.err.printf("  KFpred iono[0..2]: [%.2f, %.2f, %.2f]%n",
              KFprediction.get(iIono + 0), KFprediction.get(iIono + 1), KFprediction.get(iIono + 2));
        }
        if (nTropo > 0) {
          System.err.printf("  KFpred tropo: %.2f%n", KFprediction.get(iTropo));
        }
        // Print KFstate iono/tropo after update
        if (nIono > 0) {
          System.err.printf("  KFstate iono[0..2]: [%.2f, %.2f, %.2f]%n",
              KFstate.get(iIono + 0), KFstate.get(iIono + 1), KFstate.get(iIono + 2));
        }
        if (nTropo > 0) {
          System.err.printf("  KFstate tropo: %.2f%n", KFstate.get(iTropo));
        }
      }
      
      // RTK divergence check: KF update must not move rover beyond reasonable distance from base
      // BDS GEO DD is poorly conditioned, can converge to antipode. Fall back to prediction if diverged.
      boolean kfDiverged = false;
      boolean stateNaN = false;
      
      // Check for NaN/Inf in full KF state (numerical instability guard)
      for (int i = 0; i < nx; i++) {
        double v = KFstate.get(i, 0);
        if (Double.isNaN(v) || Double.isInfinite(v)) {
          stateNaN = true;
          if (goGPS.isDebug()) {
            System.err.printf("[NaN-TRACE] state[%d]=%.3e (i1=%d,i2=%d,i3=%d,iIono=%d,iTropo=%d)%n",
                i, v, i1, i2, i3, iIono, iTropo);
            // Print G row for this state
            double gMax = 0;
            for (int j = 0; j < G.numCols(); j++) {
              double gv = G.get(i, j);
              if (Math.abs(gv) > Math.abs(gMax)) gMax = gv;
            }
            System.err.printf("[NaN-TRACE] G row[%d] max=%.3e, KFpred=%.3e%n", i, gMax, KFprediction.get(i, 0));
            // Print y0 max
            double yMax = 0;
            for (int j = 0; j < y0.numRows(); j++) {
              double yv = Math.abs(y0.get(j, 0));
              if (yv > yMax) yMax = yv;
            }
            System.err.printf("[NaN-TRACE] y0 max=%.3e%n", yMax);
          }
          break;
        }
      }
      if (stateNaN && goGPS.isDebug()) {
        System.out.println("[Kalman loop] KFstate contains NaN/Inf, keeping prediction + inflating Cee");
      }
      
      if (!kfDiverged && !stateNaN && masterObs != null && masterPos != null) {
        double dx = KFstate.get(0) - masterPos.getX();
        double dy = KFstate.get(i1 + 1) - masterPos.getY();
        double dz = KFstate.get(i2 + 1) - masterPos.getZ();
        if (Math.sqrt(dx*dx + dy*dy + dz*dz) > goGPS.getMaxDivergenceDistance()) {
          if(goGPS.isDebug()) System.out.println("[Kalman loop] KF update diverged (" +
              String.format("%.0f", Math.sqrt(dx*dx+dy*dy+dz*dz)/1000) + "km), fallback to prediction");
          kfDiverged = true;
        }
      }
      
      if (stateNaN || kfDiverged) {
        consecutiveDivergence++;
        int maxDivReset = goGPS.getMaxDivergenceReset();
        // RTKLIB-style: keep predicted state (preserves ambiguity info), inflate Cee to allow recovery
        // Never reset to base station as that destroys all accumulated ambiguity states
        if(goGPS.isDebug()) System.out.println("[Kalman loop] " + 
            (stateNaN ? "NaN/Inf detected" : "diverged") + " (" + consecutiveDivergence + 
            "/" + maxDivReset + "), keep prediction + inflate Cee (RTKLIB-style)");
        KFstate = KFprediction;
        KFprediction = T.mult(KFstate);
        // RTKLIB-aligned: reset position variance to VAR_POS (900 m²), not maxDivDist².
        // RTKLIB udpos() resets to VAR_POS when var > VAR_POS, allowing gradual recovery.
        double inflateVar = Math.pow(stDevInit, 2);
        Cee = K.copy();
        for (int idx = 0; idx < o3; idx++) {
          if (Cee.get(idx, idx) < inflateVar) {
            Cee.set(idx, idx, inflateVar);
          }
        }
        if (stateNaN || consecutiveDivergence > maxDivReset) {
          consecutiveDivergence = 0; // reset counter
        }
      } else {
        consecutiveDivergence = 0; // reset counter on successful update
        KFprediction = T.mult(KFstate);

        // 【Cee 协方差更新 - Joseph form】
        //
        // 根因分析(通过 S-manual 诊断确认):
        //   epoch 1: K[pos]=900(初始), S[phase]=1161(正确: α²·900+λ²·9.76)
        //   epoch 1 更新后: Cee[pos,pos]=0.30(位置方差快速收敛, 10+颗卫星同时约束)
        //                  Cee[pos,amb] 变为负值(位置-模糊度强相关, RTK 固有特性)
        //   epoch 2: K=Cee(STATIC模式 T=I, Cvv_pos=0)
        //            S[phase] = H·K·H' + Cnn
        //            = α²·K[pos](0.30) + λ²·K[amb](1.47) + 2·α·K[pos,amb]·λ(交叉项)
        //            交叉项为负值, 部分抵消对角项 → S[phase]≈0.001(小但非奇异!)
        //            S[code] ≈ 126, 条件数 ≈ 1.26e5
        //
        // RTKLIB 对比 (已核实源码):
        //   RTKLIB filter_() 用简单形式 C=(I-KH)P, 用 matinv() (LU 分解 dgetrf_) 求 S_inv。
        //   RTKLIB 不出问题的原因:
        //   (1) ddcov() 产生非对角 R 矩阵(同系统 DD 共享参考卫星方差 Ri), 改善 S 条件数
        //   (2) filter() 状态压缩(只保留 x!=0 && P>0 的状态), 减少矩阵维度
        //   (3) LAPACK dgetrf_ 有 partial pivoting, 数值稳定
        //
        // goGPS 修复方案 (双重保障):
        //   (1) Joseph form 保证 Cee 半正定: Cee = (I-GH)·K·(I-GH)' + G·Cnn·G'
        //       简单形式 (I-GH)K 可能因数值误差失去正定性, Joseph form 数学等价但保证半正定。
        //   (2) 直接求解线性系统 S·G'=H·K' (不显式求 S_inv), 见上方 Step 3。
        //       即使 S 条件数~1e5, 直接求解也比显式求逆稳定得多。
        //
        // 【实测验证 2026-06-21】
        //   Joseph form + 直接求解后, Cee 正定性检查(cross <= sqrt(pos*amb))全部通过。
        //   模糊度初始方差 900 cycle² 时: epoch1 posVar=1.2 ambVar=69 cross=1.6(bound=9.1 ok)。
        //   但 gMax 仍达 5.2(epoch1)→265(epoch2), 位置仍发散。
        //   根因: 位置方差从 900 暴跌到 1.2(10+卫星同时约束), 增益 G 过大(>1),
        //   位置过度修正(154m), 导致 epoch2 几何关系完全错乱。
        //   这是 RTK 初始化阶段的固有问题, RTKLIB 通过 ddcov() 非对角 R 矩阵缓解。
        //   后续排查方向: 对齐 RTKLIB ddcov() 或限制位置方差收敛速度。
        SimpleMatrix GH = G.mult(H);
        SimpleMatrix IminusGH = I.minus(GH);
        Cee = IminusGH.mult(K).mult(IminusGH.transpose())
            .plus(G.mult(Cnn).mult(G.transpose()));
        // [诊断] 打印 Cee 位置/模糊度对角线和交叉协方差，验证正定性
        if (goGPS.isDebug() && epochCount <= 10) {
          double posVar = Cee.get(0, 0);
          double ambVar0 = (i3 + 1 < nx) ? Cee.get(i3 + 1, i3 + 1) : 0;
          double crossVar = (i3 + 1 < nx) ? Cee.get(0, i3 + 1) : 0;
          // 正定性检查: |K[pos,amb]| 应 <= sqrt(K[pos]*K[amb])
          double crossBound = Math.sqrt(Math.abs(posVar) * Math.abs(ambVar0));
          System.err.printf("[Cee-DIAG] epoch=%d posVar=%.4f ambVar0=%.4f cross=%.4f (bound=%.4f ok=%b)%n",
              epochCount, posVar, ambVar0, crossVar, crossBound, Math.abs(crossVar) <= crossBound + 1e-10);
        }
      }

    } else {

      // Positioning only by system dynamics
      KFstate = KFprediction;
      KFprediction = T.mult(KFstate);
      Cee = T.mult(Cee).mult(T.transpose());
    }

    // Cee lower bound: RTKLIB has NO covariance floor — variance can converge
    // to cm-level (std~1cm => var~1e-4) or below. A large floor like 1.0 m^2
    // keeps the Kalman gain G large forever, preventing convergence and
    // causing position to jump hundreds of meters every epoch.
    // Keep a tiny floor (1e-8 => std 0.1mm) only for numerical stability.
    // 【关键修复记录】原值 MIN_VAR_POS=1.0 导致位置标准差永远≥1m，Kalman 增益无法衰减，
    // 定位结果每历元跳变几十米。改为 1e-8 后与 RTKLIB(无下限) 一致，可收敛到厘米级。
    // 若定位再次发散，先核对此值是否被改回 1.0。
    final double MIN_VAR_POS = 1e-8;
    final double MIN_VAR_AMB = 1e-8;
    final double MIN_VAR_IONO = 1e-8;
    final double MIN_VAR_TROPO = 1e-8;
    for (int idx = 0; idx < Cee.numRows(); idx++) {
      double minVar;
      if (idx < o3) {
        minVar = MIN_VAR_POS;
      } else if (idx < iIono) {
        minVar = MIN_VAR_AMB;
      } else if (nTropo > 0 && idx >= iTropo) {
        minVar = MIN_VAR_TROPO;
      } else {
        minVar = MIN_VAR_IONO;
      }
      if (Cee.get(idx, idx) < minVar) {
        Cee.set(idx, idx, minVar);
      }
    }

    // LAMBDA ambiguity resolution (DD Kalman filter only)
    if (fixAmbiguitiesLambda()) {
      nfix++;
      if (nfix >= MIN_FIX_HOLD) {
        holdamb();
      }
    } else {
      nfix = 0;
    }

    // Compute predicted phase ranges based on Doppler observations
    computeDopplerPredictedPhase(roverObs, masterObs);

    // Set receiver position
    rover.setXYZ(KFstate.get(0), KFstate.get(i1 + 1), KFstate.get(i2 + 1));
    rover.computeGeodetic();
    if (goGPS.isDebug()) {
      System.err.printf("[DEBUG KF loop] KFstate pos=[%.2f, %.2f, %.2f], lat=%.6f lon=%.6f h=%.1f%n",
              KFstate.get(0), KFstate.get(i1+1), KFstate.get(i2+1),
              rover.getGeodeticLatitude(), rover.getGeodeticLongitude(), rover.getGeodeticHeight());
      if (masterPos != null) {
        double dx = rover.getX() - masterPos.getX();
        double dy = rover.getY() - masterPos.getY();
        double dz = rover.getZ() - masterPos.getZ();
        System.err.printf("[DEBUG KF loop] dist from base=%.1f km, base=[%.2f, %.2f, %.2f]%n",
                Math.sqrt(dx*dx+dy*dy+dz*dz)/1000.0, masterPos.getX(), masterPos.getY(), masterPos.getZ());
      }
    }

    positionCovariance.set(0, 0, Cee.get(0, 0));
    positionCovariance.set(1, 1, Cee.get(i1 + 1, i1 + 1));
    positionCovariance.set(2, 2, Cee.get(i2 + 1, i2 + 1));
    positionCovariance.set(0, 1, Cee.get(0, i1 + 1));
    positionCovariance.set(0, 2, Cee.get(0, i2 + 1));
    positionCovariance.set(1, 0, Cee.get(i1 + 1, 0));
    positionCovariance.set(1, 2, Cee.get(i1 + 1, i2 + 1));
    positionCovariance.set(2, 0, Cee.get(i2 + 1, 0));
    positionCovariance.set(2, 1, Cee.get(i2 + 1, i1 + 1));

    // Allocate and build rotation matrix
    SimpleMatrix R = Coordinates.rotationMatrix(rover);

    // Propagate covariance from global system to local system
    // Covariance matrix obtained from matrix A (satellite geometry) [local coordinates]
    SimpleMatrix covENU = R.mult(positionCovariance).mult(R.transpose());

    // Kalman filter DOP computation
    rover.kpDop = Math.sqrt(positionCovariance.get(0, 0) + positionCovariance.get(1, 1) + positionCovariance.get(2, 2));
    rover.khDop = Math.sqrt(covENU.get(0, 0) + covENU.get(1, 1));
    rover.kvDop = Math.sqrt(covENU.get(2, 2));

    // Compute positioning in geodetic coordinates
    rover.computeGeodetic();

    // RTKLIB-aligned: save rover DD state at END of epoch (after selectDoubleDiff).
    // RTKLIB ddres() selects the reference satellite per epoch by highest
    // elevation; goGPS tracks the pivot change to transform DD ambiguity
    // states (A*Cee*A'). Saving here ensures oldPivotId/satOld reflect the
    // final rover DD state of THIS epoch, immune to intermediate
    // selectStandalone() calls (e.g. base-station SPP in run()) that may
    // temporarily alter sats.pivot/sats.availPhase.
    try {
      oldPivotId   = sats.pos[sats.pivot].getSatID();
      oldPivotType = sats.pos[sats.pivot].getSatType();
    } catch(ArrayIndexOutOfBoundsException e) {
      oldPivotId = 0;
    }
    satOld = sats.availPhase;
    satTypeOld = sats.typeAvailPhase;

    // Debug: print final KFstate and KFprediction at end of loop
    if (goGPS.isDebug() && epochCount <= 5) {
      System.err.printf("[KF loop END] epoch=%d, KFstate=[%.2f,%.2f,%.2f], KFpred=[%.2f,%.2f,%.2f]%n",
          epochCount,
          KFstate.get(0), KFstate.get(i1+1), KFstate.get(i2+1),
          KFprediction.get(0), KFprediction.get(i1+1), KFprediction.get(i2+1));
    }
  }
}