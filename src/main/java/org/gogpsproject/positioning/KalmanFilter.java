package org.gogpsproject.positioning;

import java.util.ArrayList;
import java.util.Objects;

import org.ejml.simple.SimpleMatrix;
import org.gogpsproject.GoGPS;
import org.gogpsproject.producer.ObservationSet;
import org.gogpsproject.producer.Observations;

public abstract class KalmanFilter extends LS_DD_code {

  /** Initial position st dev (m) */
  private static final double stDevInit = 1;

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
    nAmbL2 = estimIono ? nN : 0;
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
    // When DD diverged, SPP position is uncertain (~100km), use large initial covariance
    double initPosVar = ddDiverged ? Math.pow(maxDivDist, 2) : Math.pow(stDevInit, 2);
    if(positionCovariance != null) {
      // Full copy of 3x3 position covariance matrix (RTKLIB-aligned: preserve off-diagonal terms)
      for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
          int kr = (row == 0) ? 0 : (row == 1) ? (i1 + 1) : (i2 + 1);
          int kc = (col == 0) ? 0 : (col == 1) ? (i1 + 1) : (i2 + 1);
          Cee.set(kr, kc, positionCovariance.get(row, col));
        }
      }
    } else {
      positionCovariance = new SimpleMatrix(3, 3);
      Cee.set(0, 0, initPosVar);
      Cee.set(i1 + 1, i1 + 1, initPosVar);
      Cee.set(i2 + 1, i2 + 1, initPosVar);
    }
    for (int i = 1; i < o1; i++) {
      Cee.set(i, i, initPosVar);
      Cee.set(i + i1 + 1, i + i1 + 1, initPosVar);
      Cee.set(i + i2 + 1, i + i2 + 1, initPosVar);
    }

    // Initialize ionospheric and tropospheric states (RTKLIB-aligned)
    if (estimIono) {
      double sigIono2 = Math.pow(goGPS.getSigIono(), 2);
      for (int i = 0; i < nIono; i++) {
        Cee.set(iIono + i, iIono + i, sigIono2);
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

    final int minNumSat = 2;

    // Set linearization point (approximate coordinates by KF prediction at previous step)
    rover.setXYZ(KFprediction.get(0), KFprediction.get(i1 + 1), KFprediction.get(i2 + 1));

    // Debug: print KFprediction at the very beginning of loop
    if (goGPS.isDebug() && epochCount <= 5) {
      System.err.printf("[KF loop START] epoch=%d, KFpred=[%.2f, %.2f, %.2f]%n",
          epochCount, KFprediction.get(0), KFprediction.get(i1+1), KFprediction.get(i2+1));
    }

    // Save previous list of available satellites with phase
    satOld = sats.availPhase;
    satTypeOld = sats.typeAvailPhase;

    // Save the ID and index of the previous sats.pivot satellite
    try {
      oldPivotId   = sats.pos[sats.pivot].getSatID();
      oldPivotType = sats.pos[sats.pivot].getSatType();
    } catch(ArrayIndexOutOfBoundsException e) {
      oldPivotId = 0;
    }

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
    int nObs = goGPS.isDualFreq() ? l1Obs * 2 : l1Obs;

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

      // Ionospheric and tropospheric process noise (RTKLIB-aligned, dynamic dt)
      if (nIono > 0) {
        double prnI = goGPS.getPrnIono() * dt;
        for (int i = 0; i < nIono; i++) {
          Cvv.set(iIono + i, iIono + i, prnI);
        }
      }
      if (nTropo > 0) {
        Cvv.set(iTropo, iTropo, goGPS.getPrnTropo() * dt);
      }
      
      // Rebuild state transition matrix with current dt (RTKLIB-aligned)
      buildTransitionMatrix(dt);
      
      // Fill in Kalman filter transformation matrix, observation vector and observation error covariance matrix
      setup(roverObs, masterObs, masterPos);
      
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
      SimpleMatrix S = H.mult(K).mult(H.transpose()).plus(Cnn);
      
      // Step 3: check if S is invertible (singularity guard, RTKLIB-style)
      SimpleMatrix S_inv = null;
      try {
        S_inv = S.invert();
      } catch (Exception e) {
        if(goGPS.isDebug()) System.out.println("[Kalman] S matrix singular, skipping update");
        // Fall back to prediction only
        KFstate = KFprediction;
        KFprediction = T.mult(KFstate);
        Cee = T.mult(Cee).mult(T.transpose());
        return;
      }
      
      // Step 4: Kalman gain G = K*H'*S^{-1}
      G = K.mult(H.transpose()).mult(S_inv);
      
      // Unified outlier detection (RTKLIB-aligned: merge residual + chi-square into one round)
      // Detect bad observations, down-weight them, then recompute KF state once
      if( goGPS.searchForOutliers() ) {
        SimpleMatrix Xhat_t_t = I.minus(G.mult(H)).mult(KFprediction).plus(G.mult(y0));
        SimpleMatrix residuals = compute_residuals(Xhat_t_t);
        SimpleMatrix innovation = y0.minus(H.mult(KFprediction));
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
          // Recompute with down-weighted observations
          SimpleMatrix S_new = H.mult(K).mult(H.transpose()).plus(Cnn);
          try {
            SimpleMatrix S_inv_new = S_new.invert();
            G = K.mult(H.transpose()).mult(S_inv_new);
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
      
      KFstate = I.minus(G.mult(H)).mult(KFprediction).plus(G.mult(y0));
      
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
        for (int i = 0; i < Math.min(y0.numRows(), 10); i++) {
          double innov = y0.get(i, 0);
          for (int j = 0; j < nx; j++) {
            innov -= H.get(i, j) * KFprediction.get(j, 0);
          }
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
        for (int i = 0; i < Math.min(y0.numRows(), 5); i++) {
          double innov = y0.get(i, 0);
          for (int j = 0; j < nx; j++) {
            innov -= H.get(i, j) * KFprediction.get(j, 0);
          }
          System.err.printf("  obs[%d] innov=%.2f, y0=%.2f, HdotPred=%.2f%n",
              i, innov, y0.get(i, 0), y0.get(i, 0) - innov);
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
        // Inflate position covariance to allow recovery
        double inflateVar = Math.pow(goGPS.getMaxDivergenceDistance() / 1000.0, 2);
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
        Cee = I.minus(G.mult(H)).mult(K);
      }

    } else {

      // Positioning only by system dynamics
      KFstate = KFprediction;
      KFprediction = T.mult(KFstate);
      Cee = T.mult(Cee).mult(T.transpose());
    }

    // Cee lower bound: prevent KF over-confidence (KHDOP collapsing to 0.37)
    // RTKLIB-style: position states need larger floor (1.0 m^2) to absorb
    // observation quality degradation; ambiguity states use smaller floor (1e-4)
    final double MIN_VAR_POS = 1.0;
    final double MIN_VAR_AMB = 1e-4;
    final double MIN_VAR_IONO = 1e-6;
    final double MIN_VAR_TROPO = 1e-6;
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

    // Debug: print final KFstate and KFprediction at end of loop
    if (goGPS.isDebug() && epochCount <= 5) {
      System.err.printf("[KF loop END] epoch=%d, KFstate=[%.2f,%.2f,%.2f], KFpred=[%.2f,%.2f,%.2f]%n",
          epochCount,
          KFstate.get(0), KFstate.get(i1+1), KFstate.get(i2+1),
          KFprediction.get(0), KFprediction.get(i1+1), KFprediction.get(i2+1));
    }
  }
}