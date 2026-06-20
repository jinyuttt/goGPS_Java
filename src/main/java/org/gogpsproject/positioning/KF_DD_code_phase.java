package org.gogpsproject.positioning;

import java.util.ArrayList;

import org.ejml.simple.SimpleMatrix;
import org.gogpsproject.GoGPS;
import org.gogpsproject.GoGPS.*;
import org.gogpsproject.Status;
import org.gogpsproject.consumer.PositionConsumer;
import org.gogpsproject.positioning.RoverPosition.DopType;
import org.gogpsproject.producer.Observations;
import org.gogpsproject.producer.ObservationsProducer;

public class KF_DD_code_phase extends KalmanFilter {

  private static final int BDS_GEO_MIN = 1;
  private static final int BDS_GEO_MAX = 5;
  private static final double GEO_NOISE_SCALE = 8.0;

  /** RTKLIB-aligned: constraint variance to hold fixed ambiguity (cycle^2) */
  private static final double VAR_HOLDAMB = 0.001;

  /** RTKLIB-aligned: ionospheric mapping function single-layer height (m) */
  private static final double IONO_H = 350000.0;

  /** RTKLIB-aligned: GMF constants */
  private static final double GMF_A = 1.001;
  private static final double GMF_B = 0.002001;

  /**
   * Check if a BDS satellite is GEO based on PRN.
   * BDS-2 GEO: C01-C05 (PRN 1-5). BDS-3 GEO: may use different PRN ranges.
   * TODO: replace PRN range with navigation ephemeris orbit type when available.
   */
  private static boolean isBdsGeo(char satType, int satPrn) {
    return satType == 'C' && satPrn >= BDS_GEO_MIN && satPrn <= BDS_GEO_MAX;
  }

  public KF_DD_code_phase(GoGPS goGPS) {
    super(goGPS);
  }

  @Override
  void setup(Observations roverObs, Observations masterObs, Coordinates masterPos) {

    epochCount++;
    boolean dualFreq = goGPS.isDualFreq();
    boolean estimIono = goGPS.getIonoOpt() != GoGPS.IonoOpt.OFF && dualFreq;
    boolean estimTropo = goGPS.getTropOpt() != GoGPS.TropOpt.OFF && dualFreq;

    // Number of GPS observations
    int nObs = roverObs.getNumSat();

    // Number of available satellites (i.e. observations)
    int nObsAvail = sats.avail.size();

    // Double differences with respect to pivot satellite reduce observations by 1
    nObsAvail--;

    // Number of available satellites with phase (DD)
    int nObsAvailPhase = sats.availPhase.size() - 1;

    // Matrix containing parameters obtained from the linearization of the observation equations
    SimpleMatrix A = new SimpleMatrix(nObsAvail, 3);

    // Pivot satellite ID
    int pivotId = roverObs.getSatID(sats.pivot);
    char satType = roverObs.getGnssType(sats.pivot);

    // Store rover-pivot and master-pivot observed pseudoranges (L1)
    double roverPivotCodeObs = roverObs.getSatByIDType(pivotId, satType).getPseudorange(0);
    double masterPivotCodeObs = masterObs.getSatByIDType(pivotId, satType).getPseudorange(0);

    // Compute and store rover-pivot and master-pivot observed phase ranges (L1)
    double roverPivotPhaseObs = roverObs.getSatByIDType(pivotId, satType).getPhaserange(0);
    double masterPivotPhaseObs = masterObs.getSatByIDType(pivotId, satType).getPhaserange(0);

    // L2 pivot observations (for dual-frequency)
    double roverPivotCodeObsL2 = 0, masterPivotCodeObsL2 = 0;
    double roverPivotPhaseObsL2 = 0, masterPivotPhaseObsL2 = 0;
    double gamma = 1.0; // (f1/f2)^2 = (lambda1/lambda2)^2
    if (dualFreq) {
      roverPivotCodeObsL2 = roverObs.getSatByIDType(pivotId, satType).getPseudorange(1);
      masterPivotCodeObsL2 = masterObs.getSatByIDType(pivotId, satType).getPseudorange(1);
      roverPivotPhaseObsL2 = roverObs.getSatByIDType(pivotId, satType).getPhaserange(1);
      masterPivotPhaseObsL2 = masterObs.getSatByIDType(pivotId, satType).getPhaserange(1);
      double lam1 = roverObs.getSatByIDType(pivotId, satType).getWavelength(0);
      double lam2 = roverObs.getSatByIDType(pivotId, satType).getWavelength(1);
      gamma = (lam1 * lam1) / (lam2 * lam2);
    }

    // Rover-pivot approximate pseudoranges
    SimpleMatrix diffRoverPivot = rover.diffSat[sats.pivot];
    double roverPivotAppRange = rover.satAppRange[sats.pivot];

    // Master-pivot approximate pseudoranges
    double masterPivotAppRange = master.satAppRange[sats.pivot];

    // Rover-pivot and master-pivot troposphere correction
    double roverPivotTropoCorr = rover.satTropoCorr[sats.pivot];
    double masterPivotTropoCorr = master.satTropoCorr[sats.pivot];

    // Rover-pivot and master-pivot ionosphere correction (L1)
    double roverPivotIonoCorr = rover.satIonoCorr[sats.pivot];
    double masterPivotIonoCorr = master.satIonoCorr[sats.pivot];

    // Rover-pivot and master-pivot antenna correction
    double roverPivotAntCorr = rover.satAntennaCorr[sats.pivot];
    double masterPivotAntCorr = master.satAntennaCorr[sats.pivot];

    // Rover-pivot and master-pivot phase wind-up correction
    double roverPivotWindUp = rover.satWindUp[sats.pivot];
    double masterPivotWindUp = master.satWindUp[sats.pivot];

    // Compute rover-pivot and master-pivot elevations
    double roverElevation = rover.topo[sats.pivot].getElevation();
    double masterElevation = master.topo[sats.pivot].getElevation();

    // Troposphere wet mapping function (RTKLIB GMF-aligned)
    // mw(el) = 1.001 / sqrt(0.002001 + sin^2(el))
    double roverPivotMw = tropoMapWet(roverElevation);
    double masterPivotMw = tropoMapWet(masterElevation);

    // Start filling in the observation error covariance matrix
    Cnn.zero();
    int nSatAvail = sats.avail.size() - 1;
    int nSatAvailPhase = (sats.availPhase.size() > 0) ? sats.availPhase.size() - 1 : 0;
    int l1Rows = nSatAvail + nSatAvailPhase;
    int totalObs = dualFreq ? l1Rows * 2 : l1Rows;

    if (goGPS.isDebug() && epochCount <= 2) {
      System.err.printf("[KF setup] dualFreq=%b, estimIono=%b, estimTropo=%b, nObs=%d, nObsAvail=%d, nSatAvailPhase=%d, l1Rows=%d, totalObs=%d%n",
          dualFreq, estimIono, estimTropo, nObs, nObsAvail, nSatAvailPhase, l1Rows, totalObs);
    }

    // Pre-fill Cnn with pivot baseline noise for L1 rows
    for (int i = 0; i < l1Rows; i++) {
      double baseNoise;
      if (i < nSatAvail) {
        baseNoise = varerr(Math.toRadians(roverElevation), false, satType, 0)
                  + varerr(Math.toRadians(masterElevation), false, satType, 0);
      } else {
        baseNoise = varerr(Math.toRadians(roverElevation), true, satType, 0)
                  + varerr(Math.toRadians(masterElevation), true, satType, 0);
      }
      Cnn.set(i, i, baseNoise);
    }
    // Pre-fill Cnn with pivot baseline noise for L2 rows
    if (dualFreq) {
      for (int i = 0; i < l1Rows; i++) {
        double baseNoise;
        int idx = l1Rows + i;
        if (i < nSatAvail) {
          baseNoise = varerr(Math.toRadians(roverElevation), false, satType, 1)
                    + varerr(Math.toRadians(masterElevation), false, satType, 1);
        } else {
          baseNoise = varerr(Math.toRadians(roverElevation), true, satType, 1)
                    + varerr(Math.toRadians(masterElevation), true, satType, 1);
        }
        Cnn.set(idx, idx, baseNoise);
      }
    }

    // Counter for available satellites
    int k = 0;
    int p = 0;

    for (int i = 0; i < nObs; i++) {

      int id = roverObs.getSatID(i);
      satType = roverObs.getGnssType(i);
      String checkAvailGnss = String.valueOf(satType) + String.valueOf(id);

      if (sats.pos[i]!=null && sats.gnssAvail.contains(checkAvailGnss) && i != sats.pivot) {

        // Compute parameters obtained from linearization of observation equations
        double alphaX = rover.diffSat[i].get(0) / rover.satAppRange[i]
            - diffRoverPivot.get(0) / roverPivotAppRange;
        double alphaY = rover.diffSat[i].get(1) / rover.satAppRange[i]
            - diffRoverPivot.get(1) / roverPivotAppRange;
        double alphaZ = rover.diffSat[i].get(2) / rover.satAppRange[i]
            - diffRoverPivot.get(2) / roverPivotAppRange;

        // Fill in the A matrix
        A.set(k, 0, alphaX);
        A.set(k, 1, alphaY);
        A.set(k, 2, alphaZ);

        // Approximate code double difference
        double ddcApp = (rover.satAppRange[i] - master.satAppRange[i])
            - (roverPivotAppRange - masterPivotAppRange);

        // Observed code double difference (L1)
        double ddcObs = (roverObs.getSatByIDType(id, satType).getPseudorange(0) - masterObs.getSatByIDType(id, satType).getPseudorange(0))
            - (roverPivotCodeObs - masterPivotCodeObs);

        // Observed phase double difference (L1)
        double ddpObs = (roverObs.getSatByIDType(id, satType).getPhaserange(0) - masterObs.getSatByIDType(id, satType).getPhaserange(0))
            - (roverPivotPhaseObs - masterPivotPhaseObs);

        // Compute troposphere, ionosphere and antenna residuals
        double tropoResiduals = (rover.satTropoCorr[i] - master.satTropoCorr[i])
            - (roverPivotTropoCorr - masterPivotTropoCorr);
        double ionoResiduals = (rover.satIonoCorr[i] - master.satIonoCorr[i])
            - (roverPivotIonoCorr - masterPivotIonoCorr);
        double antResiduals = (rover.satAntennaCorr[i] - master.satAntennaCorr[i])
            - (roverPivotAntCorr - masterPivotAntCorr);
        double windUpResiduals = ((rover.satWindUp[i] - master.satWindUp[i])
            - (roverPivotWindUp - masterPivotWindUp))
            * roverObs.getSatByIDType(id, satType).getWavelength(0);

        // Troposphere DD wet mapping function
        double roverMw = tropoMapWet(rover.topo[i].getElevation());
        double masterMw = tropoMapWet(master.topo[i].getElevation());
        double mwDD = (roverMw - roverPivotMw) - (masterMw - masterPivotMw);

        // Compute approximate ranges (with/without iono estimation)
        double appRangeCode;
        double appRangePhase;
        if (estimIono) {
          // IONOOPT_EST: ionosphere is estimated, NOT added to approximate range
          appRangeCode = ddcApp + tropoResiduals + antResiduals;
          appRangePhase = ddcApp + tropoResiduals + antResiduals + windUpResiduals;
        } else {
          appRangeCode = ddcApp + tropoResiduals + ionoResiduals + antResiduals;
          appRangePhase = ddcApp + tropoResiduals - ionoResiduals + antResiduals + windUpResiduals;
        }

        // === L1 Code Row ===
        H.set(k, 0, alphaX);
        H.set(k, i1 + 1, alphaY);
        H.set(k, i2 + 1, alphaZ);

        // Ionosphere column (RTKLIB: +I for L1 code)
        if (estimIono) {
          H.set(k, iIono + id, 1.0);
        }
        // Troposphere column (RTKLIB: +mw_dd * dTrop)
        if (estimTropo) {
          H.set(k, iTropo, mwDD);
        }

        y0.set(k, 0, ddcObs - appRangeCode + alphaX * rover.getX()
            + alphaY * rover.getY() + alphaZ * rover.getZ());

        if (goGPS.isDebug() && k == 0) {
          System.err.printf("[DEBUG KF setup] sat %c%d: ddcObs=%.2f, appRangeCode=%.2f, ddcObs-appRangeCode=%.2f%n",
                  satType, id, ddcObs, appRangeCode, ddcObs - appRangeCode);
          System.err.printf("[DEBUG KF setup] sat %c%d: alpha=[%.6f,%.6f,%.6f], rover=[%.2f,%.2f,%.2f]%n",
                  satType, id, alphaX, alphaY, alphaZ, rover.getX(), rover.getY(), rover.getZ());
          System.err.printf("[DEBUG KF setup] sat %c%d: satPos=[%.2f,%.2f,%.2f], roverAppRange=%.2f, masterAppRange=%.2f%n",
                  satType, id, sats.pos[i].getX(), sats.pos[i].getY(), sats.pos[i].getZ(),
                  rover.satAppRange[i], master.satAppRange[i]);
          System.err.printf("[DEBUG KF setup] sat %c%d: ddcApp=%.2f, tropoRes=%.2f, ionoRes=%.2f, antRes=%.2f%n",
                  satType, id, ddcApp, tropoResiduals, ionoResiduals, antResiduals);
        }

        // Print all sat DD residuals for first epoch
        if (goGPS.isDebug() && epochCount == 1) {
          System.err.printf("[DEBUG KF DD] %c%02d: ddcObs=%.2f, appRangeCode=%.2f, res=%.2f, ddpObs=%.2f, appRangePhase=%.2f, phaseRes=%.2f%n",
                  satType, id, ddcObs, appRangeCode, ddcObs - appRangeCode,
                  ddpObs, appRangePhase, ddpObs - appRangePhase);
        }

        // L1 code noise
        double roverCodeVar = varerr(Math.toRadians(rover.topo[i].getElevation()), false, satType, 0);
        double masterCodeVar = varerr(Math.toRadians(master.topo[i].getElevation()), false, satType, 0);

        int satPrn = id;
        if (isBdsGeo(satType, satPrn)) {
          roverCodeVar *= GEO_NOISE_SCALE;
          masterCodeVar *= GEO_NOISE_SCALE;
        }

        double CnnBase = Cnn.get(k, k);
        Cnn.set(k, k, CnnBase + roverCodeVar + masterCodeVar);

        // === L1 Phase Row ===
        if (sats.gnssAvail.contains(checkAvailGnss)) {

          H.set(nObsAvail + p, 0, alphaX);
          H.set(nObsAvail + p, i1 + 1, alphaY);
          H.set(nObsAvail + p, i2 + 1, alphaZ);
          H.set(nObsAvail + p, i3 + id, -roverObs.getSatByIDType(id, satType).getWavelength(0));

          // Ionosphere column (RTKLIB: -I for L1 phase)
          if (estimIono) {
            H.set(nObsAvail + p, iIono + id, -1.0);
          }
          // Troposphere column
          if (estimTropo) {
            H.set(nObsAvail + p, iTropo, mwDD);
          }

          y0.set(nObsAvail + p, 0, ddpObs - appRangePhase + alphaX * rover.getX()
              + alphaY * rover.getY() + alphaZ * rover.getZ());

          double roverPhaseVar = varerr(Math.toRadians(rover.topo[i].getElevation()), true, satType, 0);
          double masterPhaseVar = varerr(Math.toRadians(master.topo[i].getElevation()), true, satType, 0);

          if (isBdsGeo(satType, satPrn)) {
            roverPhaseVar *= GEO_NOISE_SCALE;
            masterPhaseVar *= GEO_NOISE_SCALE;
          }

          CnnBase = Cnn.get(nObsAvail + p, nObsAvail + p);
          Cnn.set(nObsAvail + p, nObsAvail + p, CnnBase + roverPhaseVar + masterPhaseVar);

          p++;
        }

        k++;
      }
    }

    // === L2 Observations (dual-frequency) ===
    if (dualFreq) {
      int l2Base = l1Rows; // L2 rows start after L1 rows
      int k2 = 0;
      int p2 = 0;

      for (int i = 0; i < nObs; i++) {

        int id = roverObs.getSatID(i);
        satType = roverObs.getGnssType(i);
        String checkAvailGnss = String.valueOf(satType) + String.valueOf(id);

        if (sats.pos[i]!=null && sats.gnssAvail.contains(checkAvailGnss) && i != sats.pivot) {

          double alphaX = rover.diffSat[i].get(0) / rover.satAppRange[i]
              - diffRoverPivot.get(0) / roverPivotAppRange;
          double alphaY = rover.diffSat[i].get(1) / rover.satAppRange[i]
              - diffRoverPivot.get(1) / roverPivotAppRange;
          double alphaZ = rover.diffSat[i].get(2) / rover.satAppRange[i]
              - diffRoverPivot.get(2) / roverPivotAppRange;

          double ddcApp = (rover.satAppRange[i] - master.satAppRange[i])
              - (roverPivotAppRange - masterPivotAppRange);

          // L2 observed code DD
          double ddcObsL2 = (roverObs.getSatByIDType(id, satType).getPseudorange(1) - masterObs.getSatByIDType(id, satType).getPseudorange(1))
              - (roverPivotCodeObsL2 - masterPivotCodeObsL2);

          // L2 observed phase DD
          double ddpObsL2 = (roverObs.getSatByIDType(id, satType).getPhaserange(1) - masterObs.getSatByIDType(id, satType).getPhaserange(1))
              - (roverPivotPhaseObsL2 - masterPivotPhaseObsL2);

          double tropoResiduals = (rover.satTropoCorr[i] - master.satTropoCorr[i])
              - (roverPivotTropoCorr - masterPivotTropoCorr);
          double ionoResiduals = (rover.satIonoCorr[i] - master.satIonoCorr[i])
              - (roverPivotIonoCorr - masterPivotIonoCorr);
          double antResiduals = (rover.satAntennaCorr[i] - master.satAntennaCorr[i])
              - (roverPivotAntCorr - masterPivotAntCorr);
          double windUpResidualsL2 = ((rover.satWindUp[i] - master.satWindUp[i])
              - (roverPivotWindUp - masterPivotWindUp))
              * roverObs.getSatByIDType(id, satType).getWavelength(1);

          double roverMw = tropoMapWet(rover.topo[i].getElevation());
          double masterMw = tropoMapWet(master.topo[i].getElevation());
          double mwDD = (roverMw - roverPivotMw) - (masterMw - masterPivotMw);

          // L2 approximate range (iono scaled by gamma = (f1/f2)^2)
          double appRangeCodeL2;
          double appRangePhaseL2;
          if (estimIono) {
            appRangeCodeL2 = ddcApp + tropoResiduals + antResiduals;
            appRangePhaseL2 = ddcApp + tropoResiduals + antResiduals + windUpResidualsL2;
          } else {
            appRangeCodeL2 = ddcApp + tropoResiduals + ionoResiduals * gamma + antResiduals;
            appRangePhaseL2 = ddcApp + tropoResiduals - ionoResiduals * gamma + antResiduals + windUpResidualsL2;
          }

          // === L2 Code Row ===
          int rowL2Code = l2Base + k2;
          H.set(rowL2Code, 0, alphaX);
          H.set(rowL2Code, i1 + 1, alphaY);
          H.set(rowL2Code, i2 + 1, alphaZ);

          if (estimIono) {
            H.set(rowL2Code, iIono + id, gamma); // +gamma*I for L2 code
          }
          if (estimTropo) {
            H.set(rowL2Code, iTropo, mwDD);
          }

          y0.set(rowL2Code, 0, ddcObsL2 - appRangeCodeL2 + alphaX * rover.getX()
              + alphaY * rover.getY() + alphaZ * rover.getZ());

          double roverCodeVarL2 = varerr(Math.toRadians(rover.topo[i].getElevation()), false, satType, 1);
          double masterCodeVarL2 = varerr(Math.toRadians(master.topo[i].getElevation()), false, satType, 1);

          int satPrn = id;
          if (isBdsGeo(satType, satPrn)) {
            roverCodeVarL2 *= GEO_NOISE_SCALE;
            masterCodeVarL2 *= GEO_NOISE_SCALE;
          }

          double CnnBaseL2 = Cnn.get(rowL2Code, rowL2Code);
          Cnn.set(rowL2Code, rowL2Code, CnnBaseL2 + roverCodeVarL2 + masterCodeVarL2);

          // === L2 Phase Row ===
          if (sats.gnssAvail.contains(checkAvailGnss)) {
            int rowL2Phase = l2Base + nSatAvail + p2;
            H.set(rowL2Phase, 0, alphaX);
            H.set(rowL2Phase, i1 + 1, alphaY);
            H.set(rowL2Phase, i2 + 1, alphaZ);
            H.set(rowL2Phase, iAmbL2 + id, -roverObs.getSatByIDType(id, satType).getWavelength(1));

            if (estimIono) {
              H.set(rowL2Phase, iIono + id, -gamma); // -gamma*I for L2 phase
            }
            if (estimTropo) {
              H.set(rowL2Phase, iTropo, mwDD);
            }

            y0.set(rowL2Phase, 0, ddpObsL2 - appRangePhaseL2 + alphaX * rover.getX()
                + alphaY * rover.getY() + alphaZ * rover.getZ());

            double roverPhaseVarL2 = varerr(Math.toRadians(rover.topo[i].getElevation()), true, satType, 1);
            double masterPhaseVarL2 = varerr(Math.toRadians(master.topo[i].getElevation()), true, satType, 1);

            if (isBdsGeo(satType, satPrn)) {
              roverPhaseVarL2 *= GEO_NOISE_SCALE;
              masterPhaseVarL2 *= GEO_NOISE_SCALE;
            }

            CnnBaseL2 = Cnn.get(rowL2Phase, rowL2Phase);
            Cnn.set(rowL2Phase, rowL2Phase, CnnBaseL2 + roverPhaseVarL2 + masterPhaseVarL2);

            p2++;
          }

          k2++;
        }
      }
    }

    // RTKLIB ddcov alignment: add off-diagonal elements to Cnn
    // Within each system-frequency block, DD observations share the same
    // reference satellite (pivot), so their noise is correlated through
    // the pivot noise. Cov(DD_i, DD_j) = pivotNoise for i != j.
    // Reference: RTKLIB rtkpos.c ddcov()

    // L1 code block: rows 0..nSatAvail-1
    double pivotCodeNoise = varerr(Math.toRadians(roverElevation), false, satType, 0)
                          + varerr(Math.toRadians(masterElevation), false, satType, 0);
    if (isBdsGeo(satType, pivotId)) {
      pivotCodeNoise *= GEO_NOISE_SCALE;
    }
    for (int i = 0; i < nSatAvail; i++) {
      for (int j = i + 1; j < nSatAvail; j++) {
        Cnn.set(i, j, pivotCodeNoise);
        Cnn.set(j, i, pivotCodeNoise);
      }
    }

    // L1 phase block: rows nSatAvail..l1Rows-1
    double pivotPhaseNoise = varerr(Math.toRadians(roverElevation), true, satType, 0)
                           + varerr(Math.toRadians(masterElevation), true, satType, 0);
    if (isBdsGeo(satType, pivotId)) {
      pivotPhaseNoise *= GEO_NOISE_SCALE;
    }
    for (int i = nSatAvail; i < l1Rows; i++) {
      for (int j = i + 1; j < l1Rows; j++) {
        Cnn.set(i, j, pivotPhaseNoise);
        Cnn.set(j, i, pivotPhaseNoise);
      }
    }

    if (dualFreq) {
      // L2 code block: rows l1Rows..l1Rows+nSatAvail-1
      double pivotCodeNoiseL2 = varerr(Math.toRadians(roverElevation), false, satType, 1)
                              + varerr(Math.toRadians(masterElevation), false, satType, 1);
      if (isBdsGeo(satType, pivotId)) {
        pivotCodeNoiseL2 *= GEO_NOISE_SCALE;
      }
      for (int i = l1Rows; i < l1Rows + nSatAvail; i++) {
        for (int j = i + 1; j < l1Rows + nSatAvail; j++) {
          Cnn.set(i, j, pivotCodeNoiseL2);
          Cnn.set(j, i, pivotCodeNoiseL2);
        }
      }

      // L2 phase block: rows l1Rows+nSatAvail..totalObs-1
      double pivotPhaseNoiseL2 = varerr(Math.toRadians(roverElevation), true, satType, 1)
                               + varerr(Math.toRadians(masterElevation), true, satType, 1);
      if (isBdsGeo(satType, pivotId)) {
        pivotPhaseNoiseL2 *= GEO_NOISE_SCALE;
      }
      for (int i = l1Rows + nSatAvail; i < totalObs; i++) {
        for (int j = i + 1; j < totalObs; j++) {
          Cnn.set(i, j, pivotPhaseNoiseL2);
          Cnn.set(j, i, pivotPhaseNoiseL2);
        }
      }
    }

    // Debug: print first epoch DD info
    if (goGPS.isDebug() && epochCount <= 5) {
      System.err.printf("%n=== [KF_DD setup] epoch=%d, pivot=%c%d, nSatAvail=%d, nSatAvailPhase=%d, totalObs=%d ===%n",
          epochCount, satType, pivotId, nSatAvail, nSatAvailPhase, totalObs);
      System.err.printf("  roverPos: [%.2f, %.2f, %.2f]%n", rover.getX(), rover.getY(), rover.getZ());
      System.err.printf("  masterPos: [%.2f, %.2f, %.2f]%n", masterPos.getX(), masterPos.getY(), masterPos.getZ());
      for (int row = 0; row < Math.min(6, totalObs); row++) {
        String rowType = "";
        if (row < nSatAvail) rowType = "L1Code";
        else if (row < l1Rows) rowType = "L1Phase";
        else if (dualFreq && row < l1Rows + nSatAvail) rowType = "L2Code";
        else if (dualFreq) rowType = "L2Phase";
        System.err.printf("  [%d %s] y0=%.3f, H=[%.6f, %.6f, %.6f], Cnn=%.4f%n",
            row, rowType, y0.get(row, 0),
            H.get(row, 0), H.get(row, i1 + 1), H.get(row, i2 + 1),
            Cnn.get(row, row));
      }
    }

    updateDops(A);
  }

  /** Troposphere wet mapping function (RTKLIB GMF-aligned).
   *  mw(el) = 1.001 / sqrt(0.002001 + sin^2(el)) */
  private static double tropoMapWet(double elevDeg) {
    double sinE = Math.sin(Math.toRadians(elevDeg));
    return GMF_A / Math.sqrt(GMF_B + sinE * sinE);
  }

  /** Ionospheric single-layer mapping function (RTKLIB-aligned).
   *  im(el) = 1.0 / sqrt(1.0 - (Re * cos(el) / (Re + H))^2)
   *  where Re = 6378137 m, H = 350000 m.
   *  Reference: RTKLIB rtkcmn.c ionmapf() */
  private static double ionoMapF(double elevDeg) {
    double sinE = Math.sin(Math.toRadians(elevDeg));
    double cosE = Math.cos(Math.toRadians(elevDeg));
    double re = 6378137.0;
    double factor = re / (re + IONO_H) * cosE;
    if (factor >= 1.0) return 1.0;
    return 1.0 / Math.sqrt(1.0 - factor * factor);
  }
  
  /**
   * @param roverObs
   * @param masterObs
   * @param masterPos
   */
  @Override
  void estimateAmbiguities( Observations roverObs, Observations masterObs, Coordinates masterPos, ArrayList<Integer> satAmb, int pivotIndex, boolean init) {

    // Check if pivot is in satAmb, in case remove it
    if (satAmb.contains(sats.pos[pivotIndex].getSatID()))
      satAmb.remove(satAmb.indexOf(sats.pos[pivotIndex].getSatID()));

    // Number of GPS observations
    int nObs = roverObs.getNumSat();

    // Number of available satellites (i.e. observations)
    int nObsAvail = sats.avail.size();

    // Number of available satellites (i.e. observations) with phase
    int nObsAvailPhase = sats.availPhase.size();

    // Double differences with respect to pivot satellite reduce
    // observations by 1
    nObsAvail--;
    nObsAvailPhase--;

    // Number of unknown parameters
    int nUnknowns = 3 + satAmb.size();

    // Pivot satellite ID
    int pivotId = roverObs.getSatID(pivotIndex);
    char satType = roverObs.getGnssType(pivotIndex);

    // Rover-pivot and master-pivot observed pseudorange
    double roverPivotCodeObs = roverObs.getSatByIDType(pivotId, satType).getPseudorange(goGPS.getFreq());
    double masterPivotCodeObs = masterObs.getSatByIDType(pivotId, satType).getPseudorange(goGPS.getFreq());

    // Rover-pivot and master-pivot observed phase
    double roverPivotPhaseObs = roverObs.getSatByIDType(pivotId, satType).getPhaserange(goGPS.getFreq());
    double masterPivotPhaseObs = masterObs.getSatByIDType(pivotId, satType).getPhaserange(goGPS.getFreq());

    // Rover-pivot approximate pseudoranges
    SimpleMatrix diffRoverPivot = rover.diffSat[pivotIndex];
    double roverPivotAppRange = rover.satAppRange[pivotIndex];

    // Master-pivot approximate pseudoranges
    double masterPivotAppRange = master.satAppRange[pivotIndex];

    // Estimated ambiguity combinations (double differences)
    double[] estimatedAmbiguityComb = new double[satAmb.size()];

    // Covariance of estimated ambiguity combinations
    double[] estimatedAmbiguityCombCovariance = new double[satAmb.size()];

    if (goGPS.getAmbiguityStrategy() == AmbiguityStrategy.OBSERV) {

      for (int i = 0; i < nObs; i++) {

        int id = roverObs.getSatID(i);
        satType = roverObs.getGnssType(i);

        if (sats.pos[i]!=null && satAmb.contains(id) && id != pivotId) {

          // Rover-satellite and master-satellite observed code
          double roverSatCodeObs = roverObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq());
          double masterSatCodeObs = masterObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq());

          // Rover-satellite and master-satellite observed phase
          double roverSatPhaseObs = roverObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq());
          double masterSatPhaseObs = masterObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq());

          // Observed code double difference
          double codeDoubleDiffObserv = (roverSatCodeObs - masterSatCodeObs) - (roverPivotCodeObs - masterPivotCodeObs);

          // Observed phase double difference
          double phaseDoubleDiffObserv = (roverSatPhaseObs - masterSatPhaseObs) - (roverPivotPhaseObs - masterPivotPhaseObs);

          // Store estimated ambiguity combinations and their covariance
          estimatedAmbiguityComb[satAmb.indexOf(id)] = (codeDoubleDiffObserv - phaseDoubleDiffObserv) / roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq());
          estimatedAmbiguityCombCovariance[satAmb.indexOf(id)] = 4
          * getStDevCode(roverObs.getSatByIDType(id, satType), goGPS.getFreq())
          * getStDevCode(masterObs.getSatByIDType(id, satType), goGPS.getFreq()) / Math.pow(roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq()), 2);
        }
      }
    } 
    else if(goGPS.getAmbiguityStrategy() == AmbiguityStrategy.APPROX | (nObsAvail + nObsAvailPhase <= nUnknowns)) {

      for (int i = 0; i < nObs; i++) {

        int id = roverObs.getSatID(i);
        satType = roverObs.getGnssType(i);

        if( sats.pos[i]!=null && satAmb.contains(id) && id != pivotId) {

          // Rover-satellite and master-satellite approximate pseudorange
          double roverSatCodeAppRange  = rover.satAppRange[i];
          double masterSatCodeAppRange = master.satAppRange[i];

          // Rover-satellite and master-satellite observed phase
          double roverSatPhaseObs  = roverObs.getSatByIDType(id, satType) .getPhaserange(goGPS.getFreq());
          double masterSatPhaseObs = masterObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq());

          // Estimated code pseudorange double differences
          double codeDoubleDiffApprox = (roverSatCodeAppRange - masterSatCodeAppRange)
                                      - (roverPivotAppRange - masterPivotAppRange);

          // Observed phase double differences
          double phaseDoubleDiffObserv = (roverSatPhaseObs - masterSatPhaseObs)
                          - (roverPivotPhaseObs - masterPivotPhaseObs);

          // Store estimated ambiguity combinations and their covariance
          estimatedAmbiguityComb[satAmb.indexOf(id)] = (codeDoubleDiffApprox - phaseDoubleDiffObserv) / roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq());
          estimatedAmbiguityCombCovariance[satAmb.indexOf(id)] = 4
            * getStDevCode(roverObs.getSatByIDType(id, satType), goGPS.getFreq())
            * getStDevCode(masterObs.getSatByIDType(id, satType), goGPS.getFreq()) / Math.pow(roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq()), 2);
        }
      }
    } 
    else if ( goGPS.getAmbiguityStrategy() == AmbiguityStrategy.LS ) {

      // Least squares design matrix
      SimpleMatrix A = new SimpleMatrix(nObsAvail+nObsAvailPhase, nUnknowns);

      // Vector for approximate pseudoranges
      SimpleMatrix b = new SimpleMatrix(nObsAvail+nObsAvailPhase, 1);

      // Vector for observed pseudoranges
      SimpleMatrix y0 = new SimpleMatrix(nObsAvail+nObsAvailPhase, 1);

      // Cofactor matrices
      SimpleMatrix Qcode = new SimpleMatrix(nObsAvail, nObsAvail);
      SimpleMatrix Qphase = new SimpleMatrix(nObsAvailPhase, nObsAvailPhase);
      SimpleMatrix Q = new SimpleMatrix(nObsAvail+nObsAvailPhase, nObsAvail+nObsAvailPhase);

      // Solution vector
      SimpleMatrix x = new SimpleMatrix(nUnknowns, 1);

      // Vector for observation error
      SimpleMatrix vEstim = new SimpleMatrix(nObsAvail, 1);

      // Error covariance matrix
      SimpleMatrix covariance = new SimpleMatrix(nUnknowns, nUnknowns);

      // Vectors for troposphere and ionosphere corrections
      SimpleMatrix tropoCorr = new SimpleMatrix(nObsAvail+nObsAvailPhase, 1);
      SimpleMatrix ionoCorr = new SimpleMatrix(nObsAvail+nObsAvailPhase, 1);

      // Counters for available satellites
      int k = 0;
      int p = 0;

      // Rover-pivot and master-pivot troposphere correction
      double roverPivotTropoCorr  = rover.satTropoCorr[pivotIndex];
      double masterPivotTropoCorr = master.satTropoCorr[pivotIndex];;

      // Rover-pivot and master-pivot ionosphere correction
      double roverPivotIonoCorr  = rover.satIonoCorr[pivotIndex];
      double masterPivotIonoCorr = master.satIonoCorr[pivotIndex];

      // Compute weights using varerr
      double roverPivotElevation = rover.topo[pivotIndex].getElevation();
      double masterPivotElevation = master.topo[pivotIndex].getElevation();
      double roverPivotCodeVar = varerr(Math.toRadians(roverPivotElevation), false, satType, goGPS.getFreq());
      double masterPivotCodeVar = varerr(Math.toRadians(masterPivotElevation), false, satType, goGPS.getFreq());
      double roverPivotPhaseVar = varerr(Math.toRadians(roverPivotElevation), true, satType, goGPS.getFreq());
      double masterPivotPhaseVar = varerr(Math.toRadians(masterPivotElevation), true, satType, goGPS.getFreq());
      Qcode.fill(roverPivotCodeVar + masterPivotCodeVar);
      Qphase.fill(roverPivotPhaseVar + masterPivotPhaseVar);

      // Set up the least squares matrices...
      // ... for code ...
      for (int i = 0; i < nObs; i++) {

        int id = roverObs.getSatID(i);
        satType = roverObs.getGnssType(i);
        String checkAvailGnss = String.valueOf(satType) + String.valueOf(id);
        
        if (sats.pos[i] !=null && sats.gnssAvail.contains(checkAvailGnss)
            && i != pivotIndex) {

          // Fill in one row in the design matrix
          A.set(k, 0, rover.diffSat[i].get(0) / rover.satAppRange[i] - diffRoverPivot.get(0) / roverPivotAppRange); /* X */

          A.set(k, 1, rover.diffSat[i].get(1) / rover.satAppRange[i] - diffRoverPivot.get(1) / roverPivotAppRange); /* Y */

          A.set(k, 2, rover.diffSat[i].get(2) / rover.satAppRange[i] - diffRoverPivot.get(2) / roverPivotAppRange); /* Z */

          // Add the differenced approximate pseudorange value to b
          b.set(k, 0, (rover.satAppRange[i] - master.satAppRange[i])
              - (roverPivotAppRange - masterPivotAppRange));

          // Add the differenced observed pseudorange value to y0
          y0.set(k, 0, (roverObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq()) - masterObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq()))
              - (roverPivotCodeObs - masterPivotCodeObs));

          // Fill in troposphere and ionosphere double differenced
          // corrections
          tropoCorr.set(k, 0, (rover.satTropoCorr[i] - master.satTropoCorr[i])
              - (roverPivotTropoCorr - masterPivotTropoCorr));
          ionoCorr.set(k, 0, (rover.satIonoCorr[i] - master.satIonoCorr[i])
              - (roverPivotIonoCorr - masterPivotIonoCorr));

          // Fill in the cofactor matrix
          double roverCodeVar = varerr(Math.toRadians(rover.topo[i].getElevation()), false, satType, goGPS.getFreq());
          double masterCodeVar = varerr(Math.toRadians(master.topo[i].getElevation()), false, satType, goGPS.getFreq());
          Qcode.set(k, k, Qcode.get(k, k) + roverCodeVar + masterCodeVar);

          // Increment available satellites counter
          k++;
        }
      }

      // ... and phase
      for (int i = 0; i < nObs; i++) {

        int id = roverObs.getSatID(i);
        satType = roverObs.getGnssType(i);
        String checkAvailGnss = String.valueOf(satType) + String.valueOf(id);

        if( sats.pos[i] !=null && sats.gnssAvail.contains(checkAvailGnss)
            && i != pivotIndex) {

          // Fill in one row in the design matrix
          A.set(k, 0, rover.diffSat[i].get(0) / rover.satAppRange[i] - diffRoverPivot.get(0) / roverPivotAppRange); /* X */
          A.set(k, 1, rover.diffSat[i].get(1) / rover.satAppRange[i] - diffRoverPivot.get(1) / roverPivotAppRange); /* Y */
          A.set(k, 2, rover.diffSat[i].get(2) / rover.satAppRange[i] - diffRoverPivot.get(2) / roverPivotAppRange); /* Z */

          if (satAmb.contains(id)) {
            A.set(k, 3 + satAmb.indexOf(id), -roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq())); /* N */

            // Add the differenced observed pseudorange value to y0
            y0.set(k, 0, (roverObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq()) - masterObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq()))
                - (roverPivotPhaseObs - masterPivotPhaseObs));
          } 
          else {
            // Add the differenced observed pseudorange value + known N to y0
            y0.set(k, 0, (roverObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq()) - masterObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq()))
                - (roverPivotPhaseObs - masterPivotPhaseObs) + KFprediction.get(i3 + id));
          }

          // Add the differenced approximate pseudorange value to b
          b.set(k, 0, (rover.satAppRange[i] - master.satAppRange[i])
              - (roverPivotAppRange - masterPivotAppRange));

          // Fill in troposphere and ionosphere double differenced corrections
          tropoCorr.set(k, 0, (rover.satTropoCorr[i] - master.satTropoCorr[i]) - (roverPivotTropoCorr - masterPivotTropoCorr));
          ionoCorr.set(k, 0, -((rover.satIonoCorr[i] - master.satIonoCorr[i]) - (roverPivotIonoCorr - masterPivotIonoCorr)));

          // Fill in the cofactor matrix
          double roverPhaseVar = varerr(Math.toRadians(rover.topo[i].getElevation()), true, satType, goGPS.getFreq());
          double masterPhaseVar = varerr(Math.toRadians(master.topo[i].getElevation()), true, satType, goGPS.getFreq());
          
          Qphase.set(p, p, Qphase.get(p, p)
              + (Math.pow(stDevPhase, 2) + Math.pow(roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq()), 2) * Cee.get(i3 + id, i3 + id))
              * (roverPivotPhaseVar + masterPivotPhaseVar)
              + (Math.pow(stDevPhase, 2) + Math.pow(roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq()), 2) * Cee.get(i3 + id, i3 + id))
              * (roverPhaseVar + masterPhaseVar));
          
          int r = 1;
          for (int m = i+1; m < nObs; m++) {
            if (sats.pos[m] !=null && sats.availPhase.contains(sats.pos[m].getSatID()) && m != pivotIndex) {
              Qphase.set(p, p+r, 0);
              Qphase.set(p+r, p, 0);
              r++;
            }
          }
          //          int r = 1;
          //          for (int j = i+1; j < nObs; j++) {
          //            if (sats.pos[j] !=null && sats.availPhase.contains(sats.pos[j].getSatID()) && j != pivotIndex) {
          //              Qphase.set(p, p+r, Qphase.get(p, p+r)
          //                  + (Math.pow(lambda, 2) * Cee.get(i3 + sats.pos[i].getSatID(), i3 + sats.pos[j].getSatID()))
          //                  * (roverPivotWeight + masterPivotWeight));
          //              Qphase.set(p+r, p, Qphase.get(p, p+r));
          //              r++;
          //            }
          //          }

          // Increment available satellite counters
          k++;
          p++;
        }
      }

      // Apply troposphere and ionosphere correction
      b = b.plus(tropoCorr);
      b = b.plus(ionoCorr);

      //Build complete cofactor matrix (code and phase)
      Q.insertIntoThis(0, 0, Qcode);
      Q.insertIntoThis(nObsAvail, nObsAvail, Qphase);

      // Least squares solution x = ((A'*Q^-1*A)^-1)*A'*Q^-1*(y0-b);
      x = A.transpose().mult(Q.invert()).mult(A).invert().mult(A.transpose()).mult(Q.invert()).mult(y0.minus(b));

      // Estimation of the variance of the observation error
      vEstim = y0.minus(A.mult(x).plus(b));
      
      double varianceEstim = (vEstim.transpose().mult(Q.invert())
          .mult(vEstim)).get(0)
          / (nObsAvail + nObsAvailPhase - nUnknowns);

      // Covariance matrix of the estimation error
      covariance = A.transpose().mult(Q.invert()).mult(A).invert().scale(varianceEstim);

      // Store estimated ambiguity combinations and their covariance
      for (int m = 0; m < satAmb.size(); m++) {
        estimatedAmbiguityComb[m] = x.get(3 + m);
        estimatedAmbiguityCombCovariance[m] = covariance.get(3 + m, 3 + m);
      }
    }

    if (init) {
      for (int i = 0; i < satAmb.size(); i++) {
        // Estimated ambiguity
        KFstate.set(i3 + satAmb.get(i), 0, estimatedAmbiguityComb[i]);

        // Store the variance of the estimated ambiguity
        Cee.set(i3 + satAmb.get(i), i3 + satAmb.get(i),
            estimatedAmbiguityCombCovariance[i]);
      }
    } else {
      for (int i = 0; i < satAmb.size(); i++) {
        // Estimated ambiguity
        KFprediction.set(i3 + satAmb.get(i), 0, estimatedAmbiguityComb[i]);

        // Store the variance of the estimated ambiguity
        Cvv.set(i3 + satAmb.get(i), i3 + satAmb.get(i), Math.pow( stDevAmbiguity, 2));
      }
    }
  }

  
  /**
   * @param roverObs
   * @param masterObs
   * @param masterPos
   */
  @Override
  void checkSatelliteConfiguration(Observations roverObs, Observations masterObs, Coordinates masterPos) {

    // Lists for keeping track of satellites that need ambiguity (re-)estimation
    ArrayList<Integer> newSatellites = new ArrayList<Integer>(0);
    ArrayList<Integer> slippedSatellites = new ArrayList<Integer>(0);

    // Check if satellites were lost since the previous epoch
    for (int i = 0; i < satOld.size(); i++) {

      // Set ambiguity of lost satellites to zero
//      if (!sats.gnssAvailPhase.contains(satOld.get(i))) {
      if (!sats.availPhase.contains(satOld.get(i)) && sats.typeAvailPhase.contains(satOld.get(i))) {

        if(goGPS.isDebug()) System.out.println("Lost satellite "+satOld.get(i));

        KFprediction.set(i3 + satOld.get(i), 0, 0);
      }
    }

    // Check if new satellites are available since the previous epoch
    int temporaryPivot = 0;
    boolean newPivot = false;
    for (int i = 0; i < sats.pos.length; i++) {

      if (sats.pos[i] != null && sats.availPhase.contains(sats.pos[i].getSatID()) && sats.typeAvailPhase.contains(sats.pos[i].getSatType())
          && !satOld.contains(sats.pos[i].getSatID()) && satTypeOld.contains(sats.pos[i].getSatType())) {

        newSatellites.add(sats.pos[i].getSatID());

        if (sats.pos[i].getSatID() == sats.pos[sats.pivot].getSatID() && sats.pos[i].getSatType() == sats.pos[sats.pivot].getSatType()) {
          newPivot = true;
          if(goGPS.isDebug()) System.out.println("New satellite "+sats.pos[i].getSatID()+" (new pivot)");
        } else {
          if(goGPS.isDebug()) System.out.println("New satellite "+sats.pos[i].getSatID());
        }
      }
    }

    // If a new satellite is going to be the pivot, its ambiguity needs to be estimated before switching pivot
    if (newPivot) {
      // If it is not the only satellite with phase
      if (sats.availPhase.size() > 1) {
        // If the former pivot is still among satellites with phase
        if (sats.availPhase.contains(oldPivotId) && sats.typeAvailPhase.contains(oldPivotType)) {
          // Find the index of the old pivot
          for (int j = 0; j < sats.pos.length; j ++) {
            if (sats.pos[j] != null && sats.pos[j].getSatID() == oldPivotId && sats.pos[j].getSatType() == oldPivotType) {
              temporaryPivot = j;
            }
          }
        } else {
          double maxEl = 0;
          // Find a temporary pivot with phase
          for (int j = 0; j < sats.pos.length; j ++) {
            if (sats.pos[j] != null && sats.availPhase.contains(sats.pos[j].getSatID()) && sats.typeAvailPhase.contains(sats.pos[j].getSatType())
                && j != sats.pivot
                && rover.topo[j].getElevation() > maxEl) {
              temporaryPivot = j;
              maxEl = rover.topo[j].getElevation();
            }
          }
          // Reset the ambiguities of other satellites according to the temporary pivot
          newSatellites.clear();
          newSatellites.addAll(sats.availPhase);
          oldPivotId = sats.pos[temporaryPivot].getSatID();
          oldPivotType = sats.pos[temporaryPivot].getSatType();
          
        }
        // Estimate the ambiguity of the new pivot and other (new) satellites, using the temporary pivot
        estimateAmbiguities(roverObs, masterObs, masterPos, newSatellites, temporaryPivot, false);
        newSatellites.clear();
      }
    }

    // Check if pivot satellite changed since the previous epoch
    // Guard: oldPivotId must be a valid satellite (>= 1) and must be in the ambiguity range
    if (oldPivotId > 0 && oldPivotId != sats.pos[sats.pivot].getSatID() && oldPivotType == sats.pos[sats.pivot].getSatType()  && sats.availPhase.size() > 1) {

      if(goGPS.isDebug()) System.out.println("Pivot change from satellite "+oldPivotId+" to satellite "+sats.pos[sats.pivot].getSatID());

      // Matrix construction to manage the change of pivot satellite
      // RTKLIB-aligned ambswap: DD ambiguity N_i = SD_i - SD_pivot
      // When pivot changes from p to q: N_i_new = N_i_old - N_q_old (for i ≠ q)
      // N_q_new = 0 (new pivot's DD ambiguity is zero by definition)
      // Use full state dimension to cover position + L1 amb + L2 amb + iono + tropo
      int totalStateDim = stateSize();
      SimpleMatrix A = SimpleMatrix.identity(totalStateDim);

      int newPivotIndex = i3 + sats.pos[sats.pivot].getSatID();
      int oldPivotIndex = i3 + oldPivotId;

      for (int i = 0; i < sats.availPhase.size(); i++) {
        int satIndex = i3 + sats.availPhase.get(i);
        if (satIndex != newPivotIndex) {
          A.set(satIndex, satIndex, 1);
          A.set(satIndex, newPivotIndex, -1);
        }
      }
      // Zero out old pivot row: its ambiguity is now expressed relative to new pivot
      A.set(oldPivotIndex, oldPivotIndex, 0);
      A.set(newPivotIndex, newPivotIndex, 0);

      // Update predicted state
      KFprediction = A.mult(KFprediction);

      // Re-computation of the Cee covariance matrix at the previous epoch
      Cee = A.mult(Cee).mult(A.transpose());
    }

    // Cycle-slip detection
    boolean lossOfLockCycleSlipRover;
    boolean lossOfLockCycleSlipMaster;
    boolean dopplerCycleSlipRover;
    boolean dopplerCycleSlipMaster;
    boolean approxRangeCycleSlip;
    boolean cycleSlip;
    //boolean slippedPivot = false;
    
    // Pivot satellite ID
    int pivotId = sats.pos[sats.pivot].getSatID();
    
    // Rover-pivot and master-pivot observed phase
    char satType = roverObs.getGnssType(0);
    double roverPivotPhaseObs = roverObs.getSatByIDType(pivotId, satType).getPhaserange(goGPS.getFreq());
    double masterPivotPhaseObs = masterObs.getSatByIDType(pivotId, satType).getPhaserange(goGPS.getFreq());
    
    // Rover-pivot and master-pivot approximate pseudoranges
    double roverPivotAppRange = rover.satAppRange[sats.pivot];
    double masterPivotAppRange = master.satAppRange[sats.pivot];
    
    for (int i = 0; i < roverObs.getNumSat(); i++) {

      int satID = roverObs.getSatID(i);
      satType = roverObs.getGnssType(i);
      String checkAvailGnss = String.valueOf(satType) + String.valueOf(satID);

      if (sats.gnssAvailPhase.contains(checkAvailGnss)) {

        // cycle slip detected by loss of lock indicator (RTKLIB-aligned: LLI flag)
        lossOfLockCycleSlipRover = roverObs.getSatByIDType(satID, satType).isPossibleCycleSlip(goGPS.getFreq());
        lossOfLockCycleSlipMaster = masterObs.getSatByIDType(satID, satType).isPossibleCycleSlip(goGPS.getFreq());

        // cycle slip detected by Doppler predicted phase range
        if (goGPS.getCycleSlipDetectionStrategy() == CycleSlipDetectionStrategy.DOPPLER_PREDICTED_PHASE_RANGE) {
          dopplerCycleSlipRover = rover.getDopplerPredictedPhase(satID) != 0.0 && (Math.abs(roverObs.getSatByIDType(satID, satType).getPhaseCycles(goGPS.getFreq())
              - rover.getDopplerPredictedPhase(satID)) > goGPS.getCycleSlipThreshold());
          dopplerCycleSlipMaster = master.getDopplerPredictedPhase(satID) != 0.0 && (Math.abs(masterObs.getSatByIDType(satID, satType).getPhaseCycles(goGPS.getFreq())
              - master.getDopplerPredictedPhase(satID)) > goGPS.getCycleSlipThreshold());
        } else {
          dopplerCycleSlipRover = false;
          dopplerCycleSlipMaster = false;
        }

        // cycle slip detected by GF (Geometry-Free) combination (dual-frequency only)
        boolean gfCycleSlipRover = false;
        boolean gfCycleSlipMaster = false;
        if (goGPS.isDualFreq()) {
          double gfThresh = goGPS.getGfCycleSlipThreshold();
          double gfRover = rover.getPrevGf(satID);
          double gfMaster = master.getPrevGf(satID);
          if (gfRover != 0.0) {
            double gfCurr = roverObs.getSatByIDType(satID, satType).getPhaserange(0)
                - roverObs.getSatByIDType(satID, satType).getPhaserange(1);
            gfCycleSlipRover = Math.abs(gfCurr - gfRover) > gfThresh;
          }
          rover.setPrevGf(satID, roverObs.getSatByIDType(satID, satType).getPhaserange(0)
              - roverObs.getSatByIDType(satID, satType).getPhaserange(1));
          if (gfMaster != 0.0) {
            double gfCurr = masterObs.getSatByIDType(satID, satType).getPhaserange(0)
                - masterObs.getSatByIDType(satID, satType).getPhaserange(1);
            gfCycleSlipMaster = Math.abs(gfCurr - gfMaster) > gfThresh;
          }
          master.setPrevGf(satID, masterObs.getSatByIDType(satID, satType).getPhaserange(0)
              - masterObs.getSatByIDType(satID, satType).getPhaserange(1));
        }

        // cycle slip detected by approximate pseudorange
        approxRangeCycleSlip = false;
        if (goGPS.getCycleSlipDetectionStrategy() == CycleSlipDetectionStrategy.APPROX_PSEUDORANGE && satID != pivotId) {

          // Rover-satellite and master-satellite approximate pseudorange
          double roverSatCodeAppRange = rover.satAppRange[i];
          double masterSatCodeAppRange = master.satAppRange[i];

          // Rover-satellite and master-satellite observed phase
          double roverSatPhaseObs = roverObs.getSatByIDType(satID, satType).getPhaserange(goGPS.getFreq());
          double masterSatPhaseObs = masterObs.getSatByIDType(satID, satType).getPhaserange(goGPS.getFreq());

          // Estimated code pseudorange double differences
          double codeDoubleDiffApprox = (roverSatCodeAppRange - masterSatCodeAppRange) - (roverPivotAppRange - masterPivotAppRange);

          // Observed phase double differences
          double phaseDoubleDiffObserv = (roverSatPhaseObs - masterSatPhaseObs) - (roverPivotPhaseObs - masterPivotPhaseObs);

          // Store estimated ambiguity combinations and their covariance
          double estimatedAmbiguityComb = (codeDoubleDiffApprox - phaseDoubleDiffObserv) / roverObs.getSatByIDType(satID, satType).getWavelength(goGPS.getFreq());

          approxRangeCycleSlip = (Math.abs(KFprediction.get(i3+satID) - estimatedAmbiguityComb)) > goGPS.getCycleSlipThreshold();

        } else {
          approxRangeCycleSlip = false;
        }

        cycleSlip = (lossOfLockCycleSlipRover || lossOfLockCycleSlipMaster || dopplerCycleSlipRover || dopplerCycleSlipMaster || gfCycleSlipRover || gfCycleSlipMaster || approxRangeCycleSlip);

        if (satID != sats.pos[sats.pivot].getSatID() && !newSatellites.contains(satID) && cycleSlip) {

          slippedSatellites.add(satID);

          //        if (satID != sats.pos[sats.pivot].getSatID()) {
          if (dopplerCycleSlipRover)
            if(goGPS.isDebug()) System.out.println("[ROVER] Cycle slip on satellite "+satID+" (range diff = "+Math.abs(roverObs.getSatByIDType(satID, satType).getPhaseCycles(goGPS.getFreq())
                - rover.getDopplerPredictedPhase(satID))+")");
          if (dopplerCycleSlipMaster)
            if(goGPS.isDebug()) System.out.println("[MASTER] Cycle slip on satellite "+satID+" (range diff = "+Math.abs(masterObs.getSatByIDType(satID, satType).getPhaseCycles(goGPS.getFreq())
                - master.getDopplerPredictedPhase(satID))+")");
          //        } else {
          //          boolean slippedPivot = true;
          //          if (dopplerCycleSlipRover)
          //            System.out.println("[ROVER] Cycle slip on pivot satellite "+satID+" (range diff = "+Math.abs(roverObs.getGpsByID(satID).getPhase(goGPS.getFreq())
          //                - this.rover.getDopplerPredictedPhase(satID))+")");
          //          if (dopplerCycleSlipMaster)
          //            System.out.println("[MASTER] Cycle slip on pivot satellite "+satID+" (range diff = "+Math.abs(masterObs.getGpsByID(satID).getPhase(goGPS.getFreq())
          //                - this.master.getDopplerPredictedPhase(satID))+")");
          //        }
        }
      }
    }

//    // If the pivot satellites slipped, the ambiguities of all the other satellites must be re-estimated
//    if (slippedPivot) {
//      // If it is not the only satellite with phase
//      if (sats.availPhase.size() > 1) {
//        // Reset the ambiguities of other satellites
//        newSatellites.clear();
//        slippedSatellites.clear();
//        slippedSatellites.addAll(sats.availPhase);
//      }
//    }

    // Ambiguity estimation
    if (newSatellites.size() != 0 || slippedSatellites.size() != 0) {
      // List of satellites that need ambiguity estimation
      ArrayList<Integer> satAmb = newSatellites;
      satAmb.addAll(slippedSatellites);
      estimateAmbiguities(roverObs, masterObs, masterPos, satAmb, sats.pivot, false);
    }
  }

  /**
   * LAMBDA ambiguity resolution: fix DD ambiguities to integers.
   * Called after each KF update in the loop() method.
   */
  boolean fixAmbiguitiesLambda() {

    int nPhase = sats.availPhase.size();
    if (nPhase < 2) return false; // Need at least 2 to form DD

    int pivotId = sats.pos[sats.pivot].getSatID();

    // Count active DD ambiguity states (exclude pivot)
    int nb = 0;
    int[] satIds = new int[nPhase];
    for (int id : sats.availPhase) {
      if (id != pivotId) {
        satIds[nb++] = id;
      }
    }
    if (nb < 2) return false; // Need at least 2 DD ambiguities

    // Ratio test threshold (RTKLIB default: 3.0)
    double thres = goGPS.getAmbiguityRatioThreshold();
    if (thres <= 0) thres = 3.0;

    // Extract float ambiguity states and covariance submatrix
    double[] a = new double[nb];
    double[] Q = new double[nb * nb];

    for (int i = 0; i < nb; i++) {
      a[i] = KFstate.get(i3 + satIds[i]);
      for (int j = 0; j < nb; j++) {
        Q[i + j * nb] = Cee.get(i3 + satIds[i], i3 + satIds[j]);
      }
    }

    // Run LAMBDA (2 candidates)
    double[] F = new double[nb * 2];
    double[] s = new double[2];
    int info = Lambda.lambda(nb, 2, a, Q, F, s);

    if (goGPS.isDebug()) {
      System.err.printf("[LAMBDA debug] nb=%d, pivot=%d%n", nb, pivotId);
      for (int i = 0; i < nb; i++) {
        double floatAmb = a[i];
        double nearestInt = Math.round(floatAmb);
        double fracDist = floatAmb - nearestInt;
        double diagCov = Q[i + i * nb];
        System.err.printf("  sat C%02d: float=%.3f, nearestInt=%.0f, fracDist=%.3f, sqrtCov=%.3f%n",
            satIds[i], floatAmb, nearestInt, fracDist, Math.sqrt(Math.max(diagCov, 0)));
      }

      // Verify LAMBDA: compute Mahalanobis distance of simple rounding
      double[] dRound = new double[nb];
      for (int i = 0; i < nb; i++) {
        dRound[i] = a[i] - Math.round(a[i]);
      }
      // Compute dRound' * Q^{-1} * dRound using SimpleMatrix
      SimpleMatrix Qmat = new SimpleMatrix(nb, nb);
      SimpleMatrix dVec = new SimpleMatrix(nb, 1);
      for (int i = 0; i < nb; i++) {
        dVec.set(i, 0, dRound[i]);
        for (int j = 0; j < nb; j++) {
          Qmat.set(i, j, Q[i + j * nb]);
        }
      }
      double sRound = dVec.transpose().mult(Qmat.invert()).mult(dVec).get(0);
      System.err.printf("[LAMBDA debug] Simple rounding Mahalanobis² = %.1f, LAMBDA s[0]=%.1f, s[1]=%.1f%n",
          sRound, s[0], s[1]);

      // Also print LAMBDA fixed candidates
      System.err.printf("[LAMBDA debug] LAMBDA fixed: ");
      for (int i = 0; i < Math.min(nb, 5); i++) {
        System.err.printf("%.0f ", F[i]);
      }
      System.err.println();
    }

    if (info != 0) {
      if (goGPS.isDebug()) System.out.println("[LAMBDA] Failed (info=" + info + ")");
      return false;
    }

    // Ratio test
    double ratio = (s[0] > 0) ? s[1] / s[0] : 0.0;
    if (ratio >= thres) {
      // Fix ambiguities: update KFstate with fixed integer values
      for (int i = 0; i < nb; i++) {
        KFstate.set(i3 + satIds[i], 0, F[i]); // F[i] = first (best) candidate
      }

      // Conditional covariance update (RTKLIB-aligned: resamb_LAMBDA)
      int na = i3 + 1; // number of non-ambiguity states (0..i3)

      // Build Qb (nb x nb) and Qab (na x nb) from Cee
      SimpleMatrix Qb = new SimpleMatrix(nb, nb);
      SimpleMatrix Qab = new SimpleMatrix(na, nb);
      for (int i = 0; i < nb; i++) {
        for (int j = 0; j < nb; j++) {
          Qb.set(i, j, Cee.get(i3 + satIds[i], i3 + satIds[j]));
        }
        for (int j = 0; j < na; j++) {
          Qab.set(j, i, Cee.get(j, i3 + satIds[i]));
        }
      }

      // Build residual vector: db = b0 - b (float - fixed)
      SimpleMatrix dbVec = new SimpleMatrix(nb, 1);
      for (int i = 0; i < nb; i++) {
        dbVec.set(i, 0, a[i] - F[i]);
      }

      // Check numerical stability of Qb before inversion
      // Use trace-based condition estimate: cond_est = trace(Qb) / nb * ||Qb^{-1}||_approx
      // If Qb is near-singular, skip covariance update to avoid numerical explosion
      double trace = 0;
      double maxAbs = 0;
      for (int i = 0; i < nb; i++) {
        double diag = Math.abs(Qb.get(i, i));
        trace += diag;
        if (diag > maxAbs) maxAbs = diag;
      }
      // Condition number estimate: if trace is tiny or maxAbs is huge, Qb is ill-conditioned
      boolean illCond = (trace < 1e-20) || (maxAbs / Math.max(trace / nb, 1e-30) > 1e12);
      if (illCond) {
        if (goGPS.isDebug()) {
          System.out.printf("[LAMBDA] Qb ill-conditioned (trace=%.2e, maxDiag=%.2e), skip covariance update%n", trace, maxAbs);
        }
      } else {
        try {
          // Qb_inv = Qb^(-1)
          SimpleMatrix QbInv = Qb.invert();

          // db = Qb^(-1) * (b0 - b)
          SimpleMatrix db = QbInv.mult(dbVec);

          // Update non-ambiguity states: xa = xa - Qab * db
          SimpleMatrix xaOld = new SimpleMatrix(na, 1);
          for (int j = 0; j < na; j++) {
            xaOld.set(j, 0, KFstate.get(j, 0));
          }
          SimpleMatrix xaNew = xaOld.minus(Qab.mult(db));
          for (int j = 0; j < na; j++) {
            KFstate.set(j, 0, xaNew.get(j, 0));
          }

          // Update non-ambiguity covariance: Pa = Pa - Qab * Qb^(-1) * Qab^T
          SimpleMatrix QabT = Qab.transpose();
          SimpleMatrix correction = Qab.mult(QbInv).mult(QabT);
          for (int j = 0; j < na; j++) {
            for (int k = 0; k < na; k++) {
              double newVal = Cee.get(j, k) - correction.get(j, k);
              Cee.set(j, k, newVal);
            }
          }
        } catch (Exception e) {
          if (goGPS.isDebug()) {
            System.out.println("[LAMBDA] Qb inversion failed, skip covariance update: " + e.getMessage());
          }
        }
      }

      if (goGPS.isDebug()) {
        System.out.printf("[LAMBDA] Fixed %d ambiguities, ratio=%.2f, s=[%.4f, %.4f]%n",
            nb, ratio, s[0], s[1]);
      }
      return true;
    } else {
      if (goGPS.isDebug()) {
        System.out.printf("[LAMBDA] Ratio test failed: %.2f < %.2f (nb=%d, s=[%.4f, %.4f])%n",
            ratio, thres, nb, s[0], s[1]);
      }
      return false;
    }
  }

  /**
   * RTKLIB-aligned holdamb: constrain fixed DD ambiguities.
   * After LAMBDA fix, apply pseudo-observation constraints to prevent
   * ambiguity states from drifting. This is equivalent to RTKLIB's
   * holdamb() which calls filter() with VAR_HOLDAMB constraints.
   *
   * In GoGPS DD model, each ambiguity state is already a DD ambiguity
   * (relative to pivot). The constraint is: x[i] - fixed_value = 0
   * with variance VAR_HOLDAMB.
   */
  @Override
  void holdamb() {
    int pivotId = sats.pos[sats.pivot].getSatID();
    int nb = 0;
    for (int id : sats.availPhase) {
      if (id != pivotId) nb++;
    }
    if (nb == 0) return;

    SimpleMatrix H_hold = new SimpleMatrix(nb, stateSize());
    SimpleMatrix v_hold = new SimpleMatrix(nb, 1);
    SimpleMatrix R_hold = new SimpleMatrix(nb, nb);

    int nv = 0;
    for (int id : sats.availPhase) {
      if (id == pivotId) continue;
      int idx = i3 + id;
      H_hold.set(nv, idx, 1.0);
      v_hold.set(nv, 0, 0.0);
      R_hold.set(nv, nv, VAR_HOLDAMB);
      nv++;
    }

    try {
      SimpleMatrix S_hold = H_hold.mult(Cee).mult(H_hold.transpose()).plus(R_hold);
      SimpleMatrix S_inv = S_hold.invert();
      SimpleMatrix K_hold = Cee.mult(H_hold.transpose()).mult(S_inv);

      SimpleMatrix innov = v_hold.minus(H_hold.mult(KFstate));
      KFstate = KFstate.plus(K_hold.mult(innov));

      SimpleMatrix I = SimpleMatrix.identity(stateSize());
      Cee = I.minus(K_hold.mult(H_hold)).mult(Cee);
    } catch (Exception e) {
      if (goGPS.isDebug()) {
        System.out.println("[holdamb] Constraint update failed: " + e.getMessage());
      }
    }

    if (goGPS.isDebug()) {
      System.out.printf("[holdamb] Held %d DD ambiguities%n", nb);
    }
  }

  /**
   * Run kalman filter on code and phase double differences.
   */
  public static void run( GoGPS goGPS ) {
    
    RoverPosition rover   = goGPS.getRoverPos();
    MasterPosition master = goGPS.getMasterPos();
    Satellites sats       = goGPS.getSats();
    ObservationsProducer roverIn = goGPS.getRoverIn();
    ObservationsProducer masterIn = goGPS.getMasterIn();
    boolean debug = goGPS.isDebug();
    
    long timeRead = System.currentTimeMillis();
    long depRead = 0;

    long timeProc = 0;
    long depProc = 0;

    KalmanFilter kf = new KF_DD_code_phase(goGPS);
    
    // Flag to check if Kalman filter has been initialized
    boolean kalmanInitialized = false;

    try {
      boolean validPosition = false;

      timeRead = System.currentTimeMillis() - timeRead;
      depRead = depRead + timeRead;

      Observations obsR = roverIn.getNextObservations();
      Observations obsM = masterIn.getNextObservations();

      // SPP fallback: if no valid base station position (no 1005/1006 message),
      // run single point positioning on base station observations
      if (obsM != null && obsM.getNumSat() >= 4) {
        Coordinates masterCoord = masterIn.getDefinedPosition();
        if (debug) System.err.println("[KF_DD run] masterIn.getDefinedPosition()=[" +
            (masterCoord != null ? masterCoord.getX() + ", " + masterCoord.getY() + ", " + masterCoord.getZ() : "null") + "]");
        if (masterCoord == null || !masterCoord.isValidXYZ() ||
            (masterCoord.getX() == 0 && masterCoord.getY() == 0 && masterCoord.getZ() == 0)) {
          if (debug) System.out.println("[SPP] No valid base station position (no 1005/1006 msg), running SPP...");

          // Save rover position (will be restored after SPP)
          double origRoverX = rover.getX(), origRoverY = rover.getY(), origRoverZ = rover.getZ();
          double origRoverClk = rover.getClockError();

          // Run iterative SPP on base station observations
          rover.setXYZ(0, 0, 0);
          rover.setClockError(0);
          double prevX = 0, prevY = 0, prevZ = 0;
          for (int iter = 0; iter < 10; iter++) {
            sats.selectStandalone(obsM, goGPS.getCutoff());
            if (sats.getAvailNumber() >= 4) {
              kf.codeStandalone(obsM, false, true);
            }
            if (rover.isValidXYZ() && iter > 0) {
              double dx = rover.getX() - prevX;
              double dy = rover.getY() - prevY;
              double dz = rover.getZ() - prevZ;
              if (Math.sqrt(dx*dx + dy*dy + dz*dz) < 100) {
                if (debug) System.out.println("[SPP] Base station SPP converged at iter " + (iter + 1));
                break;
              }
            }
            prevX = rover.getX();
            prevY = rover.getY();
            prevZ = rover.getZ();
          }

          if (rover.isValidXYZ() && (prevX != 0 || prevY != 0 || prevZ != 0)) {
            masterCoord = Coordinates.globalXYZInstance(rover.getX(), rover.getY(), rover.getZ());
            masterCoord.computeGeodetic();
            goGPS.setMasterPos(masterCoord);
            if (debug) System.out.println("[SPP] Base station SPP result: X=" + String.format("%.4f", rover.getX()) +
                " Y=" + String.format("%.4f", rover.getY()) + " Z=" + String.format("%.4f", rover.getZ()));
          } else {
            if (debug) System.out.println("[SPP] Base station SPP failed, using (0,0,0)");
          }

          // Restore rover position
          rover.setXYZ(origRoverX, origRoverY, origRoverZ);
          rover.setClockError(origRoverClk);
        }
      }

      // SKIP_DUP_MARKER: 同一历元多条MSM消息会产生重复Observations，此处跳过已处理的时间戳
      // 注释掉while循环内的skip块可恢复输出重复数据（调试MSM消息合并时使用）
      long lastEpochTime = -1;
      // END_SKIP_DUP_MARKER

      while (obsR != null && obsM != null) {
//        System.out.println("obsR: " + obsR);

        
        if(debug)System.out.println("R:"+obsR.getRefTime().getMsec()+" M:"+obsM.getRefTime().getMsec());

        timeRead = System.currentTimeMillis();

        long maxTimeDiffMs = goGPS.getMaxTimeDiffMs();

        // Discard master epochs that are behind rover by more than tolerance
        while (obsM != null && obsR != null
            && (obsR.getRefTime().getMsec() - obsM.getRefTime().getMsec()) > maxTimeDiffMs) {
          obsM = masterIn.getNextObservations();
        }
        if (obsM == null) {
          if(debug) System.out.println("Couldn't find an obsM in a valid time span: " + obsR.getRefTime());
          break;
        }

        // Discard rover epochs that are behind master by more than tolerance
        while (obsM != null && obsR != null
            && (obsM.getRefTime().getMsec() - obsR.getRefTime().getMsec()) > maxTimeDiffMs) {
          obsR = roverIn.getNextObservations();
        }
        if (obsR == null) {
          if(debug) System.out.println("Couldn't find an obsR in a valid time span: " + obsM.getRefTime());
          break;
        }

        // SKIP_DUP_MARKER: 跳过同一历元的重复MSM消息
        if (obsR != null && obsR.getRefTime().getMsec() == lastEpochTime) {
          if (debug) System.out.println("[Skip] Duplicate epoch: " + obsR.getRefTime());
          obsR = roverIn.getNextObservations();
          obsM = masterIn.getNextObservations();
          continue;
        }
        // END_SKIP_DUP_MARKER

        if(obsM!=null && obsR!=null){
          lastEpochTime = obsR.getRefTime().getMsec();  // SKIP_DUP_MARKER: 记录当前历元
          timeRead = System.currentTimeMillis() - timeRead;
          depRead = depRead + timeRead;
          timeProc = System.currentTimeMillis();

          try {
            // If Kalman filter was not initialized and if there are at least four satellites
            boolean valid = true;
            if (!kalmanInitialized && obsR.getNumSat() >= 4) {

            // Compute approximate positioning by iterative least-squares
            if( roverIn.getDefinedPosition() != null )
              roverIn.getDefinedPosition().cloneInto(rover);
            
            boolean initConverged = false;
            double prevX = 0, prevY = 0, prevZ = 0;
            for (int iter = 0; iter < 3; iter++) {
              // Save previous position for convergence check
              if (iter > 0) {
                prevX = rover.getX();
                prevY = rover.getY();
                prevZ = rover.getZ();
              }
              
              // Select all satellites
              sats.selectStandalone( obsR, -100);
              
              if (sats.getAvailNumber() >= 4) {
                kf.codeStandalone( obsR, false, true );
              }
              
              // Convergence check: position change < 100m after 2nd iteration
              if (iter >= 1 && rover.isValidXYZ()) {
                double dx = rover.getX() - prevX;
                double dy = rover.getY() - prevY;
                double dz = rover.getZ() - prevZ;
                double change = Math.sqrt(dx*dx + dy*dy + dz*dz);
                if (change < 100) {
                  initConverged = true;
                  if(debug) System.out.println("[Init] Code standalone converged at iter " + (iter+1) + ", change=" + String.format("%.1f", change) + "m");
                  break;
                }
              }
            }
            
            if (!initConverged && rover.isValidXYZ()) {
              if(debug) System.out.println("[Init] Code standalone did NOT converge after 3 iterations");
            }
            
            // Height sanity check: reject if height > 10000m (likely wrong solution)
            // Fallback: use base station position as initial guess for short-baseline RTK
            if (rover.isValidXYZ()) {
              rover.computeGeodetic();
              double h = rover.getGeodeticHeight();
              if (Math.abs(h) > 10000) {
                if(debug) System.out.println("[Init] Height out of range: " + String.format("%.1f", h) + "m, reject init");
                if (masterIn.getDefinedPosition() != null) {
                  masterIn.getDefinedPosition().cloneInto(rover);
                  if(debug) System.out.println("[Init] Fallback to base station position: " +
                      String.format("%.1f, %.1f, %.1f", rover.getX(), rover.getY(), rover.getZ()));
                } else {
                  rover.setXYZ(0, 0, 0);
                }
              }
            }

            // If an approximate position was computed
            if (rover.isValidXYZ()) {
              
              // Initialize Kalman filter
              kf.init(obsR, obsM, masterIn.getDefinedPosition());

              if (rover.isValidXYZ()) {
                kalmanInitialized = true;
                if(debug)System.out.println("Kalman filter initialized.");
              } else {
                if(debug)System.out.println("Kalman filter not initialized.");
              }
            }else{
              if(debug) System.out.println("A-priori position (from code observations) is not valid.");
              // Fallback: use base station position for short-baseline RTK
              if (masterIn.getDefinedPosition() != null) {
                masterIn.getDefinedPosition().cloneInto(rover);
                if(debug) System.out.println("[Init] Fallback to base station position (SPP failed): " +
                    String.format("%.1f, %.1f, %.1f", rover.getX(), rover.getY(), rover.getZ()));
                // Retry Kalman filter initialization with base position
                kf.init(obsR, obsM, masterIn.getDefinedPosition());
                if (rover.isValidXYZ()) {
                  kalmanInitialized = true;
                  if(debug) System.out.println("Kalman filter initialized (base station fallback).");
                }
              }
            }
          } else if (kalmanInitialized) {

            // Do a Kalman filter loop
            try{
              kf.loop(obsR,obsM, masterIn.getDefinedPosition());
            }catch(Exception e){
              e.printStackTrace();
              valid = false;
            }
          }

          timeProc = System.currentTimeMillis() - timeProc;
          depProc = depProc + timeProc;

          if(kalmanInitialized && valid){
            // Coordinate validity check: intercept NaN, zero, and out-of-range positions
            boolean coordValid = rover.isValidXYZ()
                && !Double.isNaN(rover.getX()) && !Double.isNaN(rover.getY()) && !Double.isNaN(rover.getZ())
                && !Double.isInfinite(rover.getX()) && !Double.isInfinite(rover.getY()) && !Double.isInfinite(rover.getZ())
                && !(rover.getX() == 0 && rover.getY() == 0 && rover.getZ() == 0);

            // Check distance from base station (RTK short baseline guard)
            if (coordValid && masterIn.getDefinedPosition() != null) {
              double dx = rover.getX() - masterIn.getDefinedPosition().getX();
              double dy = rover.getY() - masterIn.getDefinedPosition().getY();
              double dz = rover.getZ() - masterIn.getDefinedPosition().getZ();
              double distFromBase = Math.sqrt(dx*dx + dy*dy + dz*dz);
              if (debug) System.err.printf("[Output guard] rover=[%.2f,%.2f,%.2f] base=[%.2f,%.2f,%.2f] dist=%.1fm%n",
                  rover.getX(), rover.getY(), rover.getZ(),
                  masterIn.getDefinedPosition().getX(), masterIn.getDefinedPosition().getY(), masterIn.getDefinedPosition().getZ(),
                  distFromBase);
              if (distFromBase > goGPS.getMaxDivergenceDistance()) {
                if(debug) System.out.println("[Output guard] Rover too far from base: " + String.format("%.1f", distFromBase/1000) + "km, skip output");
                coordValid = false;
              }
            }

            if (coordValid) {
              if(!validPosition){
                goGPS.notifyPositionConsumerEvent(PositionConsumer.EVENT_START_OF_TRACK);
                validPosition = true;
              }else
              if(goGPS.getPositionConsumers().size()>0){
                RoverPosition coord = new RoverPosition(rover, DopType.KALMAN, rover.getKpDop(), rover.getKhDop(), rover.getKvDop());
                coord.setRefTime(new Time(obsR.getRefTime().getMsec()));
                coord.sampleTime = obsR.getRefTime();
                coord.obs = obsR;

                // Build satsInUse bitmask from available satellites (for downstream consumers)
                long satsMask = 0;
                for (int satId : sats.avail.keySet()) {
                    if (satId > 0 && satId < 64) {
                        satsMask |= (1L << (satId - 1));
                    }
                }
                coord.satsInUse = satsMask;

                // Determine FIX/FLOAT status based on LAMBDA ambiguity resolution
                coord.status = (kf.nfix >= MIN_FIX_HOLD) ? Status.Valid : Status.Low_Sat;

                goGPS.notifyPositionConsumerAddCoordinate(coord);
              }
            } else {
              if(debug) System.out.println("[Output guard] Invalid coordinates, skip output");
            }

          }
          //System.out.println("--------------------");

          } catch (Exception e) {
            System.err.println("[Epoch] Exception in epoch processing: " + e.getMessage());
            if (debug) {
              e.printStackTrace();
            }
          }

          if(debug)System.out.println("-- Get next epoch ---------------------------------------------------");
          // get next epoch
          obsR = roverIn.getNextObservations();
          obsM = masterIn.getNextObservations();

        }else{
          if(debug)System.out.println("Missing M or R obs ");
        }
      }

    } catch (Exception e) {
      e.printStackTrace();
    } finally {
      goGPS.notifyPositionConsumerEvent(PositionConsumer.EVENT_END_OF_TRACK);
    }

    int elapsedTimeSec = (int) Math.floor(depRead / 1000);
    int elapsedTimeMillisec = (int) (depRead - elapsedTimeSec * 1000);
    if(debug)System.out.println("\nElapsed time (read): " + elapsedTimeSec
        + " seconds " + elapsedTimeMillisec + " milliseconds.");

    elapsedTimeSec = (int) Math.floor(depProc / 1000);
    elapsedTimeMillisec = (int) (depProc - elapsedTimeSec * 1000);
    if(debug)System.out.println("\nElapsed time (proc): " + elapsedTimeSec
        + " seconds " + elapsedTimeMillisec + " milliseconds.");
  }

}