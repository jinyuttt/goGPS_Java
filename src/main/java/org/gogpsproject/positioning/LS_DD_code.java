package org.gogpsproject.positioning;

import org.ejml.simple.SimpleMatrix;
import org.gogpsproject.GoGPS;
import org.gogpsproject.consumer.PositionConsumer;
import org.gogpsproject.positioning.RoverPosition.DopType;
import org.gogpsproject.producer.Observations;
import org.gogpsproject.producer.ObservationsProducer;

public class LS_DD_code extends LS_SA_code {

  public LS_DD_code(GoGPS goGPS) {
    super(goGPS);
  }

  /**
   * @param roverObs
   * @param masterObs
   * @param masterPos
   */
  public void codeDoubleDifferences( Observations roverObs,Observations masterObs, Coordinates masterPos) {

    // Number of GPS observations
    int nObs = roverObs.getNumSat();

    // Number of unknown parameters
    int nUnknowns = 3;

    // Number of available satellites (i.e. observations)
    int nObsAvail = sats.avail.size();

    // Full design matrix for DOP computation
    SimpleMatrix Adop = new SimpleMatrix(nObsAvail, 3);

    // Double differences with respect to pivot satellite reduce
    // observations by 1
    nObsAvail--;

    // Least squares design matrix
    SimpleMatrix A = new SimpleMatrix(nObsAvail, nUnknowns);

    // Vector for approximate pseudoranges
    SimpleMatrix b = new SimpleMatrix(nObsAvail, 1);

    // Vector for observed pseudoranges
    SimpleMatrix y0 = new SimpleMatrix(nObsAvail, 1);

    // Cofactor matrix
    SimpleMatrix Q = new SimpleMatrix(nObsAvail, nObsAvail);

    // Solution vector
    SimpleMatrix x = new SimpleMatrix(nUnknowns, 1);

    // Vector for observation error
    SimpleMatrix vEstim = new SimpleMatrix(nObsAvail, 1);

    // Vectors for troposphere and ionosphere corrections
    SimpleMatrix tropoCorr = new SimpleMatrix(nObsAvail, 1);
    SimpleMatrix ionoCorr = new SimpleMatrix(nObsAvail, 1);

    // Counter for available satellites (with pivot)
    int d = 0;

    // Pivot satellite index
    int pivotId = roverObs.getSatID(sats.pivot);
    char satType = roverObs.getGnssType(sats.pivot);      
    
    // Store rover-pivot and master-pivot observed pseudoranges
    double roverPivotObs = roverObs.getSatByIDType(pivotId, satType).getPseudorange(goGPS.getFreq());
    double masterPivotObs = masterObs.getSatByIDType(pivotId, satType).getPseudorange(goGPS.getFreq());

    // Rover-pivot approximate pseudoranges
    SimpleMatrix diffRoverPivot = rover.diffSat[sats.pivot];
    double roverPivotAppRange   = rover.satAppRange[sats.pivot];

    // Master-pivot approximate pseudoranges
    double masterPivotAppRange = master.satAppRange[sats.pivot];

    // Computation of rover-pivot troposphere correction
    double roverPivotTropoCorr = rover.satTropoCorr[sats.pivot];

    // Computation of master-pivot troposphere correction
    double masterPivotTropoCorr = master.satTropoCorr[sats.pivot];;

    // Computation of rover-pivot ionosphere correction
    double roverPivotIonoCorr = rover.satIonoCorr[sats.pivot];

    // Computation of master-pivot ionosphere correction
    double masterPivotIonoCorr = master.satIonoCorr[sats.pivot];

    // Compute pivot variances for DD covariance matrix
    double roverPivotElevation = rover.topo[sats.pivot].getElevation();
    double masterPivotElevation = master.topo[sats.pivot].getElevation();
    double roverPivotSigma = varerr(Math.toRadians(roverPivotElevation), false, satType, goGPS.getFreq());
    double masterPivotSigma = varerr(Math.toRadians(masterPivotElevation), false, satType, goGPS.getFreq());
    double pivotVar = roverPivotSigma * roverPivotSigma + masterPivotSigma * masterPivotSigma;

    // Store satellite variances for later covariance matrix construction
    double[] satVars = new double[nObsAvail];

    // Set up the least squares matrices
    for (int i = 0, k = 0; i < nObs; i++) {

      // Satellite ID
      int id = roverObs.getSatID(i);
      satType = roverObs.getGnssType(i);
      String checkAvailGnss = String.valueOf(satType) + String.valueOf(id);

      if (sats.pos[i] !=null && sats.avail.containsKey(id) && sats.gnssAvail.contains(checkAvailGnss) && i != sats.pivot) {
//      if (sats.pos[i] !=null && sats.avail.contains(id) && satTypeAvail.contains(satType) && i != pivot) {

        // Fill in one row in the design matrix
        A.set(k, 0, rover.diffSat[i].get(0) / rover.satAppRange[i]
            - diffRoverPivot.get(0) / roverPivotAppRange); /* X */

        A.set(k, 1, rover.diffSat[i].get(1) / rover.satAppRange[i]
            - diffRoverPivot.get(1) / roverPivotAppRange); /* Y */

        A.set(k, 2, rover.diffSat[i].get(2) / rover.satAppRange[i]
            - diffRoverPivot.get(2) / roverPivotAppRange); /* Z */

        // Add the differenced approximate pseudorange value to b
        b.set(k, 0, (rover.satAppRange[i] - master.satAppRange[i])
            - (roverPivotAppRange - masterPivotAppRange));

        // Add the differenced observed pseudorange value to y0
        y0.set(k, 0, (roverObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq()) - masterObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq()))
            - (roverPivotObs - masterPivotObs));

        // Fill in troposphere and ionosphere double differenced
        // corrections
        tropoCorr.set(k, 0, (rover.satTropoCorr[i] - master.satTropoCorr[i])
            - (roverPivotTropoCorr - masterPivotTropoCorr));
        ionoCorr.set(k, 0, (rover.satIonoCorr[i] - master.satIonoCorr[i])
            - (roverPivotIonoCorr - masterPivotIonoCorr));

        // Fill in the cofactor matrix (covariance of DD observations)
        double roverSatSigma = varerr(Math.toRadians(rover.topo[i].getElevation()), false, satType, goGPS.getFreq());
        double masterSatSigma = varerr(Math.toRadians(master.topo[i].getElevation()), false, satType, goGPS.getFreq());
        satVars[k] = roverSatSigma * roverSatSigma + masterSatSigma * masterSatSigma;

        // Increment available satellites counter
        k++;
      }

      // Design matrix for DOP computation
      if (sats.pos[i] !=null && sats.avail.containsKey(id) && sats.gnssAvail.contains(checkAvailGnss)) {
//      if (sats.pos[i] != null && sats.avail.contains(id) && satTypeAvail.contains(satType)) {
        // Fill in one row in the design matrix (complete one, for DOP)
        Adop.set(d, 0, rover.diffSat[i].get(0) / rover.satAppRange[i]); /* X */
        Adop.set(d, 1, rover.diffSat[i].get(1) / rover.satAppRange[i]); /* Y */
        Adop.set(d, 2, rover.diffSat[i].get(2) / rover.satAppRange[i]); /* Z */
        d++;
      }
    }

    // Apply troposphere and ionosphere correction
    b = b.plus(tropoCorr);
    b = b.plus(ionoCorr);

    // Build DD covariance matrix Q:
    // Q[i][i] = var(sat_i) + var(pivot)
    // Q[i][j] = var(pivot)  for i != j (common pivot correlation)
    Q.zero();
    for (int i = 0; i < nObsAvail; i++) {
      Q.set(i, i, satVars[i] + pivotVar);
      for (int j = 0; j < i; j++) {
        Q.set(i, j, pivotVar);
        Q.set(j, i, pivotVar);
      }
    }

    // Least squares solution x = ((A'*Q^-1*A)^-1)*A'*Q^-1*(y0-b);
    x = A.transpose().mult(Q.invert()).mult(A).invert().mult(A.transpose()).mult(Q.invert()).mult(y0.minus(b));

    // Receiver position
    rover.setPlusXYZ(x);

    // Estimation of the variance of the observation error
    vEstim = y0.minus(A.mult(x).plus(b));

    // Covariance matrix of the estimation error
    if (nObsAvail >= nUnknowns){
      SimpleMatrix cofactor = A.transpose().mult(Q.invert()).mult(A).invert();
      if (nObsAvail > nUnknowns) {
        double varianceEstim = (vEstim.transpose().mult(Q.invert())
            .mult(vEstim)).get(0)
            / (nObsAvail - nUnknowns);
        positionCovariance = cofactor.scale(varianceEstim).extractMatrix(0, 3, 0, 3);
      } else {
        // nObsAvail == nUnknowns: no redundancy, use cofactor directly
        positionCovariance = cofactor.extractMatrix(0, 3, 0, 3);
      }
    }else{
      positionCovariance = null;
    }

    updateDops(Adop);

    // Compute positioning in geodetic coordinates
    rover.computeGeodetic();
  }
  
  /**
   * Run code double differences.
   */
  public static void run( GoGPS goGPS ) {
    
    RoverPosition rover   = goGPS.getRoverPos();
    MasterPosition master = goGPS.getMasterPos();
    Satellites sats       = goGPS.getSats();
    ObservationsProducer roverIn = goGPS.getRoverIn();
    ObservationsProducer masterIn = goGPS.getMasterIn();
    boolean debug = goGPS.isDebug();
    boolean validPosition = false;
    
    try {
      LS_DD_code dd = new LS_DD_code(goGPS);

      Observations obsR = roverIn.getNextObservations();
      Observations obsM = masterIn.getNextObservations();

      while (obsR != null && obsM != null) {

        // Discard master epochs if correspondent rover epochs are not available
        double obsRtime = obsR.getRefTime().getGpsTime();
        while (obsM!=null && obsR!=null && obsRtime > obsM.getRefTime().getGpsTime()) {
          obsM = masterIn.getNextObservations();
        }

        // Discard rover epochs if correspondent master epochs are not available
        double obsMtime = obsM.getRefTime().getGpsTime();
        while (obsM!=null && obsR!=null && obsR.getRefTime().getGpsTime() < obsMtime) {
          obsR = roverIn.getNextObservations();
        }


        // If there are at least four satellites
        if (obsM!=null && obsR!=null){
          if(obsR.getNumSat() >= 4) {

            // Compute approximate positioning by iterative least-squares
            if (!rover.isValidXYZ()) {
              Coordinates definedPos = roverIn.getDefinedPosition();
              if (definedPos != null && definedPos.isValidXYZ()) {
                definedPos.cloneInto(rover);
              } else {
                rover.setXYZ(0, 0, 0);
              }
            }
            rover.setClockError(0);
            
            // 增加迭代次数到10次，与RTKLIB一致
            // 在大误差下需要更多迭代才能收敛
            for (int iter = 0; iter < 10; iter++) {
              
              // Select all satellites
              sats.selectStandalone( obsR, -100);
              
              if (sats.getAvailNumber() >= 4) {
                dd.codeStandalone( obsR, false, true);
                
                // 检测发散：如果位置超出合理范围（距离地球中心超过50000公里），重置为基站坐标
                double r = Math.sqrt(rover.getX()*rover.getX() + rover.getY()*rover.getY() + rover.getZ()*rover.getZ());
                if (r > 5.0e7) { // 超过50000公里
                  System.err.println("[DEBUG LS_DD] 迭代 " + iter + " 位置发散 (r=" + r + " m)，重置为基站坐标");
                  if (roverIn.getDefinedPosition() != null && roverIn.getDefinedPosition().isValidXYZ()) {
                    roverIn.getDefinedPosition().cloneInto(rover);
                  } else if (masterIn != null && masterIn.getDefinedPosition() != null && masterIn.getDefinedPosition().isValidXYZ()) {
                    masterIn.getDefinedPosition().cloneInto(rover);
                  } else {
                    rover.setXYZ(0, 0, 0);
                  }
                  rover.setClockError(0);
                }
              }
            }

            // If an approximate position was computed
            if (rover.isValidXYZ()) {

              // Select satellites available for double differences
              sats.selectDoubleDiff( obsR, obsM, masterIn.getDefinedPosition());

              if (sats.getAvailNumber() >= 4)
                // Compute code double differences positioning
                // (epoch-by-epoch solution)
                dd.codeDoubleDifferences( obsR, obsM, masterIn.getDefinedPosition());
              else {
                // Discard approximate positioning, fallback to base position
                if (masterIn != null && masterIn.getDefinedPosition() != null && masterIn.getDefinedPosition().isValidXYZ()) {
                  masterIn.getDefinedPosition().cloneInto(rover);
                } else {
                  rover.setXYZ(0, 0, 0);
                }
              }
            }

            if (rover.isValidXYZ()) {
              if(!validPosition){
                goGPS.notifyPositionConsumerEvent(PositionConsumer.EVENT_START_OF_TRACK);
                validPosition = true;
              }else{
                RoverPosition coord = new RoverPosition(rover, DopType.KALMAN, rover.getpDop(), rover.gethDop(), rover.getvDop());

                if(goGPS.getPositionConsumers().size()>0){
                  coord.setRefTime(new Time(obsR.getRefTime().getMsec()));
                  goGPS.notifyPositionConsumerAddCoordinate(coord);
                }
                if(debug)System.out.println("-------------------- "+rover.getpDop());
              }
            }
          }
        }
        // get next epoch
        obsR = roverIn.getNextObservations();
        obsM = masterIn.getNextObservations();
      }
    } catch (Exception e) {
      e.printStackTrace();
    } finally {
      goGPS.notifyPositionConsumerEvent(PositionConsumer.EVENT_END_OF_TRACK);
    }
  }
  
}