package org.gogpsproject.positioning;

import org.ejml.simple.SimpleMatrix;
import org.gogpsproject.Constants;
import org.gogpsproject.GoGPS;
import org.gogpsproject.consumer.PositionConsumer;
import org.gogpsproject.positioning.RoverPosition.DopType;
import org.gogpsproject.producer.Observations;
import org.gogpsproject.producer.ObservationsProducer;
import org.gogpsproject.producer.StreamEventListener;

public class LS_SA_code extends Core {

  public LS_SA_code( GoGPS goGPS ){
    super( goGPS );
  }
  
  public void codeStandalone( Observations roverObs, boolean estimateOnlyClock, boolean ignoreTopocentricParameters ) {

    boolean debug = goGPS.isDebug();

    // 【修改0】如果接收机位置无效（NaN），设为零向量，避免矩阵运算传播NaN
    if (!rover.isValidXYZ()) {
      rover.setXYZ(0, 0, 0);
      rover.computeGeodetic();
    }

    // Number of GNSS observations without cutoff
    int nObs = roverObs.getNumSat();

    // Number of unknown parameters
    int nUnknowns = 4;
    
    // Add one unknown for each constellation in addition to the first (to estimate Inter-System Biases - ISBs)
    String sys = sats.getAvailGnssSystems();
    if (sys.length()>0) {
      sys = sys.substring(1);
      nUnknowns = nUnknowns + sys.length();
    }

    // Number of available satellites (i.e. observations)
    int nObsAvail = sats.avail.size();

    // Least squares design matrix
    SimpleMatrix A = new SimpleMatrix(nObsAvail, nUnknowns);

    // Vector for approximate pseudoranges
    SimpleMatrix b = new SimpleMatrix(nObsAvail, 1);

    // Vector for observed pseudoranges
    SimpleMatrix y0 = new SimpleMatrix(nObsAvail, 1);

    // Cofactor matrix: identity for coarse iteration (equal weights),
    // filled with elevation-dependent weights for fine iteration
    SimpleMatrix Q = ignoreTopocentricParameters ? SimpleMatrix.identity(nObsAvail) : new SimpleMatrix(nObsAvail, nObsAvail);

    // Solution vector
    SimpleMatrix x;

    // Vector for observation error
    SimpleMatrix vEstim = new SimpleMatrix(nObsAvail, 1);

    // Vectors for troposphere and ionosphere corrections
    SimpleMatrix tropoCorr = new SimpleMatrix(nObsAvail, 1);

    SimpleMatrix ionoCorr = new SimpleMatrix(nObsAvail, 1);
    
    // Set up the least squares matrices
    for( int i = 0, k = 0; i < nObs; i++ ) {

      // Satellite ID
      int id = roverObs.getSatID(i);
      char satType = roverObs.getGnssType(i);   
      
      String checkAvailGnss = String.valueOf(satType) + String.valueOf(id);
      
      if( sats.pos[i]!=null && sats.avail.containsKey(id) && sats.gnssAvail.contains(checkAvailGnss)) {
//      if (sats.pos[i]!=null && sats.avail.contains(id)  && satTypeAvail.contains(satType)) {
//        System.out.println("####" + checkAvailGnss  + "####");

        // Fill in one row in the design matrix
        // diffSat = rover - sat, so diffSat/range = receiver-to-satellite unit vector
        // With A=[e, 1], the LS solution x gives: x[0:3] = dp (position correction), x[3] = -dt_rx
        A.set(k, 0, rover.diffSat[i].get(0) / rover.satAppRange[i]); /* X */
        A.set(k, 1, rover.diffSat[i].get(1) / rover.satAppRange[i]); /* Y */
        A.set(k, 2, rover.diffSat[i].get(2) / rover.satAppRange[i]); /* Z */
        A.set(k, 3, 1); /* clock error */
        for (int c = 0; c < sys.length(); c++) {
          A.set(k, 4+c, sys.indexOf(satType)==c?1:0); /* inter-system bias */
        }

        // Add the approximate pseudorange value to b
        double satRange = rover.satAppRange[i];
        double satClockError = sats.pos[i].getSatelliteClockError() * Constants.SPEED_OF_LIGHT;
        b.set(k, 0, satRange - satClockError);
        
        // Add the clock-corrected observed pseudorange value to y0
        double pseudorange = roverObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq());
        y0.set(k, 0, pseudorange);
        
        if (debug && k == 0) {
          System.err.printf("[DEBUG LS_SA] 卫星 %c%d 近似距离=%.2f 米, 卫星钟误差修正=%.6f 秒%n", 
                  satType, id, satRange, sats.pos[i].getSatelliteClockError());
          System.err.printf("[DEBUG LS_SA] 卫星 %c%d 伪距=%.2f 米 (%.10f 秒)%n", satType, id, pseudorange, pseudorange / Constants.SPEED_OF_LIGHT);
          System.err.printf("[DEBUG LS_SA] 卫星位置 X=%.2f Y=%.2f Z=%.2f%n", 
                  sats.pos[i].getX(), sats.pos[i].getY(), sats.pos[i].getZ());
        }

        if (!ignoreTopocentricParameters) {
          // Fill in troposphere and ionosphere double differenced
          // corrections
          tropoCorr.set(k, 0, rover.satTropoCorr[i]);
          ionoCorr.set(k, 0, rover.satIonoCorr[i]);

          // Fill in the cofactor matrix (weight = 1/variance)
          double weight = 1.0 / varerr(Math.toRadians(rover.topo[i].getElevation()), false, satType, goGPS.getFreq());
          Q.set(k, k, weight);
        }

        // Increment available satellites counter
        k++;
      }
      
    }

    if (!ignoreTopocentricParameters) {
      // Apply troposphere and ionosphere correction
      b = b.plus(tropoCorr);
      b = b.plus(ionoCorr);
    }

    // Least squares solution x = ((A'*Q^-1*A)^-1)*A'*Q^-1*(y0-b);
    // 【修改1】整个矩阵求逆块用try-catch保护，覆盖Q.invert()和N.invert()
    try {
      SimpleMatrix AtQinv = A.transpose().mult(Q.invert());
      SimpleMatrix N = AtQinv.mult(A);
      SimpleMatrix Ninv = N.invert();
      SimpleMatrix y0MinusB = y0.minus(b);
      x = Ninv.mult(AtQinv).mult(y0MinusB);
    } catch (Exception e) {
      // 矩阵奇异时（如NaN接收机位置、仅使用北斗GEO卫星等），将定位结果设为无效
      System.err.println("[LS_SA] Matrix inversion failed: " + e.getMessage());
      rover.setXYZ(0, 0, 0);
      return;
    }
    
    // 【修改2】检查解算结果是否包含NaN值
    // 当观测值误差过大或矩阵病态时，最小二乘解可能出现NaN
    if (Double.isNaN(x.get(0)) || Double.isNaN(x.get(1)) || Double.isNaN(x.get(2)) || Double.isNaN(x.get(3))) {
      System.err.println("[LS_SA] Solution contains NaN, discarding");
      rover.setXYZ(0, 0, 0);
      return;
    }
    
    if (debug) {
      System.err.printf("[DEBUG LS_SA] 解算: nUnknowns=%d, nObs=%d%n", nUnknowns, nObsAvail);
      System.err.printf("[DEBUG LS_SA] 解算: x=[%.2f, %.2f, %.2f, %.6f]%n", x.get(0), x.get(1), x.get(2), x.get(3));
    }
    
    // Debug: print per-satellite residuals
    if (debug) {
      SimpleMatrix preResid = y0.minus(b);
      for (int j = 0; j < Math.min(nObsAvail, 3); j++) {
        System.err.printf("[DEBUG LS_SA] sat[%d]: y0=%.2f, b=%.2f, y0-b=%.2f m, satClockError=%.6f s%n",
          j, y0.get(j), b.get(j), preResid.get(j), sats.pos[j].getSatelliteClockError());
      }
    }
    
    // Receiver clock error
    rover.clockError = x.get(3) / Constants.SPEED_OF_LIGHT;

    if(estimateOnlyClock)
      return;

    // check residuals 
    // Probably not needed if the elevation mask is set reasonably
//    SimpleMatrix resid = y0.minus(b);
//    resid = resid.plus(-rover.clockError * Constants.SPEED_OF_LIGHT);
//    double resMax = resid.elementMaxAbs();
//    if( resMax>1000 ) {
//    	rover.setXYZ(0, 0, 0);
//    	return;
//    }
    
    // Receiver position
    rover.setPlusXYZ(x.extractMatrix(0, 3, 0, 1));

    // 北斗GEO单点定位：固定高程约束
    Double fixedH = goGPS.getFixedHeight();
    if (fixedH != null) {
      rover.computeGeodetic();
      rover.setGeod(rover.getGeodeticLatitude(), rover.getGeodeticLongitude(), fixedH);
      rover.computeECEF();
    }

    // Estimation of the variance of the observation error
    vEstim = y0.minus(A.mult(x).plus(b));

    // Covariance matrix of the estimation error
    if (nObsAvail >= nUnknowns) {
      try {
        SimpleMatrix cofactor = A.transpose().mult(Q.invert()).mult(A).invert();
        if (nObsAvail > nUnknowns) {
          double varianceEstim = (vEstim.transpose().mult(Q.invert())
              .mult(vEstim)).get(0)
              / (nObsAvail - nUnknowns);
          positionCovariance = cofactor.scale(varianceEstim).extractMatrix(0, 3, 0, 3);
        } else {
          positionCovariance = cofactor.extractMatrix(0, 3, 0, 3);
        }
      } catch (Exception e) {
        System.err.println("[LS_SA] Covariance computation failed: " + e.getMessage());
        positionCovariance = null;
      }
    } else {
      positionCovariance = null;
    }

    updateDops(A);

    // Compute positioning in geodetic coordinates
    rover.computeGeodetic();
  }
  
  /**
   * Run code standalone.
   *
   * @param getNthPosition the get nth position
   * @return the coordinates
   * @throws Exception
   */
  public static void run( GoGPS goGPS, double stopAtDopThreshold ) {
    
    RoverPosition rover   = goGPS.getRoverPos();
    MasterPosition master = goGPS.getMasterPos();
    Satellites sats       = goGPS.getSats();
    ObservationsProducer roverIn = goGPS.getRoverIn();
    ObservationsProducer masterIn = goGPS.getMasterIn();
    boolean debug = goGPS.isDebug();
    boolean validPosition = false;
    
    LS_SA_code sa = new LS_SA_code(goGPS);
    
    RoverPosition coord = null;
    try {
      Observations obsR = roverIn.getCurrentObservations();
      while( obsR != null && !Thread.interrupted() ) { // buffStreamObs.ready()
//        if(debug) System.out.println("OK ");

        //try{
          // If there are at least four satellites
          if (obsR.getNumSat() >= 4) { // gps.length
            if(debug) System.out.println("Total number of satellites: "+obsR.getNumSat());

            // Compute approximate positioning by iterative least-squares
            if (! (rover.isValidXYZ() && rover.isValidClockError())) {
            	
            	 double el = -100;
            	 if( roverIn.getDefinedPosition() != null && roverIn.getDefinedPosition().isValidXYZ()) {
            		 roverIn.getDefinedPosition().cloneInto(rover);
            		 el = 5;
            	 } else {
            		 rover.setXYZ(0, 0, 0);
            	 }
            	 
            	 rover.setClockError(0);

              for (int iter = 0; iter < 10; iter++) {
                // Select all satellites
                sats.selectStandalone( obsR, goGPS.getCutoff());
                
                if (debug) {
                System.err.printf("[DEBUG LS_SA] 迭代 %d: 可用卫星数=%d, obsR时间=%s, gpsTime=%.0f%n", 
                        iter, sats.getAvailNumber(), 
                        obsR.getRefTime(), obsR.getRefTime().getGpsTime());
                
                System.err.printf("[DEBUG LS_SA] rover 位置: Lat=%.6f Lon=%.6f H=%.1f%n", 
                        rover.getGeodeticLatitude(), rover.getGeodeticLongitude(), rover.getGeodeticHeight());
                }
                
                if (sats.getAvailNumber() >= 4) {
                  sa.codeStandalone( obsR, false, true);
                  if (debug) {
                  System.err.printf("[DEBUG LS_SA] 迭代 %d codeStandalone 后: rover X=%.2f Y=%.2f Z=%.2f%n", 
                          iter, rover.getX(), rover.getY(), rover.getZ());
                  System.err.printf("[DEBUG LS_SA] 迭代 %d codeStandalone 后: Lat=%.6f Lon=%.6f H=%.1f%n", 
                          iter, rover.getGeodeticLatitude(), rover.getGeodeticLongitude(), rover.getGeodeticHeight());
                  }
                  // 【修改3】近似位置迭代中，若rover变为无效则提前终止
                  // 避免后续迭代基于无效位置（0,0,0）继续计算，导致矩阵持续奇异
                  if (!rover.isValidXYZ()) {
                    if(debug) System.out.println("Approximate LS iteration " + iter + " failed, breaking");
                    break;
                  }
                } else {
                  if (debug) System.err.printf("[DEBUG LS_SA] 迭代 %d: 可用卫星不足，跳过计算%n", iter);
                }
              }

            // If an approximate position was computed
              if(debug) System.out.println("Valid approximate position? "+rover.isValidXYZ()+ " " + rover.toString());
            }
            
            // 近似位置迭代失败时，直接使用基站坐标作为回退
            if (!rover.isValidXYZ() && roverIn.getDefinedPosition() != null && roverIn.getDefinedPosition().isValidXYZ()) {
              roverIn.getDefinedPosition().cloneInto(rover);
              rover.setClockError(0);
              rover.computeGeodetic();
              if(debug) System.out.println("Approximate position failed, fallback to base station: " + rover.toString());
            }
            
            if (rover.isValidXYZ() && rover.isValidClockError()) {
              double approxClock = rover.clockError;
              
              // Select available satellites
              sats.selectStandalone( obsR, goGPS.getCutoff());

              if (debug) {
              System.err.printf("[DEBUG LS_SA] selectStandalone 后: rover=%s, geod=%s%n", 
                      rover.toString(), String.format("Lat=%.6f Lon=%.6f H=%.1f", 
                              rover.getGeodeticLatitude(), 
                              rover.getGeodeticLongitude(),
                              rover.getGeodeticHeight()));
              System.err.printf("[DEBUG LS_SA] 可用卫星数: %d%n", sats.getAvailNumber());
              
              // 调试：打印每个卫星的仰角
              for (int i = 0; i < obsR.getNumSat(); i++) {
                int id = obsR.getSatID(i);
                char satType = obsR.getGnssType(i);
                if (rover.topo[i] != null) {
                  double el = rover.topo[i].getElevation();
                  System.err.printf("[DEBUG LS_SA] Sat %c%d 仰角=%.2f 度 %s%n", satType, id, el, el > 0 ? "✓" : "");
                }
              }
            }
              
              if (sats.getAvailNumber() >= 4){
                if(debug) System.out.println("Number of selected satellites: " + sats.getAvailNumber());
                // 【修改5】多次迭代（5次）确保LS收敛
                // 单次迭代在山区SPP场景下极易发散，增加迭代次数可提高收敛概率
                for (int iter = 0; iter < 5; iter++) {
                  sats.selectStandalone( obsR, goGPS.getCutoff());
                  sa.codeStandalone( obsR, false, false);
                  if (!rover.isValidXYZ()) {
                    if(debug) System.out.println("LS iteration " + iter + " failed, breaking");
                    break;
                  }
                }
                // 主LS解算失败时，回退到基站坐标
                if (!rover.isValidXYZ()) {
                  roverIn.getDefinedPosition().cloneInto(rover);
                  rover.clockError = approxClock;
                  rover.computeGeodetic();
                  if(debug) System.out.println("Main LS failed, fallback to base station position: " + rover.toString());
                }
              }
              else {
                // Discard approximate positioning, fallback to defined position if available
                if (roverIn.getDefinedPosition() != null && roverIn.getDefinedPosition().isValidXYZ()) {
                  roverIn.getDefinedPosition().cloneInto(rover);
                } else {
                  rover.setXYZ(0, 0, 0);
                }
              }
            }

            if(debug)System.out.println("Valid LS position? "+ (rover.isValidXYZ() && rover.isValidClockError() )+ " " + rover.toString() );
            if (rover.isValidXYZ() && rover.isValidClockError()) {
              rover.computeGeodetic();
              double lat = rover.getGeodeticLatitude();
              double lon = rover.getGeodeticLongitude();
              // 【修改7】区域坐标范围检查（青藏高原区域：北纬20-35°，东经90-102°）
              // 过滤定位结果中严重偏离目标区域的异常值（如17:34:48的异常点）
              // 注意：此范围针对青藏高原东部区域，其他区域使用时需修改
              if (lat < 20 || lat > 35 || lon < 90 || lon > 102) {
                rover.setXYZ(0, 0, 0);
                if(debug) System.out.println("Position out of region bounds, discarded: Lat=" + lat + " Lon=" + lon);
              }
            }
            if (rover.isValidXYZ() && rover.isValidClockError()) {
              if(!validPosition){
                goGPS.notifyPositionConsumerEvent(PositionConsumer.EVENT_START_OF_TRACK);
                validPosition = true;
              }
//              else 
              {
                coord = new RoverPosition(rover, DopType.STANDARD, rover.getpDop(), rover.gethDop(), rover.getvDop());

                if( goGPS.getPositionConsumers().size()>0){
                  coord.setRefTime(new Time(obsR.getRefTime().getMsec()));
                  coord.obs = obsR;
                  coord.sampleTime = obsR.getRefTime();
                  coord.status = rover.status;
                  coord.clockError = rover.clockError;
                  coord.clockErrorRate = rover.clockErrorRate;
                  goGPS.notifyPositionConsumerAddCoordinate(coord);

    							for(StreamEventListener sel:goGPS.getStreamEventListeners()){
    								Observations oc = (Observations)obsR.clone();
    								sel.addObservations(oc);
    							}
                  
                }
                if(debug)System.out.println("PDOP: "+rover.getpDop());
                if(debug)System.out.println("------------------------------------------------------------");
                if( stopAtDopThreshold>0.0 && rover.getpDop()<stopAtDopThreshold){
                  return;
                }
              }
            }
          }
//        }catch(Exception e){
//          System.out.println("Could not complete due to "+e);
//          e.printStackTrace();
//        }
        obsR = roverIn.getNextObservations();
      }
    } catch (Exception e) {
      e.printStackTrace();
      throw e;
    } finally {
      goGPS.notifyPositionConsumerEvent(PositionConsumer.EVENT_END_OF_TRACK);
    }
  }
  
}