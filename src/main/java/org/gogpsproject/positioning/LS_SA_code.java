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

    // Cofactor matrix (initialized to identity)
    SimpleMatrix Q = SimpleMatrix.identity(nObsAvail);

    // Solution vector
    SimpleMatrix x = new SimpleMatrix(nUnknowns, 1);

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
      
      if( sats.pos[i]!=null && sats.gnssAvail.contains(checkAvailGnss)) {
//      if (sats.pos[i]!=null && sats.avail.contains(id)  && satTypeAvail.contains(satType)) {
//        System.out.println("####" + checkAvailGnss  + "####");

        // Fill in one row in the design matrix
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
        
        if (k == 0) {
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

          // Fill in the cofactor matrix
          double weight = Q.get(k, k)
              + computeWeight(rover.topo[i].getElevation(),
                  roverObs.getSatByIDType(id, satType).getSignalStrength(goGPS.getFreq()));
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
    SimpleMatrix AtQinv = A.transpose().mult(Q.invert());
    SimpleMatrix N = AtQinv.mult(A);
    SimpleMatrix Ninv = N.invert();
    SimpleMatrix y0MinusB = y0.minus(b);
    x = Ninv.mult(AtQinv).mult(y0MinusB);
    
    System.err.printf("[DEBUG LS_SA] 解算: nUnknowns=%d, nObs=%d%n", nUnknowns, nObsAvail);
    System.err.printf("[DEBUG LS_SA] 解算: x=[%.2f, %.2f, %.2f, %.6f]%n", x.get(0), x.get(1), x.get(2), x.get(3));
    
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

    // Estimation of the variance of the observation error
    vEstim = y0.minus(A.mult(x).plus(b));
    double varianceEstim = (vEstim.transpose().mult(Q.invert())
        .mult(vEstim)).get(0)
        / (nObsAvail - nUnknowns);

    // Covariance matrix of the estimation error
    if (nObsAvail > nUnknowns) {
      positionCovariance = A.transpose().mult(Q.invert()).mult(A).invert()
      .scale(varianceEstim)
      .extractMatrix(0, 3, 0, 3);
    }else{
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
      // 检查是否为 RTCM3FileReader，如果是则先读取消息收集基站坐标和星历
      System.err.println("[DEBUG LS_SA] roverIn 类型: " + roverIn.getClass().getName());
      if (roverIn instanceof org.gogpsproject.producer.parser.rtcm3.RTCM3FileReader) {
        System.err.println("[DEBUG LS_SA] 检测到 RTCM3FileReader，开始预处理...");
        org.gogpsproject.producer.parser.rtcm3.RTCM3FileReader rtcmReader = 
            (org.gogpsproject.producer.parser.rtcm3.RTCM3FileReader) roverIn;
        
        // 先读取所有消息，收集基站坐标和星历
        // 注意：基站坐标消息可能在观测数据之后
        Object readNextResult = null;
        int preObsCount = 0;
        while ((readNextResult = rtcmReader.readNext()) != null) {
          if (readNextResult instanceof Observations) {
            preObsCount++;
          }
        }
        System.err.println("[DEBUG LS_SA] 预处理完成: 读取了 " + preObsCount + " 条观测消息");
        
        // 打印读取到的基站坐标
        org.gogpsproject.positioning.Coordinates definedPos = rtcmReader.getDefinedPosition();
        System.err.println("[DEBUG LS_SA] 基站坐标检查: definedPos=" + (definedPos != null ? definedPos.toString() : "null") + ", isValid=" + (definedPos != null ? definedPos.isValidXYZ() : false));
        // 也检查 RTCM3Client 中的基站位置
        if (definedPos != null && definedPos.isValidXYZ()) {
          System.err.println("[DEBUG LS_SA] 最终基站坐标: X=" + definedPos.getX() + " Y=" + definedPos.getY() + " Z=" + definedPos.getZ());
        }
      }
      Observations obsR = roverIn.getCurrentObservations();
      // 如果已经读取完所有消息，尝试重新初始化文件读取器
      if (obsR == null && roverIn instanceof org.gogpsproject.producer.parser.rtcm3.RTCM3FileReader) {
        org.gogpsproject.producer.parser.rtcm3.RTCM3FileReader rtcmReader = 
            (org.gogpsproject.producer.parser.rtcm3.RTCM3FileReader) roverIn;
        try {
          rtcmReader.init();
          obsR = rtcmReader.getNextObservations();
        } catch (Exception e) {
          System.err.println("重新初始化失败: " + e.getMessage());
        }
      }
      while( obsR != null && !Thread.interrupted() ) { // buffStreamObs.ready()
//        if(debug) System.out.println("OK ");

        //try{
          // If there are at least four satellites
          if (obsR.getNumSat() >= 4) { // gps.length
            if(debug) System.out.println("Total number of satellites: "+obsR.getNumSat());

            // Compute approximate positioning by iterative least-squares
            if (! (rover.isValidXYZ() && rover.isValidClockError())) {
            	
            	 double el = -100;
            	 rover.setXYZ(0, 0, 0);            	 
            	 if( roverIn.getDefinedPosition() != null && roverIn.getDefinedPosition().isValidXYZ()) {
            		 roverIn.getDefinedPosition().cloneInto(rover);
            		 el = 5;
            	 }
            	 
            	 rover.setClockError(0);

              for (int iter = 0; iter < 3; iter++) {
                // Select all satellites - 临时使用 -200 cutoff 来禁用仰角过滤
                sats.selectStandalone( obsR, -200);
                
                System.err.printf("[DEBUG LS_SA] 迭代 %d: 可用卫星数=%d%n", iter, sats.getAvailNumber());
                System.err.printf("[DEBUG LS_SA] rover 位置: Lat=%.6f Lon=%.6f H=%.1f%n", 
                        rover.getGeodeticLatitude(), rover.getGeodeticLongitude(), rover.getGeodeticHeight());
                
                if (sats.getAvailNumber() >= 4) {
                  sa.codeStandalone( obsR, false, true);
                  System.err.printf("[DEBUG LS_SA] 迭代 %d codeStandalone 后: rover X=%.2f Y=%.2f Z=%.2f%n", 
                          iter, rover.getX(), rover.getY(), rover.getZ());
                  System.err.printf("[DEBUG LS_SA] 迭代 %d codeStandalone 后: Lat=%.6f Lon=%.6f H=%.1f%n", 
                          iter, rover.getGeodeticLatitude(), rover.getGeodeticLongitude(), rover.getGeodeticHeight());
                } else {
                  System.err.printf("[DEBUG LS_SA] 迭代 %d: 可用卫星不足，跳过计算%n", iter);
                }
              }

            // If an approximate position was computed
              if(debug) System.out.println("Valid approximate position? "+rover.isValidXYZ()+ " " + rover.toString());
            }
            
            if (rover.isValidXYZ() && rover.isValidClockError()) {
              // Select available satellites - 临时使用 -200 cutoff
              sats.selectStandalone( obsR, -200);

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
              
              if (sats.getAvailNumber() >= 4){
                if(debug) System.out.println("Number of selected satellites: " + sats.getAvailNumber());
                // Compute code stand-alone positioning (epoch-by-epoch solution)
                sa.codeStandalone( obsR, false, false);
              }
              else {
                // Discard approximate positioning
                rover.setXYZ(0, 0, 0);
              }
            }

            if(debug)System.out.println("Valid LS position? "+ (rover.isValidXYZ() && rover.isValidClockError() )+ " " + rover.toString() );
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
