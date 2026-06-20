package org.gogpsproject.positioning;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;

import org.ejml.simple.SimpleMatrix;
import org.gogpsproject.Constants;
import org.gogpsproject.GoGPS;
import org.gogpsproject.Status;
import org.gogpsproject.producer.NavigationProducer;
import org.gogpsproject.producer.ObservationSet;
import org.gogpsproject.producer.Observations;
import org.gogpsproject.producer.parser.IonoGps;

public class Satellites {
  
  GoGPS goGPS;
  RoverPosition rover;
  MasterPosition master; 
  NavigationProducer navigation;

  /** Absolute position of all visible satellites (ECEF) */
  SatellitePosition[] pos; 

  /** List of satellites available for processing */
  Map<Integer, SatellitePosition> avail; 
 
  /** List of satellites available for processing */
  ArrayList<Integer> availPhase; 
  
  /** List of satellite Types available for processing */
  ArrayList<Character> typeAvail; 
  
  /** List of satellite Type available for processing */
  ArrayList<Character> typeAvailPhase; 
  
  /** List of satellite Types & Id available for processing */
  ArrayList<String> gnssAvail;  
  
  /** List of satellite Types & Id available for processing */
  ArrayList<String> gnssAvailPhase;  

  /** Index of the satellite with highest elevation in satAvail list */
  int pivot;

  public Satellites(GoGPS goGPS) {
    this.goGPS = goGPS;
    this.rover = goGPS.getRoverPos();
    this.master = goGPS.getMasterPos();
    this.navigation = goGPS.getNavigation();
  }

  /** @return the number of available satellites */
  public int getAvailNumber() {
    return avail.size();
  }

  /** @return the number of available satellites (with phase) */
  public int getAvailPhaseNumber() {
    return availPhase.size();
  }
  
  public String getAvailGnssSystems(){
    if( typeAvail.isEmpty()) return "";
    String GnssSys = "";
    for(int i=0;i< typeAvail.size();i++) {
      if (GnssSys.indexOf(( typeAvail.get(i))) < 0)
        GnssSys = GnssSys + typeAvail.get(i);
    }
    return GnssSys;
  }
  
  private static final double REL_HUMI = 0.7;

  /** Previous phase wind-up value per satellite (cycles), keyed by satID */
  private Map<Integer, Double> prevPhwRover = new LinkedHashMap<>();
  private Map<Integer, Double> prevPhwMaster = new LinkedHashMap<>();
  private long prevTimeRover = 0;
  private long prevTimeMaster = 0;

  /**
   * RTKLIB-aligned tropmodel: compute tropospheric delay by standard atmosphere
   * and Saastamoinen model.
   * @param lat   geodetic latitude (rad)
   * @param height height above ellipsoid (m)
   * @param el    elevation angle (rad)
   * @return tropospheric delay (m)
   */
  static double computeTroposphereCorrection(double lat, double height, double el) {

    if (height < -100.0 || height > 1E4 || el <= 0.0) return 0.0;

    double hgt = height < 0.0 ? 0.0 : height;

    double pres = 1013.25 * Math.pow(1.0 - 2.2557E-5 * hgt, 5.2568);
    double temp = 15.0 - 6.5E-3 * hgt + 273.16;
    double e = 6.108 * REL_HUMI * Math.exp((17.15 * temp - 4684.0) / (temp - 38.45));

    double z = Math.PI / 2.0 - el;
    double trph = 0.0022768 * pres / (1.0 - 0.00266 * Math.cos(2.0 * lat) - 0.00028 * hgt / 1E3) / Math.cos(z);
    double trpw = 0.002277 * (1255.0 / temp + 0.05) * e / Math.cos(z);

    return trph + trpw;
  }
  
  private static final double[] ION_DEFAULT = {
    0.1118E-07, -0.7451E-08, -0.5961E-07,  0.1192E-06,
    0.1167E+06, -0.2294E+06, -0.1311E+06,  0.1049E+07
  };

  /**
   * RTKLIB-aligned ionmodel: compute ionospheric delay by broadcast
   * Klobuchar model.
   * @param t     time (GPST seconds of week)
   * @param ion   iono parameters {a0,a1,a2,a3,b0,b1,b2,b3}
   * @param lat   geodetic latitude (rad)
   * @param lon   geodetic longitude (rad)
   * @param az    azimuth angle (rad)
   * @param el    elevation angle (rad)
   * @return ionospheric delay L1 (m)
   */
  static double computeIonosphereCorrection(double t, double[] ion,
      double lat, double lon, double az, double el) {

    if (el <= 0.0) return 0.0;

    if (ion == null || norm(ion) <= 0.0) ion = ION_DEFAULT;

    double psi = 0.0137 / (el / Math.PI + 0.11) - 0.022;

    double phi = lat / Math.PI + psi * Math.cos(az);
    if      (phi >  0.416) phi =  0.416;
    else if (phi < -0.416) phi = -0.416;
    double lam = lon / Math.PI + psi * Math.sin(az) / Math.cos(phi * Math.PI);

    phi += 0.064 * Math.cos((lam - 1.617) * Math.PI);

    double tt = 43200.0 * lam + t;
    tt -= Math.floor(tt / 86400.0) * 86400.0;

    double f = 1.0 + 16.0 * Math.pow(0.53 - el / Math.PI, 3.0);

    double amp = ion[0] + phi * (ion[1] + phi * (ion[2] + phi * ion[3]));
    double per = ion[4] + phi * (ion[5] + phi * (ion[6] + phi * ion[7]));
    if (amp <     0.0) amp =     0.0;
    if (per < 72000.0) per = 72000.0;
    double x = 2.0 * Math.PI * (tt - 50400.0) / per;

    return Constants.SPEED_OF_LIGHT * f * (Math.abs(x) < 1.57
        ? 5E-9 + amp * (1.0 + x * x * (-0.5 + x * x / 24.0))
        : 5E-9);
  }

  private static double norm(double[] v) {
    double s = 0.0;
    for (double d : v) s += d * d;
    return Math.sqrt(s);
  }

  /**
   * RTKLIB-aligned interpvar: linear interpolation of PCV table.
   * PCV table has 19 values covering 0-90 deg zenith angle at 5 deg steps.
   * @param ang  zenith angle (deg) for receiver; nadir*5 (deg) for satellite
   * @param var  PCV variation table (19 values)
   * @return interpolated PCV value (m)
   */
  private static double interpvar(double ang, double[] var) {
    double a = ang / 5.0;
    int i = (int) a;
    if (i < 0) return var[0];
    if (i >= 18) return var[18];
    return var[i] * (1.0 - a + i) + var[i + 1] * (a - i);
  }

  /**
   * RTKLIB-aligned antmodel: receiver antenna phase center correction.
   * Computes the range offset from PCO (phase center offset) and PCV
   * (phase center variation) for the receiver antenna.
   * @param pcv   receiver antenna PCV parameters
   * @param del   antenna delta {e, n, u} (m)
   * @param az    azimuth angle (rad)
   * @param el    elevation angle (rad)
   * @param opt   0: PCO only, 1: PCO + PCV
   * @param freq  frequency index (0=L1, 1=L2, 2=L5)
   * @return range offset (m) to be subtracted from observation
   */
  static double antennaCorrection(AntennaPcv pcv, double[] del,
      double az, double el, int opt, int freq) {

    if (pcv == null) return 0.0;

    double cosel = Math.cos(el);
    double[] e = new double[3];
    e[0] = Math.sin(az) * cosel;
    e[1] = Math.cos(az) * cosel;
    e[2] = Math.sin(el);

    double[] off = new double[3];
    for (int j = 0; j < 3; j++) {
      off[j] = pcv.getOff(freq, j) + (del != null ? del[j] : 0.0);
    }

    double dant = -dot3(off, e);
    if (opt != 0) {
      dant += interpvar(90.0 - Math.toDegrees(el), pcv.getVar()[freq]);
    }
    return dant;
  }

  /**
   * RTKLIB-aligned antmodel_s: satellite antenna phase center variation.
   * Computes PCV from nadir angle.
   * @param pcv    satellite antenna PCV parameters
   * @param nadir  nadir angle (rad)
   * @param freq   frequency index (0=L1, 1=L2, 2=L5)
   * @return range offset (m) to be subtracted from observation
   */
  static double satelliteAntennaVariation(AntennaPcv pcv, double nadir, int freq) {
    if (pcv == null) return 0.0;
    return interpvar(Math.toDegrees(nadir) * 5.0, pcv.getVar()[freq]);
  }

  /**
   * RTKLIB-aligned satantpcv: compute satellite antenna PCV from
   * satellite and receiver positions.
   * @param rs  satellite position {x, y, z} (m, ECEF)
   * @param rr  receiver position {x, y, z} (m, ECEF)
   * @param pcv satellite antenna PCV parameters
   * @param freq frequency index (0=L1, 1=L2, 2=L5)
   * @return PCV range offset (m)
   */
  static double satelliteAntennaCorrection(double[] rs, double[] rr,
      AntennaPcv pcv, int freq) {

    if (pcv == null) return 0.0;

    double[] ru = new double[3];
    double[] rz = new double[3];
    for (int i = 0; i < 3; i++) {
      ru[i] = rr[i] - rs[i];
      rz[i] = -rs[i];
    }

    double[] eu = new double[3];
    double[] ez = new double[3];
    if (!normv3(ru, eu) || !normv3(rz, ez)) return 0.0;

    double cosa = dot3(eu, ez);
    cosa = cosa < -1.0 ? -1.0 : (cosa > 1.0 ? 1.0 : cosa);
    double nadir = Math.acos(cosa);

    return satelliteAntennaVariation(pcv, nadir, freq);
  }

  private static double dot3(double[] a, double[] b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  }

  private static boolean normv3(double[] a, double[] b) {
    double r = Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (r <= 0.0) return false;
    b[0] = a[0] / r;
    b[1] = a[1] / r;
    b[2] = a[2] / r;
    return true;
  }

  private static double[] cross3(double[] a, double[] b) {
    return new double[] {
      a[1] * b[2] - a[2] * b[1],
      a[2] * b[0] - a[0] * b[2],
      a[0] * b[1] - a[1] * b[0]
    };
  }

  /** Convert ECEF (x,y,z) to geodetic (lat,lon,h) in radians/meters. pos[0]=lat, pos[1]=lon, pos[2]=h */
  private static void ecef2pos(double[] r, double[] pos) {
    double e2 = Constants.WGS84_ECCENTRICITY;
    double r2 = r[0] * r[0] + r[1] * r[1];
    double z = r[2], zk;
    double v = Constants.WGS84_SEMI_MAJOR_AXIS;
    for (zk = 0.0; Math.abs(z - zk) >= 1E-4; ) {
      zk = z;
      double sinp = z / Math.sqrt(r2 + z * z);
      v = Constants.WGS84_SEMI_MAJOR_AXIS / Math.sqrt(1.0 - e2 * sinp * sinp);
      z = r[2] + v * e2 * sinp;
    }
    pos[0] = r2 > 1E-12 ? Math.atan(z / Math.sqrt(r2)) : (r[2] > 0.0 ? Math.PI / 2.0 : -Math.PI / 2.0);
    pos[1] = r2 > 1E-12 ? Math.atan2(r[1], r[0]) : 0.0;
    pos[2] = Math.sqrt(r2 + z * z) - v;
  }

  /** Compute ECEF to ENU rotation matrix. pos[0]=lat(rad), pos[1]=lon(rad). Returns 9-element array. */
  private static double[] xyz2enu(double[] pos) {
    double sinp = Math.sin(pos[0]), cosp = Math.cos(pos[0]);
    double sinl = Math.sin(pos[1]), cosl = Math.cos(pos[1]);
    return new double[] {
      -sinl,       cosl,       0.0,
      -sinp * cosl, -sinp * sinl, cosp,
       cosp * cosl,  cosp * sinl, sinp
    };
  }

  /**
   * RTKLIB-aligned phase wind-up correction (model_phw).
   * Uses nominal yaw-steering model: satellite Z=radial, Y=Z×sun, X=Y×Z.
   * @param time      GPS time (milliseconds of week)
   * @param rs        satellite position ECEF (m)
   * @param rr        receiver position ECEF (m)
   * @param prevPhw   previous wind-up value (cycles), updated in-place
   * @return phase wind-up correction (cycles), 0 if first epoch
   */
  double computeWindUpCorrection(double time, double[] rs, double[] rr,
      Map<Integer, Double> prevPhw, long prevTime, int satId) {

    Double prev = prevPhw.get(satId);
    if (prev == null) {
      prevPhw.put(satId, 0.0);
      return 0.0;
    }

    double[] exs = new double[3];
    double[] eys = new double[3];
    double[] ek = new double[3];

    // Satellite body axes: nominal yaw-steering
    // Z-axis: satellite to Earth center (radial)
    double[] ezs = new double[] { -rs[0], -rs[1], -rs[2] };
    if (!normv3(ezs, ezs)) return 0.0;

    // Sun position (simplified analytical model)
    double[] rsun = computeSunPosition(time);
    double[] esun = new double[3];
    if (!normv3(rsun, esun)) return 0.0;

    // Y-axis: cross product of Z and sun direction
    double[] ey = cross3(ezs, esun);
    if (!normv3(ey, eys)) return 0.0;

    // X-axis: completes right-hand system
    double[] ex = cross3(eys, ezs);
    if (!normv3(ex, exs)) return 0.0;

    // Unit vector satellite to receiver
    double[] r = new double[] { rr[0] - rs[0], rr[1] - rs[1], rr[2] - rs[2] };
    if (!normv3(r, ek)) return 0.0;

    // Receiver antenna axes (ENU: x=North, y=West)
    double[] pos = new double[3];
    ecef2pos(rr, pos);
    double[] E = xyz2enu(pos);
    double[] exr = new double[] { E[1], E[4], E[7] };  // x = north
    double[] eyr = new double[] { -E[0], -E[3], -E[6] }; // y = west

    // Phase wind-up effect
    double[] eks = cross3(ek, eys);
    double[] ekr = cross3(ek, eyr);

    double[] ds = new double[3];
    double[] dr = new double[3];
    double dotEkExs = ek[0] * exs[0] + ek[1] * exs[1] + ek[2] * exs[2];
    double dotEkExr = ek[0] * exr[0] + ek[1] * exr[1] + ek[2] * exr[2];
    for (int i = 0; i < 3; i++) {
      ds[i] = exs[i] - ek[i] * dotEkExs - eks[i];
      dr[i] = exr[i] - ek[i] * dotEkExr + ekr[i];
    }

    double cosp = (ds[0] * dr[0] + ds[1] * dr[1] + ds[2] * dr[2])
        / Math.sqrt(ds[0] * ds[0] + ds[1] * ds[1] + ds[2] * ds[2])
        / Math.sqrt(dr[0] * dr[0] + dr[1] * dr[1] + dr[2] * dr[2]);
    if (cosp < -1.0) cosp = -1.0;
    else if (cosp > 1.0) cosp = 1.0;
    double ph = Math.acos(cosp) / (2.0 * Math.PI);

    double[] drs = cross3(ds, dr);
    if (ek[0] * drs[0] + ek[1] * drs[1] + ek[2] * drs[2] < 0.0) ph = -ph;

    double phw = ph + Math.floor(prev - ph + 0.5);
    prevPhw.put(satId, phw);
    return phw;
  }

  /**
   * Simplified sun position in ECEF (m) from GPS time.
   * Uses analytical solar ephemeris.
   */
  private double[] computeSunPosition(double gpsTime) {
    // GPS time to Julian centuries since J2000.0
    // gpsTime: total GPS seconds since GPS epoch (1980-01-06 00:00:00)
    double jd = 2444244.5 + gpsTime / 86400.0;
    double T = (jd - 2451545.0) / 36525.0;

    // Mean anomaly of sun
    double M = Math.toRadians(357.5291 + 35999.0503 * T - 0.0001559 * T * T);
    // Mean longitude of sun
    double L0 = Math.toRadians(280.4665 + 36000.7698 * T);
    // Ecliptic longitude
    double lambda = L0 + Math.toRadians((1.9147 - 0.0048 * T) * Math.sin(M)
        + 0.0200 * Math.sin(2 * M));
    // Obliquity of ecliptic
    double eps = Math.toRadians(23.4393 - 0.0130 * T);
    // Distance (AU)
    double r = 1.00014 - 0.01671 * Math.cos(M) - 0.00014 * Math.cos(2 * M);

    double AU = 149597870700.0;
    double rse = r * AU;
    return new double[] {
      rse * Math.cos(lambda),
      rse * Math.sin(lambda) * Math.cos(eps),
      rse * Math.sin(lambda) * Math.sin(eps)
    };
  }

  void init( Observations roverObs ) {

    int nObs = roverObs.getNumSat();

    // Allocate an array to store GPS satellite positions
    pos = new SatellitePosition[nObs];

    // Create a list for available satellites
    avail = new LinkedHashMap<>();
    typeAvail = new ArrayList<>(0);
    gnssAvail = new ArrayList<>(0);

    // Create a list for available satellites with phase
    availPhase = new ArrayList<>(0);
    typeAvailPhase = new ArrayList<>(0);
    gnssAvailPhase = new ArrayList<>(0);

    // Allocate arrays to store receiver-satellite vectors
    rover.diffSat = new SimpleMatrix[nObs];
    master.diffSat = new SimpleMatrix[nObs];

    // Allocate arrays to store receiver-satellite approximate range
    rover.satAppRange = new double[nObs];
    master.satAppRange = new double[nObs];

    // Allocate arrays to store receiver-satellite atmospheric corrections
    rover.satTropoCorr = new double[nObs];
    rover.satIonoCorr = new double[nObs];
    master.satTropoCorr = new double[nObs];
    master.satIonoCorr = new double[nObs];

    // Allocate arrays to store receiver-satellite antenna corrections
    rover.satAntennaCorr = new double[nObs];
    master.satAntennaCorr = new double[nObs];

    // Allocate arrays to store receiver-satellite phase wind-up corrections
    rover.satWindUp = new double[nObs];
    master.satWindUp = new double[nObs];

    // Allocate arrays for GF (Geometry-Free) combination cycle slip detection
    // Sized by max satellite ID (BDS=63) to support satID-based indexing
    rover.prevGf = new double[64];
    master.prevGf = new double[64];

    // Allocate arrays of topocentric coordinates
    rover.topo = new TopocentricCoordinates[nObs];
    master.topo = new TopocentricCoordinates[nObs];
    
    rover.satsInUse = 0;
  }
  
  /**
   * @param roverObs
   */
  public void selectStandalone( Observations roverObs ) {
    selectStandalone( roverObs, goGPS.getCutoff() );
  }

  /**
   * @param roverObs
   * @param cutoff
   */
  public void selectStandalone( Observations roverObs, double cutoff) {

    init( roverObs );
    
    // Compute topocentric coordinates and
    // select satellites above the cutoff level
    for( int i = 0; i < roverObs.getNumSat(); i++) {

      int id = roverObs.getSatID(i);
      char satType = roverObs.getGnssType(i);

      // Compute GPS satellite positions getGpsByIdx(idx).getSatType()
      pos[i] = navigation.getGpsSatPosition( roverObs, id, satType, rover.getClockError());
      
      if(pos[i]!=null){

    	  	if( pos[i].equals( SatellitePosition.UnhealthySat )) {
    	        pos[i] = null;
              roverObs.getSatByIdx(i).setInUse(false);
    	        continue;
    	  	}
    	  	
        // Compute rover-satellite approximate pseudorange
        rover.diffSat[i] = rover.minusXYZ(pos[i]);
        rover.satAppRange[i] = Math.sqrt(Math.pow(rover.diffSat[i].get(0), 2)
            + Math.pow(rover.diffSat[i].get(1), 2)
            + Math.pow(rover.diffSat[i].get(2), 2));

        // Compute azimuth, elevation and distance for each satellite
        rover.topo[i] = new TopocentricCoordinates();
        rover.topo[i].computeTopocentric(rover, pos[i]);

        // Correct approximate pseudorange for troposphere (RTKLIB-aligned)
        rover.satTropoCorr[i] = computeTroposphereCorrection(
            Math.toRadians(rover.getGeodeticLatitude()),
            rover.getGeodeticHeight(),
            Math.toRadians(rover.topo[i].getElevation()));

        // Correct approximate pseudorange for ionosphere (RTKLIB-aligned)
        IonoGps ionoGps = navigation.getIono(roverObs.getRefTime().getMsec());
        double[] ionParams = (ionoGps != null)
            ? new double[]{ionoGps.getAlpha(0), ionoGps.getAlpha(1), ionoGps.getAlpha(2), ionoGps.getAlpha(3),
                           ionoGps.getBeta(0),  ionoGps.getBeta(1),  ionoGps.getBeta(2),  ionoGps.getBeta(3)}
            : null;
        rover.satIonoCorr[i] = computeIonosphereCorrection(
            roverObs.getRefTime().getGpsTime(),
            ionParams,
            Math.toRadians(rover.getGeodeticLatitude()),
            Math.toRadians(rover.getGeodeticLongitude()),
            Math.toRadians(rover.topo[i].getAzimuth()),
            Math.toRadians(rover.topo[i].getElevation()));

        // Correct approximate pseudorange for antenna phase center (RTKLIB-aligned)
        double azRad = Math.toRadians(rover.topo[i].getAzimuth());
        double elRad = Math.toRadians(rover.topo[i].getElevation());
        double rcvAntCorr = antennaCorrection(goGPS.getReceiverAntennaPcv(),
            goGPS.getAntennaDelta(), azRad, elRad, 1, goGPS.getFreq());
        double[] rs = {pos[i].getX(), pos[i].getY(), pos[i].getZ()};
        double[] rr = {rover.getX(), rover.getY(), rover.getZ()};
        double satAntCorr = satelliteAntennaCorrection(rs, rr,
            goGPS.getSatelliteAntennaPcv(id), goGPS.getFreq());
        rover.satAntennaCorr[i] = rcvAntCorr + satAntCorr;

        // Phase wind-up correction (RTKLIB-aligned)
        long gpsWeek = roverObs.getRefTime().getGpsWeek();
        double gpsSec = roverObs.getRefTime().getGpsTime();
        double gpsTime = gpsWeek * 604800.0 + gpsSec;
        rover.satWindUp[i] = computeWindUpCorrection(gpsTime, rs, rr,
            prevPhwRover, prevTimeRover, id);
        prevTimeRover = (long) gpsTime;

//        System.out.println("getElevation: " + id + "::" + rover.topo[i].getElevation() ); 
        // Check if satellite elevation is higher than cutoff
        // 如果仰角为 NaN（接收机位置未知），则不进行仰角过滤
        double elevation = rover.topo[i].getElevation();
        if (Double.isNaN(elevation) || elevation > cutoff) {
          
          avail.put(id, pos[i]);
          typeAvail.add(satType);
          gnssAvail.add(String.valueOf(satType) + String.valueOf(id));
          roverObs.getSatByIdx(i).setInUse(true);
          
          // Check if also phase is available
          if (!Double.isNaN(roverObs.getSatByIDType(id, satType).getPhaseCycles(goGPS.getFreq()))) {
            availPhase.add(id);
            typeAvailPhase.add(satType);
            gnssAvailPhase.add(String.valueOf(satType) + String.valueOf(id));       
            
          }
        }else{
          if(goGPS.isDebug()) System.out.println("Not useful sat "+roverObs.getSatID(i)+" for too low elevation "+rover.topo[i].getElevation()+" < "+cutoff);
          roverObs.getSatByIdx(i).setInUse(false);
        }
      }
    }
  }

  /**
   * @param roverObs
   * @param masterObs
   * @param masterPos
   */
  public void selectDoubleDiff( Observations roverObs, Observations masterObs, Coordinates masterPos ) {

    // Retrieve options from goGPS class
    double cutoff = goGPS.getCutoff();

    init( roverObs );
    
    // Variables to store highest elevation
    double maxElevCode = 0;
    double maxElevPhase = 0;

    // Variables for code pivot and phase pivot
    int pivotCode = -1;
    int pivotPhase = -1;

    // Satellite ID
    int id = 0;

    // Compute topocentric coordinates and
    // select satellites above the cutoff level
    for (int i = 0; i < roverObs.getNumSat(); i++) {

      id = roverObs.getSatID(i);
      char satType = roverObs.getGnssType(i);

      // Compute GPS satellite positions
      pos[i] = navigation.getGpsSatPosition(roverObs, id, satType, rover.getClockError());

      if(pos[i]!=null){

        // Compute rover-satellite approximate pseudorange
        rover.diffSat[i] = rover.minusXYZ(pos[i]);
        rover.satAppRange[i] = Math.sqrt(Math.pow(rover.diffSat[i].get(0), 2)
                             + Math.pow(rover.diffSat[i].get(1), 2)
                             + Math.pow(rover.diffSat[i].get(2), 2));

        // Compute master-satellite approximate pseudorange
        master.diffSat[i]     = masterPos.minusXYZ(pos[i]);
        master.satAppRange[i] = Math.sqrt(Math.pow(master.diffSat[i].get(0), 2)
                              + Math.pow(master.diffSat[i].get(1), 2)
                              + Math.pow(master.diffSat[i].get(2), 2));

        // Compute azimuth, elevation and distance for each satellite from rover
        rover.topo[i] = new TopocentricCoordinates();
        rover.topo[i].computeTopocentric(rover, pos[i]);

        // Compute azimuth, elevation and distance for each satellite from master
        master.topo[i] = new TopocentricCoordinates();
        master.topo[i].computeTopocentric(masterPos, pos[i]);

        // Computation of rover-satellite troposphere correction (RTKLIB-aligned)
        rover.satTropoCorr[i] = computeTroposphereCorrection(
            Math.toRadians(rover.getGeodeticLatitude()),
            rover.getGeodeticHeight(),
            Math.toRadians(rover.topo[i].getElevation()));

        // Computation of master-satellite troposphere correction (RTKLIB-aligned)
        master.satTropoCorr[i] = computeTroposphereCorrection(
            Math.toRadians(masterPos.getGeodeticLatitude()),
            masterPos.getGeodeticHeight(),
            Math.toRadians(master.topo[i].getElevation()));

        // Computation of rover-satellite ionosphere correction (RTKLIB-aligned)
        IonoGps ionoGpsDD = navigation.getIono(roverObs.getRefTime().getMsec());
        double[] ionParamsDD = (ionoGpsDD != null)
            ? new double[]{ionoGpsDD.getAlpha(0), ionoGpsDD.getAlpha(1), ionoGpsDD.getAlpha(2), ionoGpsDD.getAlpha(3),
                           ionoGpsDD.getBeta(0),  ionoGpsDD.getBeta(1),  ionoGpsDD.getBeta(2),  ionoGpsDD.getBeta(3)}
            : null;
        rover.satIonoCorr[i] = computeIonosphereCorrection(
            roverObs.getRefTime().getGpsTime(),
            ionParamsDD,
            Math.toRadians(rover.getGeodeticLatitude()),
            Math.toRadians(rover.getGeodeticLongitude()),
            Math.toRadians(rover.topo[i].getAzimuth()),
            Math.toRadians(rover.topo[i].getElevation()));

        // Computation of master-satellite ionosphere correction (RTKLIB-aligned)
        master.satIonoCorr[i] = computeIonosphereCorrection(
            roverObs.getRefTime().getGpsTime(),
            ionParamsDD,
            Math.toRadians(masterPos.getGeodeticLatitude()),
            Math.toRadians(masterPos.getGeodeticLongitude()),
            Math.toRadians(master.topo[i].getAzimuth()),
            Math.toRadians(master.topo[i].getElevation()));

        // Computation of rover-satellite antenna correction (RTKLIB-aligned)
        double azRadR = Math.toRadians(rover.topo[i].getAzimuth());
        double elRadR = Math.toRadians(rover.topo[i].getElevation());
        double rcvAntCorrR = antennaCorrection(goGPS.getReceiverAntennaPcv(),
            goGPS.getAntennaDelta(), azRadR, elRadR, 1, goGPS.getFreq());
        double[] rsDD = {pos[i].getX(), pos[i].getY(), pos[i].getZ()};
        double[] rrR = {rover.getX(), rover.getY(), rover.getZ()};
        double satAntCorrR = satelliteAntennaCorrection(rsDD, rrR,
            goGPS.getSatelliteAntennaPcv(id), goGPS.getFreq());
        rover.satAntennaCorr[i] = rcvAntCorrR + satAntCorrR;

        // Computation of master-satellite antenna correction (RTKLIB-aligned)
        double azRadM = Math.toRadians(master.topo[i].getAzimuth());
        double elRadM = Math.toRadians(master.topo[i].getElevation());
        double rcvAntCorrM = antennaCorrection(goGPS.getReceiverAntennaPcv(),
            goGPS.getAntennaDelta(), azRadM, elRadM, 1, goGPS.getFreq());
        double[] rrM = {masterPos.getX(), masterPos.getY(), masterPos.getZ()};
        double satAntCorrM = satelliteAntennaCorrection(rsDD, rrM,
            goGPS.getSatelliteAntennaPcv(id), goGPS.getFreq());
        master.satAntennaCorr[i] = rcvAntCorrM + satAntCorrM;

        // Phase wind-up correction for rover and master (RTKLIB-aligned)
        long gpsWeekDD = roverObs.getRefTime().getGpsWeek();
        double gpsSecDD = roverObs.getRefTime().getGpsTime();
        double gpsTimeDD = gpsWeekDD * 604800.0 + gpsSecDD;
        rover.satWindUp[i] = computeWindUpCorrection(gpsTimeDD, rsDD, rrR,
            prevPhwRover, prevTimeRover, id);
        master.satWindUp[i] = computeWindUpCorrection(gpsTimeDD, rsDD, rrM,
            prevPhwMaster, prevTimeMaster, id);
        prevTimeRover = (long) gpsTimeDD;
        prevTimeMaster = (long) gpsTimeDD;

        // Check if satellite is available for double differences, after cutoff
        // Both rover and master must have valid elevation above cutoff
        double roverElev = rover.topo[i].getElevation();
        double masterElev = master.topo[i].getElevation();
        if (masterObs.containsSatIDType(roverObs.getSatID(i), roverObs.getGnssType(i))
            && !Double.isNaN(roverElev) && roverElev > cutoff
            && !Double.isNaN(masterElev) && masterElev > cutoff) {

          // Find code pivot satellite (with highest elevation)
          if (roverElev > maxElevCode) {
            pivotCode = i;
            maxElevCode = roverElev;
          }

          avail.put(id,pos[i]);
          typeAvail.add(satType);
          gnssAvail.add(String.valueOf(satType) + String.valueOf(id));  

          // Check if also phase is available for both rover and master
          if (!Double.isNaN(roverObs.getSatByIDType(id, satType).getPhaseCycles(goGPS.getFreq())) &&
              !Double.isNaN(masterObs.getSatByIDType(id, satType).getPhaseCycles(goGPS.getFreq()))) {

            // Find phase pivot satellite (with highest elevation)
            if (roverElev > maxElevPhase) {
              pivotPhase = i;
              maxElevPhase = roverElev;
            }

            availPhase.add(id);
            typeAvailPhase.add(satType);
            gnssAvailPhase.add(String.valueOf(satType) + String.valueOf(id));
          }
        }
      }
    }

    // Select best pivot satellite
    if( pivotPhase != -1 ){
      pivot = pivotPhase;
    }else{
      pivot = pivotCode;
    }
  }
}