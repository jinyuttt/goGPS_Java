package org.gogpsproject.positioning;

import org.ejml.simple.SimpleMatrix;

import java.util.Arrays;

public abstract class ReceiverPosition extends Coordinates {

  // Fields related to receiver-satellite geometry
  
  /** receiver-satellite approximate range */
  double[] satAppRange; 
  
  /** receiver-satellite troposphere correction */
  double[] satTropoCorr; 

  /** receiver-satellite ionosphere correction */
  double[] satIonoCorr;

  /** receiver-satellite antenna phase center correction (receiver + satellite) */
  double[] satAntennaCorr; 

  /** receiver-satellite phase wind-up correction (cycles) */
  double[] satWindUp;

  /** previous epoch GF (Geometry-Free) combination value for cycle slip detection */
  double[] prevGf; 

  /** receiver-satellite vector */
  SimpleMatrix[] diffSat; 

  // Fields for satellite selection
  TopocentricCoordinates[] topo;

  // Fields for storing values from previous epoch
  /** rover L Carrier Phase predicted from previous epoch (based on Doppler) [cycle] */
  double[] dopplerPredPhase; 

  /** @return the rover Doppler predicted phase */
  public double getDopplerPredictedPhase(int satID) {
    return dopplerPredPhase[satID - 1];
  }
  
  /** @param roverDopplerPredictedPhase the Doppler predicted phase to set */
  public void setDopplerPredictedPhase(int satID, double roverDopplerPredictedPhase) {
    dopplerPredPhase[satID - 1] = roverDopplerPredictedPhase;
  }

  /** @return the previous GF combination value for cycle slip detection */
  public double getPrevGf(int satID) {
    if (satID < 1 || satID >prevGf.length) {
      throw new IllegalArgumentException("Invalid satellite ID: " + satID);
    }
    return prevGf[satID - 1];
  }

  /** @param gf the GF combination value to store */
  public void setPrevGf(int satID, double gf) {
    if ( satID >prevGf.length) {
      //存储可以动态扩容
      double[] tmp=new double[satID+1];
      System.arraycopy(prevGf, 0, tmp, 0, prevGf.length);
      prevGf=tmp;
    }
    prevGf[satID - 1] = gf;
  }

}