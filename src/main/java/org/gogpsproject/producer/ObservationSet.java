/*
 * Copyright (c) 2010, Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
 *
 * This file is part of goGPS Project (goGPS).
 *
 * goGPS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3
 * of the License, or (at your option) any later version.
 *
 * goGPS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with goGPS.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
package org.gogpsproject.producer;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import org.gogpsproject.Constants;

/**
 * <p>
 * Set of observations for one epoch and one satellite
 * </p>
 *
 * @author Eugenio Realini, Cryms.com
 */
public class ObservationSet implements Streamable {

	private final static int STREAM_V = 1;


	public final static int L1 = 0;
	public final static int L2 = 1;
	public final static int NFREQ = 5; /* max frequency channels (RTKLIB-aligned: BDS needs up to 5) */

	/* RTKLIB obs code per frequency channel.
	 * Stores the RTKLIB CODE_??? value (e.g. CODE_L2I=40 for B1I, CODE_L6I=42 for B3I)
	 * so that getWavelength() can compute the exact carrier frequency for the
	 * signal actually stored in each channel. This is necessary because multiple
	 * signals can share the same freq-index (e.g. B1I and B1C both map to idx=0
	 * but have different frequencies: 1561.098 vs 1575.42 MHz).
	 * 0 means "not set" (fallback to legacy satType+freqIdx logic). */
	private int[] code = {0,0,0,0,0};

	private int satID;	/* Satellite number */
	private char satType;	/* Satellite Type */

	private boolean tgdApplied = false;	/* Flag to prevent repeated TGD correction */

	/* Array of [L1..L5] */
	private double[] codeC = {Double.NaN,Double.NaN,Double.NaN,Double.NaN,Double.NaN};			/* C Coarse/Acquisition (C/A) code [m] */
	private double[] codeP = {Double.NaN,Double.NaN,Double.NaN,Double.NaN,Double.NaN};			/* P Code Pseudorange [m] */
	private double[] phase = {Double.NaN,Double.NaN,Double.NaN,Double.NaN,Double.NaN};			/* L Carrier Phase [cycle] */
	private float[] signalStrength = {Float.NaN,Float.NaN,Float.NaN,Float.NaN,Float.NaN};		/* C/N0 (signal strength) [dBHz] */
	private float[] doppler = {Float.NaN,Float.NaN,Float.NaN,Float.NaN,Float.NaN};			/* Doppler value [Hz] */

	private int[] qualityInd = {-1,-1,-1,-1,-1};	/* Nav Measurements Quality Ind. ublox proprietary? */

	/*
	 * Loss of lock indicator (LLI). Range: 0-7
	 *  0 or blank: OK or not known
	 *  Bit 0 set : Lost lock between previous and current observation: cycle slip possible
	 *  Bit 1 set : Opposite wavelength factor to the one defined for the satellite by a previous WAVELENGTH FACT L1/2 line. Valid for the current epoch only.
	 *  Bit 2 set : Observation under Antispoofing (may suffer from increased noise)
	 * Bits 0 and 1 for phase only.
	 */
	private int[] lossLockInd = {-1,-1,-1,-1,-1};

	/*
	 * Signal strength indicator projected into interval 1-9:
	 *  1: minimum possible signal strength
 	 *  5: threshold for good S/N ratio
 	 *  9: maximum possible signal strength
	 * 0 or blank: not known, don't care
	 */
	private int[] signalStrengthInd = {-1,-1,-1,-1,-1};

	private int freqNum;

	/* Sets whether this obs is in use or not:
		 could be below the elevation threshold for example
     or unhealthy
  */
	private boolean inUse = false;

  /* residual error */
  public double eRes;

  /**
   * topocentric elevation
   */
  public double el;

	public ObservationSet(){
	}

	public ObservationSet(DataInputStream dai, boolean oldVersion) throws IOException{
		read(dai,oldVersion);
	}

	/**
	 * @return the satID
	 */
	public int getSatID() {
		return satID;
	}

	/**
	 * @param satID the satID to set
	 */
	public void setSatID(int satID) {
		this.satID = satID;
	}

	/**
	 * @return the satType
	 */
	public char getSatType() {
		return satType;
	}

	/**
	 * @param satType the satType to set
	 */
	public void setSatType(char satType) {
		this.satType = satType;
	}
	
	/**
	 * @return the phase range (in meters)
	 */
	public double getPhaserange(int i) {
		return phase[i] * getWavelength(i);
	}

	public double getWavelength(int i) {
		/* RTKLIB-aligned: if code[i] is set, compute frequency from the actual
		 * signal code. This correctly handles cases where multiple signals share
		 * the same freq-index (e.g. B1I and B1C both at idx=0 but with different
		 * frequencies: 1561.098 vs 1575.42 MHz). */
		if (i >= 0 && i < NFREQ && code[i] != 0) {
			double freq = codeToFrequency(satType, code[i]);
			if (freq > 0) {
				return Constants.SPEED_OF_LIGHT / freq;
			}
		}
		/* Fallback: legacy logic based on satType + freqIdx (for backward
		 * compatibility with RINEX reader and other code paths that don't
		 * set code[]). */
		double frequency = 0;
		switch (this.satType) {
		case 'G':
			frequency = (i==0)?Constants.FL1:Constants.FL2;
			break;
		case 'R':
			frequency = (i==0)?freqNum*Constants.FR1_delta+Constants.FR1_base:freqNum*Constants.FR2_delta+Constants.FR2_base;
			break;
		case 'E':
			frequency = (i==0)?Constants.FE1:Constants.FE5a;
			break;
		case 'C':
			frequency = (i==0)?Constants.FC2:Constants.FC6;
			break;
		case 'J':
			frequency = (i==0)?Constants.FJ1:Constants.FJ2;
			break;
		}
		return Constants.SPEED_OF_LIGHT/frequency;
	}

	/**
	 * Convert RTKLIB obs code to carrier frequency (Hz).
	 * Mirrors RTKLIB's code2freq() / code2freq_BDS() etc.
	 *
	 * BeiDou signal expansion (BDS Phase 3/4, pre-2035):
	 *   idx 0: B1I (1561.098) / B1C (1575.42)
	 *   idx 1: B2I/B2b (1207.140)
	 *   idx 2: B2a (1176.450)
	 *   idx 3: B3I (1268.520)
	 *   idx 4: B2ab (1191.795)
	 * New BDS-3/BDS-4 signals should be added here as they are defined in
	 * RTCM 10410.x and ICD updates.
	 */
	private double codeToFrequency(char satType, int code) {
		String obs = codeToObs(code);
		if (obs == null || obs.isEmpty()) return 0;
		char c0 = obs.charAt(0);
		switch (satType) {
		case 'G': // GPS
			if (c0 == '1') return Constants.FL1;
			if (c0 == '2') return Constants.FL2;
			if (c0 == '5') return Constants.FL5;
			break;
		case 'R': // GLONASS (nominal, fcn-dependent not handled here)
			if (c0 == '1') return freqNum*Constants.FR1_delta+Constants.FR1_base;
			if (c0 == '2') return freqNum*Constants.FR2_delta+Constants.FR2_base;
			break;
		case 'E': // Galileo
			if (c0 == '1') return Constants.FE1;
			if (c0 == '7') return Constants.FE5b;
			if (c0 == '5') return Constants.FE5a;
			if (c0 == '6') return Constants.FE6;
			if (c0 == '8') return Constants.FE5;
			break;
		case 'C': // BeiDou (ref RTKLIB code2freq_BDS)
			if (c0 == '2') return Constants.FC2;   // B1I  1561.098 MHz
			if (c0 == '1') return Constants.FC1C; // B1C  1575.420 MHz
			if (c0 == '7') return Constants.FC2b; // B2b  1207.140 MHz
			if (c0 == '5') return Constants.FC2a; // B2a  1176.450 MHz
			if (c0 == '6') return Constants.FC6;  // B3I  1268.520 MHz
			if (c0 == '8') return Constants.FE5;  // B2ab 1191.795 MHz
			break;
		case 'J': // QZSS
			if (c0 == '1') return Constants.FJ1;
			if (c0 == '2') return Constants.FJ2;
			if (c0 == '5') return Constants.FJ5;
			if (c0 == '6') return Constants.FJ6;
			break;
		}
		return 0;
	}

	/**
	 * Convert RTKLIB numeric code to 2-char obs string (mirrors RTKLIB code2obs).
	 * Index matches RTKLIB's obscodes[] table.
	 */
	private static final String[] OBSCODES = {
		""  ,"1C","1P","1W","1Y","1M","1N","1S","1L","1E",  /*  0- 9 */
		"1A","1B","1X","1Z","2C","2D","2S","2L","2X","2P",  /* 10-19 */
		"2W","2Y","2M","2N","5I","5Q","5X","7I","7Q","7X",  /* 20-29 */
		"6A","6B","6C","6X","6Z","6S","6L","8I","8Q","8X",  /* 30-39 */
		"2I","2Q","6I","6Q","3I","3Q","3X","1I","1Q","5A",  /* 40-49 */
		"5B","5C","9A","9B","9C","9X","1D","5D","5P","5Z",  /* 50-59 */
		"6E","7D","7P","7Z","8D","8P","4A","4B","4X",""     /* 60-69 */
	};
	private String codeToObs(int code) {
		if (code < 0 || code >= OBSCODES.length) return "";
		return OBSCODES[code];
	}
	
	/**
	 * @return the pseudorange (in meters)
	 */
	public double getPseudorange(int i) {
		return Double.isNaN(codeP[i])?codeC[i]:codeP[i];
	}

	public boolean isPseudorangeP(int i){
		return !Double.isNaN(codeP[i]);
	}

	/**
	 * Get the RTKLIB obs code stored for frequency channel i.
	 * @return code value (0 if not set)
	 */
	public int getCode(int i) {
		return code[i];
	}

	/**
	 * Set the RTKLIB obs code for frequency channel i.
	 * This enables getWavelength() to compute the exact carrier frequency
	 * for the signal actually stored in this channel.
	 */
	public void setCode(int i, int code) {
		this.code[i] = code;
	}

	/**
	 * @return the c
	 */
	public double getCodeC(int i) {
		return codeC[i];
	}

	/**
	 * @param c the c to set
	 */
	public void setCodeC(int i,double c) {
		codeC[i] = c;
	}

	/**
	 * @return the p
	 */
	public double getCodeP(int i) {
		return codeP[i];
	}

	/**
	 * @param p the p to set
	 */
	public void setCodeP(int i, double p) {
		codeP[i] = p;
	}

	public boolean isTgdApplied() {
		return tgdApplied;
	}

	public void setTgdApplied(boolean tgdApplied) {
		this.tgdApplied = tgdApplied;
	}

	/**
	 * @return the l
	 */
	public double getPhaseCycles(int i) {
		return phase[i];
	}

	/**
	 * @param l the l to set
	 */
	public void setPhaseCycles(int i, double l) {
		phase[i] = l;
	}

	/**
	 * @return the s
	 */
	public float getSignalStrength(int i) {
		return signalStrength[i];
	}

	/**
	 * @param s the s to set
	 */
	public void setSignalStrength(int i, float s) {
		signalStrength[i] = s;
	}

	/**
	 * @return the d
	 */
	public float getDoppler(int i) {
		return doppler[i];
	}

	/**
	 * @param d the d to set
	 */
	public void setDoppler(int i, float d) {
		doppler[i] = d;
	}

	/* (non-Javadoc)
	 * @see java.lang.Object#equals(java.lang.Object)
	 */
	@Override
	public boolean equals(Object obj) {
		if(obj instanceof ObservationSet){
			return ((ObservationSet)obj).getSatID() == satID;
		}else{
			return super.equals(obj);
		}
	}

	/**
	 * @return the qualityInd
	 */
	public int getQualityInd(int i) {
		return qualityInd[i];
	}

	/**
	 * @param qualityInd the qualityInd to set
	 */
	public void setQualityInd(int i,int qualityInd) {
		this.qualityInd[i] = qualityInd;
	}

	/**
	 * @return the lossLockInd
	 */
	public int getLossLockInd(int i) {
		return lossLockInd[i];
	}

	/**
	 * @param lossLockInd the lossLockInd to set
	 */
	public void setLossLockInd(int i,int lossLockInd) {
		this.lossLockInd[i] = lossLockInd;
	}

	public boolean isLocked(int i){
		return lossLockInd[i] == 0;
	}
	public boolean isPossibleCycleSlip(int i){
		return lossLockInd[i]>0 && ((lossLockInd[i]&0x1) == 0x1);
	}
	public boolean isHalfWavelength(int i){
		return lossLockInd[i]>0 && ((lossLockInd[i]&0x2) == 0x2);
	}
	public boolean isUnderAntispoof(int i){
		return lossLockInd[i]>0 && ((lossLockInd[i]&0x4) == 0x4);
	}

	public int write(DataOutputStream dos) throws IOException{
		int size = 0;
		dos.writeUTF(MESSAGE_OBSERVATIONS_SET); // 5

		dos.writeInt(STREAM_V); size +=4;
		dos.write(satID);size +=1;		// 1
		dos.write(satType);size +=1;		// 1
		// L1 data
		dos.write((byte)qualityInd[L1]);	size+=1;
		dos.write((byte)lossLockInd[L1]);	size+=1;
		dos.writeDouble(codeC[L1]); size+=8;
		dos.writeDouble(codeP[L1]); size+=8;
		dos.writeDouble(phase[L1]); size+=8;
		dos.writeFloat(signalStrength[L1]); size+=4;
		dos.writeFloat(doppler[L1]); size+=4;
		// write L2 data ?
		boolean hasL2 = false;
		if(!Double.isNaN(codeC[L2])) hasL2 = true;
		if(!Double.isNaN(codeP[L2])) hasL2 = true;
		if(!Double.isNaN(phase[L2])) hasL2 = true;
		if(!Float.isNaN(signalStrength[L2])) hasL2 = true;
		if(!Float.isNaN(doppler[L2])) hasL2 = true;
		dos.writeBoolean(hasL2); size+=1;
		if(hasL2){
			dos.write((byte)qualityInd[L2]);	size+=1;
			dos.write((byte)lossLockInd[L2]);	size+=1;
			dos.writeDouble(codeC[L2]); size+=8;
			dos.writeDouble(codeP[L2]); size+=8;
			dos.writeDouble(phase[L2]); size+=8;
			dos.writeFloat(signalStrength[L2]); size+=4;
			dos.writeFloat(doppler[L2]); size+=4;
		}
		dos.writeBoolean(inUse);
		return size;
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.Streamable#read(java.io.DataInputStream)
	 */
	@Override
	public void read(DataInputStream dai, boolean oldVersion) throws IOException {
		int v = 1;
		if(!oldVersion) v = dai.readInt();

		if(v==1){
			satID = dai.read();
			satType = (char) dai.read();

			// L1 data
			qualityInd[L1] = (int)dai.read();
			if(qualityInd[L1]==255) qualityInd[L1] = -1;
			lossLockInd[L1] = (int)dai.read();
			if(lossLockInd[L1]==255) lossLockInd[L1] = -1;
			codeC[L1] = dai.readDouble();
			codeP[L1] = dai.readDouble();
			phase[L1] = dai.readDouble();
			signalStrength[L1] = dai.readFloat();
			doppler[L1] = dai.readFloat();
			if(dai.readBoolean()){
				// L2 data
				qualityInd[L2] = (int)dai.read();
				if(qualityInd[L2]==255) qualityInd[L2] = -1;
				lossLockInd[L2] = (int)dai.read();
				if(lossLockInd[L2]==255) lossLockInd[L2] = -1;
				codeC[L2] = dai.readDouble();
				codeP[L2] = dai.readDouble();
				phase[L2] = dai.readDouble();
				signalStrength[L2] = dai.readFloat();
				doppler[L2] = dai.readFloat();
			}
			inUse = dai.readBoolean();
		}else{
			throw new IOException("Unknown format version:"+v);
		}
	}

	/**
	 * @param signalStrengthInd the signalStrengthInd to set
	 */
	public void setSignalStrengthInd(int i,int signalStrengthInd) {
		this.signalStrengthInd[i] = signalStrengthInd;
	}

	/**
	 * @return the signalStrengthInd
	 */
	public int getSignalStrengthInd(int i) {
		return signalStrengthInd[i];
	}
	
	/**
	 * @param signalStrengthInd the signalStrengthInd to set
	 */
	public void setFreqNum(int freqNum) {
		this.freqNum = freqNum;
	}

	/**
	 * @return the signalStrengthInd
	 */
	public int getFreqNum(int i) {
		return freqNum;
	}

  public boolean inUse() {
    return isInUse();
  }
  
  public void inUse(boolean inUse) {
    this.setInUse(inUse);
  }

  public boolean isInUse() {
    return inUse;
  }

  public void setInUse(boolean inUse) {
    this.inUse = inUse;
  }

    public void setHalfCycleAmb(int i, boolean b) {

    }
}