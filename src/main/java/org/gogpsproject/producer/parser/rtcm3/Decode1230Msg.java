/*
 * Copyright (c) 2010 Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
 *
 * This file is part of goGPS Project (goGPS).
 */

package org.gogpsproject.producer.parser.rtcm3;

/**
 * RTCM3 Message Type 1230 - GLONASS Code Phase Biases
 * 
 * Contains GLONASS inter-signal code biases for precise positioning
 * 
 * @author goGPS Project
 */
public class Decode1230Msg implements Decode {

	public Decode1230Msg() {
	}

	@Override
	public Object decode(boolean[] bits, int week) {
		
		int i = 0;
		
		GloCodePhaseBias bias = new GloCodePhaseBias();
		
		// Message Number: 12 bits
		i += 12;
		
		// Station ID: 12 bits
		int stationId = (int) decodeUnsigned(bits, i, 12);
		bias.setStationId(stationId);
		i += 12;
		
		// GLONASS Code-Phase indicator: 1 bit
		int cpaIndicator = (int) decodeUnsigned(bits, i, 1);
		bias.setCpaIndicator(cpaIndicator);
		i += 1;
		
		// GLONASS FDMA signals mask: 4 bits
		int fdmaSignalsMask = (int) decodeUnsigned(bits, i, 4);
		bias.setFdmaSignalsMask(fdmaSignalsMask);
		i += 4;
		
		// GLONASS L1 C/A Code-Phase bias: 16 bits, scale 0.02m
		if ((fdmaSignalsMask & 0x08) != 0) {
			int biasL1 = (int) decodeSigned(bits, i, 16);
			if (biasL1 != -32768) {
				bias.setBiasL1CA(biasL1 * 0.02);
			}
			i += 16;
		}
		
		// GLONASS L1 P Code-Phase bias: 16 bits, scale 0.02m
		if ((fdmaSignalsMask & 0x04) != 0) {
			int biasL1P = (int) decodeSigned(bits, i, 16);
			if (biasL1P != -32768) {
				bias.setBiasL1P(biasL1P * 0.02);
			}
			i += 16;
		}
		
		// GLONASS L2 C/A Code-Phase bias: 16 bits, scale 0.02m
		if ((fdmaSignalsMask & 0x02) != 0) {
			int biasL2CA = (int) decodeSigned(bits, i, 16);
			if (biasL2CA != -32768) {
				bias.setBiasL2CA(biasL2CA * 0.02);
			}
			i += 16;
		}
		
		// GLONASS L2 P Code-Phase bias: 16 bits, scale 0.02m
		if ((fdmaSignalsMask & 0x01) != 0) {
			int biasL2P = (int) decodeSigned(bits, i, 16);
			if (biasL2P != -32768) {
				bias.setBiasL2P(biasL2P * 0.02);
			}
			i += 16;
		}
		
		return bias;
	}
	
	private long decodeUnsigned(boolean[] bits, int start, int length) {
		long result = 0;
		for (int j = 0; j < length; j++) {
			if (bits[start + j]) {
				result |= (1L << (length - 1 - j));
			}
		}
		return result;
	}
	
	private long decodeSigned(boolean[] bits, int start, int length) {
		long unsigned = decodeUnsigned(bits, start, length);
		long msb = 1L << (length - 1);
		if ((unsigned & msb) != 0) {
			return unsigned - (1L << length);
		}
		return unsigned;
	}
	
	/**
	 * GLONASS Code Phase Bias data class
	 */
	public static class GloCodePhaseBias {
		private int stationId;
		private int cpaIndicator;
		private int fdmaSignalsMask;
		private double biasL1CA;
		private double biasL1P;
		private double biasL2CA;
		private double biasL2P;
		
		public int getStationId() { return stationId; }
		public void setStationId(int stationId) { this.stationId = stationId; }
		public int getCpaIndicator() { return cpaIndicator; }
		public void setCpaIndicator(int cpaIndicator) { this.cpaIndicator = cpaIndicator; }
		public int getFdmaSignalsMask() { return fdmaSignalsMask; }
		public void setFdmaSignalsMask(int fdmaSignalsMask) { this.fdmaSignalsMask = fdmaSignalsMask; }
		public double getBiasL1CA() { return biasL1CA; }
		public void setBiasL1CA(double bias) { this.biasL1CA = bias; }
		public double getBiasL1P() { return biasL1P; }
		public void setBiasL1P(double bias) { this.biasL1P = bias; }
		public double getBiasL2CA() { return biasL2CA; }
		public void setBiasL2CA(double bias) { this.biasL2CA = bias; }
		public double getBiasL2P() { return biasL2P; }
		public void setBiasL2P(double bias) { this.biasL2P = bias; }
	}
}