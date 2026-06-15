/*
 * Copyright (c) 2010 Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
 *
 * This file is part of goGPS Project (goGPS).
 */

package org.gogpsproject.producer.parser.rtcm3;

/**
 * RTCM3 Message Type 1013 - Reference Station Parameters
 * 
 * Contains reference station information including station ID, receiver type,
 * antenna type, and other metadata.
 * 
 * @author goGPS Project
 */
public class Decode1013Msg implements Decode {

	public Decode1013Msg() {
	}

	@Override
	public Object decode(boolean[] bits, int week) {
		
		int i = 0;
		
		RefStationParams params = new RefStationParams();
		
		// Message Number: 12 bits
		i += 12;
		
		// Station ID: 12 bits
		int stationId = (int) decodeUnsigned(bits, i, 12);
		params.setStationId(stationId);
		i += 12;
		
		// Receiver Type: 24 bits (3 ASCII characters)
		StringBuilder recvType = new StringBuilder();
		for (int j = 0; j < 3; j++) {
			int charVal = (int) decodeUnsigned(bits, i, 8);
			if (charVal >= 32 && charVal < 127) {
				recvType.append((char) charVal);
			}
			i += 8;
		}
		params.setReceiverType(recvType.toString().trim());
		
		// Receiver Version: 24 bits (3 ASCII characters)
		StringBuilder recvVer = new StringBuilder();
		for (int j = 0; j < 3; j++) {
			int charVal = (int) decodeUnsigned(bits, i, 8);
			if (charVal >= 32 && charVal < 127) {
				recvVer.append((char) charVal);
			}
			i += 8;
		}
		params.setReceiverVersion(recvVer.toString().trim());
		
		// Antenna Type: 24 bits (3 ASCII characters)
		StringBuilder antType = new StringBuilder();
		for (int j = 0; j < 3; j++) {
			int charVal = (int) decodeUnsigned(bits, i, 8);
			if (charVal >= 32 && charVal < 127) {
				antType.append((char) charVal);
			}
			i += 8;
		}
		params.setAntennaType(antType.toString().trim());
		
		// Antenna Serial Number: 40 bits (5 ASCII characters)
		StringBuilder antSerial = new StringBuilder();
		for (int j = 0; j < 5; j++) {
			int charVal = (int) decodeUnsigned(bits, i, 8);
			if (charVal >= 32 && charVal < 127) {
				antSerial.append((char) charVal);
			}
			i += 8;
		}
		params.setAntennaSerialNumber(antSerial.toString().trim());
		
		// Approximate Latitude: 32 bits (signed), scale 1e-7 degrees
		long latRaw = decodeSigned(bits, i, 32);
		params.setApproximateLatitude(latRaw * 1e-7);
		i += 32;
		
		// Approximate Longitude: 32 bits (signed), scale 1e-7 degrees
		long lonRaw = decodeSigned(bits, i, 32);
		params.setApproximateLongitude(lonRaw * 1e-7);
		i += 32;
		
		// Approximate Height: 24 bits (signed), scale 1 mm
		long heightRaw = decodeSigned(bits, i, 24);
		params.setApproximateHeight(heightRaw * 0.001);
		i += 24;
		
		return params;
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
	 * Reference Station Parameters data class
	 */
	public static class RefStationParams {
		private int stationId;
		private String receiverType;
		private String receiverVersion;
		private String antennaType;
		private String antennaSerialNumber;
		private double approximateLatitude;
		private double approximateLongitude;
		private double approximateHeight;
		
		public int getStationId() { return stationId; }
		public void setStationId(int stationId) { this.stationId = stationId; }
		public String getReceiverType() { return receiverType; }
		public void setReceiverType(String receiverType) { this.receiverType = receiverType; }
		public String getReceiverVersion() { return receiverVersion; }
		public void setReceiverVersion(String receiverVersion) { this.receiverVersion = receiverVersion; }
		public String getAntennaType() { return antennaType; }
		public void setAntennaType(String antennaType) { this.antennaType = antennaType; }
		public String getAntennaSerialNumber() { return antennaSerialNumber; }
		public void setAntennaSerialNumber(String antennaSerialNumber) { this.antennaSerialNumber = antennaSerialNumber; }
		public double getApproximateLatitude() { return approximateLatitude; }
		public void setApproximateLatitude(double approximateLatitude) { this.approximateLatitude = approximateLatitude; }
		public double getApproximateLongitude() { return approximateLongitude; }
		public void setApproximateLongitude(double approximateLongitude) { this.approximateLongitude = approximateLongitude; }
		public double getApproximateHeight() { return approximateHeight; }
		public void setApproximateHeight(double approximateHeight) { this.approximateHeight = approximateHeight; }
		
		@Override
		public String toString() {
			return String.format("RefStationParams{stationId=%d, receiver='%s', antenna='%s', lat=%.6f, lon=%.6f, height=%.3f}",
				stationId, receiverType, antennaType, approximateLatitude, approximateLongitude, approximateHeight);
		}
	}
}