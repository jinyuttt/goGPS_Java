/*
 * Copyright (c) 2010 Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
 *
 * This file is part of goGPS Project (goGPS).
 */

package org.gogpsproject.producer.parser.rtcm3;

/**
 * RTCM3 Message Type 1029 - Text Message
 * 
 * @author goGPS Project
 */
public class Decode1029Msg implements Decode {

	public Decode1029Msg() {
	}

	@Override
	public Object decode(boolean[] bits, int week) {
		
		int i = 0;
		
		// Message Number: 12 bits
		i += 12;
		
		// Station ID: 12 bits
		int stationId = (int) decodeUnsigned(bits, i, 12);
		i += 12;
		
		// Modified Julian Day (MJD): 16 bits
		int mjd = (int) decodeUnsigned(bits, i, 16);
		i += 16;
		
		// Seconds of Day: 17 bits, scale 0.001s
		double sow = decodeUnsigned(bits, i, 17) * 0.001;
		i += 17;
		
		// Number of Characters: 7 bits
		int nchar = (int) decodeUnsigned(bits, i, 7);
		i += 7;
		
		// Number of Stations: 8 bits
		int nstations = (int) decodeUnsigned(bits, i, 8);
		i += 8;
		
		// Text Message: nchar * 8 bits
		StringBuilder text = new StringBuilder();
		for (int c = 0; c < nchar; c++) {
			int charVal = (int) decodeUnsigned(bits, i, 8);
			if (charVal >= 32 && charVal < 127) {
				text.append((char) charVal);
			} else {
				text.append(' ');
			}
			i += 8;
		}
		
		TextMessage textMsg = new TextMessage();
		textMsg.setStationId(stationId);
		textMsg.setMjd(mjd);
		textMsg.setTimeOfDay(sow);
		textMsg.setMessage(text.toString());
		
		return textMsg;
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
	 * Text Message data class
	 */
	public static class TextMessage {
		private int stationId;
		private int mjd;
		private double timeOfDay;
		private String message;
		
		public int getStationId() { return stationId; }
		public void setStationId(int stationId) { this.stationId = stationId; }
		public int getMjd() { return mjd; }
		public void setMjd(int mjd) { this.mjd = mjd; }
		public double getTimeOfDay() { return timeOfDay; }
		public void setTimeOfDay(double timeOfDay) { this.timeOfDay = timeOfDay; }
		public String getMessage() { return message; }
		public void setMessage(String message) { this.message = message; }
	}
}