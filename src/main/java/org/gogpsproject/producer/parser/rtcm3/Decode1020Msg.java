/*
 * Copyright (c) 2010 Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
 *
 * This file is part of goGPS Project (goGPS).
 */

package org.gogpsproject.producer.parser.rtcm3;

import org.gogpsproject.ephemeris.EphGlo;

/**
 * RTCM3 Message Type 1020 - GLONASS Ephemeris Data
 * 
 * @author goGPS Project
 */
public class Decode1020Msg implements Decode {

	public Decode1020Msg() {
	}

	@Override
	public Object decode(boolean[] bits, int week) {
		
		int i = 0;
		
		EphGlo eph = new EphGlo();
		
		eph.setSatType('R');
		
		// Message Number: 12 bits
		i += 12;
		
		// Satellite ID: 6 bits
		int satID = (int) decodeUnsigned(bits, i, 6);
		eph.setSatID(satID);
		i += 6;
		
		// Frequency Number: 5 bits
		int freqNum = (int) decodeSigned(bits, i, 5);
		eph.setFreqNum(freqNum);
		i += 5;
		
		// SV Health: 1 bit
		int svHealth = (int) decodeUnsigned(bits, i, 1);
		eph.setSvHealth(svHealth);
		i += 1;
		
		// String Number (Nt): 16 bits
		int strNum = (int) decodeUnsigned(bits, i, 16);
		eph.setStrNum(strNum);
		i += 16;
		
		// Toe (Time of Ephemeris): 27 bits (GLONASS time)
		double toe = decodeUnsigned(bits, i, 27);
		eph.setToe(toe);
		i += 27;
		
		// X Position: 32 bits, scale 2^-11 m
		double posX = decodeSigned(bits, i, 32) * Math.pow(2, -11);
		eph.setPosX(posX);
		i += 32;
		
		// Y Position: 32 bits, scale 2^-11 m
		double posY = decodeSigned(bits, i, 32) * Math.pow(2, -11);
		eph.setPosY(posY);
		i += 32;
		
		// Z Position: 32 bits, scale 2^-11 m
		double posZ = decodeSigned(bits, i, 32) * Math.pow(2, -11);
		eph.setPosZ(posZ);
		i += 32;
		
		// X Velocity: 24 bits, scale 2^-20 m/s
		double velX = decodeSigned(bits, i, 24) * Math.pow(2, -20);
		eph.setVelX(velX);
		i += 24;
		
		// Y Velocity: 24 bits, scale 2^-20 m/s
		double velY = decodeSigned(bits, i, 24) * Math.pow(2, -20);
		eph.setVelY(velY);
		i += 24;
		
		// Z Velocity: 24 bits, scale 2^-20 m/s
		double velZ = decodeSigned(bits, i, 24) * Math.pow(2, -20);
		eph.setVelZ(velZ);
		i += 24;
		
		// X Acceleration: 16 bits, scale 2^-30 m/s^2
		double accX = decodeSigned(bits, i, 16) * Math.pow(2, -30);
		eph.setAccX(accX);
		i += 16;
		
		// Y Acceleration: 16 bits, scale 2^-30 m/s^2
		double accY = decodeSigned(bits, i, 16) * Math.pow(2, -30);
		eph.setAccY(accY);
		i += 16;
		
		// Z Acceleration: 16 bits, scale 2^-30 m/s^2
		double accZ = decodeSigned(bits, i, 16) * Math.pow(2, -30);
		eph.setAccZ(accZ);
		i += 16;
		
		// SV Clock Bias: 22 bits, scale 2^-30 s
		double tauN = decodeSigned(bits, i, 22) * Math.pow(2, -30);
		eph.setTauN(tauN);
		i += 22;
		
		// SV Clock Drift: 14 bits, scale 2^-45 s/s
		double gammaN = decodeSigned(bits, i, 14) * Math.pow(2, -45);
		eph.setGammaN(gammaN);
		i += 14;
		
		// Age of Data: 5 bits
		int ageOfData = (int) decodeUnsigned(bits, i, 5);
		eph.setAgeOfData(ageOfData);
		i += 5;
		
		return eph;
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
}