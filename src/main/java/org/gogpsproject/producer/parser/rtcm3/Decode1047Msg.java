/*
 * Copyright (c) 2010 Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
 *
 * This file is part of goGPS Project (goGPS).
 */

package org.gogpsproject.producer.parser.rtcm3;

import org.gogpsproject.ephemeris.EphQzs;

/**
 * RTCM3 Message Type 1047 - QZSS Ephemeris Data
 * 
 * @author goGPS Project
 */
public class Decode1047Msg implements Decode {

	public Decode1047Msg() {
	}

	@Override
	public Object decode(boolean[] bits, int week) {
		
		int i = 0;
		
		EphQzs eph = new EphQzs();
		
		eph.setSatType('J');
		
		// Message Number: 12 bits
		i += 12;
		
		// Satellite ID: 6 bits
		int satID = (int) decodeUnsigned(bits, i, 6);
		eph.setSatID(satID);
		i += 6;
		
		// IODE: 8 bits
		int iode = (int) decodeUnsigned(bits, i, 8);
		eph.setIode(iode);
		i += 8;
		
		// Toe (Time of Ephemeris): 32 bits (GPS seconds of week)
		double toe = decodeUnsigned(bits, i, 32);
		eph.setToe(toe);
		i += 32;
		
		// AODE (Age of Data Ephemeris): 16 bits (seconds)
		i += 16;
		
		// Crs: 16 bits, scale 0.1m
		double crs = decodeSigned(bits, i, 16) * 0.1;
		eph.setCrs(crs);
		i += 16;
		
		// Delta n: 16 bits, scale 2^-43
		double deltaN = decodeSigned(bits, i, 16) * Math.pow(2, -43);
		eph.setDeltaN(deltaN);
		i += 16;
		
		// M0: 32 bits, scale 2^-31
		double M0 = decodeSigned(bits, i, 32) * Math.pow(2, -31);
		eph.setM0(M0);
		i += 32;
		
		// Cuc: 16 bits, scale 2^-29
		double cuc = decodeSigned(bits, i, 16) * Math.pow(2, -29);
		eph.setCuc(cuc);
		i += 16;
		
		// E (Eccentricity): 32 bits, scale 2^-33
		double e = decodeUnsigned(bits, i, 32) * Math.pow(2, -33);
		eph.setE(e);
		i += 32;
		
		// Cus: 16 bits, scale 2^-29
		double cus = decodeSigned(bits, i, 16) * Math.pow(2, -29);
		eph.setCus(cus);
		i += 16;
		
		// sqrt(A): 32 bits, scale 2^-19 (m^0.5)
		double rootA = decodeUnsigned(bits, i, 32) * Math.pow(2, -19);
		eph.setRootA(rootA);
		i += 32;
		
		// ToC (Time of Clock): 32 bits (GPS seconds of week)
		double toc = decodeUnsigned(bits, i, 32);
		eph.setToc(toc);
		i += 32;
		
		// Cic: 16 bits, scale 2^-29
		double cic = decodeSigned(bits, i, 16) * Math.pow(2, -29);
		eph.setCic(cic);
		i += 16;
		
		// OMEGA0 (Longitude of ascending node): 32 bits, scale 2^-31
		double omega0 = decodeSigned(bits, i, 32) * Math.pow(2, -31);
		eph.setOmega0(omega0);
		i += 32;
		
		// Cis: 16 bits, scale 2^-29
		double cis = decodeSigned(bits, i, 16) * Math.pow(2, -29);
		eph.setCis(cis);
		i += 16;
		
		// I0 (Inclination): 32 bits, scale 2^-31
		double i0 = decodeSigned(bits, i, 32) * Math.pow(2, -31);
		eph.setI0(i0);
		i += 32;
		
		// Crc: 16 bits, scale 0.1m
		double crc = decodeSigned(bits, i, 16) * 0.1;
		eph.setCrc(crc);
		i += 16;
		
		// OMEGA (Argument of perigee): 32 bits, scale 2^-31
		double omega = decodeSigned(bits, i, 32) * Math.pow(2, -31);
		eph.setOmega(omega);
		i += 32;
		
		// OMEGA DOT (Rate of right ascension): 24 bits, scale 2^-42
		double omegaDot = decodeSigned(bits, i, 24) * Math.pow(2, -42);
		eph.setOmegaDot(omegaDot);
		i += 24;
		
		// IDOT (Rate of inclination): 14 bits, scale 2^-45
		double iDot = decodeSigned(bits, i, 14) * Math.pow(2, -45);
		eph.setiDot(iDot);
		i += 14;
		
		// GPS Week (WN): 10 bits
		int weekNum = (int) decodeUnsigned(bits, i, 10);
		eph.setWeek(weekNum);
		i += 10;
		
		// SV Accuracy: 4 bits
		int svAccur = (int) decodeUnsigned(bits, i, 4);
		eph.setSvAccur(svAccur);
		i += 4;
		
		// SV Health: 6 bits
		int svHealth = (int) decodeUnsigned(bits, i, 6);
		eph.setSvHealth(svHealth);
		i += 6;
		
		// TGD: 16 bits, scale 2^-35 seconds
		double tgd = decodeSigned(bits, i, 16) * Math.pow(2, -35);
		eph.setTgd(tgd);
		i += 16;
		
		// IODC (Issue of Data Clock): 8 bits
		int iodc = (int) decodeUnsigned(bits, i, 8);
		eph.setIodc(iodc);
		i += 8;
		
		// AF0 (Clock offset): 22 bits, scale 2^-38 seconds
		double af0 = decodeSigned(bits, i, 22) * Math.pow(2, -38);
		eph.setAf0(af0);
		i += 22;
		
		// AF1 (Clock drift): 16 bits, scale 2^-51
		double af1 = decodeSigned(bits, i, 16) * Math.pow(2, -51);
		eph.setAf1(af1);
		i += 16;
		
		// AF2 (Clock drift rate): 8 bits, scale 2^-66
		double af2 = decodeSigned(bits, i, 8) * Math.pow(2, -66);
		eph.setAf2(af2);
		i += 8;
		
		// Fit interval: 4 bits
		long fitInt = (long) decodeUnsigned(bits, i, 4);
		eph.setFitInt(fitInt);
		i += 4;
		
		// Reserved: 2 bits
		i += 2;
		
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