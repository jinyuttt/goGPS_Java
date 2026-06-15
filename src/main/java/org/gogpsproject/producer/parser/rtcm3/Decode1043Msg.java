/*
 * Copyright (c) 2010 Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
 *
 * This file is part of goGPS Project (goGPS).
 */

package org.gogpsproject.producer.parser.rtcm3;

import org.gogpsproject.ephemeris.EphBds;

/**
 * RTCM3 Message Type 1043 - BeiDou Ephemeris (Alternate)
 * 
 * Contains BeiDou satellite ephemeris data in alternate format.
 * 
 * @author goGPS Project
 */
public class Decode1043Msg implements Decode {

	public Decode1043Msg() {
	}

	@Override
	public Object decode(boolean[] bits, int week) {
		
		EphBds eph = new EphBds();
		int i = 0;
		
		// Message Number: 12 bits
		i += 12;
		
		// Satellite ID: 6 bits (PRN number - 1)
		int prn = (int) decodeUnsigned(bits, i, 6) + 1;
		eph.setSatID(prn);
		i += 6;
		
		// GPS Week Number: 12 bits
		int gpsWeek = (int) decodeUnsigned(bits, i, 12);
		eph.setGpsWeek(gpsWeek);
		i += 12;
		
		// Toe (Time of Ephemeris): 32 bits, scale 1 second
		long toe = decodeUnsigned(bits, i, 32);
		eph.setToe((double) toe);
		i += 32;
		
		// Toc (Time of Clock): 32 bits, scale 1 second
		long toc = decodeUnsigned(bits, i, 32);
		eph.setToc((double) toc);
		i += 32;
		
		// Af0: 22 bits (signed), scale 2^-34 seconds
		long af0Raw = decodeSigned(bits, i, 22);
		eph.setAf0(af0Raw * Math.pow(2, -34));
		i += 22;
		
		// Af1: 16 bits (signed), scale 2^-46 seconds/second
		long af1Raw = decodeSigned(bits, i, 16);
		eph.setAf1(af1Raw * Math.pow(2, -46));
		i += 16;
		
		// Af2: 8 bits (signed), scale 2^-59 seconds/second^2
		long af2Raw = decodeSigned(bits, i, 8);
		eph.setAf2(af2Raw * Math.pow(2, -59));
		i += 8;
		
		// IODE (Issue of Data, Ephemeris): 8 bits
		int iode = (int) decodeUnsigned(bits, i, 8);
		eph.setIoDE(iode);
		i += 8;
		
		// Crs: 16 bits (signed), scale 2^-5 meters
		long crsRaw = decodeSigned(bits, i, 16);
		eph.setCrs(crsRaw * Math.pow(2, -5));
		i += 16;
		
		// Delta n: 16 bits (signed), scale 2^-43 semi-circles/second
		long dnRaw = decodeSigned(bits, i, 16);
		eph.setDn(dnRaw * Math.pow(2, -43));
		i += 16;
		
		// M0: 32 bits (signed), scale 2^-31 semi-circles
		long m0Raw = decodeSigned(bits, i, 32);
		eph.setM0(m0Raw * Math.pow(2, -31));
		i += 32;
		
		// Cuc: 16 bits (signed), scale 2^-29 radians
		long cucRaw = decodeSigned(bits, i, 16);
		eph.setCuc(cucRaw * Math.pow(2, -29));
		i += 16;
		
		// Eccentricity: 32 bits (signed), scale 2^-33
		long eRaw = decodeSigned(bits, i, 32);
		eph.setE(eRaw * Math.pow(2, -33));
		i += 32;
		
		// Cus: 16 bits (signed), scale 2^-29 radians
		long cusRaw = decodeSigned(bits, i, 16);
		eph.setCus(cusRaw * Math.pow(2, -29));
		i += 16;
		
		// Sqrt(A): 32 bits (signed), scale 2^-19 meters^0.5
		long sqrtA = decodeUnsigned(bits, i, 32);
		eph.setSqrtA(sqrtA * Math.pow(2, -19));
		i += 32;
		
		// Toe (Time of Ephemeris, alternate): 32 bits, scale 1 second
		// (This is redundant but included for completeness)
		i += 32;
		
		// Cic: 16 bits (signed), scale 2^-29 radians
		long cicRaw = decodeSigned(bits, i, 16);
		eph.setCic(cicRaw * Math.pow(2, -29));
		i += 16;
		
		// OMEGA0 (Longitude of ascending node): 32 bits (signed), scale 2^-31 semi-circles
		long omega0Raw = decodeSigned(bits, i, 32);
		eph.setOmega0(omega0Raw * Math.pow(2, -31));
		i += 32;
		
		// Cis: 16 bits (signed), scale 2^-29 radians
		long cisRaw = decodeSigned(bits, i, 16);
		eph.setCis(cisRaw * Math.pow(2, -29));
		i += 16;
		
		// I0 (Inclination angle): 32 bits (signed), scale 2^-31 semi-circles
		long i0Raw = decodeSigned(bits, i, 32);
		eph.setI0(i0Raw * Math.pow(2, -31));
		i += 32;
		
		// Crc: 16 bits (signed), scale 2^-5 meters
		long crcRaw = decodeSigned(bits, i, 16);
		eph.setCrc(crcRaw * Math.pow(2, -5));
		i += 16;
		
		// OMEGA (Argument of perigee): 32 bits (signed), scale 2^-31 semi-circles
		long omegaRaw = decodeSigned(bits, i, 32);
		eph.setOmega(omegaRaw * Math.pow(2, -31));
		i += 32;
		
		// OMEGADOT (Rate of right ascension): 24 bits (signed), scale 2^-42 semi-circles/second
		long omegaDotRaw = decodeSigned(bits, i, 24);
		eph.setOmegaDot(omegaDotRaw * Math.pow(2, -42));
		i += 24;
		
		// IDOT (Rate of inclination): 14 bits (signed), scale 2^-42 semi-circles/second
		long iDotRaw = decodeSigned(bits, i, 14);
		eph.setiDot(iDotRaw * Math.pow(2, -42));
		i += 14;
		
		// Codes on L2 channel: 2 bits
		int codesL2 = (int) decodeUnsigned(bits, i, 2);
		i += 2;
		
		// GPS Week Number (for Toe): 10 bits
		i += 10;
		
		// L2 P data flag: 1 bit
		i += 1;
		
		// SV accuracy: 4 bits
		int svAccuracy = (int) decodeUnsigned(bits, i, 4);
		eph.setSvAccuracy(svAccuracy);
		i += 4;
		
		// SV health: 6 bits
		int svHealth = (int) decodeUnsigned(bits, i, 6);
		eph.setSvHealth(svHealth);
		i += 6;
		
		// TGD1: 10 bits (signed), scale 2^-35 seconds
		long tgd1Raw = decodeSigned(bits, i, 10);
		eph.setTgd1(tgd1Raw * Math.pow(2, -35));
		i += 10;
		
		// TGD2: 10 bits (signed), scale 2^-35 seconds
		long tgd2Raw = decodeSigned(bits, i, 10);
		eph.setTgd2(tgd2Raw * Math.pow(2, -35));
		i += 10;
		
		// AODC (Age of Data, Clock): 8 bits
		int aodc = (int) decodeUnsigned(bits, i, 8);
		eph.setAodc(aodc);
		i += 8;
		
		// AODE (Age of Data, Ephemeris): 8 bits
		int aode = (int) decodeUnsigned(bits, i, 8);
		eph.setAode(aode);
		i += 8;
		
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