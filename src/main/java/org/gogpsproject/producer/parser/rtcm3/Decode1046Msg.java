/*
 * Copyright (c) 2010 Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
 *
 * This file is part of goGPS Project (goGPS).
 */

package org.gogpsproject.producer.parser.rtcm3;

import org.gogpsproject.ephemeris.EphGal;

/**
 * RTCM3 Message Type 1046 - Galileo Ephemeris Data (I/NAV)
 * 
 * Field definitions based on RTKLIB rtcm3.c decode_type1046() reference implementation.
 * Total message content: 492 bits data after message type.
 * 
 * @author goGPS Project
 */
public class Decode1046Msg implements Decode {

	/* Scale factors matching RTKLIB P2_xx constants */
	private static final double P2_5  = Math.pow(2, -5);
	private static final double P2_19 = Math.pow(2, -19);
	private static final double P2_29 = Math.pow(2, -29);
	private static final double P2_31 = Math.pow(2, -31);
	private static final double P2_32 = Math.pow(2, -32);
	private static final double P2_33 = Math.pow(2, -33);
	private static final double P2_34 = Math.pow(2, -34);
	private static final double P2_43 = Math.pow(2, -43);
	private static final double P2_46 = Math.pow(2, -46);
	private static final double P2_59 = Math.pow(2, -59);

	public Decode1046Msg() {
	}

	@Override
	public Object decode(boolean[] bits, int week) {
		
		if (bits == null || bits.length < 12) {
			System.err.printf("RTCM3 1046 数据不足: 仅 %d bits, 跳过此消息%n",
					bits == null ? 0 : bits.length);
			return null;
		}
		
		int i = 0;
		final int maxBits = bits.length;
		
		EphGal eph = new EphGal();
		eph.setSatType('E');
		
		// Message Number: 12 bits (already read by caller)
		i += 12;
		
		// ========== 按 RTKLIB decode_type1046 顺序解析 ==========
		
		// Satellite ID: 6 bits
		if (i + 6 > maxBits) return partialResult(eph, "SatID", i, maxBits);
		int satID = (int) decodeUnsigned(bits, i, 6);
		eph.setSatID(satID);
		i += 6;
		
		// GST Week: 12 bits
		if (i + 12 > maxBits) return partialResult(eph, "Week", i, maxBits);
		int weekNum = (int) decodeUnsigned(bits, i, 12);
		eph.setWeek(weekNum);
		i += 12;
		
		// IODE: 10 bits
		if (i + 10 > maxBits) return partialResult(eph, "IODE", i, maxBits);
		int iode = (int) decodeUnsigned(bits, i, 10);
		eph.setIode(iode);
		i += 10;
		
		// SV Accuracy (SISA): 8 bits
		if (i + 8 > maxBits) return partialResult(eph, "SISA", i, maxBits);
		int svAccur = (int) decodeUnsigned(bits, i, 8);
		eph.setSvAccur(svAccur);
		i += 8;
		
		// IDOT: 14 bits, scale 2^-43 * PI
		if (i + 14 > maxBits) return partialResult(eph, "IDOT", i, maxBits);
		double iDot = decodeSigned(bits, i, 14) * P2_43 * Math.PI;
		eph.setiDot(iDot);
		i += 14;
		
		// ToC: 14 bits, scale 60.0 seconds
		if (i + 14 > maxBits) return partialResult(eph, "ToC", i, maxBits);
		double toc = decodeUnsigned(bits, i, 14) * 60.0;
		eph.setToc(toc);
		i += 14;
		
		// af2: 6 bits, scale 2^-59
		if (i + 6 > maxBits) return partialResult(eph, "af2", i, maxBits);
		double af2 = decodeSigned(bits, i, 6) * P2_59;
		eph.setAf2(af2);
		i += 6;
		
		// af1: 21 bits, scale 2^-46
		if (i + 21 > maxBits) return partialResult(eph, "af1", i, maxBits);
		double af1 = decodeSigned(bits, i, 21) * P2_46;
		eph.setAf1(af1);
		i += 21;
		
		// af0: 31 bits, scale 2^-34
		if (i + 31 > maxBits) return partialResult(eph, "af0", i, maxBits);
		double af0 = decodeSigned(bits, i, 31) * P2_34;
		eph.setAf0(af0);
		i += 31;
		
		// Crs: 16 bits, scale 2^-5
		if (i + 16 > maxBits) return partialResult(eph, "Crs", i, maxBits);
		double crs = decodeSigned(bits, i, 16) * P2_5;
		eph.setCrs(crs);
		i += 16;
		
		// Delta n: 16 bits, scale 2^-43 * PI
		if (i + 16 > maxBits) return partialResult(eph, "DeltaN", i, maxBits);
		double deltaN = decodeSigned(bits, i, 16) * P2_43 * Math.PI;
		eph.setDeltaN(deltaN);
		i += 16;
		
		// M0: 32 bits, scale 2^-31 * PI
		if (i + 32 > maxBits) return partialResult(eph, "M0", i, maxBits);
		double M0 = decodeSigned(bits, i, 32) * P2_31 * Math.PI;
		eph.setM0(M0);
		i += 32;
		
		// Cuc: 16 bits, scale 2^-29
		if (i + 16 > maxBits) return partialResult(eph, "Cuc", i, maxBits);
		double cuc = decodeSigned(bits, i, 16) * P2_29;
		eph.setCuc(cuc);
		i += 16;
		
		// Eccentricity e: 32 bits, scale 2^-33
		if (i + 32 > maxBits) return partialResult(eph, "e", i, maxBits);
		double ecc = decodeUnsigned(bits, i, 32) * P2_33;
		eph.setE(ecc);
		i += 32;
		
		// Cus: 16 bits, scale 2^-29
		if (i + 16 > maxBits) return partialResult(eph, "Cus", i, maxBits);
		double cus = decodeSigned(bits, i, 16) * P2_29;
		eph.setCus(cus);
		i += 16;
		
		// sqrt(A): 32 bits, scale 2^-19
		if (i + 32 > maxBits) return partialResult(eph, "sqrtA", i, maxBits);
		double rootA = decodeUnsigned(bits, i, 32) * P2_19;
		eph.setRootA(rootA);
		i += 32;
		
		// Toe: 14 bits, scale 60.0 seconds
		if (i + 14 > maxBits) return partialResult(eph, "Toe", i, maxBits);
		double toe = decodeUnsigned(bits, i, 14) * 60.0;
		eph.setToe(toe);
		i += 14;
		
		// Cic: 16 bits, scale 2^-29
		if (i + 16 > maxBits) return partialResult(eph, "Cic", i, maxBits);
		double cic = decodeSigned(bits, i, 16) * P2_29;
		eph.setCic(cic);
		i += 16;
		
		// OMEGA0: 32 bits, scale 2^-31 * PI
		if (i + 32 > maxBits) return partialResult(eph, "OMEGA0", i, maxBits);
		double omega0 = decodeSigned(bits, i, 32) * P2_31 * Math.PI;
		eph.setOmega0(omega0);
		i += 32;
		
		// Cis: 16 bits, scale 2^-29
		if (i + 16 > maxBits) return partialResult(eph, "Cis", i, maxBits);
		double cis = decodeSigned(bits, i, 16) * P2_29;
		eph.setCis(cis);
		i += 16;
		
		// i0: 32 bits, scale 2^-31 * PI
		if (i + 32 > maxBits) return partialResult(eph, "i0", i, maxBits);
		double i0 = decodeSigned(bits, i, 32) * P2_31 * Math.PI;
		eph.setI0(i0);
		i += 32;
		
		// Crc: 16 bits, scale 2^-5
		if (i + 16 > maxBits) return partialResult(eph, "Crc", i, maxBits);
		double crc = decodeSigned(bits, i, 16) * P2_5;
		eph.setCrc(crc);
		i += 16;
		
		// omega (argument of perigee): 32 bits, scale 2^-31 * PI
		if (i + 32 > maxBits) return partialResult(eph, "omega", i, maxBits);
		double omega = decodeSigned(bits, i, 32) * P2_31 * Math.PI;
		eph.setOmega(omega);
		i += 32;
		
		// OMEGA_DOT (rate of right ascension): 24 bits, scale 2^-43 * PI
		if (i + 24 > maxBits) return partialResult(eph, "OMEGA_DOT", i, maxBits);
		double omegaDot = decodeSigned(bits, i, 24) * P2_43 * Math.PI;
		eph.setOmegaDot(omegaDot);
		i += 24;
		
		// TGD0 (E5a/E1 group delay): 10 bits, scale 2^-32
		if (i + 10 > maxBits) return partialResult(eph, "TGD_E5aE1", i, maxBits);
		double tgdE5aE1 = decodeSigned(bits, i, 10) * P2_32;
		eph.setBgdE5aE1(tgdE5aE1);
		i += 10;
		
		// TGD1 (E5b/E1 group delay): 10 bits, scale 2^-32
		if (i + 10 > maxBits) return partialResult(eph, "TGD_E5bE1", i, maxBits);
		double tgdE5bE1 = decodeSigned(bits, i, 10) * P2_32;
		eph.setBgdE5bE1(tgdE5bE1);
		i += 10;
		
		// E5b OSHS (Signal Health Status): 2 bits
		if (i + 2 > maxBits) return partialResult(eph, "E5b_HS", i, maxBits);
		int e5b_hs = (int) decodeUnsigned(bits, i, 2);
		i += 2;
		
		// E5b OSDVS (Data Validity Status): 1 bit
		if (i + 1 > maxBits) return partialResult(eph, "E5b_DVS", i, maxBits);
		int e5b_dvs = (int) decodeUnsigned(bits, i, 1);
		i += 1;
		
		// E1 OSHS (Signal Health Status): 2 bits
		if (i + 2 > maxBits) return partialResult(eph, "E1_HS", i, maxBits);
		int e1_hs = (int) decodeUnsigned(bits, i, 2);
		i += 2;
		
		// E1 OSDVS (Data Validity Status): 1 bit
		if (i + 1 > maxBits) return partialResult(eph, "E1_DVS", i, maxBits);
		int e1_dvs = (int) decodeUnsigned(bits, i, 1);
		i += 1;
		
		// Combine health status: E1 health as primary SV health
		eph.setSvHealth((e1_hs << 1) | e1_dvs);
		
		return eph;
	}
	
	/**
	 * 当数据不足以解析某个字段时, 打印警告并返回 null。
	 */
	private EphGal partialResult(EphGal eph, String fieldName, int bitPos, int maxBits) {
		System.err.printf("RTCM3 1046 数据截断: 字段 %s 需要 bit[%d..], 但仅有 %d bits (satID=%d), 跳过此消息%n",
				fieldName, bitPos, maxBits, eph.getSatID());
		return null;
	}
	
	private long decodeUnsigned(boolean[] bits, int start, int length) {
		long result = 0;
		for (int j = 0; j < length; j++) {
			int idx = start + j;
			if (idx >= bits.length) break;
			if (bits[idx]) {
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
