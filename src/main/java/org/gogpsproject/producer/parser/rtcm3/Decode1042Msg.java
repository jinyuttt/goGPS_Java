/*
 * Copyright (c) 2010 Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
 *
 * This file is part of goGPS Project (goGPS).
 */

package org.gogpsproject.producer.parser.rtcm3;

import org.gogpsproject.ephemeris.EphBds;
import org.gogpsproject.positioning.Time;
import org.gogpsproject.util.Bits;

/**
 * RTCM3 Message Type 1042/63 - BeiDou Ephemeris Data
 * 
 * Field definitions based on RTKLIB rtcm3.c decode_type1042() reference implementation.
 * Total message content: 511 bits data + 1 padding bit = 512 bits (64 bytes).
 * CRC (24 bits / 3 bytes) is read separately by RTCM3Client.readMessage().
 * 
 * @author goGPS Project
 */
public class Decode1042Msg implements Decode {

	/** Expected total content length in bits (12 msg type + 499 data) */
	private static final int EXPECTED_BITS = 511;

	/* Scale factors matching RTKLIB P2_xx constants */
	private static final double P2_6  = Math.pow(2, -6);   /* 0.015625 */
	private static final double P2_9  = Math.pow(2, -9);   /* 0.001953125 */
	private static final double P2_19 = Math.pow(2, -19);
	private static final double P2_31 = Math.pow(2, -31);
	private static final double P2_32 = Math.pow(2, -32);
	private static final double P2_33 = Math.pow(2, -33);
	private static final double P2_43 = Math.pow(2, -43);
	private static final double P2_50 = Math.pow(2, -50);
	private static final double P2_66 = Math.pow(2, -66);

	public Decode1042Msg() {
	}

	@Override
	public Object decode(boolean[] bits, int week) {
		
		if (bits == null || bits.length < 12) {
			System.err.printf("RTCM3 1042 数据不足: 仅 %d bits, 跳过此消息%n",
					bits == null ? 0 : bits.length);
			return null;
		}
		
		int i = 0;
		final int maxBits = bits.length;
		
		EphBds eph = new EphBds();
		eph.setSatType('C');
		
		// Message Number: 12 bits (already read by caller)
		// Note: bits array contains message content only (preamble+length stripped by RTCM3Client)
		i += 12;
		
		// ========== 按 RTKLIB decode_type1042 顺序解析 ==========
		
		// SatID: 6 bits
		if (i + 6 > maxBits) return partialResult(eph, "SatID", i, maxBits);
		int satID = (int) decodeUnsigned(bits, i, 6);
		eph.setSatID(satID);
		i += 6;
		
		// BDS Week: 13 bits
		if (i + 13 > maxBits) return partialResult(eph, "Week", i, maxBits);
		int bdsWeek = (int) decodeUnsigned(bits, i, 13);
		// 转换 BDS 周 → GPS 周 (BDS 周 0 = GPS 周 1356)
		eph.setWeek(bdsWeek + 1356);
		i += 13;
		
		// SV Accuracy (URA): 4 bits
		if (i + 4 > maxBits) return partialResult(eph, "URA", i, maxBits);
		int svAccur = (int) decodeUnsigned(bits, i, 4);
		eph.setSvAccur(svAccur);
		i += 4;
		
		// IDOT: 14 bits, scale 2^-43 * PI (semi-circles/s -> rad/s)
		if (i + 14 > maxBits) return partialResult(eph, "IDOT", i, maxBits);
		double iDot = decodeSigned(bits, i, 14) * P2_43 * Math.PI;
		eph.setiDot(iDot);
		i += 14;
		
		// AODE (stored as IODE): 5 bits
		if (i + 5 > maxBits) return partialResult(eph, "IODE/AODE", i, maxBits);
		int iode = (int) decodeUnsigned(bits, i, 5);
		eph.setIode(iode);
		eph.setAode(iode);
		i += 5;
		
		// ToC: 17 bits, scale 8.0 seconds
		if (i + 17 > maxBits) return partialResult(eph, "ToC", i, maxBits);
		long rawToc = decodeUnsigned(bits, i, 17);
		double toc = rawToc * 8.0;
		// BDT ToC转换为GPST ToC（BDT = GPST - 14秒）
		toc = toc + 14.0;
		eph.setToc(toc);

		i += 17;
		
		// af2: 11 bits, scale 2^-66
		if (i + 11 > maxBits) return partialResult(eph, "af2", i, maxBits);
		double af2 = decodeSigned(bits, i, 11) * P2_66;
		eph.setAf2(af2);
		i += 11;
		
		// af1: 22 bits, scale 2^-50
		if (i + 22 > maxBits) return partialResult(eph, "af1", i, maxBits);
		double af1 = decodeSigned(bits, i, 22) * P2_50;
		eph.setAf1(af1);
		i += 22;
		
		// af0: 24 bits, scale 2^-33 (BDS clock bias)
		if (i + 24 > maxBits) return partialResult(eph, "af0", i, maxBits);
		double af0 = decodeSigned(bits, i, 24) * P2_33;
		eph.setAf0(af0);
		i += 24;
		
		// AODC (stored as IODC): 5 bits
		if (i + 5 > maxBits) return partialResult(eph, "IODC/AODC", i, maxBits);
		int iodc = (int) decodeUnsigned(bits, i, 5);
		eph.setIodc(iodc);
		i += 5;
		
		// Crs: 18 bits, scale 2^-6 (0.015625 m)
		if (i + 18 > maxBits) return partialResult(eph, "Crs", i, maxBits);
		double crs = decodeSigned(bits, i, 18) * P2_6;
		eph.setCrs(crs);
		i += 18;
		
		// Delta n: 16 bits, scale 2^-43 * PI (semi-circles/s -> rad/s)
		if (i + 16 > maxBits) return partialResult(eph, "DeltaN", i, maxBits);
		double deltaN = decodeSigned(bits, i, 16) * P2_43 * Math.PI;
		eph.setDeltaN(deltaN);
		i += 16;
		
		// M0: 32 bits, scale 2^-31 * PI (semi-circles -> radians)
		if (i + 32 > maxBits) return partialResult(eph, "M0", i, maxBits);
		double M0 = decodeSigned(bits, i, 32) * P2_31 * Math.PI;
		eph.setM0(M0);
		i += 32;
		
		// Cuc: 18 bits, scale 2^-31 radians
		if (i + 18 > maxBits) return partialResult(eph, "Cuc", i, maxBits);
		double cuc = decodeSigned(bits, i, 18) * P2_31;
		eph.setCuc(cuc);
		i += 18;
		
		// Eccentricity e: 32 bits, scale 2^-33
		if (i + 32 > maxBits) return partialResult(eph, "e", i, maxBits);
		double ecc = decodeUnsigned(bits, i, 32) * P2_33;
		eph.setE(ecc);
		i += 32;
		
		// Cus: 18 bits, scale 2^-31 radians
		if (i + 18 > maxBits) return partialResult(eph, "Cus", i, maxBits);
		double cus = decodeSigned(bits, i, 18) * P2_31;
		eph.setCus(cus);
		i += 18;
		
		// sqrtA: 32 bits, scale 2^-19 (BDS standard, unit: m^0.5)
		if (i + 32 > maxBits) return partialResult(eph, "sqrtA", i, maxBits);
		long rawSqrtA = decodeUnsigned(bits, i, 32);
		double sqrtA = rawSqrtA * P2_19;
		eph.setRootA(sqrtA);
		i += 32;
		
		// Toe: 17 bits, scale 8.0 seconds
		if (i + 17 > maxBits) return partialResult(eph, "Toe", i, maxBits);
		long rawToe = decodeUnsigned(bits, i, 17);
		double toe = rawToe * 8.0;
		// BDT ToE转换为GPST ToE（BDT = GPST - 14秒）
		toe = toe + 14.0;
		eph.setToe(toe);
		i += 17;
		
		// Cic: 18 bits, scale 2^-31 radians
		if (i + 18 > maxBits) return partialResult(eph, "Cic", i, maxBits);
		double cic = decodeSigned(bits, i, 18) * P2_31;
		eph.setCic(cic);
		i += 18;
		
		// OMEGA0: 32 bits, scale 2^-31 * PI (semi-circles -> radians)
		if (i + 32 > maxBits) return partialResult(eph, "OMEGA0", i, maxBits);
		double omega0 = decodeSigned(bits, i, 32) * P2_31 * Math.PI;
		eph.setOmega0(omega0);
		i += 32;
		
		// Cis: 18 bits, scale 2^-31 radians
		if (i + 18 > maxBits) return partialResult(eph, "Cis", i, maxBits);
		double cis = decodeSigned(bits, i, 18) * P2_31;
		eph.setCis(cis);
		i += 18;
		
		// i0: 32 bits, scale 2^-31 * PI (semi-circles -> radians)
		if (i + 32 > maxBits) return partialResult(eph, "i0", i, maxBits);
		double i0 = decodeSigned(bits, i, 32) * P2_31 * Math.PI;
		eph.setI0(i0);
		i += 32;
		
		// Crc: 18 bits, scale 2^-6 (0.015625 m)
		if (i + 18 > maxBits) return partialResult(eph, "Crc", i, maxBits);
		double crc = decodeSigned(bits, i, 18) * P2_6;
		eph.setCrc(crc);
		i += 18;
		
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
		
		// TGD1 (B1I group delay): 10 bits, scale 2^-33 s
		if (i + 10 > maxBits) return partialResult(eph, "TGD1", i, maxBits);
		double tgd1 = decodeSigned(bits, i, 10) * P2_33;
		eph.setTgd(tgd1);
		i += 10;
		
		// TGD2 (B2I group delay): 10 bits, scale 2^-33 s
		if (i + 10 > maxBits) return partialResult(eph, "TGD2", i, maxBits);
		double tgd2 = decodeSigned(bits, i, 10) * P2_33;
		eph.setTgd2(tgd2);
		i += 10;
		
		// SV Health: 1 bit
		if (i + 1 > maxBits) return partialResult(eph, "SVHealth", i, maxBits);
		int svHealth = (int) decodeUnsigned(bits, i, 1);
		eph.setSvHealth(svHealth);
		i += 1;
		
		// 验证总读取位数 (511 data + 1 padding = 512)
		if (i > EXPECTED_BITS) {
			System.err.printf("RTCM3 1042 内部错误: 预期 %d bits, 实际读取 %d bits%n", EXPECTED_BITS, i);
		}
		
		// 设置星历参考时间 refTime（使用GPST时间基准）
		// ToC已在上面转换为GPST格式
		Time refTime = new Time(bdsWeek + 1356, toc);  // GPS周 + GPST时间
		eph.setRefTime(refTime);
		
		return eph;
	}
	
	/**
	 * 当数据不足以解析某个字段时, 打印警告并返回 null。
	 */
	private EphBds partialResult(EphBds eph, String fieldName, int bitPos, int maxBits) {
		System.err.printf("RTCM3 1042 数据截断: 字段 %s 需要 bit[%d..], 但仅有 %d bits (satID=%d), 跳过此消息%n",
				fieldName, bitPos, maxBits, eph.getSatID());
		return null;
	}
	
	private long decodeUnsigned(boolean[] bits, int start, int length) {
		long result = 0;
		for (int j = 0; j < length; j++) {
			int readIdx = start + j;
			if (readIdx < bits.length && bits[readIdx]) {
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