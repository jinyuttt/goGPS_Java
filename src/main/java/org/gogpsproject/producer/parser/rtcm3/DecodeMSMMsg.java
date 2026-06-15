/*
 * Copyright (c) 2010 Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
 *
 * This file is part of goGPS Project (goGPS).
 */

package org.gogpsproject.producer.parser.rtcm3;

import java.util.ArrayList;

import org.gogpsproject.positioning.Time;
import org.gogpsproject.producer.ObservationSet;
import org.gogpsproject.producer.Observations;
import org.gogpsproject.util.Bits;

/**
 * RTCM3 Multiple Signal Message (MSM) Decoder
 * 
 * Supports MSM1-MSM7 for:
 * - GPS (1071-1077)
 * - GLONASS (1081-1087)
 * - Galileo (1091-1097)
 * - SBAS (1101-1107)
 * - QZSS (1111-1117)
 * - BeiDou (1121-1127)
 * - Multi-GNSS (1131-1137)
 * 
 * @author goGPS Project
 */
public class DecodeMSMMsg implements Decode {

	private static final double RANGE_MS = 299792.458;  // 光速 * 0.001 米/毫秒
	private static final double P2_10 = 1.0 / 1024.0;  // 2^-10
	private static final double P2_20 = 1.0 / 1048576.0; // 2^-20
	private static final double P2_29 = 1.0 / (1L << 29); // 2^-29
	private static final double P2_30 = 1.0 / (1L << 30); // 2^-30
	private static final double P2_31 = 1.0 / (1L << 31); // 2^-31
	private static final double P2_34 = 1.0 / (1L << 34); // 2^-34
	private static final double P2_43 = 1.0 / (1L << 43); // 2^-43
	private static final double P2_48 = 1.0 / (1L << 48); // 2^-48

	// MSM参数表: {roughBits, fineBits, cnrBits, phaseUnit}
	private static final double[][] MSM_PARAMS = {
		{8, 8, 6, 0.0001},    // MSM1 - 紧凑伪距
		{8, 10, 6, 0.0001},   // MSM2 - 紧凑相位+伪距
		{8, 15, 6, 0.0005},   // MSM3 - 紧凑+CNR
		{8, 10, 6, 0.0001},   // MSM4 - 完整（最常用）
		{8, 15, 6, 0.0005},   // MSM5 - 完整+相位变化率
		{12, 15, 10, 0.0005}, // MSM6 - 高精度
		{12, 22, 10, 0.0005}  // MSM7 - 高精度+相位变化率
	};

	private int msgType;
	private int msmLevel;

	public DecodeMSMMsg(int msgType) {
		this.msgType = msgType;
		this.msmLevel = msgType % 10;
	}

	/**
	 * 根据消息类型获取卫星系统
	 */
	private char getSatelliteSystem() {
		if (msgType >= 1071 && msgType <= 1077) return 'G'; // GPS
		if (msgType >= 1081 && msgType <= 1087) return 'R'; // GLONASS
		if (msgType >= 1091 && msgType <= 1097) return 'E'; // Galileo
		if (msgType >= 1101 && msgType <= 1107) return 'S'; // SBAS
		if (msgType >= 1111 && msgType <= 1117) return 'J'; // QZSS
		if (msgType >= 1121 && msgType <= 1127) return 'C'; // BeiDou
		if (msgType >= 1131 && msgType <= 1137) return 'M'; // Multi-GNSS
		return 'G';
	}

	@Override
	public Object decode(boolean[] bits, int week) {
		if (bits.length < 64) {
			System.err.printf("MSM解码失败: 数据长度不足, type=%d, length=%d%n", msgType, bits.length);
			return null;
		}

		int i = 12; // 跳过消息类型
		double[] params = MSM_PARAMS[msmLevel - 1];
		int roughBits = (int) params[0];
		int fineBits = (int) params[1];
		int cnrBits = (int) params[2];
		double phaseScale = params[3];

		try {
			// ========== 消息头 ==========
			// Station ID: 12 bits
			int stationId = (int) bitsToUInt(bits, i, 12);
			i += 12;

			// Epoch Time: 30 bits, scale 0.001s
			double tow = bitsToUInt(bits, i, 30) * 0.001;
			// 调试：打印原始 tow 值
			System.err.printf("[DEBUG DecodeMSM] 原始 tow=%f, satSys=%c%n", tow, getSatelliteSystem());
			// 注意：RTCM3 MSM 消息中的 BeiDou 时间标签通常是 BDT 格式
			// 需要转换为 GPST（加 14 秒）
			if (getSatelliteSystem() == 'C') {
				tow = tow + 14.0;
			}
			System.err.printf("[DEBUG DecodeMSM] 转换后 tow=%f%n", tow);
			i += 30;

			// Multiple Message: 1 bit
			i += 1;

			// IOD: 3 bits
			int iod = (int) bitsToUInt(bits, i, 3);
			i += 3;

			// Reserved: 7 bits
			i += 7;

			// Clock Steering: 2 bits
			i += 2;

			// External Clock: 2 bits
			i += 2;

			// Smoothing: 1 bit
			i += 1;

			// Smoothing Interval: 3 bits
			i += 3;

			// ========== 卫星掩码 (64 bits) ==========
			ArrayList<Integer> satellites = new ArrayList<>();
			for (int s = 0; s < 64; s++) {
				if (bitsToUInt(bits, i + s, 1) == 1) {
					satellites.add(s + 1); // PRN从1开始
				}
			}
			i += 64;
			int satCount = satellites.size();
			System.err.printf("[DEBUG DecodeMSM] MSM类型=%d (level=%d), roughBits=%d, satCount=%d%n", 
                msgType, msmLevel, roughBits, satCount);

			// ========== 信号掩码 (32 bits) ==========
			ArrayList<Integer> signals = new ArrayList<>();
			for (int s = 0; s < 32; s++) {
				if (bitsToUInt(bits, i + s, 1) == 1) {
					signals.add(s + 1); // Signal ID从1开始
				}
			}
			i += 32;
			int sigCount = signals.size();

			if (satCount == 0 || sigCount == 0) {
				return null;
			}

			// ========== Cell Mask ==========
			int totalCells = satCount * sigCount;
			boolean[] cellMask = new boolean[totalCells];
			for (int c = 0; c < totalCells; c++) {
				cellMask[c] = bitsToUInt(bits, i + c, 1) == 1;
			}
			i += totalCells;

			// ========== 粗糙范围 (roughRange) ==========
			int[] roughRange = new int[satCount];
			System.err.printf("[DEBUG DecodeMSM] 开始解码 roughRange, roughBits=%d, satCount=%d%n", roughBits, satCount);
			for (int s = 0; s < satCount; s++) {
				roughRange[s] = (int) bitsToUInt(bits, i, roughBits);
				System.err.printf("[DEBUG DecodeMSM] roughRange[%d]=%d%n", s, roughRange[s]);
				i += roughBits;
			}

			// ========== 扩展信息 (MSM5/7) ==========
			int[] extendedInfo = null;
			if (msmLevel == 5 || msmLevel == 7) {
				extendedInfo = new int[satCount];
				for (int s = 0; s < satCount; s++) {
					extendedInfo[s] = (int) bitsToUInt(bits, i, 4);
					i += 4;
				}
			}

			// ========== 粗糙范围小数部分 ==========
			int[] roughRangeFine = new int[satCount];
			for (int s = 0; s < satCount; s++) {
				roughRangeFine[s] = (int) bitsToUInt(bits, i, 10);
				i += 10;
			}

			// ========== 粗糙相位变化率 (MSM5/7) ==========
			int[] roughPhaseRate = null;
			if (msmLevel == 5 || msmLevel == 7) {
				roughPhaseRate = new int[satCount];
				for (int s = 0; s < satCount; s++) {
					roughPhaseRate[s] = (int) bitsToUInt(bits, i, 14);
					i += 14;
				}
			}

			// ========== 精细值数组 ==========
			double[][] fineRange = new double[satCount][sigCount];
			double[][] finePhase = new double[satCount][sigCount];
			int[][] lockTime = new int[satCount][sigCount];
			int[][] halfAmb = new int[satCount][sigCount];
			double[][] cnr = new double[satCount][sigCount];
			double[][] finePhaseRate = new double[satCount][sigCount];

			// ========== 精细伪距 ==========
			for (int s = 0; s < satCount; s++) {
				for (int sig = 0; sig < sigCount; sig++) {
					int cellIdx = s * sigCount + sig;
					if (cellMask[cellIdx]) {
						fineRange[s][sig] = bitsSigned(bits, i, fineBits);
						i += fineBits;
					}
				}
			}

			// ========== 精细载波相位 ==========
			for (int s = 0; s < satCount; s++) {
				for (int sig = 0; sig < sigCount; sig++) {
					int cellIdx = s * sigCount + sig;
					if (cellMask[cellIdx]) {
						finePhase[s][sig] = bitsSigned(bits, i, fineBits);
						i += fineBits;
					}
				}
			}

			// ========== 锁定时长 ==========
			int lockBits = (msmLevel == 6 || msmLevel == 7) ? 10 : 4;
			for (int s = 0; s < satCount; s++) {
				for (int sig = 0; sig < sigCount; sig++) {
					int cellIdx = s * sigCount + sig;
					if (cellMask[cellIdx]) {
						lockTime[s][sig] = (int) bitsToUInt(bits, i, lockBits);
						i += lockBits;
					}
				}
			}

			// ========== 半周模糊度 ==========
			for (int s = 0; s < satCount; s++) {
				for (int sig = 0; sig < sigCount; sig++) {
					int cellIdx = s * sigCount + sig;
					if (cellMask[cellIdx]) {
						halfAmb[s][sig] = (int) bitsToUInt(bits, i, 1);
						i += 1;
					}
				}
			}

			// ========== CNR ==========
			double cnrUnit = (msmLevel == 6 || msmLevel == 7) ? 0.0625 : 1.0;
			for (int s = 0; s < satCount; s++) {
				for (int sig = 0; sig < sigCount; sig++) {
					int cellIdx = s * sigCount + sig;
					if (cellMask[cellIdx]) {
						cnr[s][sig] = bitsToUInt(bits, i, cnrBits) * cnrUnit;
						i += cnrBits;
					}
				}
			}

			// ========== 精细相位变化率 (MSM5/7) ==========
			if (msmLevel == 5 || msmLevel == 7) {
				for (int s = 0; s < satCount; s++) {
					for (int sig = 0; sig < sigCount; sig++) {
						int cellIdx = s * sigCount + sig;
						if (cellMask[cellIdx]) {
							finePhaseRate[s][sig] = bitsSigned(bits, i, 15) * 0.001; // 米/秒
							i += 15;
						}
					}
				}
			}

			// ========== 生成观测值记录 ==========
			Observations obs = new Observations(new Time(week, tow), stationId);
			char satType = getSatelliteSystem();

			// 根据MSM等级选择精细值缩放因子
			double fineScale;
			switch (msmLevel) {
				case 1: fineScale = RANGE_MS / (1L << 8); break;
				case 2: fineScale = RANGE_MS / (1L << 10); break;
				case 3: fineScale = RANGE_MS / (1L << 15); break;
				case 4: fineScale = RANGE_MS / (1L << 10); break;
				case 5: fineScale = RANGE_MS / (1L << 15); break;
				case 6: fineScale = RANGE_MS / (1L << 15); break;
				case 7: fineScale = RANGE_MS / (1L << 22); break;
				default: fineScale = RANGE_MS / (1L << 10); break;
			}

			for (int s = 0; s < satCount; s++) {
				int satId = satellites.get(s);
				
				// 计算伪距
				// 根据 RTCM3 MSM5 规范:
				// - roughRange: 整数毫秒 (已经是毫秒，不需要 * 0.001)
				// - fineRange: 2^-10 毫秒 (需要 * P2_10)
				double roughRangeMs = roughRange[s]; // 粗糙值已经是毫秒
				double fineRangeMs = fineRange[s][0] * P2_10; // 精细值转换为毫秒 (P2_10 = 2^-10)
				double pseudorange = (roughRangeMs + fineRangeMs) * RANGE_MS;
				System.err.printf("[DEBUG DecodeMSM] 卫星 %c%d: roughRange=%d, roughRangeMs=%.6f ms, fineRangeMs=%.6f ms, pseudorange=%.2f m (对应时间=%.6f ms)%n", 
						satType, satId, roughRange[s], roughRangeMs, fineRangeMs, pseudorange, pseudorange / RANGE_MS);

				// 创建观测集
				ObservationSet os = new ObservationSet();
				os.setSatType(satType);
				os.setSatID(satId);

				// 设置伪距
				os.setCodeC(0, pseudorange);

				// 设置载波相位 (MSM2及以上)
				if (msmLevel >= 2) {
					double phase = finePhase[s][0] * phaseScale; // 单位: 米
					// 转换为周 (载波相位/波长)
					double wavelength = getWavelength(satType, signals.get(0));
					if (wavelength > 0) {
						os.setPhaseCycles(0, phase / wavelength);
					}
				}

				// 设置CNR
				os.setSignalStrength(0, (float) cnr[s][0]);

				// 设置半周模糊度标志
				os.setHalfCycleAmb(0, halfAmb[s][0] == 1);

				// 设置LLI (Loss of Lock Indicator)
				int lli = 0;
				if (lockTime[s][0] == 0) {
					lli |= 1; // LLI bit set
				}
				os.setLossLockInd(0, lli);

				// 设置多普勒 (MSM5/7)
				if (msmLevel == 5 || msmLevel == 7) {
					os.setDoppler(0, (float) finePhaseRate[s][0]);
				}

				obs.setGps(s, os);
			}

			return obs;

		} catch (Exception e) {
			System.err.printf("MSM解析异常: type=%d, msmLevel=%d, error=%s%n", msgType, msmLevel, e.getMessage());
			return null;
		}
	}

	/**
	 * 获取载波波长 (米)
	 */
	private double getWavelength(char satType, int signalId) {
		switch (satType) {
			case 'G': // GPS L1
				if (signalId == 1) return 0.1902936727984; // L1
				if (signalId == 2) return 0.2442102136241; // L2
				return 0.1902936727984;
			case 'R': // GLONASS
				if (signalId == 1) return 0.1902936727984; // L1
				if (signalId == 2) return 0.2442102136241; // L2
				return 0.1902936727984;
			case 'E': // Galileo
				if (signalId == 1) return 0.1902936727984; // E1
				if (signalId == 5) return 0.2515140450406; // E5a
				if (signalId == 6) return 0.2550232057122; // E5b
				if (signalId == 7) return 0.2530548706806; // E5a+E5b
				return 0.1902936727984;
			case 'C': // BeiDou
				if (signalId == 1) return 0.1902936727984; // B1
				if (signalId == 2) return 0.2442102136241; // B2
				if (signalId == 5) return 0.1936760860390; // B3
				return 0.1902936727984;
			case 'J': // QZSS
				if (signalId == 1) return 0.1902936727984; // L1
				if (signalId == 2) return 0.2442102136241; // L2
				if (signalId == 5) return 0.2515140450406; // L5
				return 0.1902936727984;
			default:
				return 0.1902936727984;
		}
	}

	private long bitsToUInt(boolean[] bits, int start, int length) {
		long result = 0;
		for (int j = 0; j < length; j++) {
			if (bits[start + j]) {
				result |= (1L << (length - 1 - j));
			}
		}
		return result;
	}

	private long bitsSigned(boolean[] bits, int start, int length) {
		long unsigned = bitsToUInt(bits, start, length);
		long msb = 1L << (length - 1);
		if ((unsigned & msb) != 0) {
			return unsigned - (1L << length);
		}
		return unsigned;
	}
}