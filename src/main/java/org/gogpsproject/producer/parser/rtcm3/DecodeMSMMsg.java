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

	private static final double RANGE_MS = 299792.458;  // CLIGHT * 0.001 m/ms
	private static final double P2_10 = 1.0 / 1024.0;       // 2^-10
	private static final double P2_24 = 1.0 / 16777216.0;    // 2^-24
	private static final double P2_29 = 1.0 / (1L << 29);    // 2^-29
	private static final double P2_31 = 1.0 / (1L << 31);    // 2^-31

	// MSM参数表 (RTKLIB-aligned):
	// {roughBits, finePRBits, finePhaseBits, cnrBits, lockBits, cnrUnit}
	// finePRBits: 精细伪距位宽 (signed), finePhaseBits: 精细相位位宽 (signed)
	// 精细伪距缩放: MSM4/5 → P2_24*RANGE_MS, MSM6/7 → P2_29*RANGE_MS
	// 精细相位缩放: MSM4/5 → P2_29*RANGE_MS, MSM6/7 → P2_31*RANGE_MS
	private static final int[][] MSM_PARAMS = {
		{ 8,  8,  8,  6, 4, 100},  // MSM1
		{ 8, 10, 10,  6, 4, 100},  // MSM2
		{ 8, 15, 15,  6, 4, 100},  // MSM3
		{ 8, 15, 22,  6, 4, 100},  // MSM4: finePR=15bit, finePhase=22bit
		{ 8, 15, 22,  6, 4, 100},  // MSM5: finePR=15bit, finePhase=22bit
		{12, 20, 24, 10,10,  16},  // MSM6: finePR=20bit, finePhase=24bit
		{12, 20, 24, 10,10,  16}   // MSM7: finePR=20bit, finePhase=24bit
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
		int[] params = MSM_PARAMS[msmLevel - 1];
		int roughBits = params[0];
		int finePRBits = params[1];
		int finePhaseBits = params[2];
		int cnrBits = params[3];
		int lockBits = params[4];
		double cnrUnit = params[5] / 100.0; // 除以100恢复真实值 (1.0 or 0.0625)

		// 精细值缩放因子 (RTKLIB-aligned)
		boolean isHighRes = (msmLevel == 6 || msmLevel == 7);
		double finePRScale = isHighRes ? P2_29 : P2_24;
		double finePhaseScale = isHighRes ? P2_31 : P2_29;

		// 精细值哨兵值 (最小有符号值 = -2^(bits-1))
		long finePRSentinel = -(1L << (finePRBits - 1));
		long finePhaseSentinel = -(1L << (finePhaseBits - 1));

		try {
			// ========== 消息头 ==========
			// Station ID: 12 bits
			int stationId = (int) bitsToUInt(bits, i, 12);
			i += 12;

			// Epoch Time: 30 bits, scale 0.001s
			long rawTowBits = bitsToUInt(bits, i, 30);
			double tow = rawTowBits * 0.001;
			double towGPST = tow;
			if (getSatelliteSystem() == 'C') {
				towGPST = tow + 14.0;
			}
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
			for (int s = 0; s < satCount; s++) {
				roughRange[s] = (int) bitsToUInt(bits, i, roughBits);
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

			// ========== 精细伪距 (RTKLIB: getbits signed) ==========
			for (int s = 0; s < satCount; s++) {
				for (int sig = 0; sig < sigCount; sig++) {
					int cellIdx = s * sigCount + sig;
					if (cellMask[cellIdx]) {
						fineRange[s][sig] = bitsSigned(bits, i, finePRBits);
						i += finePRBits;
					}
				}
			}

			// ========== 精细载波相位 (RTKLIB: getbits signed, 位宽与伪距不同!) ==========
			for (int s = 0; s < satCount; s++) {
				for (int sig = 0; sig < sigCount; sig++) {
					int cellIdx = s * sigCount + sig;
					if (cellMask[cellIdx]) {
						finePhase[s][sig] = bitsSigned(bits, i, finePhaseBits);
						i += finePhaseBits;
					}
				}
			}

			// ========== 锁定时长 ==========
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

			// ========== 生成观测值记录 (RTKLIB-aligned) ==========
			Observations obs = new Observations(new Time(week, towGPST), stationId);
			char satType = getSatelliteSystem();

			for (int s = 0; s < satCount; s++) {
				int satId = satellites.get(s);

				// === RTKLIB-aligned 伪距计算 ===
				// 公式: P = (roughRange + roughRangeFine * P2_10) * RANGE_MS + fineRange * finePRScale * RANGE_MS
				// roughRange: 整数毫秒 (roughBits位, 无符号)
				// roughRangeFine: 小数毫秒 (10位, 无符号), P2_10 = 2^-10
				// fineRange: 精细伪距修正 (finePRBits位, 有符号), finePRScale = P2_24 or P2_29
				double roughMs = roughRange[s];
				double roughFineMs = roughRangeFine[s] * P2_10;
				double roughRangeM = (roughMs + roughFineMs) * RANGE_MS;

				ObservationSet os = new ObservationSet();
				os.setSatType(satType);
				os.setSatID(satId);

				// Decode all available signals (up to 2 for dual-frequency L1+L2)
				// Sort signals by wavelength: shorter wavelength (higher freq) → L1, longer → L2
				int nSigDecode = Math.min(sigCount, 2);
				int[] sigIdsSorted = new int[nSigDecode];
				double[] sigWavelengths = new double[nSigDecode];
				int[] sigOrigIdx = new int[nSigDecode]; // original index in fineRange/finePhase arrays
				for (int sig = 0; sig < nSigDecode; sig++) {
					sigIdsSorted[sig] = signals.get(sig);
					sigWavelengths[sig] = getWavelength(satType, sigIdsSorted[sig]);
					sigOrigIdx[sig] = sig;
				}
				// Sort by wavelength ascending (shortest → L1)
				for (int ki = 0; ki < nSigDecode; ki++) {
					for (int kj = ki + 1; kj < nSigDecode; kj++) {
						if (sigWavelengths[ki] > sigWavelengths[kj]) {
							double tmpW = sigWavelengths[ki]; sigWavelengths[ki] = sigWavelengths[kj]; sigWavelengths[kj] = tmpW;
							int tmpId = sigIdsSorted[ki]; sigIdsSorted[ki] = sigIdsSorted[kj]; sigIdsSorted[kj] = tmpId;
							int tmpOi = sigOrigIdx[ki]; sigOrigIdx[ki] = sigOrigIdx[kj]; sigOrigIdx[kj] = tmpOi;
						}
					}
				}

				if (s == 0 && satType == 'C') {
					// System.err.printf("[MSM decode] BDS sat=%d, sigCount=%d, sig[0]=%d(wl=%.4f), sig[1]=%s%n",
					// 	satId, sigCount, sigIdsSorted.length > 0 ? sigIdsSorted[0] : -1,
					// 	sigIdsSorted.length > 0 ? sigWavelengths[0] : 0.0,
					// 	sigIdsSorted.length > 1 ? sigIdsSorted[1] + "(wl=" + String.format("%.4f", sigWavelengths[1]) + ")" : "N/A");
				}

				for (int si = 0; si < nSigDecode; si++) {
					int signalId = sigIdsSorted[si];
					int origIdx = sigOrigIdx[si];
					int freqIdx = si; // 0=L1 (shorter wavelength), 1=L2 (longer wavelength)

					// Pseudorange (RTKLIB: P = (roughRange + roughRangeFine*P2_10)*RANGE_MS + fineRange*finePRScale*RANGE_MS)
					if (roughRange[s] != 255) {
						long finePR = (long) fineRange[s][origIdx];
						double finePRM = (finePR != finePRSentinel) ? finePR * finePRScale * RANGE_MS : 0;
						double pseudorange = roughRangeM + finePRM;
						os.setCodeC(freqIdx, pseudorange);
					}

					// Carrier phase (RTKLIB: L = (roughRangeM + finePhase*finePhaseScale*RANGE_MS) / wavelength)
					if (msmLevel >= 2 && roughRange[s] != 255) {
						long fineCP = (long) finePhase[s][origIdx];
						double fineCPM = (fineCP != finePhaseSentinel) ? fineCP * finePhaseScale * RANGE_MS : 0;
						double phaseM = roughRangeM + fineCPM;
						double wavelength = getWavelength(satType, signalId);
						if (wavelength > 0) {
							os.setPhaseCycles(freqIdx, phaseM / wavelength);
						}
					}

					// CNR
					os.setSignalStrength(freqIdx, (float) cnr[s][origIdx]);

					// Half-cycle ambiguity flag
					os.setHalfCycleAmb(freqIdx, halfAmb[s][origIdx] == 1);

					// LLI (Loss of Lock Indicator)
					int lli = 0;
					if (lockTime[s][origIdx] == 0) {
						lli |= 1;
					}
					os.setLossLockInd(freqIdx, lli);

					// Doppler (MSM5/7)
					if (msmLevel == 5 || msmLevel == 7) {
						os.setDoppler(freqIdx, (float) finePhaseRate[s][origIdx]);
					}
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
			case 'G': // GPS
				if (signalId == 1) return 0.1902936727984; // L1 (1575.42 MHz)
				if (signalId == 2) return 0.2442102136241; // L2 (1227.60 MHz)
				if (signalId == 5) return 0.2548282314958; // L5 (1176.45 MHz)
				return 0.1902936727984;
			case 'R': // GLONASS (frequency-dependent, using nominal values)
				if (signalId == 1) return 0.1902936727984; // L1 (nominal)
				if (signalId == 2) return 0.2442102136241; // L2 (nominal)
				return 0.1902936727984;
			case 'E': // Galileo
				if (signalId == 1) return 0.1902936727984; // E1 (1575.42 MHz)
				if (signalId == 5) return 0.2548282314958; // E5a (1176.45 MHz)
				if (signalId == 6) return 0.2483451365472; // E5b (1207.14 MHz)
				if (signalId == 7) return 0.2515140450406; // E5 AltBOC (1191.795 MHz)
				return 0.1902936727984;
			case 'C': // BeiDou
				if (signalId == 1) return 0.1920473045472; // B1I (1561.098 MHz)
				if (signalId == 2) return 0.2483451365472; // B2I (1207.14 MHz)
				if (signalId == 5) return 0.2363478644346; // B3I (1268.52 MHz)
				if (signalId == 8) return 0.1902936727984; // B1C (1575.42 MHz)
				if (signalId == 23) return 0.2548282314958; // B2a (1176.45 MHz)
				return 0.1920473045472;
			case 'J': // QZSS
				if (signalId == 1) return 0.1902936727984; // L1 (1575.42 MHz)
				if (signalId == 2) return 0.2442102136241; // L2 (1227.60 MHz)
				if (signalId == 5) return 0.2548282314958; // L5 (1176.45 MHz)
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