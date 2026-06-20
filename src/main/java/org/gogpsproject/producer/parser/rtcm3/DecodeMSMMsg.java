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

				/* RTKLIB-aligned signal selection (sigindex + getcodepri).
				 *
				 * Each RTCM MSM signal ID is mapped to an RTKLIB obs code (e.g.
				 * sigId=2 -> "2I" -> CODE_L2I=40 -> B1I). The obs code determines:
				 *   - freq-index (code2idx): which frequency channel (0..4)
				 *   - carrier frequency (code2freq): exact Hz for wavelength
				 *   - priority (getcodepri): which signal wins when multiple
				 *     signals compete for the same freq-index.
				 *
				 * For each freq-index, only the highest-priority signal is kept;
				 * lower-priority signals are dropped (RTKLIB stores them in
				 * extended obs slots, but goGPS only uses NFREQ=5 main channels).
				 *
				 * BeiDou priority (codepris[5], ref RTKLIB rtkcmn.c):
				 *   idx 0: "IQXDPAN" -> B1I(I) > B1C(X/D/P) > B1A(A) > B1N(N)
				 *   idx 1: "IQXDPZ"  -> B2b(I/Q/X) > B2a(D/P) > B2ab(Z)
				 *   idx 2: "DPX"     -> B2a(D/P/X)
				 *   idx 3: "IQXA"    -> B3I(I/Q/X) > B3A(A)
				 *   idx 4: "DPX"     -> B2ab(D/P/X)
				 *
				 * BDS Phase 4 (pre-2035) will add new signals; extend
				 * getObsCode() and the priority tables as ICDs are published.
				 */
				int[] sigCodeCh = new int[ObservationSet.NFREQ];      // RTKLIB code per freq-index
				int[] sigOrigCh = new int[ObservationSet.NFREQ];      // original signal index per freq-index
				int[] sigPriCh  = new int[ObservationSet.NFREQ];      // priority per freq-index
				java.util.Arrays.fill(sigCodeCh, 0);
				java.util.Arrays.fill(sigOrigCh, -1);
				java.util.Arrays.fill(sigPriCh, -1);
				for (int sig = 0; sig < sigCount; sig++) {
					int sid = signals.get(sig);
					int obsCode = getObsCode(satType, sid);
					if (obsCode == 0) continue; // unsupported signal
					int freqIdx = code2freqIndex(satType, obsCode);
					if (freqIdx < 0 || freqIdx >= ObservationSet.NFREQ) continue;
					int pri = getCodePriority(satType, obsCode);
					// Select highest priority signal for this freq-index
					if (sigOrigCh[freqIdx] < 0 || pri > sigPriCh[freqIdx]) {
						sigCodeCh[freqIdx] = obsCode;
						sigOrigCh[freqIdx] = sig;
						sigPriCh[freqIdx] = pri;
					}
				}
				// Build compact arrays of selected signals (only filled channels)
				int nSigDecode = 0;
				for (int fi = 0; fi < ObservationSet.NFREQ; fi++) {
					if (sigOrigCh[fi] >= 0) nSigDecode = fi + 1;
				}
				int[] sigCodes = new int[nSigDecode];
				int[] sigOrigIdx = new int[nSigDecode];
				int[] sigFreqIdx = new int[nSigDecode];
				for (int fi = 0; fi < nSigDecode; fi++) {
					sigCodes[fi] = sigCodeCh[fi];
					sigOrigIdx[fi] = sigOrigCh[fi];
					sigFreqIdx[fi] = fi;
				}

				if (s == 0 && satType == 'C') {
					StringBuilder sb = new StringBuilder();
					sb.append(String.format("[MSM decode] BDS sat=%d, sigCount=%d", satId, sigCount));
					for (int si = 0; si < nSigDecode; si++) {
						if (sigOrigIdx[si] >= 0) {
							sb.append(String.format(" | f%d: code=%d(%s) pri=%d",
								si, sigCodes[si], codeToObsStr(sigCodes[si]), getCodePriority(satType, sigCodes[si])));
						}
					}
					System.err.println(sb.toString());
				}

				for (int si = 0; si < nSigDecode; si++) {
					if (sigOrigIdx[si] < 0) continue; // no signal for this freq-index
					int obsCode = sigCodes[si];
					int origIdx = sigOrigIdx[si];
					int freqIdx = sigFreqIdx[si];

					// Store RTKLIB code so getWavelength() can compute exact frequency
					os.setCode(freqIdx, obsCode);

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
						double wavelength = os.getWavelength(freqIdx);
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

	/* ====================================================================
	 * RTKLIB-aligned signal selection (ref rtcm3.c sigindex + rtkcmn.c)
	 *
	 * The selection pipeline is:
	 *   RTCM sigId  --getObsCode()-->  RTKLIB code  --code2freqIndex()-->  freq-index
	 *                                          |
	 *                                   getCodePriority()
	 *                                          |
	 *                              highest priority wins per freq-index
	 *
	 * For each frequency channel (0..NFREQ-1), only the highest-priority
	 * signal is stored in ObservationSet. This guarantees that rover and
	 * base always use the SAME signal type for the same satellite, which
	 * is essential for double-differencing (mixing B1I and B1C, which
	 * differ in DCB by ~14 ns / ~4 m, produces huge DD residuals).
	 *
	 * BeiDou frequency channels (ref RTKLIB code2freq_BDS):
	 *   idx 0: B1I (1561.098 MHz) / B1C (1575.42 MHz)
	 *   idx 1: B2I / B2b (1207.140 MHz)
	 *   idx 2: B2a (1176.450 MHz)
	 *   idx 3: B3I (1268.520 MHz)
	 *   idx 4: B2ab (1191.795 MHz)
	 *
	 * BeiDou code priority (ref RTKLIB codepris[5]):
	 *   idx 0: "IQXDPAN" -> I(B1I) > Q > X > D(B1C) > P > A > N
	 *   idx 1: "IQXDPZ"  -> I(B2b) > Q > X > D(B2a) > P > Z(B2ab)
	 *   idx 2: "DPX"     -> D(B2a) > P > X
	 *   idx 3: "IQXA"    -> I(B3I) > Q > X > A
	 *   idx 4: "DPX"     -> D(B2ab) > P > X
	 *
	 * BDS Phase 4 (pre-2035) expansion:
	 *   When new BDS signals are defined (e.g. B1A, B3A, or future signals),
	 *   add their RTCM sigId -> obs code mapping in getObsCode() and ensure
	 *   their priority is listed in CODE_PRIS. The rest of the pipeline
	 *   (freq-index assignment, priority selection) will work automatically.
	 * ==================================================================== */

	/* RTKLIB obs code constants (subset, ref rtklib.h CODE_???).
	 * Only codes relevant to RTCM MSM are listed here. */
	private static final int CODE_NONE = 0;
	private static final int CODE_L1C  = 1;   /* L1C/A, G1C/A, E1C (GPS,GLO,GAL,QZS,SBS) */
	private static final int CODE_L1S  = 7;   /* L1C(D) (GPS,QZS) - B1C pilot */
	private static final int CODE_L1L  = 8;   /* L1C(P) (GPS,QZS) - B1C data */
	private static final int CODE_L1X  = 12;  /* L1C(D+P), B1D+P (B1C combined) */
	private static final int CODE_L2C  = 14;  /* L2C/A (GPS) */
	private static final int CODE_L2W  = 20;  /* L2 Z-track (GPS) */
	private static final int CODE_L5I  = 24;  /* L5I, E5aI, B2aD */
	private static final int CODE_L5Q  = 25;  /* L5Q, E5aQ, B2aP */
	private static final int CODE_L5X  = 26;  /* L5I+Q, B2aD+P */
	private static final int CODE_L7I  = 27;  /* E5bI, B2bI */
	private static final int CODE_L7Q  = 28;  /* E5bQ, B2bQ */
	private static final int CODE_L7X  = 29;  /* E5bI+Q, B2bI+Q */
	private static final int CODE_L6I  = 42;  /* B3I (BDS) */
	private static final int CODE_L6Q  = 43;  /* B3Q (BDS) */
	private static final int CODE_L6X  = 33;  /* B3I+Q (BDS) */
	private static final int CODE_L2I  = 40;  /* B1I (BDS) */
	private static final int CODE_L2Q  = 41;  /* B1Q (BDS) */
	private static final int CODE_L1D  = 56;  /* B1C data (BDS-3) - ref rtklib.h CODE_L1D=56 */
	private static final int CODE_L5D  = 57;  /* B2a data (BDS-3) - ref rtklib.h CODE_L5D=57 */
	private static final int CODE_L5P  = 58;  /* B2a pilot (BDS-3) - ref rtklib.h CODE_L5P=58 */

	/* RTKLIB obscodes table (index = code value, ref rtkcmn.c obscodes[]).
	 * Used to convert numeric code back to 2-char string for debug output. */
	private static final String[] OBSCODES = {
		""  ,"1C","1P","1W","1Y","1M","1N","1S","1L","1E",  /*  0- 9 */
		"1A","1B","1X","1Z","2C","2D","2S","2L","2X","2P",  /* 10-19 */
		"2W","2Y","2M","2N","5I","5Q","5X","7I","7Q","7X",  /* 20-29 */
		"6A","6B","6C","6X","6Z","6S","6L","8I","8Q","8X",  /* 30-39 */
		"2I","2Q","6I","6Q","3I","3Q","3X","1I","1Q","5A",  /* 40-49 */
		"5B","5C","9A","9B","9C","9X","1D","5D","5P","5Z",  /* 50-59 */
		"6E","7D","7P","7Z","8D","8P","4A","4B","4X",""     /* 60-69 */
	};

	/**
	 * Convert RTKLIB numeric code to 2-char obs string (for debug).
	 */
	private String codeToObsStr(int code) {
		if (code < 0 || code >= OBSCODES.length) return "?";
		return OBSCODES[code];
	}

	/**
	 * Map RTCM3 MSM signal ID to RTKLIB obs code.
	 * Mirrors RTKLIB's msm_sig_*[] tables (ref rtcm3.c).
	 *
	 * GPS (msm_sig_gps, ref RTCM 10420.1 table 3.5-106):
	 *   1=1C, 2=1P, 3=1W, 4=1Y, 5=1M, 6=1N, 7=1S, 8=1L, 9=1X
	 *   10=2C, 11=2D, 12=2S, 13=2L, 14=2X, 15=2P, 16=2W, 17=2Y, 18=2M, 19=2N
	 *   20=5I, 21=5Q, 22=5X
	 *
	 * BeiDou (msm_sig_cmp, ref RTCM 10410.1 table 3.5-108):
	 *   1=""(reserved), 2=2I(B1I), 3=2Q(B1Q), 4=2X(B1I+Q)
	 *   8=6I(B3I), 9=6Q(B3Q), 10=6X(B3I+Q)
	 *   14=7I(B2bI), 15=7Q(B2bQ), 16=7X(B2bI+Q)
	 *   23=1D(B1C data), 24=1P(B1C pilot), 25=1X(B1C data+pilot)
	 *   29=5D(B2a data), 30=5P(B2a pilot), 31=5X(B2a data+pilot)
	 *
	 * BDS Phase 4 (pre-2035): new signal IDs will be assigned by RTCM
	 * 10410.x updates. Add them here as they become available.
	 */
	private int getObsCode(char satType, int signalId) {
		switch (satType) {
			case 'G': // GPS (ref msm_sig_gps)
				switch (signalId) {
					case 1: return CODE_L1C;  // 1C = L1C/A
					case 2: return 2;         // 1P = L1P
					case 3: return 3;         // 1W = L1 Z-track
					case 4: return 4;         // 1Y
					case 5: return 5;         // 1M
					case 6: return 6;         // 1N
					case 7: return CODE_L1S;  // 1S = L1C(D)
					case 8: return CODE_L1L;  // 1L = L1C(P)
					case 9: return CODE_L1X;  // 1X = L1C(D+P)
					case 10: return CODE_L2C; // 2C = L2C/A
					case 11: return 15;       // 2D
					case 12: return 16;       // 2S = L2C(M)
					case 13: return 17;       // 2L = L2C(L)
					case 14: return 18;       // 2X = L2C(M+L)
					case 15: return 19;       // 2P
					case 16: return CODE_L2W; // 2W = L2 Z-track
					case 17: return 21;       // 2Y
					case 18: return 22;       // 2M
					case 19: return 23;       // 2N
					case 20: return CODE_L5I; // 5I
					case 21: return CODE_L5Q; // 5Q
					case 22: return CODE_L5X; // 5X
					default: return CODE_NONE;
				}
			case 'R': // GLONASS (ref msm_sig_glo)
				switch (signalId) {
					case 1: return CODE_L1C;  // 1C = G1C/A
					case 2: return 2;         // 1P = G1P
					case 3: return 14;        // 2C = G2C/A
					case 4: return 19;         // 2P = G2P
					default: return CODE_NONE;
				}
			case 'E': // Galileo (ref msm_sig_gal)
				switch (signalId) {
					case 1: return CODE_L1C;  // 1C = E1C
					case 2: return 11;         // 1B = E1B
					case 3: return CODE_L1X;   // 1X = E1B+C
					case 4: return 10;         // 1A = E1A
					case 5: return 13;          // 1Z = E1A+B+C
					case 6: return CODE_L7I;    // 7I = E5bI
					case 7: return CODE_L7Q;   // 7Q = E5bQ
					case 8: return CODE_L7X;   // 7X = E5bI+Q
					case 9: return CODE_L5I;   // 5I = E5aI
					case 10: return CODE_L5Q;  // 5Q = E5aQ
					case 11: return CODE_L5X;   // 5X = E5aI+Q
					case 12: return 30;        // 6A = E6A
					case 13: return 31;         // 6B = E6B
					case 14: return CODE_L6X;  // 6X = E6B+C
					case 15: return 34;        // 6Z = E6A+B+C
					case 16: return 37;         // 8I = E5abI
					case 17: return 38;        // 8Q = E5abQ
					case 18: return 39;         // 8X = E5abI+Q
					default: return CODE_NONE;
				}
			case 'C': // BeiDou (ref msm_sig_cmp, RTCM 10410.1)
				switch (signalId) {
					case 2: return CODE_L2I;  // 2I = B1I
					case 3: return CODE_L2Q;  // 2Q = B1Q
					case 4: return 18;        // 2X = B1I+Q (uses GPS 2X slot)
					case 8: return CODE_L6I;  // 6I = B3I
					case 9: return CODE_L6Q;  // 6Q = B3Q
					case 10: return CODE_L6X; // 6X = B3I+Q
					case 14: return CODE_L7I; // 7I = B2bI
					case 15: return CODE_L7Q; // 7Q = B2bQ
					case 16: return CODE_L7X; // 7X = B2bI+Q
					case 23: return CODE_L1D; // 1D = B1C data
					case 24: return 2;        // 1P = B1C pilot (uses CODE_L1P=2)
					case 25: return CODE_L1X; // 1X = B1C data+pilot
					case 29: return CODE_L5D; // 5D = B2a data
					case 30: return CODE_L5P; // 5P = B2a pilot
					case 31: return CODE_L5X; // 5X = B2a data+pilot
					default: return CODE_NONE; // reserved/unknown
				}
			case 'J': // QZSS (ref msm_sig_qzs)
				switch (signalId) {
					case 1: return CODE_L1C;  // 1C = L1C/A
					case 2: return 2;         // 1S = L1C(D)
					case 3: return 8;         // 1L = L1C(P)
					case 4: return CODE_L1X;  // 1X = L1C(D+P)
					case 5: return 13;        // 1Z = L1S
					case 6: return CODE_L2C;  // 2S = L2C(M)
					case 7: return 17;        // 2L = L2C(L)
					case 8: return 18;        // 2X = L2C(M+L)
					case 9: return CODE_L5I;  // 5I
					case 10: return CODE_L5Q; // 5Q
					case 11: return CODE_L5X; // 5X
					case 12: return 32;       // 6S = L6S
					case 13: return 36;        // 6L = L6L
					default: return CODE_NONE;
				}
			default:
				return CODE_NONE;
		}
	}

	/**
	 * Map RTKLIB obs code to frequency channel index (0..NFREQ-1).
	 * Mirrors RTKLIB's code2idx() / code2freq_*().
	 *
	 * BeiDou (ref code2freq_BDS):
	 *   obs[0]='1' -> idx 0 (B1C, 1575.42 MHz)
	 *   obs[0]='2' -> idx 0 (B1I, 1561.098 MHz)
	 *   obs[0]='7' -> idx 1 (B2b, 1207.140 MHz)
	 *   obs[0]='5' -> idx 2 (B2a, 1176.450 MHz)
	 *   obs[0]='6' -> idx 3 (B3I, 1268.520 MHz)
	 *   obs[0]='8' -> idx 4 (B2ab, 1191.795 MHz)
	 */
	private int code2freqIndex(char satType, int obsCode) {
		String obs = codeToObsStr(obsCode);
		if (obs.isEmpty()) return -1;
		char c0 = obs.charAt(0);
		switch (satType) {
			case 'G': // GPS: 1->0, 2->1, 5->2
				if (c0 == '1') return 0;
				if (c0 == '2') return 1;
				if (c0 == '5') return 2;
				break;
			case 'R': // GLONASS: 1->0, 2->1
				if (c0 == '1') return 0;
				if (c0 == '2') return 1;
				break;
			case 'E': // Galileo: 1->0, 7->1, 5->2, 6->3, 8->4
				if (c0 == '1') return 0;
				if (c0 == '7') return 1;
				if (c0 == '5') return 2;
				if (c0 == '6') return 3;
				if (c0 == '8') return 4;
				break;
			case 'C': // BeiDou (binary 2.5.0 EX aligned, NOT source 2.4.3-b34):
		          //   B1I(2)->0, B2b(7)->1, B2a(5)->2, B3I(6)->3, B1C(1)->4
		          // NOTE: RTKLIB source 2.4.3-b34 maps B1C to idx 0 (shares with B1I),
		          // but the 2.5.0 EX binary separates B1C into idx 4. We follow the
		          // binary because the app is one generation ahead of the open source.
			if (c0 == '2') return 0;          // B1I  -> idx 0
			if (c0 == '7') return 1;          // B2b  -> idx 1
			if (c0 == '5') return 2;          // B2a  -> idx 2
			if (c0 == '6') return 3;          // B3I  -> idx 3
			if (c0 == '1') return 4;          // B1C  -> idx 4 (binary-aligned)
			if (c0 == '8') return 4;          // B2ab -> idx 4 (conflict with B1C, resolved by priority)
			break;
			case 'J': // QZSS: 1->0, 2->1, 5->2, 6->3
				if (c0 == '1') return 0;
				if (c0 == '2') return 1;
				if (c0 == '5') return 2;
				if (c0 == '6') return 3;
				break;
		}
		return -1;
	}

	/* Code priority tables per system per freq-index (ref RTKLIB codepris).
	 * Characters earlier in the string have HIGHER priority.
	 * Priority value = 14 - (index of char in string), matching RTKLIB's
	 *   (p=strchr(codepris[i][j],obs[1]))?14-(int)(p-codepris[i][j]):0
	 *
	 * BeiDou (codepris[5]):
	 *   idx 0: "IQXDPAN" -> B1I(I,Q,X) > B1C(D,P,X) > B1A(A) > B1N(N)
	 *   idx 1: "IQXDPZ"  -> B2b(I,Q,X) > B2a(D,P) > B2ab(Z)
	 *   idx 2: "DPX"     -> B2a(D,P,X)
	 *   idx 3: "IQXA"    -> B3I(I,Q,X) > B3A(A)
	 *   idx 4: "DPX"     -> B2ab(D,P,X)
	 */
	private static final String[][] CODE_PRIS = {
		// GPS:     idx0         idx1          idx2
		{"CPYWMNSL", "PYWCMNDLSX", "IQX", "", "", ""},           // GPS
		{"CPABX",    "PCABX",      "IQX", "", "", ""},           // GLO
		{"CABXZ",    "IQX",        "IQX", "ABCXZ", "IQX", ""},   // GAL
		{"CLSXZ",    "LSX",        "IQXDPZ", "LSXEZ", "", ""},   // QZS
		{"C",        "IQX",        "", "", "", ""},              // SBS
		{"IQXDPAN",  "IQXDPZ",     "DPX", "IQXA", "DPX", ""},    // BDS
		{"ABCX",     "ABCX",       "", "", "", ""}              // IRN
	};
	/* Map satType char to CODE_PRIS row index */
	private int satTypeToPriRow(char satType) {
		switch (satType) {
			case 'G': return 0;
			case 'R': return 1;
			case 'E': return 2;
			case 'J': return 3;
			case 'S': return 4;
			case 'C': return 5;
			case 'I': return 6;
			default: return -1;
		}
	}

	/**
	 * Get code priority for a signal (mirrors RTKLIB getcodepri).
	 * Higher value = higher priority. Returns 0 if not found.
	 */
	private int getCodePriority(char satType, int obsCode) {
		String obs = codeToObsStr(obsCode);
		if (obs == null || obs.length() < 2) return 0;
		int row = satTypeToPriRow(satType);
		if (row < 0) return 0;
		int freqIdx = code2freqIndex(satType, obsCode);
		if (freqIdx < 0 || freqIdx >= CODE_PRIS[row].length) return 0;
		String pris = CODE_PRIS[row][freqIdx];
		int p = pris.indexOf(obs.charAt(1));
		return (p >= 0) ? 14 - p : 0;
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