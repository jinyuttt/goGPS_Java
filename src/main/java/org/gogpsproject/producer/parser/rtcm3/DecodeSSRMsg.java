/*
 * Copyright (c) 2010 Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
 *
 * This file is part of goGPS Project (goGPS).
 */

package org.gogpsproject.producer.parser.rtcm3;

import org.gogpsproject.ephemeris.SsrOrbitClock;
import org.gogpsproject.ephemeris.SsrOrbitClock.SsrSatelliteCorrection;

/**
 * RTCM3 SSR (State Space Representation) Messages Decoder
 * 
 * Supports:
 * - 1057-1060: GPS SSR Orbit/Clock/URA/Code Bias
 * - 1061-1064: GLONASS SSR Orbit/Clock/URA/Code Bias
 * - 1065-1068: Galileo SSR Orbit/Clock/URA/Code Bias
 * - 1240-1251: Extended SSR Messages
 * - 1258-1263: SSR Ionosphere/Troposphere
 * 
 * @author goGPS Project
 */
public class DecodeSSRMsg implements Decode {

	private int msgType;
	
	public DecodeSSRMsg(int msgType) {
		this.msgType = msgType;
	}

	@Override
	public Object decode(boolean[] bits, int week) {
		
		int i = 0;
		
		SsrOrbitClock ssr = new SsrOrbitClock();
		
		// Message Number: 12 bits
		i += 12;
		
		// Epoch Time: 20 bits, scale 0.001s
		double epochTime = decodeUnsigned(bits, i, 20) * 0.001;
		ssr.setEpochTime(epochTime);
		i += 20;
		
		// Update Interval: 4 bits
		int udi = (int) decodeUnsigned(bits, i, 4);
		ssr.setUpdateInterval(SsrOrbitClock.getUpdateInterval(udi));
		i += 4;
		
		// Multiple Message Indicator: 1 bit
		boolean multipleMessage = decodeUnsigned(bits, i, 1) == 1;
		ssr.setMultipleMessage(multipleMessage);
		i += 1;
		
		// IOD SSR: 4 bits
		int iodSsr = (int) decodeUnsigned(bits, i, 4);
		ssr.setIodSsr(iodSsr);
		i += 4;
		
		// Provider ID: 16 bits
		int providerId = (int) decodeUnsigned(bits, i, 16);
		ssr.setProviderId(providerId);
		i += 16;
		
		// Solution ID: 4 bits
		int solutionId = (int) decodeUnsigned(bits, i, 4);
		ssr.setSolutionId(solutionId);
		i += 4;
		
		// Number of Satellites: 6 bits
		int numSats = (int) decodeUnsigned(bits, i, 6);
		ssr.setSatelliteCount(numSats);
		i += 6;
		
		// Get SSR type and system from message type
		ssr.setMsgType(msgType);
		ssr.setSsrType(SsrOrbitClock.getSsrTypeFromMsgType(msgType));
		ssr.setSystem(SsrOrbitClock.getSystemFromMsgType(msgType));
		
		// Satellite-specific corrections
		for (int s = 0; s < numSats; s++) {
			SsrSatelliteCorrection satCorr = new SsrSatelliteCorrection();
			
			// Satellite ID: 6 bits
			int satId = (int) decodeUnsigned(bits, i, 6);
			satCorr.setPrn(satId);
			i += 6;
			
			int ssrType = ssr.getSsrType();
			
			// Type 1 or 4: Orbit Corrections
			if (ssrType == 1 || ssrType == 4) {
				// IODE: 8 bits
				int iode = (int) decodeUnsigned(bits, i, 8);
				satCorr.setIode(iode);
				i += 8;
				
				// Radial Orbit Correction: 22 bits, scale 0.1mm
				double radial = decodeSigned(bits, i, 22) * 0.0001;
				i += 22;
				
				// Along-Track Orbit Correction: 20 bits, scale 0.4mm
				double alongTrack = decodeSigned(bits, i, 20) * 0.0004;
				i += 20;
				
				// Cross-Track Orbit Correction: 20 bits, scale 0.4mm
				double crossTrack = decodeSigned(bits, i, 20) * 0.0004;
				i += 20;
				
				// Orbit Correction Rate: 21 bits, scale 0.001mm/s (Type 4 only)
				if (ssrType == 4) {
					i += 21;
				}
				
				// Orbit Correction Uncertainty: 8 bits
				i += 8;
				
				satCorr.setDeltaOrbit(new double[]{radial, alongTrack, crossTrack});
			}
			
			// Type 2 or 4: Clock Corrections
			if (ssrType == 2 || ssrType == 4) {
				// C0 Clock Correction: 22 bits, scale 0.1mm
				double c0 = decodeSigned(bits, i, 22) * 0.0001;
				i += 22;
				
				// C1 Clock Correction Rate: 21 bits, scale 0.001mm/s
				double c1 = decodeSigned(bits, i, 21) * 0.000001;
				i += 21;
				
				// C2 Clock Correction Acceleration: 27 bits, scale 0.00002mm/s²
				double c2 = decodeSigned(bits, i, 27) * 0.00000002;
				i += 27;
				
				satCorr.setDeltaClock(new double[]{c0, c1, c2});
			}
			
			// Type 3: Code Biases
			if (ssrType == 3) {
				// Number of Code Biases: 5 bits
				int numBiases = (int) decodeUnsigned(bits, i, 5);
				i += 5;
				
				for (int b = 0; b < numBiases; b++) {
					// Signal ID: 5 bits
					int signalId = (int) decodeUnsigned(bits, i, 5);
					i += 5;
					
					// Code Bias: 14 bits, scale 0.01m
					double bias = decodeSigned(bits, i, 14) * 0.01;
					i += 14;
					
					satCorr.addCodeBias(signalId, bias);
				}
			}
			
			// Type 5: URA
			if (ssrType == 5) {
				// URA Index: 6 bits
				int ura = (int) decodeUnsigned(bits, i, 6);
				satCorr.setUra(ura);
				i += 6;
			}
			
			// Type 6: High Rate Clock Correction
			if (ssrType == 6) {
				// HR Clock Correction: 22 bits, scale 0.1mm
				double hrClock = decodeSigned(bits, i, 22) * 0.0001;
				satCorr.setHighRateClock(hrClock);
				i += 22;
			}
			
			// Add satellite correction
			ssr.addSatelliteCorrection(satId, satCorr);
		}
		
		return ssr;
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