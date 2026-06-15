/*
 * Copyright (c) 2010 Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
 *
 * This file is part of goGPS Project (goGPS).
 */

package org.gogpsproject.producer.parser.rtcm3;

import org.gogpsproject.util.Bits;

/**
 * RTCM3 Message Type 1033 - Receiver and Antenna Information
 * 
 * Full receiver and antenna descriptor including:
 * - Station ID
 * - Antenna descriptor
 * - Antenna serial number
 * - Receiver type
 * - Receiver version
 * - Receiver serial number
 * 
 * @author goGPS Project
 */
public class Decode1033Msg implements Decode {

	private RTCM3Client client;

	public Decode1033Msg(RTCM3Client client) {
		this.client = client;
	}

	@Override
	public Object decode(boolean[] bits, int referenceTS) {
		
		StationaryAntenna antenna = new StationaryAntenna();
		
		int i = 12; // Skip message number
		
		// Station ID: 12 bits
		int stationId = (int) decodeUnsigned(bits, i, 12);
		antenna.setStationID(stationId);
		i += 12;
		
		// Antenna Descriptor Length: 8 bits
		int n = (int) decodeUnsigned(bits, i, 8);
		i += 8;
		
		// Antenna Descriptor: n * 8 bits
		StringBuilder antDesc = new StringBuilder();
		for (int j = 0; j < n; j++) {
			int charVal = (int) decodeUnsigned(bits, i, 8);
			if (charVal >= 32 && charVal < 127) {
				antDesc.append((char) charVal);
			}
			i += 8;
		}
		antenna.setAntennaDescriptor(antDesc.toString());
		
		// Antenna Setup ID: 8 bits
		antenna.setAntennaSetupId((int) decodeUnsigned(bits, i, 8));
		i += 8;
		
		// Antenna Serial Number Length: 8 bits
		int m = (int) decodeUnsigned(bits, i, 8);
		i += 8;
		
		// Antenna Serial Number: m * 8 bits
		if (m > 0) {
			StringBuilder antSerial = new StringBuilder();
			for (int j = 0; j < m; j++) {
				int charVal = (int) decodeUnsigned(bits, i, 8);
				if (charVal >= 32 && charVal < 127) {
					antSerial.append((char) charVal);
				}
				i += 8;
			}
			antenna.setAntennaSerialNumber(antSerial.toString());
		}
		
		// Receiver Type Length: 8 bits
		int n1 = (int) decodeUnsigned(bits, i, 8);
		i += 8;
		
		// Receiver Type: n1 * 8 bits
		if (n1 > 0) {
			StringBuilder recType = new StringBuilder();
			for (int j = 0; j < n1; j++) {
				int charVal = (int) decodeUnsigned(bits, i, 8);
				if (charVal >= 32 && charVal < 127) {
					recType.append((char) charVal);
				}
				i += 8;
			}
			antenna.setReceiverType(recType.toString());
		}
		
		// Receiver Firmware Version Length: 8 bits
		int n2 = (int) decodeUnsigned(bits, i, 8);
		i += 8;
		
		// Receiver Firmware Version: n2 * 8 bits
		if (n2 > 0) {
			StringBuilder recVer = new StringBuilder();
			for (int j = 0; j < n2; j++) {
				int charVal = (int) decodeUnsigned(bits, i, 8);
				if (charVal >= 32 && charVal < 127) {
					recVer.append((char) charVal);
				}
				i += 8;
			}
			antenna.setReceiverFirmware(recVer.toString());
		}
		
		// Receiver Serial Number Length: 8 bits
		int n3 = (int) decodeUnsigned(bits, i, 8);
		i += 8;
		
		// Receiver Serial Number: n3 * 8 bits
		if (n3 > 0) {
			StringBuilder recSerial = new StringBuilder();
			for (int j = 0; j < n3; j++) {
				int charVal = (int) decodeUnsigned(bits, i, 8);
				if (charVal >= 32 && charVal < 127) {
					recSerial.append((char) charVal);
				}
				i += 8;
			}
			antenna.setReceiverSerialNumber(recSerial.toString());
		}
		
		return antenna;
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