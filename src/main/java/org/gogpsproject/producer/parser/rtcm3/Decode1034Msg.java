/*
 * Copyright (c) 2010 Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
 *
 * This file is part of goGPS Project (goGPS).
 */

package org.gogpsproject.producer.parser.rtcm3;

import org.gogpsproject.util.Bits;

/**
 * RTCM3 Message Type 1034 - Reference Station Parameters (Extended)
 * 
 * @author goGPS Project
 */
public class Decode1034Msg implements Decode {

	private RTCM3Client client;

	public Decode1034Msg(RTCM3Client client) {
		this.client = client;
	}

	@Override
	public Object decode(boolean[] bits, int referenceTS) {
		int start = 12;

		StationaryAntenna antenna = new StationaryAntenna();

		// Station ID (12 bits)
		antenna.setStationID((int) Bits.bitsToUInt(Bits.subset(bits, start, 12)));
		start += 12;

		// ITRF realization year (6 bits)
		antenna.setItrl((int) Bits.bitsToUInt(Bits.subset(bits, start, 6)));
		start += 6;

		// GPS indicator (1 bit)
		antenna.setGpsIndicator((int) Bits.bitsToUInt(Bits.subset(bits, start, 1)));
		start += 1;

		// GLONASS indicator (1 bit)
		antenna.setGlonassIndicator((int) Bits.bitsToUInt(Bits.subset(bits, start, 1)));
		start += 1;

		// Galileo indicator (1 bit)
		antenna.setRgalileoIndicator((int) Bits.bitsToUInt(Bits.subset(bits, start, 1)));
		start += 1;

		// BeiDou indicator (1 bit)
		start += 1;

		// Antenna Reference Point X (38 bits, scale 0.0001 m)
		antenna.setAntennaRefPointX(Bits.bitsTwoComplement(Bits.subset(bits, start, 38)) * 0.0001);
		start += 38;

		// Antenna Reference Point Y (38 bits, scale 0.0001 m)
		antenna.setAntennaRefPointY(Bits.bitsTwoComplement(Bits.subset(bits, start, 38)) * 0.0001);
		start += 38;

		// Antenna Reference Point Z (38 bits, scale 0.0001 m)
		antenna.setAntennaRefPointZ(Bits.bitsTwoComplement(Bits.subset(bits, start, 38)) * 0.0001);
		start += 38;

		// Antenna Height (16 bits, scale 0.001 m)
		antenna.setAntennaHeight(Bits.bitsToUInt(Bits.subset(bits, start, 16)) * 0.001);
		start += 16;

		// Receiver descriptor (ASCII, 12 chars * 8 bits)
		start += 96;

		// Receiver serial number (ASCII, 12 chars * 8 bits)
		start += 96;

		// Antenna descriptor (ASCII, 12 chars * 8 bits)
		start += 96;

		// Antenna serial number (ASCII, 12 chars * 8 bits)
		start += 96;

		return antenna;
	}
}