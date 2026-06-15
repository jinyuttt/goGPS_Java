/*
 * Copyright (c) 2010 Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
 *
 * This file is part of goGPS Project (goGPS).
 */

package org.gogpsproject.ephemeris;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import org.gogpsproject.producer.Streamable;

/**
 * IRNSS (NavIC) broadcast ephemerides
 * 
 * @author goGPS Project
 */
public class EphIrnss extends EphGps {
	
	private final static int STREAM_V = 1;

	// IRNSS specific parameters
	private double tgd_l5; /* Group delay L5 */
	private double tgd_s; /* Group delay S-band */

	public EphIrnss() {
	}

	@Override
	public void read(DataInputStream dai, boolean oldVersion) throws IOException {
		if (!oldVersion) {
			int v = dai.readInt();
			if (v != STREAM_V) {
				throw new IOException("Unknown version in EphIrnss");
			}
		}
		setSatID(dai.readInt());
		setWeek(dai.readInt());
		setSvHealth(dai.readInt());
		setIode(dai.readInt());
		setIodc(dai.readInt());
		setToc(dai.readDouble());
		setToe(dai.readDouble());
		setAf0(dai.readDouble());
		setAf1(dai.readDouble());
		setAf2(dai.readDouble());
		tgd_l5 = dai.readDouble();
		tgd_s = dai.readDouble();
		setRootA(dai.readDouble());
		setE(dai.readDouble());
		setI0(dai.readDouble());
		setiDot(dai.readDouble());
		setOmega(dai.readDouble());
		setOmega0(dai.readDouble());
		setOmegaDot(dai.readDouble());
		setM0(dai.readDouble());
		setDeltaN(dai.readDouble());
		setCrc(dai.readDouble());
		setCrs(dai.readDouble());
		setCuc(dai.readDouble());
		setCus(dai.readDouble());
		setCic(dai.readDouble());
		setCis(dai.readDouble());
	}

	@Override
	public int write(DataOutputStream dao) throws IOException {
		int size = 0;
		dao.writeInt(STREAM_V); size += 4;
		dao.writeInt(getSatID()); size += 4;
		dao.writeInt(getWeek()); size += 4;
		dao.writeInt(getSvHealth()); size += 4;
		dao.writeInt(getIode()); size += 4;
		dao.writeInt(getIodc()); size += 4;
		dao.writeDouble(getToc()); size += 8;
		dao.writeDouble(getToe()); size += 8;
		dao.writeDouble(getAf0()); size += 8;
		dao.writeDouble(getAf1()); size += 8;
		dao.writeDouble(getAf2()); size += 8;
		dao.writeDouble(tgd_l5); size += 8;
		dao.writeDouble(tgd_s); size += 8;
		dao.writeDouble(getRootA()); size += 8;
		dao.writeDouble(getE()); size += 8;
		dao.writeDouble(getI0()); size += 8;
		dao.writeDouble(getiDot()); size += 8;
		dao.writeDouble(getOmega()); size += 8;
		dao.writeDouble(getOmega0()); size += 8;
		dao.writeDouble(getOmegaDot()); size += 8;
		dao.writeDouble(getM0()); size += 8;
		dao.writeDouble(getDeltaN()); size += 8;
		dao.writeDouble(getCrc()); size += 8;
		dao.writeDouble(getCrs()); size += 8;
		dao.writeDouble(getCuc()); size += 8;
		dao.writeDouble(getCus()); size += 8;
		dao.writeDouble(getCic()); size += 8;
		dao.writeDouble(getCis()); size += 8;
		return size;
	}

	// ==================== IRNSS specific Getters & Setters ====================

	public double getTgdL5() {
		return tgd_l5;
	}

	public void setTgdL5(double tgd) {
		this.tgd_l5 = tgd;
	}

	public double getTgdS() {
		return tgd_s;
	}

	public void setTgdS(double tgd) {
		this.tgd_s = tgd;
	}
}