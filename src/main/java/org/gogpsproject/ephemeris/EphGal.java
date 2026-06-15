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
 * Galileo broadcast ephemerides
 * 
 * @author goGPS Project
 */
public class EphGal extends EphGps {
	
	private final static int STREAM_V = 1;

	// Galileo specific parameters
	private double bgd_e5a_e1; /* Group delay E5a-E1 */
	private double bgd_e5b_e1; /* Group delay E5b-E1 */
	private int dataSource; /* Data source indicator */

	public EphGal() {
	}

	@Override
	public void read(DataInputStream dai, boolean oldVersion) throws IOException {
		if (!oldVersion) {
			int v = dai.readInt();
			if (v != STREAM_V) {
				throw new IOException("Unknown version in EphGal");
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
		bgd_e5a_e1 = dai.readDouble();
		bgd_e5b_e1 = dai.readDouble();
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
		dataSource = dai.readInt();
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
		dao.writeDouble(bgd_e5a_e1); size += 8;
		dao.writeDouble(bgd_e5b_e1); size += 8;
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
		dao.writeInt(dataSource); size += 4;
		return size;
	}

	// ==================== Galileo specific Getters & Setters ====================

	public double getBgdE5aE1() {
		return bgd_e5a_e1;
	}

	public void setBgdE5aE1(double bgd) {
		this.bgd_e5a_e1 = bgd;
	}

	public double getBgdE5bE1() {
		return bgd_e5b_e1;
	}

	public void setBgdE5bE1(double bgd) {
		this.bgd_e5b_e1 = bgd;
	}

	public int getDataSource() {
		return dataSource;
	}

	public void setDataSource(int dataSource) {
		this.dataSource = dataSource;
	}
}