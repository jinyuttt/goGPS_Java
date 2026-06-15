/*
 * Copyright (c) 2010, Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
 *
 * This file is part of goGPS Project (goGPS).
 *
 * goGPS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3
 * of the License, or (at your option) any later version.
 *
 * goGPS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with goGPS.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
package org.gogpsproject.ephemeris;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import org.gogpsproject.positioning.Time;
import org.gogpsproject.producer.Streamable;

/**
 * <p>
 * BeiDou (BDS) broadcast ephemerides
 * </p>
 *
 * @author goGPS Project
 */

public class EphBds extends EphGps {
	private final static int STREAM_V = 1;

	// BeiDou specific parameters
	private double tgd1; /* Group delay parameter for B1I */
	private double tgd2; /* Group delay parameter for B2I */
	private double aode; /* Age of Data Ephemeris */
	private int aodc; /* Age of Data Clock */

	public static final EphBds UnhealthyEph = new EphBds();


	public EphBds(){

	}
	public EphBds(DataInputStream dai, boolean oldVersion) throws IOException{
		read(dai,oldVersion);
	}

	/**
	 * @return the tgd1 (Group delay parameter for B1I)
	 */
	public double getTgd1() {
		return tgd1;
	}
	/**
	 * @param tgd1 the tgd1 to set
	 */
	public void setTgd1(double tgd1) {
		this.tgd1 = tgd1;
	}
	/**
	 * @return the tgd2 (Group delay parameter for B2I)
	 */
	public double getTgd2() {
		return tgd2;
	}
	/**
	 * @param tgd2 the tgd2 to set
	 */
	public void setTgd2(double tgd2) {
		this.tgd2 = tgd2;
	}
	/**
	 * @return the aode (Age of data ephemeris)
	 */
	public double getAode() {
		return aode;
	}
	/**
	 * @param aode the aode to set
	 */
	public void setAode(double aode) {
		this.aode = aode;
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.Streamable#write(java.io.DataOutputStream)
	 */
	@Override
	public int write(DataOutputStream dos) throws IOException {
		int size=5;
		dos.writeUTF(MESSAGE_EPHEMERIS); // 5
		dos.writeInt(STREAM_V); size+=4; // 4

		dos.writeLong(getRefTime()==null?-1:getRefTime().getMsec());  size +=8;
		dos.write(getSatID());  size +=1;
		dos.writeInt(getWeek()); size +=4;

		dos.writeInt(getSvAccur()); size +=4;
		dos.writeInt(getSvHealth()); size +=4;

		dos.writeInt(getIode()); size +=4;
		dos.writeInt(getIodc()); size +=4;

		dos.writeDouble(getToc()); size +=8;
		dos.writeDouble(getToe()); size +=8;

		dos.writeDouble(getAf0()); size +=8;
		dos.writeDouble(getAf1()); size +=8;
		dos.writeDouble(getAf2()); size +=8;
		dos.writeDouble(tgd1); size +=8;
		dos.writeDouble(tgd2); size +=8;

		dos.writeDouble(getRootA()); size +=8;
		dos.writeDouble(getE()); size +=8;
		dos.writeDouble(getI0()); size +=8;
		dos.writeDouble(getiDot()); size +=8;
		dos.writeDouble(getOmega()); size +=8;
		dos.writeDouble(getOmega0()); size +=8;

		dos.writeDouble(getOmegaDot()); size +=8;
		dos.writeDouble(getM0()); size +=8;
		dos.writeDouble(getDeltaN()); size +=8;
		dos.writeDouble(getCrc()); size +=8;
		dos.writeDouble(getCrs()); size +=8;
		dos.writeDouble(getCuc()); size +=8;
		dos.writeDouble(getCus()); size +=8;
		dos.writeDouble(getCic()); size +=8;
		dos.writeDouble(getCis()); size +=8;

		dos.writeLong(getFitInt()); size +=8;
		dos.writeDouble(aode); size +=8;

		return size;
	}
	/* (non-Javadoc)
	 * @see org.gogpsproject.Streamable#read(java.io.DataInputStream)
	 */
	@Override
	public void read(DataInputStream dai, boolean oldVersion) throws IOException {
		int v=1;
		if(!oldVersion) v=dai.readInt();

		if(v==1){
			long l = dai.readLong();
			setRefTime(new Time(l>0?l:System.currentTimeMillis()));
			setSatID(dai.read());
			setWeek(dai.readInt());
			setSvAccur(dai.readInt());
			setSvHealth(dai.readInt());
			setIode(dai.readInt());
			setIodc(dai.readInt());
			setToc(dai.readDouble());
			setToe(dai.readDouble());
			setAf0(dai.readDouble());
			setAf1(dai.readDouble());
			setAf2(dai.readDouble());
			tgd1 = dai.readDouble();
			tgd2 = dai.readDouble();
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
			setFitInt(dai.readLong());
			aode = dai.readDouble();
		}else{
			throw new IOException("Unknown format version:"+v);
		}
	}

	public void setGpsWeek(int gpsWeek) {
	}

	public void setIoDE(int iode) {
	}

	public void setDn(double v) {
	}

	public void setSqrtA(double v) {
	}

	public void setSvAccuracy(int svAccuracy) {
	}

	public int getAodc() {
		return aodc;
	}

	public void setAodc(int aodc) {
		this.aodc = aodc;
	}
}