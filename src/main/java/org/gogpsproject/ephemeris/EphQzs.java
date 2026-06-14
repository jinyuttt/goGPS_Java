/*
 * Copyright (c) 2010, Eugenio Realini, Mirko Reguzzoni, Cryms sagl, Daisuke Yoshida. All Rights Reserved.
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
 * QZSS broadcast ephemerides
 * </p>
 *
 * @author Eugenio Realini, Cryms.com, Daisuke Yoshida 
 */

public class EphQzs extends EphGps {
	private final static int STREAM_V = 1;

	// QZSS specific parameters (inherits most from EphGps)
	private int L2Code; /* Code on L2 */
	private int L2Flag; /* L2 P data flag */
	
	/* for GLONASS data */
	// These fields are inherited from EphGps

	public EphQzs(){
		
	}
	public EphQzs(DataInputStream dai, boolean oldVersion) throws IOException{
		read(dai,oldVersion);
	}

	/**
	 * @return the refTime
	 */
	public Time getRefTime() {
		return super.getRefTime();
	}
	/**
	 * @param refTime the refTime to set
	 */
	public void setRefTime(Time refTime) {
		super.setRefTime(refTime);
	}
	/**
	 * @return the satType
	 */
	public char getSatType() {
		return super.getSatType();
	}
	/**
	 * @param satType the satType to set
	 */
	public void setSatType(char satType) {
		super.setSatType(satType);
	}
	/**
	 * @return the satID
	 */
	public int getSatID() {
		return super.getSatID();
	}
	/**
	 * @param satID the satID to set
	 */
	public void setSatID(int satID) {
		super.setSatID(satID);
	}
	/**
	 * @return the week
	 */
	public int getWeek() {
		return super.getWeek();
	}
	/**
	 * @param week the week to set
	 */
	public void setWeek(int week) {
		super.setWeek(week);
	}
	/**
	 * @return the l2Code
	 */
	public int getL2Code() {
		return L2Code;
	}
	/**
	 * @param l2Code the l2Code to set
	 */
	public void setL2Code(int l2Code) {
		L2Code = l2Code;
	}
	/**
	 * @return the l2Flag
	 */
	public int getL2Flag() {
		return L2Flag;
	}
	/**
	 * @param l2Flag the l2Flag to set
	 */
	public void setL2Flag(int l2Flag) {
		L2Flag = l2Flag;
	}
	/**
	 * @return the svAccur
	 */
	public int getSvAccur() {
		return super.getSvAccur();
	}
	/**
	 * @param svAccur the svAccur to set
	 */
	public void setSvAccur(int svAccur) {
		super.setSvAccur(svAccur);
	}
	/**
	 * @return the svHealth
	 */
	public int getSvHealth() {
		return super.getSvHealth();
	}
	/**
	 * @param svHealth the svHealth to set
	 */
	public void setSvHealth(int svHealth) {
		super.setSvHealth(svHealth);
	}
	/**
	 * @return the iode
	 */
	public int getIode() {
		return super.getIode();
	}
	/**
	 * @param iode the iode to set
	 */
	public void setIode(int iode) {
		super.setIode(iode);
	}
	/**
	 * @return the iodc
	 */
	public int getIodc() {
		return super.getIodc();
	}
	/**
	 * @param iodc the iodc to set
	 */
	public void setIodc(int iodc) {
		super.setIodc(iodc);
	}
	/**
	 * @return the toc
	 */
	public double getToc() {
		return super.getToc();
	}
	/**
	 * @param toc the toc to set
	 */
	public void setToc(double toc) {
		super.setToc(toc);
	}
	/**
	 * @return the toe
	 */
	public double getToe() {
		return super.getToe();
	}
	/**
	 * @param toe the toe to set
	 */
	public void setToe(double toe) {
		super.setToe(toe);
	}
	/**
	 * @return the tom
	 */
	public double getTom() {
		return super.getTom();
	}
	/**
	 * @param tom the tom to set
	 */
	public void setTom(double tom) {
		super.setTom(tom);
	}
	/**
	 * @return the af0
	 */
	public double getAf0() {
		return super.getAf0();
	}
	/**
	 * @param af0 the af0 to set
	 */
	public void setAf0(double af0) {
		super.setAf0(af0);
	}
	/**
	 * @return the af1
	 */
	public double getAf1() {
		return super.getAf1();
	}
	/**
	 * @param af1 the af1 to set
	 */
	public void setAf1(double af1) {
		super.setAf1(af1);
	}
	/**
	 * @return the af2
	 */
	public double getAf2() {
		return super.getAf2();
	}
	/**
	 * @param af2 the af2 to set
	 */
	public void setAf2(double af2) {
		super.setAf2(af2);
	}
	/**
	 * @return the tgd
	 */
	public double getTgd() {
		return super.getTgd();
	}
	/**
	 * @param tgd the tgd to set
	 */
	public void setTgd(double tgd) {
		super.setTgd(tgd);
	}
	/**
	 * @return the rootA
	 */
	public double getRootA() {
		return super.getRootA();
	}
	/**
	 * @param rootA the rootA to set
	 */
	public void setRootA(double rootA) {
		super.setRootA(rootA);
	}
	/**
	 * @return the e
	 */
	public double getE() {
		return super.getE();
	}
	/**
	 * @param e the e to set
	 */
	public void setE(double e) {
		super.setE(e);
	}
	/**
	 * @return the i0
	 */
	public double getI0() {
		return super.getI0();
	}
	/**
	 * @param i0 the i0 to set
	 */
	public void setI0(double i0) {
		super.setI0(i0);
	}
	/**
	 * @return the iDot
	 */
	public double getiDot() {
		return super.getiDot();
	}
	/**
	 * @param iDot the iDot to set
	 */
	public void setiDot(double iDot) {
		super.setiDot(iDot);
	}
	/**
	 * @return the omega
	 */
	public double getOmega() {
		return super.getOmega();
	}
	/**
	 * @param omega the omega to set
	 */
	public void setOmega(double omega) {
		super.setOmega(omega);
	}
	/**
	 * @return the omega0
	 */
	public double getOmega0() {
		return super.getOmega0();
	}
	/**
	 * @param omega0 the omega0 to set
	 */
	public void setOmega0(double omega0) {
		super.setOmega0(omega0);
	}
	/**
	 * @return the omegaDot
	 */
	public double getOmegaDot() {
		return super.getOmegaDot();
	}
	/**
	 * @param omegaDot the omegaDot to set
	 */
	public void setOmegaDot(double omegaDot) {
		super.setOmegaDot(omegaDot);
	}
	/**
	 * @return the m0
	 */
	public double getM0() {
		return super.getM0();
	}
	/**
	 * @param m0 the m0 to set
	 */
	public void setM0(double m0) {
		super.setM0(m0);
	}
	/**
	 * @return the deltaN
	 */
	public double getDeltaN() {
		return super.getDeltaN();
	}
	/**
	 * @param deltaN the deltaN to set
	 */
	public void setDeltaN(double deltaN) {
		super.setDeltaN(deltaN);
	}
	/**
	 * @return the crc
	 */
	public double getCrc() {
		return super.getCrc();
	}
	/**
	 * @param crc the crc to set
	 */
	public void setCrc(double crc) {
		super.setCrc(crc);
	}
	/**
	 * @return the crs
	 */
	public double getCrs() {
		return super.getCrs();
	}
	/**
	 * @param crs the crs to set
	 */
	public void setCrs(double crs) {
		super.setCrs(crs);
	}
	/**
	 * @return the cuc
	 */
	public double getCuc() {
		return super.getCuc();
	}
	/**
	 * @param cuc the cuc to set
	 */
	public void setCuc(double cuc) {
		super.setCuc(cuc);
	}
	/**
	 * @return the cus
	 */
	public double getCus() {
		return super.getCus();
	}
	/**
	 * @param cus the cus to set
	 */
	public void setCus(double cus) {
		super.setCus(cus);
	}
	/**
	 * @return the cic
	 */
	public double getCic() {
		return super.getCic();
	}
	/**
	 * @param cic the cic to set
	 */
	public void setCic(double cic) {
		super.setCic(cic);
	}
	/**
	 * @return the cis
	 */
	public double getCis() {
		return super.getCis();
	}
	/**
	 * @param cis the cis to set
	 */
	public void setCis(double cis) {
		super.setCis(cis);
	}
	/**
	 * @return the fitInt
	 */
	public long getFitInt() {
		return super.getFitInt();
	}
	/**
	 * @param fitInt the fitInt to set
	 */
	public void setFitInt(long fitInt) {
		super.setFitInt(fitInt);
	}
	
	
	/* for GLONASS data */

	public float getTow() {
		return super.getTow();
	}
	public void setTow(float tow) {
		super.setTow(tow);
	}
	public double getTauN() {
		return super.getTauN();
	}
	public void setTauN(double tauN) {
		super.setTauN(tauN);
	}

	public double getGammaN() {
		return super.getGammaN();
	}
	public void setGammaN(double gammaN) {
		super.setGammaN(gammaN);
	}

	public double gettk() {
		return super.gettk();
	}
	public void settk(double tk) {
		super.settk(tk);
	}

	public double getX() {
		return super.getX();
	}
	public void setX(double X) {
		super.setX(X);
	}

	public double getXv() {
		return super.getXv();
	}
	public void setXv(double Xv) {
		super.setXv(Xv);
	}

	public double getXa() {
		return super.getXa();
	}
	public void setXa(double Xa) {
		super.setXa(Xa);
	}

	public double getBn() {
		return super.getBn();
	}
	public void setBn(double Bn) {
		super.setBn(Bn);
	}

	public double getY() {
		return super.getY();
	}
	public void setY(double Y) {
		super.setY(Y);
	}

	public double getYv() {
		return super.getYv();
	}
	public void setYv(double Yv) {
		super.setYv(Yv);
	}

	public double getYa() {
		return super.getYa();
	}
	public void setYa(double Ya) {
		super.setYa(Ya);
	}

	public int getfreq_num() {
		return super.getfreq_num();
	}
	public void setfreq_num(int freq_num) {
		super.setfreq_num(freq_num);
	}

	public double gettb() {
		return super.gettb();
	}
	public void settb(double tb) {
		super.settb(tb);
	}

	public double getZ() {
		return super.getZ();
	}
	public void setZ(double Z) {
		super.setZ(Z);
	}

	public double getZv() {
		return super.getZv();
	}
	public void setZv(double Zv) {
		super.setZv(Zv);
	}

	public double getZa() {
		return super.getZa();
	}
	public void setZa(double Za) {
		super.setZa(Za);
	}

	public double getEn() {
		return super.getEn();
	}
	public void setEn(double En) {
		super.setEn(En);
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

		dos.writeInt(L2Code); size +=4;
		dos.writeInt(L2Flag); size +=4;

		dos.writeInt(getSvAccur()); size +=4;
		dos.writeInt(getSvHealth()); size +=4;

		dos.writeInt(getIode()); size +=4;
		dos.writeInt(getIodc()); size +=4;

		dos.writeDouble(getToc()); size +=8;
		dos.writeDouble(getToe()); size +=8;

		dos.writeDouble(getAf0()); size +=8;
		dos.writeDouble(getAf1()); size +=8;
		dos.writeDouble(getAf2()); size +=8;
		dos.writeDouble(getTgd()); size +=8;


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
			L2Code = dai.readInt();
			L2Flag = dai.readInt();
			setSvAccur(dai.readInt());
			setSvHealth(dai.readInt());
			setIode(dai.readInt());
			setIodc(dai.readInt());
			setToc(dai.readDouble());
			setToe(dai.readDouble());
			setAf0(dai.readDouble());
			setAf1(dai.readDouble());
			setAf2(dai.readDouble());
			setTgd(dai.readDouble());
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
		}else{
			throw new IOException("Unknown format version:"+v);
		}
	}

}
