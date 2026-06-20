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
package org.gogpsproject.positioning;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import org.ejml.simple.SimpleMatrix;
import org.gogpsproject.Constants;
import org.gogpsproject.producer.Streamable;

/**
 * <p>
 * Coordinate and reference system tools
 * </p>
 *
 * @author Eugenio Realini, Cryms.com
 */
public class Coordinates implements Streamable{
	private final static int STREAM_V = 1;

	// Global systems
	private SimpleMatrix ecef = null; /* Earth-Centered, Earth-Fixed (X, Y, Z) */
	private SimpleMatrix geod = null; /* Longitude (lam), latitude (phi), height (h) */

	// Local systems (require to specify an origin)
	private SimpleMatrix enu; /* Local coordinates (East, North, Up) */

	// UTM projection
	private double utmEast = Double.NaN;
	private double utmNorth = Double.NaN;
	private int utmZone = 0;
	private boolean utmComputed = false;

	private Time refTime = null;

	protected Coordinates(){
		ecef = new SimpleMatrix(3, 1);
		geod = new SimpleMatrix(3, 1);
		enu = new SimpleMatrix(3, 1);
	}
	
	public static Coordinates readFromStream(DataInputStream dai, boolean oldVersion) throws IOException{
		Coordinates c = new Coordinates();
		c.read(dai, oldVersion);
		return c;
	}

	public static Coordinates globalXYZInstance(double x, double y, double z){
		Coordinates c = new Coordinates();
		//c.ecef = new SimpleMatrix(3, 1);
		c.setXYZ(x, y, z);
		return c;
	}
//	public static Coordinates globalXYZInstance(SimpleMatrix ecef){
//		Coordinates c = new Coordinates();
//		c.ecef = ecef.copy();
//		return c;
//	}
	public static Coordinates globalENUInstance(SimpleMatrix ecef){
		Coordinates c = new Coordinates();
		c.enu = ecef.copy();
		return c;
	}

	public static Coordinates globalGeodInstance( double lat, double lon, double alt ){
		Coordinates c = new Coordinates();
		//c.ecef = new SimpleMatrix(3, 1);
		c.setGeod( lat, lon, alt);
		c.computeECEF();

		if( !c.isValidXYZ() )
			throw new RuntimeException("Invalid ECEF: " + c);
		return c;
	}

	public SimpleMatrix minusXYZ(Coordinates coord){
		return this.ecef.minus(coord.ecef);
	}
	/**
	 * RTKLIB-aligned ecef2pos: convert ECEF (X,Y,Z) to geodetic (lat, lon, h).
	 * Uses iterative algorithm from RTKLIB rtkcmn.c.
	 * geod[0]=lon(deg), geod[1]=lat(deg), geod[2]=h(m)
	 */
	public void computeGeodetic() {
		double X = this.ecef.get(0);
		double Y = this.ecef.get(1);
		double Z = this.ecef.get(2);

		if (this.geod == null) {
			this.geod = new SimpleMatrix(3, 1);
		}

		double e2 = org.gogpsproject.Constants.WGS84_ECCENTRICITY;
		double r2 = X * X + Y * Y;
		double z = Z, zk;
		double v = org.gogpsproject.Constants.WGS84_SEMI_MAJOR_AXIS;

		for (zk = 0.0; Math.abs(z - zk) >= 1E-4;) {
			zk = z;
			double sinp = z / Math.sqrt(r2 + z * z);
			v = org.gogpsproject.Constants.WGS84_SEMI_MAJOR_AXIS / Math.sqrt(1.0 - e2 * sinp * sinp);
			z = Z + v * e2 * sinp;
		}

		double lat = r2 > 1E-12 ? Math.atan(z / Math.sqrt(r2)) : (Z > 0.0 ? Math.PI / 2.0 : -Math.PI / 2.0);
		double lon = r2 > 1E-12 ? Math.atan2(Y, X) : 0.0;
		double h = Math.sqrt(r2 + z * z) - v;

		this.geod.set(0, 0, Math.toDegrees(lon));
		this.geod.set(1, 0, Math.toDegrees(lat));
		this.geod.set(2, 0, h);
	}

	/**
	 * RTKLIB-aligned pos2ecef: convert geodetic (lat, lon, h) to ECEF (X,Y,Z).
	 * geod[0]=lon(deg), geod[1]=lat(deg), geod[2]=h(m)
	 */
	public void computeECEF() {
		double lat = Math.toRadians(this.geod.get(1));
		double lon = Math.toRadians(this.geod.get(0));
		double h = this.geod.get(2);

		double sinp = Math.sin(lat);
		double cosp = Math.cos(lat);
		double sinl = Math.sin(lon);
		double cosl = Math.cos(lon);

		double e2 = org.gogpsproject.Constants.WGS84_ECCENTRICITY;
		double v = org.gogpsproject.Constants.WGS84_SEMI_MAJOR_AXIS / Math.sqrt(1.0 - e2 * sinp * sinp);

		this.ecef.set(0, 0, (v + h) * cosp * cosl);
		this.ecef.set(1, 0, (v + h) * cosp * sinl);
		this.ecef.set(2, 0, (v * (1.0 - e2) + h) * sinp);
	}

	/**
	 * @param origin
	 * @return Local (ENU) coordinates
	 */
	public void computeLocal(Coordinates target) {
		if(this.geod==null) computeGeodetic();

		SimpleMatrix R = rotationMatrix(this);

		enu = R.mult(target.minusXYZ(this));

	}

	public double getGeodeticLongitude(){
		if(this.geod==null) computeGeodetic();
		return this.geod.get(0);
	}
	public double getGeodeticLatitude(){
		if(this.geod==null) computeGeodetic();
		return this.geod.get(1);
	}
	public double getGeodeticHeight(){
		if(this.geod==null) computeGeodetic();
		return this.geod.get(2);
	}
	public double getX(){
		return ecef.get(0);
	}
	public double getY(){
		return ecef.get(1);
	}
	public double getZ(){
		return ecef.get(2);
	}

	public void setENU(double e, double n, double u){
		this.enu.set(0, 0, e);
		this.enu.set(1, 0, n);
		this.enu.set(2, 0, u);
	}
	public double getE(){
		return enu.get(0);
	}
	public double getN(){
		return enu.get(1);
	}
	public double getU(){
		return enu.get(2);
	}


	public void setXYZ(double x, double y, double z){
		//if(this.ecef==null) this.ecef = new SimpleMatrix(3, 1);
		this.ecef.set(0, 0, x);
		this.ecef.set(1, 0, y);
		this.ecef.set(2, 0, z);
		// 清除 geodetic 缓存，以便下次需要时重新计算
		this.geod = null;
		this.utmComputed = false;
	}
	public void setGeod( double lat, double lon, double alt ){
		//if(this.ecef==null) this.ecef = new SimpleMatrix(3, 1);
		this.geod.set(1, 0, lat);
		this.geod.set(0, 0, lon);
		this.geod.set(2, 0, alt);
	}
	public void setPlusXYZ(SimpleMatrix sm){
		this.ecef.setTo(ecef.plus(sm));
		this.utmComputed = false;
	}
	public void setSMMultXYZ(SimpleMatrix sm){
		this.ecef = sm.mult(this.ecef);
		this.utmComputed = false;
	}

	public boolean isValidXYZ(){
		if (this.ecef == null) return false;
		
		double x = this.ecef.get(0);
		double y = this.ecef.get(1);
		double z = this.ecef.get(2);
		
		// 检查是否为 NaN 或 Inf
		if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(z)) return false;
		if (Double.isInfinite(x) || Double.isInfinite(y) || Double.isInfinite(z)) return false;
		
		// 检查是否全为零
		if (x == 0 && y == 0 && z == 0) return false;
		
		// 检查坐标是否在合理范围内（距离地球中心不超过 50,000 km）
		double r = Math.sqrt(x*x + y*y + z*z);
		if (r > 5.0E7 || r < 6.0E6) return false; // 合理范围: 6,000 km - 50,000 km
		
		return true;
	}

	public Object clone(){
		Coordinates c = new Coordinates();
		cloneInto(c);
		return c;
	}

	public void cloneInto(Coordinates c){
		c.ecef = this.ecef.copy();
		c.enu = this.enu.copy();
		c.geod = this.geod != null ? this.geod.copy() : null;
		c.utmEast = this.utmEast;
		c.utmNorth = this.utmNorth;
		c.utmZone = this.utmZone;
		c.utmComputed = this.utmComputed;

		if(refTime!=null) c.refTime = (Time)refTime.clone();
	}
	/**
	 * @param origin
	 * @return Rotation matrix used to switch from global to local reference systems (and vice-versa)
	 */
	public static SimpleMatrix rotationMatrix(Coordinates origin) {

		double lam = Math.toRadians(origin.getGeodeticLongitude());
		double phi = Math.toRadians(origin.getGeodeticLatitude());

		double cosLam = Math.cos(lam);
		double cosPhi = Math.cos(phi);
		double sinLam = Math.sin(lam);
		double sinPhi = Math.sin(phi);

		double[][] data = new double[3][3];
		data[0][0] = -sinLam;
		data[0][1] = cosLam;
		data[0][2] = 0;
		data[1][0] = -sinPhi * cosLam;
		data[1][1] = -sinPhi * sinLam;
		data[1][2] = cosPhi;
		data[2][0] = cosPhi * cosLam;
		data[2][1] = cosPhi * sinLam;
		data[2][2] = sinPhi;

		SimpleMatrix R = new SimpleMatrix(data);

		return R;
	}

	/**
	 * RTKLIB ecef2enu: transform ECEF vector to local ENU coordinates.
	 * @param pos  geodetic position {lat, lon} (rad)
	 * @param r    vector in ECEF {x, y, z}
	 * @return     vector in local ENU {e, n, u}
	 */
	public static double[] ecef2enu(double[] pos, double[] r) {
		double sinp = Math.sin(pos[0]), cosp = Math.cos(pos[0]);
		double sinl = Math.sin(pos[1]), cosl = Math.cos(pos[1]);

		double e = -sinl * r[0] + cosl * r[1];
		double n = -sinp * cosl * r[0] - sinp * sinl * r[1] + cosp * r[2];
		double u = cosp * cosl * r[0] + cosp * sinl * r[1] + sinp * r[2];

		return new double[] {e, n, u};
	}

	/**
	 * RTKLIB enu2ecef: transform local ENU vector to ECEF coordinates.
	 * @param pos  geodetic position {lat, lon} (rad)
	 * @param e    vector in local ENU {e, n, u}
	 * @return     vector in ECEF {x, y, z}
	 */
	public static double[] enu2ecef(double[] pos, double[] e) {
		double sinp = Math.sin(pos[0]), cosp = Math.cos(pos[0]);
		double sinl = Math.sin(pos[1]), cosl = Math.cos(pos[1]);

		double x = -sinl * e[0] - sinp * cosl * e[1] + cosp * cosl * e[2];
		double y = cosl * e[0] - sinp * sinl * e[1] + cosp * sinl * e[2];
		double z = 0.0 * e[0] + cosp * e[1] + sinp * e[2];

		return new double[] {x, y, z};
	}

	/**
	 * RTKLIB covenu: transform ECEF covariance to local ENU covariance.
	 * Q = E * P * E^T
	 * @param pos  geodetic position {lat, lon} (rad)
	 * @param P    covariance in ECEF (3x3 SimpleMatrix)
	 * @return     covariance in local ENU (3x3 SimpleMatrix)
	 */
	public static SimpleMatrix covenu(double[] pos, SimpleMatrix P) {
		double sinp = Math.sin(pos[0]), cosp = Math.cos(pos[0]);
		double sinl = Math.sin(pos[1]), cosl = Math.cos(pos[1]);

		double[][] E = new double[3][3];
		E[0][0] = -sinl;       E[0][1] = cosl;        E[0][2] = 0.0;
		E[1][0] = -sinp * cosl; E[1][1] = -sinp * sinl; E[1][2] = cosp;
		E[2][0] = cosp * cosl;  E[2][1] = cosp * sinl;  E[2][2] = sinp;

		SimpleMatrix R = new SimpleMatrix(E);
		SimpleMatrix EP = R.mult(P);
		return EP.mult(R.transpose());
	}

	/**
	 * RTKLIB covecef: transform local ENU covariance to ECEF covariance.
	 * P = E^T * Q * E
	 * @param pos  geodetic position {lat, lon} (rad)
	 * @param Q    covariance in local ENU (3x3 SimpleMatrix)
	 * @return     covariance in ECEF (3x3 SimpleMatrix)
	 */
	public static SimpleMatrix covecef(double[] pos, SimpleMatrix Q) {
		double sinp = Math.sin(pos[0]), cosp = Math.cos(pos[0]);
		double sinl = Math.sin(pos[1]), cosl = Math.cos(pos[1]);

		double[][] E = new double[3][3];
		E[0][0] = -sinl;       E[0][1] = cosl;        E[0][2] = 0.0;
		E[1][0] = -sinp * cosl; E[1][1] = -sinp * sinl; E[1][2] = cosp;
		E[2][0] = cosp * cosl;  E[2][1] = cosp * sinl;  E[2][2] = sinp;

		SimpleMatrix R = new SimpleMatrix(E);
		SimpleMatrix EQ = R.transpose().mult(Q);
		return EQ.mult(R);
	}

	/**
	 * @return the refTime
	 */
	public Time getRefTime() {
		return refTime;
	}

	/**
	 * @param refTime the refTime to set
	 */
	public void setRefTime(Time refTime) {
		this.refTime = refTime;
	}

	public int write(DataOutputStream dos) throws IOException{
		int size=0;
		dos.writeUTF(MESSAGE_COORDINATES); size+=5;// 5
		dos.writeInt(STREAM_V); size+=4; // 4

		dos.writeLong(refTime==null?-1:refTime.getMsec()); size+=8; // 8

		for(int i=0;i<3;i++){
			dos.writeDouble(ecef.get(i));  size+=8;
		}
		for(int i=0;i<3;i++){
			dos.writeDouble(enu.get(i));  size+=8;
		}
		for(int i=0;i<3;i++){
			dos.writeDouble(geod.get(i));  size+=8;
		}

		return size;
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.Streamable#read(java.io.DataInputStream)
	 */
	@Override
	public void read(DataInputStream dai, boolean oldVersion) throws IOException {
		int v = dai.readInt();

		if(v == 1){
			long l = dai.readLong();
			refTime = l==-1?null:new Time(l);
			for(int i=0;i<3;i++){
				ecef.set(i, dai.readDouble());
			}
			for(int i=0;i<3;i++){
				enu.set(i, dai.readDouble());
			}
			for(int i=0;i<3;i++){
				geod.set(i, dai.readDouble());
			}
		}else{
			throw new IOException("Unknown format version:"+v);
		}



	}

	/**
	 * WGS84经纬度 → UTM投影转换
	 * 使用标准横轴墨卡托投影公式
	 */
	public void computeUTM() {
		if (utmComputed) return;

		double lat = getGeodeticLatitude();
		double lon = getGeodeticLongitude();

		// WGS84椭球参数
		double a = 6378137.0;
		double f = 1.0 / 298.257223563;
		double k0 = 0.9996;
		double e = Math.sqrt(2 * f - f * f);
		double e2 = (e * e) / (1 - e * e);

		// UTM带号
		utmZone = (int) Math.floor((lon + 180) / 6) + 1;
		if (utmZone < 1) utmZone = 1;
		if (utmZone > 60) utmZone = 60;

		// 中央子午线
		double lon0 = Math.toRadians(utmZone * 6 - 183);

		double latRad = Math.toRadians(lat);
		double lonRad = Math.toRadians(lon);

		double sinLat = Math.sin(latRad);
		double cosLat = Math.cos(latRad);
		double tanLat = Math.tan(latRad);

		double N = a / Math.sqrt(1 - e * e * sinLat * sinLat);
		double T = tanLat * tanLat;
		double C = e2 * cosLat * cosLat;
		double A = cosLat * (lonRad - lon0);

		// 子午线弧长
		double M = a * ((1 - e * e / 4 - 3 * e * e * e * e / 64 - 5 * e * e * e * e * e * e / 256) * latRad
				- (3 * e * e / 8 + 3 * e * e * e * e / 32 + 45 * e * e * e * e * e * e / 1024) * Math.sin(2 * latRad)
				+ (15 * e * e * e * e / 256 + 45 * e * e * e * e * e * e / 1024) * Math.sin(4 * latRad)
				- (35 * e * e * e * e * e * e / 3072) * Math.sin(6 * latRad));

		// 东向
		utmEast = k0 * N * (A + (1 - T + C) * A * A * A / 6
				+ (5 - 18 * T + T * T + 72 * C - 58 * e2) * A * A * A * A * A / 120) + 500000;

		// 北向
		utmNorth = k0 * (M + N * tanLat * (A * A / 2
				+ (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24
				+ (61 - 58 * T + T * T + 600 * C - 330 * e2) * A * A * A * A * A * A / 720));

		// 南半球加10000km偏移
		if (lat < 0) {
			utmNorth += 10000000;
		}

		utmComputed = true;
	}

	public double getUTMEasting() {
		if (!utmComputed) computeUTM();
		return utmEast;
	}

	public double getUTMNorthing() {
		if (!utmComputed) computeUTM();
		return utmNorth;
	}

	public int getUTMZone() {
		if (!utmComputed) computeUTM();
		return utmZone;
	}

	public String toString(){
		String lineBreak = System.getProperty("line.separator");

		String out= String.format( "Coord ECEF: X:"+getX()+" Y:"+getY()+" Z:"+getZ()+lineBreak +
		"       ENU: E:"+getE()+" N:"+getN()+" U:"+getU()+lineBreak +
		"      GEOD: Lon:"+getGeodeticLongitude()+" Lat:"+getGeodeticLatitude()+" H:"+getGeodeticHeight()+lineBreak +
		"      UTM: Zone:"+getUTMZone()+" E:"+getUTMEasting()+" N:"+getUTMNorthing()+lineBreak +
		"      http://maps.google.com?q=%3.4f,%3.4f" + lineBreak, getGeodeticLatitude(), getGeodeticLongitude() );
		return out;
	}
}