/*
 * Copyright (c) 2010 Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
 *
 * This file is part of goGPS Project (goGPS).
 */

package org.gogpsproject.ephemeris;

import java.util.HashMap;

/**
 * SSR (State Space Representation) Corrections Data
 * 
 * Stores SSR orbit, clock, and other corrections for high-precision positioning
 * 
 * @author goGPS Project
 */
public class SsrOrbitClock {

	// SSR消息类型
	private int msgType;
	// 卫星系统 (G=GPS, R=GLONASS, E=Galileo, C=BeiDou, J=QZSS)
	private char system;
	// SSR类型 (1=轨道, 2=时钟, 3=码偏, 4=轨道+时钟, 5=URA, 6=高速时钟)
	private int ssrType;
	
	// 参考时间 (GPS秒)
	private double epochTime;
	// 更新间隔 (秒)
	private double updateInterval;
	// 多消息标志
	private boolean multipleMessage;
	// SSRIOD (Issue of Data SSR)
	private int iodSsr;
	// 服务提供商ID
	private int providerId;
	// 解决方案ID
	private int solutionId;
	
	// 卫星数量
	private int satelliteCount;
	
	// 每颗卫星的改正数
	private HashMap<Integer, SsrSatelliteCorrection> satCorrections = new HashMap<>();
	
	/**
	 * 卫星改正数内部类
	 */
	public static class SsrSatelliteCorrection {
		private int prn;
		// IODE (用于轨道改正)
		private int iode;
		// 轨道改正 [radial, along-track, cross-track] (米)
		private double[] deltaOrbit = new double[3];
		// 时钟改正 [a0, a1, a2] (米)
		private double[] deltaClock = new double[3];
		// URA指数
		private int ura;
		// 高速时钟 (米)
		private double highRateClock;
		// 码偏差 (signal -> bias)
		private HashMap<Integer, Double> codeBiases = new HashMap<>();
		
		public int getPrn() { return prn; }
		public void setPrn(int prn) { this.prn = prn; }
		public int getIode() { return iode; }
		public void setIode(int iode) { this.iode = iode; }
		public double[] getDeltaOrbit() { return deltaOrbit; }
		public void setDeltaOrbit(double[] deltaOrbit) { this.deltaOrbit = deltaOrbit; }
		public double[] getDeltaClock() { return deltaClock; }
		public void setDeltaClock(double[] deltaClock) { this.deltaClock = deltaClock; }
		public int getUra() { return ura; }
		public void setUra(int ura) { this.ura = ura; }
		public double getHighRateClock() { return highRateClock; }
		public void setHighRateClock(double hrClock) { this.highRateClock = hrClock; }
		public HashMap<Integer, Double> getCodeBiases() { return codeBiases; }
		public void addCodeBias(int signal, double bias) { this.codeBiases.put(signal, bias); }
	}
	
	public SsrOrbitClock() {
	}
	
	// ==================== Getters & Setters ====================
	
	public int getMsgType() { return msgType; }
	public void setMsgType(int msgType) { this.msgType = msgType; }
	
	public char getSystem() { return system; }
	public void setSystem(char system) { this.system = system; }
	
	public int getSsrType() { return ssrType; }
	public void setSsrType(int ssrType) { this.ssrType = ssrType; }
	
	public double getEpochTime() { return epochTime; }
	public void setEpochTime(double epochTime) { this.epochTime = epochTime; }
	
	public double getUpdateInterval() { return updateInterval; }
	public void setUpdateInterval(double updateInterval) { this.updateInterval = updateInterval; }
	
	public boolean isMultipleMessage() { return multipleMessage; }
	public void setMultipleMessage(boolean multipleMessage) { this.multipleMessage = multipleMessage; }
	
	public int getIodSsr() { return iodSsr; }
	public void setIodSsr(int iodSsr) { this.iodSsr = iodSsr; }
	
	public int getProviderId() { return providerId; }
	public void setProviderId(int providerId) { this.providerId = providerId; }
	
	public int getSolutionId() { return solutionId; }
	public void setSolutionId(int solutionId) { this.solutionId = solutionId; }
	
	public int getSatelliteCount() { return satelliteCount; }
	public void setSatelliteCount(int satelliteCount) { this.satelliteCount = satelliteCount; }
	
	public HashMap<Integer, SsrSatelliteCorrection> getSatCorrections() { return satCorrections; }
	
	/**
	 * 添加卫星改正数
	 */
	public void addSatelliteCorrection(int prn, SsrSatelliteCorrection satCorr) {
		this.satCorrections.put(prn, satCorr);
	}
	
	/**
	 * 获取指定卫星的改正数
	 */
	public SsrSatelliteCorrection getSatelliteCorrection(int prn) {
		return this.satCorrections.get(prn);
	}
	
	/**
	 * 根据消息类型获取卫星系统
	 */
	public static char getSystemFromMsgType(int msgType) {
		if (msgType >= 1057 && msgType <= 1060) return 'G'; // GPS
		if (msgType >= 1061 && msgType <= 1064) return 'R'; // GLONASS
		if (msgType >= 1065 && msgType <= 1068) return 'E'; // Galileo
		return 'G';
	}
	
	/**
	 * 根据消息类型获取SSR类型
	 */
	public static int getSsrTypeFromMsgType(int msgType) {
		int base = msgType % 10;
		if (base == 1) return 1; // 轨道
		if (base == 2) return 2; // 时钟
		if (base == 3) return 3; // 码偏
		if (base == 4) return 4; // 轨道+时钟
		if (base == 5) return 5; // URA
		if (base == 6) return 6; // 高速时钟
		if (base == 7) return 7; // 组合
		return 0;
	}
	
	/**
	 * 获取更新间隔
	 */
	public static double getUpdateInterval(int udi) {
		switch (udi) {
			case 0: return 1;
			case 1: return 2;
			case 2: return 5;
			case 3: return 10;
			case 4: return 15;
			case 5: return 30;
			case 6: return 60;
			case 7: return 120;
			case 8: return 240;
			case 9: return 300;
			case 10: return 600;
			default: return 1;
		}
	}
}