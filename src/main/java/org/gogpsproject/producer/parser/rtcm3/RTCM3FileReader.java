/*
 * Copyright (c) 2010 Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
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
 */
package org.gogpsproject.producer.parser.rtcm3;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Vector;
import java.util.concurrent.ConcurrentHashMap;

import org.gogpsproject.ephemeris.EphBds;
import org.gogpsproject.ephemeris.EphGal;
import org.gogpsproject.ephemeris.EphGlo;
import org.gogpsproject.ephemeris.EphGps;
import org.gogpsproject.ephemeris.EphIrnss;
import org.gogpsproject.ephemeris.EphQzs;
import org.gogpsproject.ephemeris.EphemerisSystem;
import org.gogpsproject.positioning.Coordinates;
import org.gogpsproject.positioning.SatellitePosition;
import org.gogpsproject.positioning.Time;
import org.gogpsproject.producer.NavigationProducer;
import org.gogpsproject.producer.Observations;
import org.gogpsproject.producer.ObservationsProducer;
import org.gogpsproject.producer.StreamEventListener;
import org.gogpsproject.producer.StreamEventProducer;
import org.gogpsproject.producer.StreamResource;
import org.gogpsproject.producer.parser.IonoGps;

/**
 * <p>
 * Read an RTCM3 file and implement Observation and Navigation producer
 * </p>
 *
 * @author Eugenio Realini GReD srl
 */

public class RTCM3FileReader extends EphemerisSystem implements ObservationsProducer, NavigationProducer, StreamResource, StreamEventProducer {

	private InputStream in;
	private RTCM3Client reader;
	private File file;
	private Observations obs = null;
	/**
	 * 当前历元累积中的 Observations（RTKLIB-style 累积）。
	 * 多条 MSM 消息属于同一历元（时间戳相同）时，合并到此对象，
	 * 直到遇到不同时间戳的消息时才返回完整历元。
	 */
	private Observations pendingObs = null;
	private IonoGps iono = null;
	// 存储每个卫星的星历列表，支持多星历选择
	private HashMap<Integer,EphGps> ephsGps = new HashMap<Integer,EphGps>();
	private HashMap<Integer,EphGlo> ephsGlo = new HashMap<Integer,EphGlo>();
	private HashMap<Integer,EphGal> ephsGal = new HashMap<Integer,EphGal>();
	private HashMap<Integer, List<EphBds>> ephsBds = new HashMap<Integer, List<EphBds>>();  // 存储每个卫星的星历列表
	private HashMap<Integer,EphQzs> ephsQzs = new HashMap<Integer,EphQzs>();
	private HashMap<Integer,EphIrnss> ephsIrnss = new HashMap<Integer,EphIrnss>();
	private int week;
	private Vector<StreamEventListener> streamEventListeners = new Vector<StreamEventListener>();

	public RTCM3FileReader(File file, int week) {
		this.file = file;
		this.week = week;
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.ObservationsProducer#getApproxPosition()
	 */
	@Override
	public Coordinates getDefinedPosition() {
		if (reader != null && reader.getMasterPosition() != null) {
			return reader.getMasterPosition();
		}
		Coordinates c = Coordinates.globalXYZInstance(0.0, 0.0, 0.0);
		c.computeGeodetic();
		return c;
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.ObservationsProducer#getCurrentObservations()
	 */
	@Override
	public Observations getCurrentObservations() {
		if (obs == null) {
			// 如果当前没有观测数据，尝试获取下一个
			getNextObservations();
		}
		return obs;
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.ObservationsProducer#hasMoreObservations()
	 */
	public boolean hasMoreObservations() {
		boolean moreObs = false;
		try {
			moreObs = in.available()>0;
		} catch (IOException e) {
		}
		return moreObs;
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.ObservationsProducer#init()
	 */
	@Override
	public void init() throws Exception {
		scanFileForEphemeris();
		this.in = new FileInputStream(file);
		this.reader = new RTCM3Client(week);
	}
	
	private void scanFileForEphemeris() throws Exception {
		FileInputStream scanIn = new FileInputStream(file);
		RTCM3Client scanReader = new RTCM3Client(week);
		try {
			while (scanIn.available() > 0) {
				int c = scanIn.read();
				if (c == 211) {
					Object o = scanReader.readMessage(scanIn);
					if (o != null) {
						if (o instanceof EphGlo) {
							EphGlo eph = (EphGlo) o;
							ephsGlo.put(new Integer(eph.getSatID()), eph);
						} else if (o instanceof EphBds) {
							EphBds eph = (EphBds) o;
							Integer satId = new Integer(eph.getSatID());
							// 获取卫星的星历列表，如果不存在则创建
							List<EphBds> ephList = ephsBds.get(satId);
							if (ephList == null) {
								ephList = new ArrayList<EphBds>();
								ephsBds.put(satId, ephList);
							}
							ephList.add(eph);
						} else if (o instanceof EphGal) {
							EphGal eph = (EphGal) o;
							ephsGal.put(new Integer(eph.getSatID()), eph);
						} else if (o instanceof EphQzs) {
							EphQzs eph = (EphQzs) o;
							ephsQzs.put(new Integer(eph.getSatID()), eph);
						} else if (o instanceof EphIrnss) {
							EphIrnss eph = (EphIrnss) o;
							ephsIrnss.put(new Integer(eph.getSatID()), eph);
						} else if (o instanceof EphGps) {
							EphGps eph = (EphGps) o;
							ephsGps.put(new Integer(eph.getSatID()), eph);
						}
					}
				}
			}
		} finally {
			scanIn.close();
		}
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.ObservationsProducer#nextObservations()
	 */
	@Override
	public Observations getNextObservations() {
		try{
			while(in.available()>0){
				int c;
				c = in.read();
				if (c == 211) {
					Object o = reader.readMessage(in);
					if(o instanceof Observations){
						Observations newObs = (Observations)o;
						long newTime = newObs.getRefTime().getMsec();

						if (pendingObs == null) {
							// 第一条消息，开始累积
							pendingObs = newObs;
						} else if (newObs.getRefTime() != null
								&& newTime == pendingObs.getRefTime().getMsec()) {
							// 同一历元，合并卫星观测值（RTKLIB-style 累积）
							pendingObs.mergeObservations(newObs);
						} else {
							// 新历元开始，返回当前累积的完整历元，缓存新消息
							this.obs = pendingObs;
							Observations result = pendingObs;
							pendingObs = newObs;
							return result;
						}
					} else if(o instanceof EphGlo){
						// 存储GLONASS星历数据
						EphGlo eph = (EphGlo)o;
						ephsGlo.put(new Integer(eph.getSatID()), eph);
					} else if(o instanceof EphBds){
						// 存储BeiDou星历数据（追加到列表）
						EphBds eph = (EphBds)o;
						Integer satId = new Integer(eph.getSatID());
						List<EphBds> ephList = ephsBds.get(satId);
						if (ephList == null) {
							ephList = new ArrayList<EphBds>();
							ephsBds.put(satId, ephList);
						}
						ephList.add(eph);
					} else if(o instanceof EphGal){
						// 存储Galileo星历数据
						EphGal eph = (EphGal)o;
						ephsGal.put(new Integer(eph.getSatID()), eph);
					} else if(o instanceof EphQzs){
						// 存储QZSS星历数据
						EphQzs eph = (EphQzs)o;
						ephsQzs.put(new Integer(eph.getSatID()), eph);
					} else if(o instanceof EphIrnss){
						// 存储IRNSS星历数据
						EphIrnss eph = (EphIrnss)o;
						ephsIrnss.put(new Integer(eph.getSatID()), eph);
					} else if(o instanceof EphGps){
						// 存储GPS星历数据（必须放在子类之后）
						EphGps eph = (EphGps)o;
						ephsGps.put(new Integer(eph.getSatID()), eph);
					} else if(o instanceof Coordinates){
						// Coordinates already stored in reader.getMasterPosition()
					}
				}
			}
			// 文件结束，返回最后累积的历元
			if (pendingObs != null) {
				this.obs = pendingObs;
				Observations result = pendingObs;
				pendingObs = null;
				return result;
			}
		}catch(IOException e){
			e.printStackTrace();
		}
		this.obs = null;  // 文件结束时清空
		return null;
	}

	/**
	 * 读取下一条解码对象（所有类型均返回）。
	 * 可返回: Observations, EphGps/EphBds/EphGal等, Coordinates, SsrOrbitClock,
	 * RefStationParams, GloCodePhaseBias, 或 null（消息被跳过/无解码器）。
	 */
	public Object readNext() {
		try {
			while (in.available() > 0) {
				int c = in.read();
				if (c == 211) { // 0xD3 preamble
					Object o = reader.readMessage(in);
					if (o != null) {
						// 存储星历到 HashMap（与 getNextObservations 保持一致）
						// 注意：子类必须放在父类 EphGps 之前判断，否则子类对象会匹配到父类分支
						if (o instanceof EphGlo) {
							EphGlo eph = (EphGlo) o;
							ephsGlo.put(new Integer(eph.getSatID()), eph);
						} else if (o instanceof EphBds) {
							EphBds eph = (EphBds) o;
							Integer satId = new Integer(eph.getSatID());
							List<EphBds> ephList = ephsBds.get(satId);
							if (ephList == null) {
								ephList = new ArrayList<EphBds>();
								ephsBds.put(satId, ephList);
							}
							ephList.add(eph);
							System.out.println("当前北斗星历数量: " + ephsBds.size() + ", PRN=" + eph.getSatID() + ", ToC=" + eph.getToc());
						} else if (o instanceof EphGal) {
							EphGal eph = (EphGal) o;
							ephsGal.put(new Integer(eph.getSatID()), eph);
						} else if (o instanceof EphQzs) {
							EphQzs eph = (EphQzs) o;
							ephsQzs.put(new Integer(eph.getSatID()), eph);
						} else if (o instanceof EphIrnss) {
							EphIrnss eph = (EphIrnss) o;
							ephsIrnss.put(new Integer(eph.getSatID()), eph);
						} else if (o instanceof EphGps) {
							EphGps eph = (EphGps) o;
							ephsGps.put(new Integer(eph.getSatID()), eph);
						} else if (o instanceof Coordinates) {
							// Coordinates already stored in reader.getMasterPosition()
						}
						return o;
					}
					// null = 解码失败/截断，继续读下一条
				}
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
		return null;
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.ObservationsProducer#release()
	 */
	@Override
	public void release(boolean waitForThread, long timeoutMs) throws InterruptedException {
		try {
			in.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	public void resetWeekAndTime() {
		if (reader != null) {
			reader.resetWeekAndTime();
		}
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.NavigationProducer#getGpsSatPosition(long, int, double)
	 */
	@Override
	public SatellitePosition getGpsSatPosition(Observations obs, int satID, char satType, double receiverClockError) {
		
		// 根据卫星类型从对应的HashMap中获取星历数据
		switch (satType) {
			case 'G': // GPS
				EphGps ephGps = ephsGps.get(new Integer(satID));
				if (ephGps != null) {
					return computePositionGps(obs, satID, satType, ephGps, receiverClockError);
				}
				break;
			case 'R': // GLONASS
				EphGlo ephGlo = ephsGlo.get(new Integer(satID));
				if (ephGlo != null) {
					// TODO: 实现GLONASS位置计算
					// return computePositionGlo(obs, satID, satType, ephGlo, receiverClockError);
				}
				break;
			case 'E': // Galileo
				EphGal ephGal = ephsGal.get(new Integer(satID));
				if (ephGal != null) {
					// TODO: 实现Galileo位置计算
					// return computePositionGal(obs, satID, satType, ephGal, receiverClockError);
				}
				break;
			case 'C': // BeiDou
				List<EphBds> ephBdsList = ephsBds.get(new Integer(satID));
				if (ephBdsList != null && !ephBdsList.isEmpty()) {
					// 获取观测时间（GPST）
					double obsTimeGPST = obs.getRefTime().getGpsTime();
					
					// 选择与观测时间最接近的星历
					EphBds ephBds = selectBestEphBds(obsTimeGPST, ephBdsList);
					
					if (ephBds != null) {
						return computePositionGps(obs, satID, satType, ephBds, receiverClockError);
					}
				}
				break;
			case 'J': // QZSS
				EphQzs ephQzs = ephsQzs.get(new Integer(satID));
				if (ephQzs != null) {
					// TODO: 实现QZSS位置计算
					// return computePositionQzs(obs, satID, satType, ephQzs, receiverClockError);
				}
				break;
			case 'I': // IRNSS
				EphIrnss ephIrnss = ephsIrnss.get(new Integer(satID));
				if (ephIrnss != null) {
					// TODO: 实现IRNSS位置计算
					// return computePositionIrnss(obs, satID, satType, ephIrnss, receiverClockError);
				}
				break;
		}
		return null;
	}

	/**
	 * 根据观测时间选择最合适的北斗星历
	 * 选择规则：
	 * 1. 使用轨道参考时刻toe匹配（轨道计算的时间基准是toe而非toc）
	 * 2. 星历有效期为参考时刻前后各2小时，取时间差绝对值最小的星历
	 * 
	 * @param obsTimeGPST 观测时间（GPST，单位：秒）
	 * @param ephList 该卫星的星历列表
	 * @return 最合适的星历，如果没有有效星历返回null
	 */
	private EphBds selectBestEphBds(double obsTimeGPST, List<EphBds> ephList) {
		EphBds bestEph = null;
		double minDelta = Double.MAX_VALUE;
		final double MAX_VALID_DELTA = 7200.0; // 星历有效期：前后各2小时
		
		for (EphBds eph : ephList) {
			double toeGPST = eph.getToe(); // 使用轨道参考时刻toe匹配
			
			// 星历有效期为参考时刻前后各2小时，取时间差绝对值最小
			double delta = Math.abs(obsTimeGPST - toeGPST);
			
			if (delta > MAX_VALID_DELTA) {
				continue;
			}
			
			// 选择时间差绝对值最小的星历（精度最高）
			if (delta < minDelta) {
				minDelta = delta;
				bestEph = eph;
			}
		}
		
		return bestEph;
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.NavigationProducer#getIono(long)
	 */
	@Override
	public IonoGps getIono(long unixTime) {
		return iono;
	}
	
	/* (non-Javadoc)
	 * @see org.gogpsproject.StreamEventProducer#addStreamEventListener(org.gogpsproject.StreamEventListener)
	 */
	@Override
	public void addStreamEventListener(StreamEventListener streamEventListener) {
		if(streamEventListener==null) return;
		if(!streamEventListeners.contains(streamEventListener))
			this.streamEventListeners.add(streamEventListener);
		if(this.reader!=null)
			this.reader.addStreamEventListener(streamEventListener);
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.StreamEventProducer#getStreamEventListeners()
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Vector<StreamEventListener> getStreamEventListeners() {
		return (Vector<StreamEventListener>) streamEventListeners.clone();
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.StreamEventProducer#removeStreamEventListener(org.gogpsproject.StreamEventListener)
	 */
	@Override
	public void removeStreamEventListener(
			StreamEventListener streamEventListener) {
		if(streamEventListener==null) return;
		if(streamEventListeners.contains(streamEventListener))
			this.streamEventListeners.remove(streamEventListener);
		this.reader.removeStreamEventListener(streamEventListener);
	}
}