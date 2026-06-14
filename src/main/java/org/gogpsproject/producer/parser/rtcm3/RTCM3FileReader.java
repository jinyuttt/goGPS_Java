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
import java.util.HashMap;
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
	private IonoGps iono = null;
	// TODO support past times, now keep only last broadcast data
	private HashMap<Integer,EphGps> ephsGps = new HashMap<Integer,EphGps>();
	private HashMap<Integer,EphGlo> ephsGlo = new HashMap<Integer,EphGlo>();
	private HashMap<Integer,EphGal> ephsGal = new HashMap<Integer,EphGal>();
	private HashMap<Integer,EphBds> ephsBds = new HashMap<Integer,EphBds>();
	private HashMap<Integer,EphQzs> ephsQzs = new HashMap<Integer,EphQzs>();
	private HashMap<Integer,EphIrnss> ephsIrnss = new HashMap<Integer,EphIrnss>();
	private int week;
	private Coordinates stationCoords = null;  // 基站坐标
	
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
		// 如果有本地存储的基站坐标，返回基站坐标作为初始位置
		if (stationCoords != null && stationCoords.isValidXYZ()) {
			System.out.printf("[DEBUG] 使用本地存储的基站坐标: X=%.4f Y=%.4f Z=%.4f%n",
					stationCoords.getX(), stationCoords.getY(), stationCoords.getZ());
			return stationCoords;
		}
		// 尝试从 RTCM3Client 获取基站位置
		if (reader != null && reader.getMasterPosition() != null) {
			System.out.printf("[DEBUG] 从RTCM3Client获取基站坐标: X=%.4f Y=%.4f Z=%.4f%n",
					reader.getMasterPosition().getX(), reader.getMasterPosition().getY(), reader.getMasterPosition().getZ());
			return reader.getMasterPosition();
		}
		// 否则返回默认值 (0,0,0)
		Coordinates coord = Coordinates.globalXYZInstance(0.0, 0.0, 0.0);
		coord.computeGeodetic();
		return coord;
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
		this.in = new FileInputStream(file);

		this.reader = new RTCM3Client(week);
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
						this.obs = (Observations)o;  // 更新当前观测数据
						return (Observations)o;
					} else if(o instanceof EphGlo){
						// 存储GLONASS星历数据
						EphGlo eph = (EphGlo)o;
						ephsGlo.put(new Integer(eph.getSatID()), eph);
					} else if(o instanceof EphBds){
						// 存储BeiDou星历数据
						EphBds eph = (EphBds)o;
						ephsBds.put(new Integer(eph.getSatID()), eph);
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
						// 存储基站坐标
						stationCoords = (Coordinates) o;
					}
				}
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
						// 临时调试：打印对象类型

						System.out.printf("[DEBUG readNext] 对象类型: %s%n", o.getClass().getName());

						// 存储星历到 HashMap（与 getNextObservations 保持一致）
						// 注意：子类必须放在父类 EphGps 之前判断，否则子类对象会匹配到父类分支
						if (o instanceof EphGlo) {
							EphGlo eph = (EphGlo) o;
							ephsGlo.put(new Integer(eph.getSatID()), eph);
						} else if (o instanceof EphBds) {
							EphBds eph = (EphBds) o;
							System.out.println("当前北斗星历数量: " + ephsBds.size());
							ephsBds.put(new Integer(eph.getSatID()), eph);
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
							// 存储基站坐标
							stationCoords = (Coordinates) o;
							System.out.printf("[DEBUG] 读取到基站坐标: X=%.4f Y=%.4f Z=%.4f%n",
									stationCoords.getX(), stationCoords.getY(), stationCoords.getZ());
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
				EphBds ephBds = ephsBds.get(new Integer(satID));
				if (ephBds != null) {
					// 临时调试：打印星历查找成功
					System.out.printf("[DEBUG] 找到星历: PRN=%d type=%c, rootA=%.0f%n", satID, satType, ephBds.getRootA());
					return computePositionGps(obs, satID, satType, ephBds, receiverClockError);
				} else {
					// 临时调试：打印HashMap大小和key列表
					System.out.printf("[DEBUG] 未找到星历: PRN=%d type=%c, HashMap大小=%d, keys=%s%n", 
							satID, satType, ephsBds.size(), ephsBds.keySet());
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
