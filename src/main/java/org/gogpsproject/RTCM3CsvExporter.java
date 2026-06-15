/*
 * RTCM3 数据 CSV 导出器
 * 支持5类数据输出：观测数据、广播星历、基准站信息、SSR改正、辅助信息
 */
package org.gogpsproject;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

import org.gogpsproject.ephemeris.*;
import org.gogpsproject.positioning.Coordinates;
import org.gogpsproject.producer.ObservationSet;
import org.gogpsproject.producer.Observations;
import org.gogpsproject.producer.parser.rtcm3.Decode1013Msg.RefStationParams;
import org.gogpsproject.producer.parser.rtcm3.Decode1230Msg.GloCodePhaseBias;
import org.gogpsproject.producer.parser.rtcm3.StationaryAntenna;

/**
 * RTCM3 解析数据 CSV 导出工具类。
 * 
 * 使用方式：
 *   RTCM3CsvExporter exporter = new RTCM3CsvExporter("output_dir");
 *   // 解析过程中收集数据
 *   exporter.addObservation(obs);
 *   exporter.addEphemeris(eph);
 *   exporter.addStationInfo(coords, msgType);
 *   exporter.addSsrCorrection(ssr);
 *   exporter.addAuxiliary(obj);
 *   // 最后一次性导出所有CSV
 *   exporter.exportAll();
 */
public class RTCM3CsvExporter {

	private final String outputDir;

	// ========== 5类数据缓存 ==========
	private final List<ObsRecord> obsRecords = new ArrayList<>();
	private final List<EphRecord> ephRecords = new ArrayList<>();
	private final List<StationRecord> stationRecords = new ArrayList<>();
	private final List<SsrRecord> ssrRecords = new ArrayList<>();
	private final List<AuxRecord> auxRecords = new ArrayList<>();

	// ========== 内部记录类 ==========

	private static class ObsRecord {
		String time;
		int satID;
		char satType;
		double codeC1, codeC2;
		double phase1, phase2;
		float snr1, snr2;
		float doppler1, doppler2;
		double azimuth;    // 方位角（度）
		double elevation;  // 仰角（度）
	}

	private static class EphRecord {
		String className;
		char satType;
		int satID;
		int week;
		double toe, toc;
		int iode, iodc;
		double af0, af1, af2;
		double rootA, eccentricity;
		double i0, iDot;
		double omega, omega0, omegaDot;
		double M0, deltaN;
		double crs, crc, cuc, cus, cic, cis;
		int svHealth;
		String extra; // 系统特有参数
	}

	private static class StationRecord {
		int msgType;
		double x, y, z;
		String description;
	}

	private static class SsrRecord {
		int msgType;
		char system;
		int ssrType;
		double epochTime;
		double updateInterval;
		int satelliteCount;
		int iodSsr;
		int providerId;
		int solutionId;
	}

	private static class AuxRecord {
		String type;
		String content;
	}

	public RTCM3CsvExporter(String outputDir) {
		this.outputDir = outputDir;
		new File(outputDir).mkdirs();
	}

	// ========== 数据收集方法 ==========

	/**
	 * 添加观测数据（MSM / 1001-1004 / 1009-1012）
	 */
	public void addObservation(Observations obs) {
		addObservation(obs, 0, 0); // 默认方位角/仰角为0
	}

	/**
	 * 添加观测数据（带方位角/仰角）
	 */
	public void addObservation(Observations obs, double azimuth, double elevation) {
		if (obs == null) return;
		String timeStr = obs.getRefTime() != null ? obs.getRefTime().toString() : "";
		for (int s = 0; s < obs.getNumSat(); s++) {
			ObservationSet os = obs.getSatByIdx(s);
			if (os == null) continue;
			ObsRecord r = new ObsRecord();
			r.time = timeStr;
			r.satID = os.getSatID();
			r.satType = os.getSatType();
			r.codeC1 = os.getCodeC(0);
			r.codeC2 = os.getCodeC(1);
			r.phase1 = os.getPhaseCycles(0);
			r.phase2 = os.getPhaseCycles(1);
			r.snr1 = os.getSignalStrength(0);
			r.snr2 = os.getSignalStrength(1);
			r.doppler1 = os.getDoppler(0);
			r.doppler2 = os.getDoppler(1);
			r.azimuth = azimuth;
			r.elevation = elevation;
			obsRecords.add(r);
		}
	}

	/**
	 * 添加观测数据（逐颗卫星带方位角/仰角）
	 */
	public void addObservationWithAzEl(Observations obs, int satIdx, double azimuth, double elevation) {
		if (obs == null || satIdx < 0 || satIdx >= obs.getNumSat()) return;
		ObservationSet os = obs.getSatByIdx(satIdx);
		if (os == null) return;
		String timeStr = obs.getRefTime() != null ? obs.getRefTime().toString() : "";
		ObsRecord r = new ObsRecord();
		r.time = timeStr;
		r.satID = os.getSatID();
		r.satType = os.getSatType();
		r.codeC1 = os.getCodeC(0);
		r.codeC2 = os.getCodeC(1);
		r.phase1 = os.getPhaseCycles(0);
		r.phase2 = os.getPhaseCycles(1);
		r.snr1 = os.getSignalStrength(0);
		r.snr2 = os.getSignalStrength(1);
		r.doppler1 = os.getDoppler(0);
		r.doppler2 = os.getDoppler(1);
		r.azimuth = azimuth;
		r.elevation = elevation;
		obsRecords.add(r);
	}

	/**
	 * 添加广播星历数据（1019/1020/1042/1044/1045/1046，支持 GPS/BDS/Galileo/QZS/IRNSS）
	 */
	public void addEphemeris(EphGps eph) {
		if (eph == null) return;
		EphRecord r = new EphRecord();
		r.className = eph.getClass().getSimpleName();
		r.satType = eph.getSatType();
		r.satID = eph.getSatID();
		r.week = eph.getWeek();
		r.toe = eph.getToe();
		r.toc = eph.getToc();
		r.iode = eph.getIode();
		r.iodc = eph.getIodc();
		r.af0 = eph.getAf0();
		r.af1 = eph.getAf1();
		r.af2 = eph.getAf2();
		r.rootA = eph.getRootA();
		r.eccentricity = eph.getE();
		r.i0 = eph.getI0();
		r.iDot = eph.getiDot();
		r.omega = eph.getOmega();
		r.omega0 = eph.getOmega0();
		r.omegaDot = eph.getOmegaDot();
		r.M0 = eph.getM0();
		r.deltaN = eph.getDeltaN();
		r.crs = eph.getCrs();
		r.crc = eph.getCrc();
		r.cuc = eph.getCuc();
		r.cus = eph.getCus();
		r.cic = eph.getCic();
		r.cis = eph.getCis();
		r.svHealth = eph.getSvHealth();

		// 系统特有参数
		StringBuilder extra = new StringBuilder();
		if (eph instanceof EphBds) {
			EphBds bds = (EphBds) eph;
			extra.append(String.format("AODE=%.0f, AODC=%d, TGD1=%.12e, TGD2=%.12e",
					bds.getAode(), bds.getAodc(), bds.getTgd1(), bds.getTgd2()));
		} else if (eph instanceof EphGal) {
			EphGal gal = (EphGal) eph;
			extra.append(String.format("BGD_E5aE1=%.12e, BGD_E5bE1=%.12e, DataSource=%d",
					gal.getBgdE5aE1(), gal.getBgdE5bE1(), gal.getDataSource()));
		}
		r.extra = extra.toString();
		ephRecords.add(r);
	}

	/**
	 * 添加 GLONASS 星历数据（1020）
	 */
	public void addEphemeris(EphGlo glo) {
		if (glo == null) return;
		EphRecord r = new EphRecord();
		r.className = "EphGlo";
		r.satType = glo.getSatType();
		r.satID = glo.getSatID();
		r.week = glo.getWeek();
		r.toe = glo.getToe();
		r.toc = glo.getToc();
		r.iode = 0;  // GLONASS 无 IODE
		r.iodc = 0;  // GLONASS 无 IODC
		r.af0 = glo.getTauN();
		r.af1 = glo.getGammaN();
		r.af2 = 0;   // GLONASS 无 AF2
		r.rootA = 0; // GLONASS 无根号A
		r.eccentricity = 0;
		r.i0 = 0;
		r.iDot = 0;
		r.omega = 0;
		r.omega0 = 0;
		r.omegaDot = 0;
		r.M0 = 0;
		r.deltaN = 0;
		r.crs = 0;
		r.crc = 0;
		r.cuc = 0;
		r.cus = 0;
		r.cic = 0;
		r.cis = 0;
		r.svHealth = (int) glo.getEn();

		// GLONASS 特有参数
		r.extra = String.format("tk=%.1f, tb=%.1f, freq_num=%.0f, Bn=%.6e, strNum=%d",
				glo.gettk(), glo.gettb(), glo.getfreq_num(), glo.getBn(), glo.getStrNum());
		ephRecords.add(r);
	}

	/**
	 * 添加基准站信息（1005/1006/1007/1008/1033）
	 */
	public void addStationInfo(Coordinates coords, int msgType) {
		if (coords == null) return;
		StationRecord r = new StationRecord();
		r.msgType = msgType;
		r.x = coords.getX();
		r.y = coords.getY();
		r.z = coords.getZ();
		r.description = "";
		stationRecords.add(r);
	}

	/**
	 * 添加基准站天线信息（1005/1006/1033等解码出的StationaryAntenna）
	 */
	public void addStationAntenna(StationaryAntenna sa) {
		if (sa == null) return;
		StationRecord r = new StationRecord();
		r.msgType = 0; // 1005/1006
		r.x = sa.getAntennaRefPointX();
		r.y = sa.getAntennaRefPointY();
		r.z = sa.getAntennaRefPointZ();
		r.description = "StationID=" + sa.getStationID()
				+ (sa.getAntennaDescriptor() != null ? ", Ant=" + sa.getAntennaDescriptor() : "")
				+ (sa.getReceiverType() != null ? ", Recv=" + sa.getReceiverType() : "");
		stationRecords.add(r);
	}

	/**
	 * 添加SSR改正信息（1057-1068, 1240-1263）
	 */
	public void addSsrCorrection(SsrOrbitClock ssr) {
		if (ssr == null) return;
		SsrRecord r = new SsrRecord();
		r.msgType = ssr.getMsgType();
		r.system = ssr.getSystem();
		r.ssrType = ssr.getSsrType();
		r.epochTime = ssr.getEpochTime();
		r.updateInterval = ssr.getUpdateInterval();
		r.satelliteCount = ssr.getSatelliteCount();
		r.iodSsr = ssr.getIodSsr();
		r.providerId = ssr.getProviderId();
		r.solutionId = ssr.getSolutionId();
		ssrRecords.add(r);
	}

	/**
	 * 添加辅助信息（1013系统参数, 1230 GLONASS码偏差等）
	 */
	public void addAuxiliary(Object obj) {
		if (obj == null) return;
		AuxRecord r = new AuxRecord();
		if (obj instanceof RefStationParams) {
			RefStationParams p = (RefStationParams) obj;
			r.type = "RefStationParams";
			r.content = String.format("StationID=%d, ReceiverType=%s, ReceiverVer=%s, AntennaType=%s, " +
							"AntennaSerial=%s, Lat=%.7f, Lon=%.7f, Height=%.3f",
					p.getStationId(), p.getReceiverType(), p.getReceiverVersion(),
					p.getAntennaType(), p.getAntennaSerialNumber(),
					p.getApproximateLatitude(), p.getApproximateLongitude(), p.getApproximateHeight());
		} else if (obj instanceof GloCodePhaseBias) {
			GloCodePhaseBias b = (GloCodePhaseBias) obj;
			r.type = "GLONASS_CodePhaseBias";
			r.content = String.format("StationID=%d, L1CA=%.4f, L1P=%.4f, L2CA=%.4f, L2P=%.4f",
					b.getStationId(), b.getBiasL1CA(), b.getBiasL1P(), b.getBiasL2CA(), b.getBiasL2P());
		} else {
			r.type = obj.getClass().getSimpleName();
			r.content = obj.toString();
		}
		auxRecords.add(r);
	}

	// ========== 导出方法 ==========

	/**
	 * 导出全部5类CSV文件
	 */
	public void exportAll() {
		exportObservations();
		exportEphemeris();
		exportStationInfo();
		exportSsrCorrections();
		exportAuxiliary();
	}

	/**
	 * 1. 导出原始观测数据 → observations.csv
	 */
	public void exportObservations() {
		if (obsRecords.isEmpty()) {
			System.out.println("[CSV] 无观测数据，跳过 observations.csv");
			return;
		}
		String path = outputDir + File.separator + "observations.csv";
		try (PrintWriter pw = new PrintWriter(new OutputStreamWriter(
				new FileOutputStream(path), StandardCharsets.UTF_8))) {
			pw.println("Time,PRN,SatType,CodeC1(m),CodeC2(m),Phase1(cyc),Phase2(cyc),SNR1(dBHz),SNR2(dBHz),Doppler1(Hz),Doppler2(Hz),Azimuth(deg),Elevation(deg)");
			for (ObsRecord r : obsRecords) {
				pw.printf("%s,%d,%c,%.4f,%.4f,%.4f,%.4f,%.1f,%.1f,%.4f,%.4f,%.1f,%.1f%n",
						r.time, r.satID, r.satType,
						r.codeC1, r.codeC2, r.phase1, r.phase2,
						r.snr1, r.snr2, r.doppler1, r.doppler2,
						r.azimuth, r.elevation);
			}
			System.out.printf("[CSV] 观测数据: %d 条记录 → %s%n", obsRecords.size(), path);
		} catch (IOException e) {
			System.err.println("[CSV] 导出观测数据失败: " + e.getMessage());
		}
	}

	/**
	 * 2. 导出广播星历 → ephemeris.csv
	 */
	public void exportEphemeris() {
		if (ephRecords.isEmpty()) {
			System.out.println("[CSV] 无星历数据，跳过 ephemeris.csv");
			return;
		}
		String path = outputDir + File.separator + "ephemeris.csv";
		try (PrintWriter pw = new PrintWriter(new OutputStreamWriter(
				new FileOutputStream(path), StandardCharsets.UTF_8))) {
			pw.println("MsgType,SatType,PRN,Week,Toe(s),ToC(s),IODE,IODC," +
					"af0(s),af1(s/s),af2(s/s2)," +
					"sqrtA(m^0.5),e,i0(rad),iDot(rad/s)," +
					"omega(rad),OMEGA0(rad),OMEGA_DOT(rad/s)," +
					"M0(rad),DeltaN(rad/s)," +
					"Crs(m),Crc(m),Cuc(rad),Cus(rad),Cic(rad),Cis(rad)," +
					"SVHealth,Extra");
			for (EphRecord r : ephRecords) {
				pw.printf("%s,%c,%d,%d,%.0f,%.0f,%d,%d," +
								"%.12e,%.12e,%.12e," +
								"%.12e,%.12e,%.12e,%.12e," +
								"%.12e,%.12e,%.12e," +
								"%.12e,%.12e," +
								"%.6f,%.6f,%.12e,%.12e,%.12e,%.12e," +
								"%d,%s%n",
						r.className, r.satType, r.satID, r.week, r.toe, r.toc, r.iode, r.iodc,
						r.af0, r.af1, r.af2,
						r.rootA, r.eccentricity, r.i0, r.iDot,
						r.omega, r.omega0, r.omegaDot,
						r.M0, r.deltaN,
						r.crs, r.crc, r.cuc, r.cus, r.cic, r.cis,
						r.svHealth, r.extra);
			}
			System.out.printf("[CSV] 广播星历: %d 条记录 → %s%n", ephRecords.size(), path);
		} catch (IOException e) {
			System.err.println("[CSV] 导出星历数据失败: " + e.getMessage());
		}
	}

	/**
	 * 3. 导出基准站信息 → station.csv
	 */
	public void exportStationInfo() {
		if (stationRecords.isEmpty()) {
			System.out.println("[CSV] 无基准站数据，跳过 station.csv");
			return;
		}
		String path = outputDir + File.separator + "station.csv";
		try (PrintWriter pw = new PrintWriter(new OutputStreamWriter(
				new FileOutputStream(path), StandardCharsets.UTF_8))) {
			pw.println("MsgType,X(m),Y(m),Z(m),Description");
			for (StationRecord r : stationRecords) {
				pw.printf("%d,%.4f,%.4f,%.4f,%s%n",
						r.msgType, r.x, r.y, r.z, r.description);
			}
			System.out.printf("[CSV] 基准站信息: %d 条记录 → %s%n", stationRecords.size(), path);
		} catch (IOException e) {
			System.err.println("[CSV] 导出基准站数据失败: " + e.getMessage());
		}
	}

	/**
	 * 4. 导出SSR改正信息 → ssr.csv
	 */
	public void exportSsrCorrections() {
		if (ssrRecords.isEmpty()) {
			System.out.println("[CSV] 无SSR数据，跳过 ssr.csv");
			return;
		}
		String path = outputDir + File.separator + "ssr.csv";
		try (PrintWriter pw = new PrintWriter(new OutputStreamWriter(
				new FileOutputStream(path), StandardCharsets.UTF_8))) {
			pw.println("MsgType,System,SSRType,EpochTime(s),UpdateInterval(s),SatCount,IOD_SSR,ProviderID,SolutionID");
			for (SsrRecord r : ssrRecords) {
				pw.printf("%d,%c,%d,%.1f,%.1f,%d,%d,%d,%d%n",
						r.msgType, r.system, r.ssrType, r.epochTime,
						r.updateInterval, r.satelliteCount,
						r.iodSsr, r.providerId, r.solutionId);
			}
			System.out.printf("[CSV] SSR改正: %d 条记录 → %s%n", ssrRecords.size(), path);
		} catch (IOException e) {
			System.err.println("[CSV] 导出SSR数据失败: " + e.getMessage());
		}
	}

	/**
	 * 5. 导出辅助信息 → auxiliary.csv
	 */
	public void exportAuxiliary() {
		if (auxRecords.isEmpty()) {
			System.out.println("[CSV] 无辅助数据，跳过 auxiliary.csv");
			return;
		}
		String path = outputDir + File.separator + "auxiliary.csv";
		try (PrintWriter pw = new PrintWriter(new OutputStreamWriter(
				new FileOutputStream(path), StandardCharsets.UTF_8))) {
			pw.println("Type,Content");
			for (AuxRecord r : auxRecords) {
				pw.printf("%s,%s%n", r.type, r.content);
			}
			System.out.printf("[CSV] 辅助信息: %d 条记录 → %s%n", auxRecords.size(), path);
		} catch (IOException e) {
			System.err.println("[CSV] 导出辅助数据失败: " + e.getMessage());
		}
	}

	// ========== 统计信息 ==========
	public int getObsCount() { return obsRecords.size(); }
	public int getEphCount() { return ephRecords.size(); }
	public int getStationCount() { return stationRecords.size(); }
	public int getSsrCount() { return ssrRecords.size(); }
	public int getAuxCount() { return auxRecords.size(); }
}
