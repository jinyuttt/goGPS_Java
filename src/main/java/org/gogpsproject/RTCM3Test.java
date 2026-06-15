/*
 * RTCM3文件解析测试程序
 * 解析RTCM3文件并输出5类CSV：观测数据、广播星历、基准站信息、SSR改正、辅助信息
 */
package org.gogpsproject;

import java.io.File;
import java.util.HashMap;

import org.gogpsproject.ephemeris.EphGps;
import org.gogpsproject.ephemeris.SsrOrbitClock;
import org.gogpsproject.positioning.Coordinates;
import org.gogpsproject.positioning.SatellitePosition;
import org.gogpsproject.positioning.TopocentricCoordinates;
import org.gogpsproject.producer.ObservationSet;
import org.gogpsproject.producer.Observations;
import org.gogpsproject.producer.parser.rtcm3.Decode1013Msg.RefStationParams;
import org.gogpsproject.producer.parser.rtcm3.Decode1230Msg.GloCodePhaseBias;
import org.gogpsproject.producer.parser.rtcm3.RTCM3FileReader;
import org.gogpsproject.producer.parser.rtcm3.StationaryAntenna;

public class RTCM3Test {

    public static void main(String[] args) {
        int week = 2422; // GPS周: 2026年6月8日对应GPS Week 2422

        if (args.length > 0) {
            testFile(args[0], week);
        } else {
            String desktop = System.getProperty("user.home") + "\\Desktop\\1.rtcm3";
            System.out.println("解析文件: " + desktop);
            testFile(desktop, week);
        }
    }

    private static void testFile(String filename, int week) {
        File file = new File(filename);
        if (!file.exists()) {
            System.out.println("文件不存在: " + filename);
            return;
        }

        // 创建CSV导出器，输出到文件所在目录的 csv_output 子目录
        String csvDir = file.getParent() + File.separator + "csv_output";
        RTCM3CsvExporter exporter = new RTCM3CsvExporter(csvDir);

        RTCM3FileReader reader = new RTCM3FileReader(file, week);
        try {
            reader.init();
        } catch (Exception e) {
            System.err.println("初始化失败: " + e.getMessage());
            return;
        }

        int obsCount = 0;
        int totalSatCount = 0;  // 观测中的卫星总数
        int ephCount = 0;
        int stationCount = 0;
        int ssrCount = 0;
        int auxCount = 0;
        int nullCount = 0;      // 被跳过(null)的消息数
        int otherCount = 0;     // 未识别的消息数
        HashMap<String, Integer> ephTypeCount = new HashMap<>(); // 星历类型统计
        Coordinates stationCoords = null;  // 基准站坐标（用于计算方位角/仰角）

        System.out.println("\n========== RTCM3 文件解析 ==========");
        System.out.println("文件: " + filename);
        System.out.println("CSV输出目录: " + csvDir);
        System.out.println();

        try {
            while (reader.hasMoreObservations()) {
                Object obj = reader.readNext();
                if (obj == null) {
                    nullCount++;
                    // 显示被跳过的消息（可能是截断的星历）
                    if (nullCount <= 5) {
                        System.out.printf("[跳过] 消息 %d (可能截断或格式错误)%n", nullCount);
                    }
                    continue;
                }

                // 1. 原始观测数据 (MSM / 1001-1004 / 1009-1012)
                if (obj instanceof Observations) {
                    Observations obs = (Observations) obj;
                    obsCount++;
                    totalSatCount += obs.getNumSat();

                    // 控制台打印摘要（仅打印前3个历元）
                    if (obsCount <= 3) {
                        System.out.printf("观测: %s 卫星数=%d%n", obs.getRefTime(), obs.getNumSat());
                        for (int s = 0; s < obs.getNumSat(); s++) {
                            ObservationSet os = obs.getSatByIdx(s);
                            if (os == null) continue;
                            StringBuilder sb = new StringBuilder();
                            sb.append(String.format("  PRN=%2d type=%c", os.getSatID(), os.getSatType()));
                            
                            double az = 0, el = 0;
                            // 计算方位角/仰角
                            if (stationCoords != null) {
                                try {
                                    SatellitePosition satPos = reader.getGpsSatPosition(obs, os.getSatID(), os.getSatType(), 0);
                                    if (satPos != null) {
                                        TopocentricCoordinates topo = new TopocentricCoordinates();
                                        topo.computeTopocentric(stationCoords, satPos);
                                        az = topo.getAzimuth();
                                        el = topo.getElevation();
                                        sb.append(String.format("  Az=%.1f El=%.1f", az, el));
                                    } else {
                                        sb.append(String.format("  [星历未找到 PRN=%d type=%c]", os.getSatID(), os.getSatType()));
                                    }
                                } catch (Exception e) {
                                    sb.append(String.format("  [计算失败: %s]", e.getMessage()));
                                }
                            } else {
                                sb.append("  [基准站坐标未设置]");
                            }
                            
                            for (int f = 0; f < 2; f++) {
                                float snr = os.getSignalStrength(f);
                                if (!Float.isNaN(snr)) {
                                    sb.append(String.format("  SNR[%d]=%.1f dBHz", f, snr));
                                }
                            }
                            System.out.println(sb.toString());
                            
                            // 导出到CSV（逐颗卫星带方位角/仰角）
                            exporter.addObservationWithAzEl(obs, s, az, el);
                        }
                    } else {
                        // 后续历元只输出统计信息
                        int azElCount = 0;
                        for (int s = 0; s < obs.getNumSat(); s++) {
                            ObservationSet os = obs.getSatByIdx(s);
                            if (os == null) continue;
                            double az = 0, el = 0;
                            if (stationCoords != null) {
                                try {
                                    SatellitePosition satPos = reader.getGpsSatPosition(obs, os.getSatID(), os.getSatType(), 0);
                                    if (satPos != null) {
                                        TopocentricCoordinates topo = new TopocentricCoordinates();
                                        topo.computeTopocentric(stationCoords, satPos);
                                        az = topo.getAzimuth();
                                        el = topo.getElevation();
                                        azElCount++;
                                    }
                                } catch (Exception e) {
                                    // 忽略
                                }
                            }
                            exporter.addObservationWithAzEl(obs, s, az, el);
                        }
                        if (obsCount <= 5 || obsCount % 50 == 0) {
                            System.out.printf("观测: %s 卫星数=%d (方位角/仰角计算: %d/%d)%n",
                                    obs.getRefTime(), obs.getNumSat(), azElCount, obs.getNumSat());
                        }
                    }
                }
                // 2. 广播星历 (1019/1020/1042/1044/1045/1046)
                else if (obj instanceof org.gogpsproject.ephemeris.EphBds) {
                    org.gogpsproject.ephemeris.EphBds eph = (org.gogpsproject.ephemeris.EphBds) obj;
                    ephCount++;
                    String typeName = eph.getClass().getSimpleName();
                    ephTypeCount.merge(typeName, 1, Integer::sum);
                    exporter.addEphemeris(eph);
                    System.out.printf("星历: %c%02d week=%d toe=%.0f type=%s aode=%.0f aodc=%d%n",
                            eph.getSatType(), eph.getSatID(), eph.getWeek(),
                            eph.getToe(), typeName, eph.getAode(), eph.getAodc());

                }
                else if (obj instanceof org.gogpsproject.ephemeris.EphGlo) {
                    org.gogpsproject.ephemeris.EphGlo eph = (org.gogpsproject.ephemeris.EphGlo) obj;
                    ephCount++;
                    String typeName = eph.getClass().getSimpleName();
                    ephTypeCount.merge(typeName, 1, Integer::sum);
                    exporter.addEphemeris(eph);
                    System.out.printf("星历: %c%02d week=%d toe=%.0f type=%s%n",
                            eph.getSatType(), eph.getSatID(), eph.getWeek(),
                            eph.getToe(), typeName);
                }
                else if (obj instanceof org.gogpsproject.ephemeris.EphGal) {
                    org.gogpsproject.ephemeris.EphGal eph = (org.gogpsproject.ephemeris.EphGal) obj;
                    ephCount++;
                    String typeName = eph.getClass().getSimpleName();
                    ephTypeCount.merge(typeName, 1, Integer::sum);
                    exporter.addEphemeris(eph);
                    System.out.printf("星历: %c%02d week=%d toe=%.0f type=%s%n",
                            eph.getSatType(), eph.getSatID(), eph.getWeek(),
                            eph.getToe(), typeName);
                }
                else if (obj instanceof EphGps) {
                    EphGps eph = (EphGps) obj;
                    ephCount++;
                    String typeName = eph.getClass().getSimpleName();
                    ephTypeCount.merge(typeName, 1, Integer::sum);
                    exporter.addEphemeris(eph);
                    System.out.printf("星历: %c%02d week=%d toe=%.0f type=%s%n",
                            eph.getSatType(), eph.getSatID(), eph.getWeek(),
                            eph.getToe(), eph.getClass().getSimpleName());
                }
                // 3. 基准站信息 (1005/1006/1007/1008/1033)
                else if (obj instanceof Coordinates) {
                    stationCount++;
                    exporter.addStationInfo((Coordinates) obj, 0);
                    Coordinates c = (Coordinates) obj;
                    stationCoords = c;  // 保存基准站坐标
                    System.out.printf("基准站: X=%.4f Y=%.4f Z=%.4f%n", c.getX(), c.getY(), c.getZ());
                }
                else if (obj instanceof StationaryAntenna) {
                    StationaryAntenna sa = (StationaryAntenna) obj;
                    // 1033消息不含坐标，仅当天线有有效坐标时才处理
                    if (Math.abs(sa.getAntennaRefPointX()) > 0.001 ||
                        Math.abs(sa.getAntennaRefPointY()) > 0.001 ||
                        Math.abs(sa.getAntennaRefPointZ()) > 0.001) {
                        stationCount++;
                        exporter.addStationAntenna(sa);
                        // 转换为Coordinates用于方位角/仰角计算
                        stationCoords = Coordinates.globalXYZInstance(
                                sa.getAntennaRefPointX(), sa.getAntennaRefPointY(), sa.getAntennaRefPointZ());
                        System.out.printf("基准站天线: X=%.4f Y=%.4f Z=%.4f%n",
                                sa.getAntennaRefPointX(), sa.getAntennaRefPointY(), sa.getAntennaRefPointZ());
                    }
                }
                // 4. SSR改正 (1057-1068, 1240-1263)
                else if (obj instanceof SsrOrbitClock) {
                    SsrOrbitClock ssr = (SsrOrbitClock) obj;
                    ssrCount++;
                    exporter.addSsrCorrection(ssr);
                    System.out.printf("SSR: type=%d sys=%c sats=%d epoch=%.1f%n",
                            ssr.getMsgType(), ssr.getSystem(),
                            ssr.getSatelliteCount(), ssr.getEpochTime());
                }
                // 5. 辅助信息 (1013/1230等)
                else if (obj instanceof RefStationParams || obj instanceof GloCodePhaseBias) {
                    auxCount++;
                    exporter.addAuxiliary(obj);
                    System.out.printf("辅助: %s%n", obj.getClass().getSimpleName());
                }
                else {
                    otherCount++;
                    System.out.println("其他: " + obj.getClass().getSimpleName());
                }
            }
        } catch (Exception e) {
            System.err.println("解析过程出错: " + e.getMessage());
            e.printStackTrace();
        }

        // ========== 导出全部5类CSV ==========
        System.out.println("\n========== 导出CSV文件 ==========");
        exporter.exportAll();

        // ========== 统计信息 ==========
        System.out.println("\n========== 统计信息 ==========");
        System.out.printf("观测历元数: %d (共 %d 颗卫星)%n", obsCount, totalSatCount);
        System.out.printf("星历记录数: %d%n", ephCount);
        if (!ephTypeCount.isEmpty()) {
            for (var entry : ephTypeCount.entrySet()) {
                System.out.printf("  %s: %d 条%n", entry.getKey(), entry.getValue());
            }
        }
        System.out.printf("基准站信息: %d 条%n", stationCount);
        System.out.printf("SSR改正数: %d 条%n", ssrCount);
        System.out.printf("辅助信息数: %d 条%n", auxCount);
        if (nullCount > 0) System.out.printf("跳过(null): %d 条%n", nullCount);
        if (otherCount > 0) System.out.printf("未识别消息: %d 条%n", otherCount);
        System.out.println("==============================");
    }
}
