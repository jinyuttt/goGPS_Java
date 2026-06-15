/*
 * 方位角/仰角验证工具
 * 用于验证 TopocentricCoordinates 计算是否正确
 */
package org.gogpsproject;

import org.gogpsproject.positioning.Coordinates;
import org.gogpsproject.positioning.TopocentricCoordinates;

public class VerifyAzEl {

    public static void main(String[] args) {
        // 基准站坐标（从RTCM3 1005消息解析）
        double stationX = -492860.3329;
        double stationY = 5551481.2314;
        double stationZ = 3092889.6323;
        
        Coordinates station = Coordinates.globalXYZInstance(stationX, stationY, stationZ);
        
        // 转换为地理坐标
        station.computeGeodetic();
        System.out.println("========== 基准站坐标 ==========");
        System.out.printf("ECEF: X=%.4f Y=%.4f Z=%.4f%n", stationX, stationY, stationZ);
        System.out.printf("地理坐标: Lat=%.6f° Lon=%.6f° H=%.2f m%n", 
                station.getGeodeticLatitude(), 
                station.getGeodeticLongitude(), 
                station.getGeodeticHeight());
        System.out.println();
        
        // 测试用例1：天顶方向卫星（应该在正上方，仰角≈90°）
        System.out.println("========== 测试用例1：天顶卫星 ==========");
        Coordinates satZenith = Coordinates.globalXYZInstance(
                stationX * 0.999,  // 稍微靠近地心
                stationY * 0.999,
                stationZ * 1.001   // 稍微向外
        );
        TopocentricCoordinates topo1 = new TopocentricCoordinates();
        topo1.computeTopocentric(station, satZenith);
        System.out.printf("方位角=%.1f° 仰角=%.1f° (预期: Az≈0°, El≈90°)%n", 
                topo1.getAzimuth(), topo1.getElevation());
        System.out.println();
        
        // 测试用例2：正东方向卫星
        System.out.println("========== 测试用例2：东方卫星 ==========");
        // 在东方向偏移
        double latRad = Math.toRadians(station.getGeodeticLatitude());
        double lonRad = Math.toRadians(station.getGeodeticLongitude());
        double cosLat = Math.cos(latRad);
        double sinLat = Math.sin(latRad);
        double cosLon = Math.cos(lonRad);
        double sinLon = Math.sin(lonRad);
        
        // 东方向单位向量
        double eastX = -sinLon;
        double eastY = cosLon;
        double eastZ = 0;
        
        // 在天顶方向基础上向东偏移
        double offset = 500000; // 500km
        Coordinates satEast = Coordinates.globalXYZInstance(
                stationX + eastX * offset,
                stationY + eastY * offset,
                stationZ + 500000 // 保持一定高度
        );
        TopocentricCoordinates topo2 = new TopocentricCoordinates();
        topo2.computeTopocentric(station, satEast);
        System.out.printf("方位角=%.1f° 仰角=%.1f° (预期: Az≈90°, El中等)%n", 
                topo2.getAzimuth(), topo2.getElevation());
        System.out.println();
        
        // 测试用例3：正北方向卫星
        System.out.println("========== 测试用例3：北方卫星 ==========");
        // 北方向单位向量
        double northX = -sinLat * cosLon;
        double northY = -sinLat * sinLon;
        double northZ = cosLat;
        
        Coordinates satNorth = Coordinates.globalXYZInstance(
                stationX + northX * 500000,
                stationY + northY * 500000,
                stationZ + northZ * 500000 + 500000
        );
        TopocentricCoordinates topo3 = new TopocentricCoordinates();
        topo3.computeTopocentric(station, satNorth);
        System.out.printf("方位角=%.1f° 仰角=%.1f° (预期: Az≈0°或360°, El中等)%n", 
                topo3.getAzimuth(), topo3.getElevation());
        System.out.println();
        
        // 测试用例4：正南方向卫星
        System.out.println("========== 测试用例4：南方卫星 ==========");
        Coordinates satSouth = Coordinates.globalXYZInstance(
                stationX - northX * 500000,
                stationY - northY * 500000,
                stationZ - northZ * 500000 + 500000
        );
        TopocentricCoordinates topo4 = new TopocentricCoordinates();
        topo4.computeTopocentric(station, satSouth);
        System.out.printf("方位角=%.1f° 仰角=%.1f° (预期: Az≈180°, El中等)%n", 
                topo4.getAzimuth(), topo4.getElevation());
        System.out.println();
        
        System.out.println("========== 验证结论 ==========");
        System.out.println("如果以上方位角/仰角符合预期，则计算正确。");
        System.out.println("方位角：0°=北, 90°=东, 180°=南, 270°=西");
        System.out.println("仰角：0°=地平线, 90°=天顶");
    }
}
