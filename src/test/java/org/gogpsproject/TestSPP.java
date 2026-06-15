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
 */
package org.gogpsproject;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

import org.gogpsproject.GoGPS.RunMode;
import org.gogpsproject.consumer.TxtProducer;
import org.gogpsproject.producer.Observations;
import org.gogpsproject.producer.parser.rtcm3.RTCM3FileReader;

/**
 * 单点定位测试类
 * 使用RTCM3文件进行单点定位（SPP - Single Point Positioning）
 * 
 * @author goGPS Project
 */
public class TestSPP {

    /**
     * @param args
     */
    public static void main(String[] args) {
        // 强制使用点作为小数分隔符
        Locale.setDefault(new Locale("en", "US"));
        int week = 2422;
        // RTCM3文件路径（默认使用桌面上的测试文件）
        String rtcm3FilePath = "C:\\Users\\jinyu\\Desktop\\1.rtcm3";
        
        // 如果命令行提供了文件路径，则使用命令行参数
        if (args.length > 0) {
            rtcm3FilePath = args[0];
        }

        System.out.println("========================================");
        System.out.println("goGPS 单点定位测试 (SPP)");
        System.out.println("========================================");
        System.out.println("输入文件: " + rtcm3FilePath);
        System.out.println();

        try {
            // 检查文件是否存在
            File rtcm3File = new File(rtcm3FilePath);
            if (!rtcm3File.exists()) {
                System.err.println("错误: 文件不存在 - " + rtcm3FilePath);
                return;
            }

            // 记录开始时间
            long start = System.currentTimeMillis();

            // 创建RTCM3文件读取器
            // RTCM3FileReader(File file, int week)
            // week参数用于GLONASS时间转换，可以使用0让系统自动处理
            RTCM3FileReader rtcm3Reader = new RTCM3FileReader(rtcm3File, week);

            // 初始化读取器
            System.out.println("正在初始化RTCM3读取器...");
            rtcm3Reader.init();

            // 创建输出文件名
            Date date = new Date();
            SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd_HHmmss");
            String dateStr = sdf.format(date);
            String outPathTxt = "./test/spp_" + dateStr + ".txt";
            
            // 确保test目录存在
            File testDir = new File("./test");
            if (!testDir.exists()) {
                testDir.mkdirs();
            }

            // 创建文本输出器
            TxtProducer txtProducer = new TxtProducer(outPathTxt);

            // 创建GoGPS实例进行单点定位
            // 注意: 单点定位只需要rover数据，不需要master基站数据
            GoGPS goGPS = new GoGPS(rtcm3Reader, rtcm3Reader)
                .addPositionConsumerListeners(txtProducer)
                .setDynamicModel(GoGPS.DynamicModel.STATIC)
                .run(RunMode.CODE_STANDALONE);

            // 等待处理完成
            System.out.println("正在处理数据...");
            goGPS.runUntilFinished();

            // 释放资源
            try {
                rtcm3Reader.release(true, 10000);
            } catch (InterruptedException ie) {
                ie.printStackTrace();
            }

            System.out.println("处理完成!");
            System.out.println("输出文件: " + outPathTxt);

            // 计算并显示耗时
            int elapsedTimeSec = (int) Math.floor((System.currentTimeMillis() - start) / 1000);
            int elapsedTimeMillisec = (int) ((System.currentTimeMillis() - start) - elapsedTimeSec * 1000);
            System.out.println("\n总耗时: " + elapsedTimeSec + " 秒 " + elapsedTimeMillisec + " 毫秒");

        } catch (Exception e) {
            System.err.println("处理过程中发生错误:");
            e.printStackTrace();
        }
    }

    /**
     * 简单的单点定位测试（不使用GoGPS框架，直接读取和计算）
     * 用于调试和验证
     */
    public static void simpleSPPTest(String rtcm3FilePath) {
        Locale.setDefault(new Locale("en", "US"));
        
        System.out.println("========================================");
        System.out.println("简单单点定位测试");
        System.out.println("========================================");

        try {
            File rtcm3File = new File(rtcm3FilePath);
            if (!rtcm3File.exists()) {
                System.err.println("错误: 文件不存在 - " + rtcm3FilePath);
                return;
            }

            RTCM3FileReader reader = new RTCM3FileReader(rtcm3File, 0);
            reader.init();

            int obsCount = 0;
            
            // 读取所有观测数据和星历
            Observations obs;
            while ((obs = reader.getNextObservations()) != null) {
                obsCount++;
                System.out.println("观测历元 #" + obsCount + ": " + obs.getRefTime());
                
                // 打印可见卫星数量
                int numSat = obs.getNumSat();
                if (numSat > 0) {
                    System.out.println("  可见卫星总数: " + numSat);
                }
            }

            reader.release(true, 5000);

        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
