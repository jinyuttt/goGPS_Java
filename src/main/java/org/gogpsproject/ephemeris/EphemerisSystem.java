/*
 * Copyright (c) 2011 Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
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
package org.gogpsproject.ephemeris;

import java.text.DateFormat;
import java.text.ParseException;
import java.util.Arrays;
import java.util.TimeZone;

import org.gogpsproject.Constants;
import org.gogpsproject.RtkLibConstants;
import org.gogpsproject.positioning.SatellitePosition;
import org.gogpsproject.positioning.Time;
import org.gogpsproject.producer.Observations;
import org.gogpsproject.producer.ObservationSet;
import org.ejml.simple.SimpleMatrix; 

/**
 * <p>
 *
 * </p>
 *
 * @author Eugenio Realini, Lorenzo Patocchi (code architecture)
 */

public abstract class EphemerisSystem {

	private static boolean debugSatPosLogged = true;
	
//	double[] pos ;
	
	
	
	public SatellitePosition computePositionGps(Observations obs, int satID, char satType, EphGps eph, double receiverClockError) {

		long unixTime = obs.getRefTime().getMsec();
		double obsPseudorange = obs.getSatByIDType(satID, satType).getPseudorange(0);
		
//		char satType2 = eph.getSatType() ;
		if(satType == 'C'){  // BeiDou

					// TGD is signal group delay, NOT part of satellite clock error
					// Reference: BDS ICD. TGD only affects pseudorange observation, not clock/transmit time
					double tgdCorrection = eph.getTgd() * Constants.SPEED_OF_LIGHT; // seconds -> meters

					// Persist TGD correction to the observation object so that all downstream
					// computations (satAppRange, double-difference, LS) use corrected pseudorange
					// IMPORTANT: TGD must be applied only ONCE per observation per epoch
					ObservationSet os = obs.getSatByIDType(satID, satType);
					double correctedPR;
					if (!os.isTgdApplied()) {
						correctedPR = obsPseudorange - tgdCorrection;
						if (os.isPseudorangeP(0)) {
							os.setCodeP(0, correctedPR);
						} else {
							os.setCodeC(0, correctedPR);
						}
						os.setTgdApplied(true);
					} else {
						correctedPR = obsPseudorange; // Already corrected, use as-is
					}

					// Compute satellite clock error using raw pseudorange (TGD NOT in clock error)
					double satelliteClockError = computeSatelliteClockError(unixTime, eph, obsPseudorange);
					if (Double.isNaN(satelliteClockError)) {
						return null; // Satellite unhealthy, exclude from solution
					}

					// Compute clock corrected transmission time (GPST) using TGD-corrected pseudorange
					// Decode1042Msg already converted BDT toe/toc to GPST (+14s)
					double transmitTimeGPST = computeClockCorrectedTransmissionTime(unixTime, satelliteClockError, correctedPR, satType);

				// Compute eccentric anomaly using BDS constants (time is GPST)
				Double EkObj = computeEccentricAnomalyBDS(transmitTimeGPST, eph);
				if (EkObj == null) {
					return null; // Satellite unhealthy, exclude from solution
				}
				double Ek = EkObj;

				// Semi-major axis
				double A = eph.getRootA() * eph.getRootA();

				// Time from the ephemerides reference epoch
				// Both transmitTime and eph.getToe() are in GPST
				double tk = checkGpsTime(transmitTimeGPST - eph.getToe());

				// 调试：打印北斗卫星位置计算关键参数（GEO卫星及BDS-3卫星）
				if (false && (satID <= 5 || satID >= 59)) {
					System.err.printf("[DEBUG BDS] PRN=%d, obsPseudorange=%.2f m, TGDcorr=%.2f m%n",
							satID, obsPseudorange, tgdCorrection);
					System.err.printf("[DEBUG BDS] satelliteClockError=%.6f s (%.2f m)%n",
							satelliteClockError, satelliteClockError * Constants.SPEED_OF_LIGHT);
					System.err.printf("[DEBUG BDS] transmitTimeGPST=%.2f, toeGPST=%.2f, tk=%.2f, Ek=%.6f rad%n",
							transmitTimeGPST, eph.getToe(), tk, Ek);
					System.err.printf("[DEBUG BDS] 星历参数: rootA=%.0f, e=%.8f, toe=%.2f, toc=%.2f%n",
							eph.getRootA(), eph.getE(), eph.getToe(), eph.getToc());
					System.err.printf("[DEBUG BDS] 钟参数: af0=%.12f, af1=%.12f, af2=%.12f, tgd=%.12f%n",
							eph.getAf0(), eph.getAf1(), eph.getAf2(), eph.getTgd());
				}

					// Position computation using BDS constants
					double fk = Math.atan2(Math.sqrt(1 - Math.pow(eph.getE(), 2))
							* Math.sin(Ek), Math.cos(Ek) - eph.getE());
					double phi = fk + eph.getOmega();
					phi = Math.IEEEremainder(phi, 2 * Math.PI);
					double u = phi + eph.getCuc() * Math.cos(2 * phi) + eph.getCus()
							* Math.sin(2 * phi);
					double r = A * (1 - eph.getE() * Math.cos(Ek)) + eph.getCrc()
							* Math.cos(2 * phi) + eph.getCrs() * Math.sin(2 * phi);
					double ik = eph.getI0() + eph.getiDot() * tk + eph.getCic() * Math.cos(2 * phi)
							+ eph.getCis() * Math.sin(2 * phi);

					double x1 = Math.cos(u) * r;
					double y1 = Math.sin(u) * r;

					SatellitePosition sp;

					// BDS GEO satellites: PRN 1-5 (BDS-2), PRN 59-63 (BDS-3)
					// Reference: BDS-SIS-ICD-B1I-3.0, RTKLIB ephemeris.c eph2pos()
					if (satID <= 5 || satID >= 59) {
						// GEO: Ω_k = Ω₀ + Ω̇ * t_k - ω_e * t_oe (BDT)
						// RTKLIB uses BDT toes (not GPST) in omge*toes term
						double toes = eph.getToe() - RtkLibConstants.BDS_TIME_OFFSET;
						double Omega = eph.getOmega0()
								+ eph.getOmegaDot() * tk
								- Constants.OMEGAE_DOT_BDS * toes;
						Omega = Math.IEEEremainder(Omega + 2 * Math.PI, 2 * Math.PI);

						double cosO = Math.cos(Omega);
						double sinO = Math.sin(Omega);
						double cosi = Math.cos(ik);
						double sini = Math.sin(ik);

						double xg = x1 * cosO - y1 * cosi * sinO;
						double yg = x1 * sinO + y1 * cosi * cosO;
						double zg = y1 * sini;

						double sino = Math.sin(Constants.OMEGAE_DOT_BDS * tk);
						double coso = Math.cos(Constants.OMEGAE_DOT_BDS * tk);

						double X =  xg * coso + yg * sino * RtkLibConstants.COS_5 + zg * sino * RtkLibConstants.SIN_5;
						double Y = -xg * sino + yg * coso * RtkLibConstants.COS_5 + zg * coso * RtkLibConstants.SIN_5;
						double Z = -yg * RtkLibConstants.SIN_5 + zg * RtkLibConstants.COS_5;

						sp = new SatellitePosition(unixTime, satID, satType, X, Y, Z);
					} else {
						// Non-GEO: Ω_k = Ω₀ + (Ω̇ - ω_e) * t_k - ω_e * t_oe (BDT)
						// RTKLIB uses BDT toes (not GPST) in omge*toes term
						double toes = eph.getToe() - RtkLibConstants.BDS_TIME_OFFSET;
						double Omega = eph.getOmega0()
								+ (eph.getOmegaDot() - Constants.OMEGAE_DOT_BDS) * tk
								- Constants.OMEGAE_DOT_BDS * toes;
						Omega = Math.IEEEremainder(Omega + 2 * Math.PI, 2 * Math.PI);

						sp = new SatellitePosition(unixTime, satID, satType,
								x1 * Math.cos(Omega) - y1 * Math.cos(ik) * Math.sin(Omega),
								x1 * Math.sin(Omega) + y1 * Math.cos(ik) * Math.cos(Omega),
								y1 * Math.sin(ik));
					}

					sp.setSatelliteClockError(satelliteClockError);

					if (debugSatPosLogged) {
						double Xdbg = sp.getX();
						double Ydbg = sp.getY();
						double Zdbg = sp.getZ();
						System.err.printf("[SATCMP] sat=C%02d tk=%.3f Ek=%.6f X=%13.3f Y=%13.3f Z=%13.3f dts=%12.3f ns%n",
								satID, tk, Ek, Xdbg, Ydbg, Zdbg, satelliteClockError * 1E9);
						System.err.printf("[SATCMP2] sat=C%02d i0=%.8f idot=%.3e ik=%.8f Omega0=%.8f OmegaDot=%.3e u=%.8f r=%.3f toe=%.1f tGPS=%.1f%n",
								satID, Math.toDegrees(eph.getI0()), eph.getiDot(), Math.toDegrees(ik),
								Math.toDegrees(eph.getOmega0()), eph.getOmegaDot(), Math.toDegrees(u), r, eph.getToe(), transmitTimeGPST);
					}

					// Apply the correction due to the Earth rotation during signal travel time
					SimpleMatrix R = computeEarthRotationCorrection(unixTime, receiverClockError, transmitTimeGPST, satType);
					sp.setSMMultXYZ(R);

					return sp;
					
		} else if(satType != 'R'){  // other than GLONASS (GPS, Galileo, QZSS)
			
//					System.out.println("### other than GLONASS data");
			
					// Compute satellite clock error
					double satelliteClockError = computeSatelliteClockError(unixTime, eph, obsPseudorange);
					if (Double.isNaN(satelliteClockError)) {
						return null; // Satellite unhealthy, exclude from solution
					}
			
					// Compute clock corrected transmission time
					double tGPS = computeClockCorrectedTransmissionTime(unixTime, satelliteClockError, obsPseudorange, satType);
			
					// Compute eccentric anomaly
					Double EkObj = computeEccentricAnomaly(tGPS, eph);
					if (EkObj == null) {
						return null;
					}
					double Ek = EkObj;
			
					// Semi-major axis
					double A = eph.getRootA() * eph.getRootA();
			
					// Time from the ephemerides reference epoch
					double tk = checkGpsTime(tGPS - eph.getToe());
			
					// Position computation
					double fk = Math.atan2(Math.sqrt(1 - Math.pow(eph.getE(), 2))
							* Math.sin(Ek), Math.cos(Ek) - eph.getE());
					double phi = fk + eph.getOmega();
					phi = Math.IEEEremainder(phi, 2 * Math.PI);
					double u = phi + eph.getCuc() * Math.cos(2 * phi) + eph.getCus()
							* Math.sin(2 * phi);
					double r = A * (1 - eph.getE() * Math.cos(Ek)) + eph.getCrc()
							* Math.cos(2 * phi) + eph.getCrs() * Math.sin(2 * phi);
					double ik = eph.getI0() + eph.getiDot() * tk + eph.getCic() * Math.cos(2 * phi)
							+ eph.getCis() * Math.sin(2 * phi);
					// Use system-specific Earth rotation rate (RTKLIB-aligned)
					double omegaE = RtkLibConstants.omgeForSatType(satType);
					double Omega = eph.getOmega0()
							+ (eph.getOmegaDot() - omegaE) * tk
							- omegaE * eph.getToe();
					Omega = Math.IEEEremainder(Omega + 2 * Math.PI, 2 * Math.PI);
					double x1 = Math.cos(u) * r;
					double y1 = Math.sin(u) * r;
			
					// Coordinates
			//			double[][] data = new double[3][1];
			//			data[0][0] = x1 * Math.cos(Omega) - y1 * Math.cos(ik) * Math.sin(Omega);
			//			data[1][0] = x1 * Math.sin(Omega) + y1 * Math.cos(ik) * Math.cos(Omega);
			//			data[2][0] = y1 * Math.sin(ik);
			
					// Fill in the satellite position matrix
					//this.coord.ecef = new SimpleMatrix(data);
					//this.coord = Coordinates.globalXYZInstance(new SimpleMatrix(data));
					SatellitePosition sp = new SatellitePosition(unixTime,satID, satType, x1 * Math.cos(Omega) - y1 * Math.cos(ik) * Math.sin(Omega),
							x1 * Math.sin(Omega) + y1 * Math.cos(ik) * Math.cos(Omega),
							y1 * Math.sin(ik));
					sp.setSatelliteClockError(satelliteClockError);
			
					// Apply the correction due to the Earth rotation during signal travel time
					SimpleMatrix R = computeEarthRotationCorrection(unixTime, receiverClockError, tGPS, satType);
					sp.setSMMultXYZ(R);
			
					return sp;
			//		this.setXYZ(x1 * Math.cos(Omega) - y1 * Math.cos(ik) * Math.sin(Omega),
			//				x1 * Math.sin(Omega) + y1 * Math.cos(ik) * Math.cos(Omega),
			//				y1 * Math.sin(ik));

		} else {   // GLONASS 
						
//					System.out.println("### GLONASS computation");
					satID = eph.getSatID();
					double X = eph.getX();  // satellite X coordinate at ephemeris reference time
					double Y = eph.getY();  // satellite Y coordinate at ephemeris reference time
					double Z = eph.getZ();  // satellite Z coordinate at ephemeris reference time
					double Xv = eph.getXv();  // satellite velocity along X at ephemeris reference time
					double Yv = eph.getYv();  // satellite velocity along Y at ephemeris reference time
					double Zv = eph.getZv();  // satellite velocity along Z at ephemeris reference time
					double Xa = eph.getXa();  // acceleration due to lunar-solar gravitational perturbation along X at ephemeris reference time
					double Ya = eph.getYa();  // acceleration due to lunar-solar gravitational perturbation along Y at ephemeris reference time
					double Za = eph.getZa();  // acceleration due to lunar-solar gravitational perturbation along Z at ephemeris reference time
					/* NOTE:  Xa,Ya,Za are considered constant within the integration interval (i.e. toe ?}15 minutes) */
				
					double tn = eph.getTauN();
					double gammaN = eph.getGammaN();
					double tk = eph.gettk();   
					double En = eph.getEn();
					double toc = eph.getToc();
					double toe = eph.getToe();
					int freqNum = eph.getfreq_num();
					
					obs.getSatByIDType(satID, satType).setFreqNum(freqNum);
					
					/*
					String refTime = eph.getRefTime().toString();
//					refTime = refTime.substring(0,10);
					refTime = refTime.substring(0,19);
//					refTime = refTime + " 00 00 00";
					System.out.println("refTime: " + refTime);
					
					try {
							// Set GMT time zone
							TimeZone zone = TimeZone.getTimeZone("GMT Time");
//							TimeZone zone = TimeZone.getTimeZone("UTC+4");
							DateFormat df = new java.text.SimpleDateFormat("yyyy MM dd HH mm ss");
							df.setTimeZone(zone);
	
							long ut = df.parse(refTime).getTime() ;
							System.out.println("ut: " + ut);
							Time tm = new Time(ut); 
							double gpsTime = tm.getGpsTime();
	//						double gpsTime = tm.getRoundedGpsTime();
							System.out.println("gpsT: " + gpsTime);
							
					} catch (ParseException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					*/
						
					
//					System.out.println("refTime: " + refTime);
//					System.out.println("toc: " + toc);
//					System.out.println("toe: " + toe);
//					System.out.println("unixTime: " + unixTime);				
//					System.out.println("satID: " + satID);
//					System.out.println("X: " + X);
//					System.out.println("Y: " + Y);
//					System.out.println("Z: " + Z);
//					System.out.println("Xv: " + Xv);
//					System.out.println("Yv: " + Yv);
//					System.out.println("Zv: " + Zv);
//					System.out.println("Xa: " + Xa);
//					System.out.println("Ya: " + Ya);
//					System.out.println("Za: " + Za);
//					System.out.println("tn: " + tn);
//					System.out.println("gammaN: " + gammaN);
////					System.out.println("tb: " + tb);
//					System.out.println("tk: " + tk);
//					System.out.println("En: " + En);
//					System.out.println("					");
					
					/* integration step */
				    int int_step = 60 ; // [s]	
					
					/* Compute satellite clock error */
				    double satelliteClockError = computeSatelliteClockError(unixTime, eph, obsPseudorange);
				    if (Double.isNaN(satelliteClockError)) {
						return null; // Satellite unhealthy, exclude from solution
					}
//				    System.out.println("satelliteClockError: " + satelliteClockError);
				    
					/* Compute clock corrected transmission time */
					double tGPS = computeClockCorrectedTransmissionTime(unixTime, satelliteClockError, obsPseudorange, satType);
//				    System.out.println("tGPS: " + tGPS);
					
				    /* Time from the ephemerides reference epoch */
					Time reftime = new Time(eph.getWeek(), tGPS);
					double tk2 = checkGpsTime(tGPS - toe - reftime.getLeapSeconds());
//					System.out.println("tk2: " + tk2);
				    
				    /* number of iterations on "full" steps */
					int n = (int) Math.floor(Math.abs(tk2 / int_step));
//					System.out.println("Number of iterations: " + n);
					
					/* array containing integration steps (same sign as tk) */
					double[] array = new double[n];
					Arrays.fill(array, 1);
					SimpleMatrix tkArray = new SimpleMatrix(n, 1, true, array);
					
//					SimpleMatrix tkArray2  = tkArray.scale(2);
					tkArray = tkArray.scale(int_step);
					tkArray = tkArray.scale(tk2/Math.abs(tk2));
//					tkArray.print();
					//double ii = tkArray * int_step * (tk2/Math.abs(tk2));
					
					/* check residual iteration step (i.e. remaining fraction of int_step) */
				    double int_step_res = tk2 % int_step;
//				    System.out.println("int_step_res: " + int_step_res);
				    
				    double[] intStepRes = new double[]{int_step_res};
					SimpleMatrix int_stepArray = new SimpleMatrix(1, 1, false, intStepRes);
//					int_stepArray.print();
					
					/* adjust the total number of iterations and the array of iteration steps */
				    if (int_step_res != 0){
				        tkArray = tkArray.combine(n, 0, int_stepArray);
//				        tkArray.print();
				        n = n + 1;
				       // tkArray = [ii; int_step_res];
				    }
//				    System.out.println("n: " + n);				
				    
					// numerical integration steps (i.e. re-calculation of satellite positions from toe to tk)
					double[] pos = {X, Y, Z};
					double[] vel = {Xv, Yv, Zv};
					double[] acc = {Xa, Ya, Za};				
					double[] pos1;
					double[] vel1;
								
					SimpleMatrix posArray = new SimpleMatrix(1, 3, true, pos);
					SimpleMatrix velArray = new SimpleMatrix(1, 3, true, vel);
					SimpleMatrix accArray = new SimpleMatrix(1, 3, true, acc);
					SimpleMatrix pos1Array;
					SimpleMatrix vel1Array;				
					SimpleMatrix pos2Array;
					SimpleMatrix vel2Array;				
					SimpleMatrix pos3Array;
					SimpleMatrix vel3Array;		
					SimpleMatrix pos4Array;
					SimpleMatrix vel4Array;						
					SimpleMatrix pos1dotArray;
					SimpleMatrix vel1dotArray;
					SimpleMatrix pos2dotArray;
					SimpleMatrix vel2dotArray;
					SimpleMatrix pos3dotArray;
					SimpleMatrix vel3dotArray;
					SimpleMatrix pos4dotArray;
					SimpleMatrix vel4dotArray;
					SimpleMatrix subPosArray;
					SimpleMatrix subVelArray;
					
					for (int i = 0 ; i < n ; i++ ){
						
							/* Runge-Kutta numerical integration algorithm */
					        // step 1 
							pos1Array = posArray; 
							//pos1 = pos;
							vel1Array = velArray;
							//vel1 = vel;
							
							// differential position
							pos1dotArray = velArray;
							//double[] pos1_dot = vel;
							vel1dotArray = satellite_motion_diff_eq(pos1Array, vel1Array, accArray, Constants.ELL_A_GLO, Constants.GM_GLO, Constants.J2_GLO, Constants.OMEGAE_DOT_GLO);
							//double[] vel1_dot = satellite_motion_diff_eq(pos1, vel1, acc, Constants.ELL_A_GLO, Constants.GM_GLO, Constants.J2_GLO, Constants.OMEGAE_DOT_GLO);
//							vel1dotArray.print();
							
							// step 2 
							pos2Array = pos1dotArray.scale(tkArray.get(i)).divide(2);
							pos2Array = posArray.plus(pos2Array);
							//double[] pos2 = pos + pos1_dot*ii(i)/2;
//							System.out.println("## pos2Array: " ); pos2Array.print();
							
							vel2Array = vel1dotArray.scale(tkArray.get(i)).divide(2);
							vel2Array = velArray.plus(vel2Array);
							//double[] vel2 = vel + vel1_dot * tkArray.get(i)/2;
//							System.out.println("## vel2Array: " ); vel2Array.print();
							
							pos2dotArray = vel2Array;
							//double[] pos2_dot = vel2;		
							vel2dotArray = satellite_motion_diff_eq(pos2Array, vel2Array, accArray, Constants.ELL_A_GLO, Constants.GM_GLO, Constants.J2_GLO, Constants.OMEGAE_DOT_GLO);
							//double[] vel2_dot = satellite_motion_diff_eq(pos2, vel2, acc, Constants.ELL_A_GLO, Constants.GM_GLO, Constants.J2_GLO, Constants.OMEGAE_DOT_GLO);
//							System.out.println("## vel2dotArray: " ); vel2dotArray.print();																			
							
							// step 3															
							pos3Array = pos2dotArray.scale(tkArray.get(i)).divide(2);
							pos3Array = posArray.plus(pos3Array);
//							double[] pos3 = pos + pos2_dot * tkArray.get(i)/2;
//							System.out.println("## pos3Array: " ); pos3Array.print();
							
							vel3Array = vel2dotArray.scale(tkArray.get(i)).divide(2);
							vel3Array = velArray.plus(vel3Array);
//					        double[] vel3 = vel + vel2_dot * tkArray.get(i)/2;
//							System.out.println("## vel3Array: " ); vel3Array.print();
							
							pos3dotArray = vel3Array;
					        //double[] pos3_dot = vel3;
							vel3dotArray = satellite_motion_diff_eq(pos3Array, vel3Array, accArray, Constants.ELL_A_GLO, Constants.GM_GLO, Constants.J2_GLO, Constants.OMEGAE_DOT_GLO);
							//double[] vel3_dot = satellite_motion_diff_eq(pos3, vel3, acc, Constants.ELL_A_GLO, Constants.GM_GLO, Constants.J2_GLO, Constants.OMEGAE_DOT_GLO);
//							System.out.println("## vel3dotArray: " ); vel3dotArray.print();	
							
							// step 4
							pos4Array = pos3dotArray.scale(tkArray.get(i));
							pos4Array = posArray.plus(pos4Array);
							//double[] pos4 = pos + pos3_dot * tkArray.get(i);
//							System.out.println("## pos4Array: " ); pos4Array.print();
							
							vel4Array = vel3dotArray.scale(tkArray.get(i));
							vel4Array = velArray.plus(vel4Array);
					        //double[] vel4 = vel + vel3_dot * tkArray.get(i);				
//							System.out.println("## vel4Array: " ); vel4Array.print();
							
							pos4dotArray = vel4Array;
							//double[] pos4_dot = vel4;						
							vel4dotArray = satellite_motion_diff_eq(pos4Array, vel4Array, accArray, Constants.ELL_A_GLO, Constants.GM_GLO, Constants.J2_GLO, Constants.OMEGAE_DOT_GLO);
							//double[] vel4_dot = satellite_motion_diff_eq(pos4, vel4, acc, Constants.ELL_A_GLO, Constants.GM_GLO, Constants.J2_GLO, Constants.OMEGAE_DOT_GLO);
//							System.out.println("## vel4dotArray: " ); vel4dotArray.print();																			
							
							// final position and velocity
							subPosArray = pos1dotArray.plus(pos2dotArray.scale(2)).plus(pos3dotArray.scale(2)).plus(pos4dotArray);
							subPosArray = subPosArray.scale(tkArray.get(i)).divide(6);
							posArray = posArray.plus(subPosArray) ;
						    //pos = pos + (pos1_dot + 2*pos2_dot + 2*pos3_dot + pos4_dot)*ii(s)/6;
//							System.out.println("## posArray: " ); posArray.print();	
							
							subVelArray = vel1dotArray.plus(vel2dotArray.scale(2)).plus(vel3dotArray.scale(2)).plus(vel4dotArray);
							subVelArray = subVelArray.scale(tkArray.get(i)).divide(6);
							velArray = velArray.plus(subVelArray) ;
						    //vel = vel + (vel1_dot + 2*vel2_dot + 2*vel3_dot + vel4_dot)*ii(s)/6;
//							System.out.println("## velArray: " ); velArray.print();	
//							System.out.println(" " );
							
						
					}
																		
					/* transformation from PZ-90.02 to WGS-84 (G1150) */
					double x1 = posArray.get(0) - 0.36;
					double y1 = posArray.get(1) + 0.08;
					double z1 = posArray.get(2) + 0.18;
					
					/* satellite velocity */
				    double Xv1 = velArray.get(0);
				    double Yv1 = velArray.get(1);
				    double Zv1 = velArray.get(2);
					
					/* Fill in the satellite position matrix */			
					SatellitePosition sp = new SatellitePosition(unixTime,satID, satType, x1, y1, z1);
					sp.setSatelliteClockError(satelliteClockError);
//		
//					/* Apply the correction due to the Earth rotation during signal travel time */
					SimpleMatrix R = computeEarthRotationCorrection(unixTime, receiverClockError, tGPS, satType);
					sp.setSMMultXYZ(R);
		
					return sp ;
//					return null ;
		
			
		}
	}

  public SatellitePosition computePositionSpeedGps(Observations obs, int satID, char satType, EphGps eph, double receiverClockError) {

    long unixTime = obs.getRefTime().getMsec();
    double obsPseudorange = obs.getSatByIDType(satID, satType).getPseudorange(0);

    // Compute satellite clock error
    double satelliteClockError = computeSatelliteClockError(unixTime, eph, obsPseudorange);
    if (Double.isNaN(satelliteClockError)) {
      return null; // Satellite unhealthy, exclude from solution
    }
  
    // Compute clock corrected transmission time
    double tGPS = computeClockCorrectedTransmissionTime(unixTime, satelliteClockError, obsPseudorange, satType);
  
    // Compute eccentric anomaly (use BDS-specific method for BeiDou satellites)
    Double EkObj;
    if (satType == 'C') {
      EkObj = computeEccentricAnomalyBDS(tGPS, eph);
    } else {
      EkObj = computeEccentricAnomaly(tGPS, eph);
    }
    if (EkObj == null) {
      return null;
    }
    double Ek = EkObj;
  
    // Semi-major axis
    double A = eph.getRootA() * eph.getRootA();
  
    // Time from the ephemerides reference epoch
    double tk = checkGpsTime(tGPS - eph.getToe());
  
    // Position computation
    double fk = Math.atan2(Math.sqrt(1 - Math.pow(eph.getE(), 2))
        * Math.sin(Ek), Math.cos(Ek) - eph.getE());
    double phi = fk + eph.getOmega();
    phi = Math.IEEEremainder(phi, 2 * Math.PI);
    double u = phi + eph.getCuc() * Math.cos(2 * phi) + eph.getCus()
        * Math.sin(2 * phi);
    double r = A * (1 - eph.getE() * Math.cos(Ek)) + eph.getCrc()
        * Math.cos(2 * phi) + eph.getCrs() * Math.sin(2 * phi);
    double ik = eph.getI0() + eph.getiDot() * tk + eph.getCic() * Math.cos(2 * phi)
        + eph.getCis() * Math.sin(2 * phi);
    double omegaDot = RtkLibConstants.omgeForSatType(satType);
    double Omega = eph.getOmega0()
        + (eph.getOmegaDot() - omegaDot) * tk
        - omegaDot * eph.getToe();
    Omega = Math.IEEEremainder(Omega + 2 * Math.PI, 2 * Math.PI);
    double x1 = Math.cos(u) * r;
    double y1 = Math.sin(u) * r;
  
    // Coordinates
  //      double[][] data = new double[3][1];
  //      data[0][0] = x1 * Math.cos(Omega) - y1 * Math.cos(ik) * Math.sin(Omega);
  //      data[1][0] = x1 * Math.sin(Omega) + y1 * Math.cos(ik) * Math.cos(Omega);
  //      data[2][0] = y1 * Math.sin(ik);
  
    // Fill in the satellite position matrix
    //this.coord.ecef = new SimpleMatrix(data);
    //this.coord = Coordinates.globalXYZInstance(new SimpleMatrix(data));
    SatellitePosition sp = new SatellitePosition(unixTime,satID, satType, x1 * Math.cos(Omega) - y1 * Math.cos(ik) * Math.sin(Omega),
        x1 * Math.sin(Omega) + y1 * Math.cos(ik) * Math.cos(Omega),
        y1 * Math.sin(ik));
    sp.setSatelliteClockError(satelliteClockError);
  
    // Apply the correction due to the Earth rotation during signal travel time
    SimpleMatrix R = computeEarthRotationCorrection(unixTime, receiverClockError, tGPS, satType);
    sp.setSMMultXYZ(R);
  
    ///////////////////////////
    // compute satellite speeds
    // The technical paper which describes the bc_velo.c program is published in
    // GPS Solutions, Volume 8, Number 2, 2004 (in press). "Computing Satellite Velocity using the Broadcast Ephemeris", by Benjamin W. Remondi
    double cus = eph.getCus();
    double cuc = eph.getCuc();
    double cis = eph.getCis();
    double crs = eph.getCrs();
    double crc = eph.getCrc();
    double cic = eph.getCic();
    double idot                     =  eph.getiDot(); // 0.342514267094e-09; 
    double e = eph.getE();
  
    double ek = Ek;
    double tak = fk;
  
    // Computed mean motion [rad/sec]
    // Use system-specific GM constant (RTKLIB-aligned)
    double GM = RtkLibConstants.muForSatType(eph.getSatType());
    double n0 = Math.sqrt(GM / Math.pow(A, 3));
  
    // Corrected mean motion [rad/sec]
    double n = n0 + eph.getDeltaN();
  
    // Mean anomaly
    double Mk = eph.getM0() + n * tk;
  
    double mkdot = n;
    double ekdot = mkdot/(1.0 - e*Math.cos(ek));
    double takdot = Math.sin(ek)*ekdot*(1.0+e*Math.cos(tak))/(Math.sin(tak)*(1.0-e*Math.cos(ek)));
    double omegakdot = ( eph.getOmegaDot() - omegaDot);
  
    double phik = phi;
    double corr_u = cus*Math.sin(2.0*phik) + cuc*Math.cos(2.0*phik);
    double corr_r = crs*Math.sin(2.0*phik) + crc*Math.cos(2.0*phik);
  
    double uk = phik + corr_u;
    double rk = A*(1.0-e*Math.cos(ek)) + corr_r;
  
    double ukdot = takdot +2.0*(cus*Math.cos(2.0*uk)-cuc*Math.sin(2.0*uk))*takdot;
    double rkdot = A*e*Math.sin(ek)*n/(1.0-e*Math.cos(ek)) + 2.0*(crs*Math.cos(2.0*uk)-crc*Math.sin(2.0*uk))*takdot;
    double ikdot = idot + (cis*Math.cos(2.0*uk)-cic*Math.sin(2.0*uk))*2.0*takdot;
  
    double xpk = rk*Math.cos(uk);
    double ypk = rk*Math.sin(uk);
  
    double xpkdot = rkdot*Math.cos(uk) - ypk*ukdot;
    double ypkdot = rkdot*Math.sin(uk) + xpk*ukdot;
  
    double xkdot = ( xpkdot-ypk*Math.cos(ik)*omegakdot )*Math.cos(Omega)
        - ( xpk*omegakdot+ypkdot*Math.cos(ik)-ypk*Math.sin(ik)*ikdot )*Math.sin(Omega);
    double ykdot = ( xpkdot-ypk*Math.cos(ik)*omegakdot )*Math.sin(Omega)
        + ( xpk*omegakdot+ypkdot*Math.cos(ik)-ypk*Math.sin(ik)*ikdot )*Math.cos(Omega);
    double zkdot = ypkdot*Math.sin(ik) + ypk*Math.cos(ik)*ikdot;
  
    sp.setSpeed( xkdot, ykdot, zkdot );
     
    return sp;
  }
  
	
	private SimpleMatrix satellite_motion_diff_eq(SimpleMatrix pos1Array,
			SimpleMatrix vel1Array, SimpleMatrix accArray, long ellAGlo,
			double gmGlo, double j2Glo, double omegaeDotGlo) {
		// TODO Auto-generated method stub
		
		/* renaming variables for better readability position */
		double X = pos1Array.get(0);
		double Y = pos1Array.get(1);
		double Z = pos1Array.get(2);
		
//		System.out.println("X: " + X);
//		System.out.println("Y: " + Y);
//		System.out.println("Z: " + Z);
		
		/* velocity */
		double Xv = vel1Array.get(0);
		double Yv = vel1Array.get(1);
		
//		System.out.println("Xv: " + Xv);
//		System.out.println("Yv: " + Yv);
		
		/* acceleration (i.e. perturbation) */
		double Xa = accArray.get(0);
		double Ya = accArray.get(1);
		double Za = accArray.get(2);
		
//		System.out.println("Xa: " + Xa);
//		System.out.println("Ya: " + Ya);
//		System.out.println("Za: " + Za);
		
		/* parameters */
		double r = Math.sqrt(Math.pow(X,2) + Math.pow(Y,2) + Math.pow(Z,2));
		double g = -gmGlo/Math.pow(r,3);
		double h = j2Glo*1.5*Math.pow((ellAGlo/r),2);
		double k = 5*Math.pow(Z,2)/Math.pow(r,2);
		
//		System.out.println("r: " + r);
//		System.out.println("g: " + g);
//		System.out.println("h: " + h);
//		System.out.println("k: " + k);
		
		/* differential velocity */
		double[] vel_dot = new double[3] ;
		vel_dot[0] = g*X*(1 - h*(k - 1)) + Xa + Math.pow(omegaeDotGlo,2)*X + 2*omegaeDotGlo*Yv;
//		System.out.println("vel1: " + vel_dot[0]);
		
		vel_dot[1] = g*Y*(1 - h*(k - 1)) + Ya + Math.pow(omegaeDotGlo,2)*Y - 2*omegaeDotGlo*Xv;
//		System.out.println("vel2: " + vel_dot[1]);
		
		vel_dot[2] = g*Z*(1 - h*(k - 3)) + Za;
//		System.out.println("vel3: " + vel_dot[2]);
		
		SimpleMatrix velDotArray = new SimpleMatrix(1, 3, true, vel_dot);
//		velDotArray.print();
		
		return velDotArray;
	}


	/**
	 * @param time
	 *            (Uncorrected GPS time)
	 * @return GPS time accounting for beginning or end of week crossover
	 */
	protected double checkGpsTime(double time) {

		// Account for beginning or end of week crossover
		if (time > Constants.SEC_IN_HALF_WEEK) {
			time = time - 2 * Constants.SEC_IN_HALF_WEEK;
		} else if (time < -Constants.SEC_IN_HALF_WEEK) {
			time = time + 2 * Constants.SEC_IN_HALF_WEEK;
		}
		return time;
	}

	/**
	 * @param traveltime
	 */
	protected SimpleMatrix computeEarthRotationCorrection(long unixTime, double receiverClockError, double transmissionTime, char satType) {

		double receptionTime = (new Time(unixTime)).getGpsTime();
		double traveltime = receptionTime + receiverClockError - transmissionTime;

		// Compute rotation angle: use system-specific Earth angular velocity
		// GPS: 7.2921151467E-5, GLO: 7.292115E-5, GAL: 7.2921151467E-5, BDS: 7.292115E-5, QZS: 7.2921151467E-5
		double omega = RtkLibConstants.omgeForSatType(satType);
		double omegatau = omega * traveltime;

		// Rotation matrix
		double[][] data = new double[3][3];
		data[0][0] = Math.cos(omegatau);
		data[0][1] = Math.sin(omegatau);
		data[0][2] = 0;
		data[1][0] = -Math.sin(omegatau);
		data[1][1] = Math.cos(omegatau);
		data[1][2] = 0;
		data[2][0] = 0;
		data[2][1] = 0;
		data[2][2] = 1;
		SimpleMatrix R = new SimpleMatrix(data);

		return R;
		// Apply rotation
		//this.coord.ecef = R.mult(this.coord.ecef);
		//this.coord.setSMMultXYZ(R);// = R.mult(this.coord.ecef);
		//satellitePosition.setSMMultXYZ(R);// = R.mult(this.coord.ecef);

	}

	/**
	 * @param eph
	 * @return Clock-corrected transmission time
	 */
	protected double computeClockCorrectedTransmissionTime(long unixTime, double satelliteClockError, double obsPseudorange, char satType) {

		double gpsTime = (new Time(unixTime)).getGpsTime();

		// 星历时间（ToC/ToE）已在Decode1042Msg中统一转换为GPST
		// 所以不需要GPST→BDT转换，统一使用GPST时间
		// 注释：之前错误地对北斗卫星做了GPST→BDT转换（减14秒），导致时间基准不匹配
		double tObs = gpsTime;

		// Remove signal travel time from observation time
		double tRaw = (tObs - obsPseudorange /*this.range*/ / Constants.SPEED_OF_LIGHT);

		return tRaw - satelliteClockError;
	}

	/**
	 * @param eph
	 * @return Satellite clock error
	 */
	protected double computeSatelliteClockError(long unixTime, EphGps eph, double obsPseudorange){
		
		if (eph.getSatType() == 'R'){   // In case of GLONASS
			
				double gpsTime = (new Time(unixTime)).getGpsTime();
//				System.out.println("gpsTime: " + gpsTime);
//				System.out.println("obsPseudorange: " + obsPseudorange);

				// Remove signal travel time from observation time
				double tRaw = (gpsTime - obsPseudorange /*this.range*/ / Constants.SPEED_OF_LIGHT);		
//				System.out.println("tRaw: " + tRaw);
				
				// Clock error computation
				double dt = checkGpsTime(tRaw - eph.getToe());
//				System.out.println("dt: " + dt);
				
				double timeCorrection =  eph.getTauN() + eph.getGammaN() * dt ;			
//				double timeCorrection =  - eph.getTauN() + eph.getGammaN() * dt ;					
				
				return timeCorrection;
			
		}else{		// other than GLONASS
				double gpsTime = (new Time(unixTime)).getGpsTime();
				double tObs = gpsTime;
				// Remove signal travel time from observation time
				double tRaw = (tObs - obsPseudorange / Constants.SPEED_OF_LIGHT);

				// Compute eccentric anomaly using BDS constants for BeiDou satellites
				Double EkObj;
				if (eph.getSatType() == 'C') {
					EkObj = computeEccentricAnomalyBDS(tRaw, eph);
				} else {
					EkObj = computeEccentricAnomaly(tRaw, eph);
				}
				if (EkObj == null) {
					return Double.NaN; // Satellite unhealthy, clock error cannot be computed
				}
				double Ek = EkObj;

				// Relativistic correction term computation
				// Use system-specific GM for relativistic correction (strictly following RTKLIB eph2pos)
				// GPS: 3.9860050E14, GAL: 3.986004418E14, BDS: 3.986004418E14, QZS: 3.9860050E14
				double gm = RtkLibConstants.muForSatType(eph.getSatType());
				double dtr = -2.0 * Math.sqrt(gm) * eph.getE() * eph.getRootA() * Math.sin(Ek)
					/ (Constants.SPEED_OF_LIGHT * Constants.SPEED_OF_LIGHT);

				// Clock error computation (2 iterations, same as RTKLIB eph2clk)
				// TGD is NOT included in satellite clock error - TGD correction is at pseudorange level
				double dt = checkGpsTime(tRaw - eph.getToc());
				double timeCorrection = (eph.getAf2() * dt + eph.getAf1()) * dt + eph.getAf0() + dtr;
				double tGPS = tRaw - timeCorrection;
				dt = checkGpsTime(tGPS - eph.getToc());
				timeCorrection = (eph.getAf2() * dt + eph.getAf1()) * dt + eph.getAf0() + dtr;

				return timeCorrection;
		}
	}

	/**
	 * @param time
	 *            (GPS time in seconds)
	 * @param eph
	 * @return Eccentric anomaly
	 */
	protected Double computeEccentricAnomaly(double time, EphGps eph) {

		// Semi-major axis
		double A = eph.getRootA() * eph.getRootA();

		// Time from the ephemerides reference epoch
		double tk = checkGpsTime(time - eph.getToe());

		// Computed mean motion [rad/sec] using system-specific GM (RTKLIB-aligned)
		double n0 = Math.sqrt(RtkLibConstants.muForSatType(eph.getSatType()) / Math.pow(A, 3));

		// Corrected mean motion [rad/sec]
		double n = n0 + eph.getDeltaN();

		// Mean anomaly
		double Mk = eph.getM0() + n * tk;

		// Eccentric anomaly starting value
		Mk = Math.IEEEremainder(Mk + 2 * Math.PI, 2 * Math.PI);
		double Ek = Mk;

		int i;
		double EkOld, dEk;

		// Eccentric anomaly iterative computation
		int maxNumIter = 12;
		for (i = 0; i < maxNumIter; i++) {
			EkOld = Ek;
			Ek = Mk + eph.getE() * Math.sin(Ek);
			dEk = Math.IEEEremainder(Ek - EkOld, 2 * Math.PI);
			if (Math.abs(dEk) < 1e-12)
				break;
		}

		// If no convergence after max iterations, return null to mark satellite as unhealthy
		if (i == maxNumIter) {
			System.err.printf("[WARN] Eccentric anomaly does not converge after %d iterations (PRN=%d), satellite excluded%n",
					maxNumIter, eph.getSatID());
			return null;
		}

		return Ek;

	}

	/**
	 * @param time
	 *            (GPST time in seconds, toe 已统一转为 GPST)
	 * @param eph
	 * @return Eccentric anomaly for BeiDou satellites
	 */
	protected Double computeEccentricAnomalyBDS(double time, EphGps eph) {

		// Semi-major axis
		double A = eph.getRootA() * eph.getRootA();

		// Time from the ephemerides reference epoch
		double tk = checkGpsTime(time - eph.getToe());

		// Computed mean motion [rad/sec] using BeiDou GM constant
		double n0 = Math.sqrt(Constants.GM_BDS / Math.pow(A, 3));

		// Corrected mean motion [rad/sec]
		double n = n0 + eph.getDeltaN();

		// Mean anomaly
		double Mk = eph.getM0() + n * tk;

		// Eccentric anomaly starting value
		Mk = Math.IEEEremainder(Mk + 2 * Math.PI, 2 * Math.PI);
		double Ek = Mk;

		int i;
		double EkOld, dEk;

		// Eccentric anomaly iterative computation
		int maxNumIter = 12;
		for (i = 0; i < maxNumIter; i++) {
			EkOld = Ek;
			Ek = Mk + eph.getE() * Math.sin(Ek);
			dEk = Math.IEEEremainder(Ek - EkOld, 2 * Math.PI);
			if (Math.abs(dEk) < 1e-12)
				break;
		}

		// If no convergence after max iterations, return null to mark satellite as unhealthy
		if (i == maxNumIter) {
			System.err.printf("[WARN BDS] Eccentric anomaly does not converge after %d iterations (PRN=%d), satellite excluded%n",
					maxNumIter, eph.getSatID());
			return null;
		}

		return Ek;

	}

}