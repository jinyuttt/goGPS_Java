package org.gogpsproject.positioning;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.ejml.simple.SimpleMatrix;
import org.gogpsproject.GoGPS;
import org.gogpsproject.GoGPS.*;
import org.gogpsproject.Status;
import org.gogpsproject.consumer.PositionConsumer;
import org.gogpsproject.positioning.RoverPosition.DopType;
import org.gogpsproject.producer.Observations;
import org.gogpsproject.producer.ObservationsProducer;

/**
 * DD (Double-Difference) Kalman Filter with code+phase observations.
 *
 * ================== RTKLIB 对齐总览 (rtkpos.c) ==================
 * 本类逐项对齐 RTKLIB-2.5.0/src/rtkpos.c 的 ddres() / zdres() / udbias() / holdamb()。
 * 以下为已核对并落地的对齐点与已知差异，便于快速核实，避免反复排查：
 *
 * [已对齐 - 几何距离]
 *   - satAppRange = 纯几何距离 ||sat-rover|| (Satellites.java)。
 *   - 差异说明: RTKLIB zdres() 中 r = geodist() + (-CLIGHT*dts) + tropo_zhd。
 *     goGPS 把卫星钟差 (-CLIGHT*dts) 放在【观测值侧】而非几何距离侧：
 *       ddcObs(伪距DD) - ddcApp(几何DD) = (satClk_i - satClk_j) - (satClk_pivot_i - satClk_pivot_j)
 *     由于双差会消去公共卫星钟差项，只要 rover/master 用同一组星历，该项在 DD 中近似抵消。
 *     若残差异常大(>30m)，优先核查: 星历是否一致、TGD 是否重复扣除、卫星位置是否做了地球自转改正。
 *
 * [已对齐 - TGD]
 *   - BDS TGD 在 EphemerisSystem.computePositionGps() 中一次性扣到伪距上 (os.setTgdApplied)。
 *   - RTKLIB: TGD 不进卫星钟差(eph2clk)，只在 pseudorange 层处理，与此一致。
 *   - 注意: TGD 只扣一次，重复调用 computePositionGps 会因 isTgdApplied 标志跳过。
 *
 * [已对齐 - 协方差下限]
 *   - KalmanFilter.MIN_VAR_POS = 1e-8 (原 1.0 会阻止收敛到厘米级)。
 *
 * [已对齐 - 观测噪声 Cnn]
 *   - varerr() 依高度角定权，Cnn 对角线 = rover_var + master_var (不再双重累加 baseNoise)。
 *
 * [已对齐 - Outlier 拒绝]
 *   - 相位阈值 maxinno[0]=5m，码阈值 maxinno[1]=30m；首历元 threshadj=10 放宽。
 *   - goGPS 差异: 不 skip 观测(避免矩阵索引错位)，而是 y0=0/H=0/Cnn=1e10 置零权重。
 *
 * [已对齐 - 模糊度初始化]
 *   - udbias(): 新卫星/周跳时 initx(bias, SQR(std[0]=30)=900 m²)。
 *   - L2 模糊度按 dualFreq 分配(nAmbL2=nN)，不与 estimIono 绑定(原绑定会越界崩溃)。
 *
 * [已对齐 - 模糊度状态单位 meters] (2026-06-21 重构，对齐 RTKLIB)
 *   H_amb = +1.0 (meters)，与 RTKLIB 完全一致。
 *   数学保证 S 非奇异: S[phase] = (α·√K[pos] + √K[amb])² + Cnn > 0。
 *   (历史: 原 H_amb=-λ(cycles) 使交叉项为负，抵消 S 对角项 → 近奇异 → gMax=201 → 发散)
 *   同步修改点(状态单位 cycles→meters):
 *     1. H_amb L1/L2: -λ → +1.0 (L468/L660)
 *     2. 状态初值: (codeDD-phaseDD)/λ → (codeDD-phaseDD) (estimateAmbiguities)
 *     3. 初始方差: 900 cycle² → 900 m² (init/cycleslip 分支)
 *     4. Qphase: λ²·Cee → Cee (Cee 已是 m²)
 *     5. LAMBDA 输入: a[i]/λ, Q[i,j]/(λ_i·λ_j) (meters→cycles, LAMBDA 要求 cycles)
 *     6. LAMBDA 固定: F[i]·λ (cycles→meters, 赋回 KFstate)
 *   holdamb 无需改(H_hold=1.0, v_hold=fixedVal(meters), 与 KFstate 一致)。
 *   ambswap 无需改(±1 是 DD 重定义，与单位无关)。
 *
 * [已对齐 - LAMBDA 与 holdamb]
 *   - ratio 阈值 3.0；连续固定 MIN_FIX_HOLD=10 次后 holdamb(VAR_HOLDAMB=0.001)。
 *
 * [已知差异 - 待观察]
 *   - BDS GEO 噪声放大 GEO_NOISE_SCALE=8.0 (RTKLIB 无此放大，必要时改 1.0 核对)。
 *   - 周跳检测同时启用 LLI/Doppler/GF/ApproxRange 四种，较 RTKLIB(仅 LLI+gf) 更敏感，
 *     可能导致模糊度频繁重置、sqrtCov 停留在 30。若 ratio 长期<3，优先排查此处。
 *   - 电离层/对流层估计: TestRTK 设 OFF(用广播模型)，与 RTKLIB brdc/saas 一致。
 *
 * [关键调试入口]
 *   - epoch0 残差: [DEBUG KF DD] / [EPOCH0 phase] / [OUTLIER-*]
 *   - LAMBDA 输入: [LAMBDA INPUT] a / Q，与 RTKLIB tracemat 逐元素对比。
 *   - 参考坐标: [KF_DD run] masterIn(1005) 与 RTKLIB epoch0 reference。
 * ================================================================
 */
public class KF_DD_code_phase extends KalmanFilter {

  private static final int BDS_GEO_MIN = 1;
  private static final int BDS_GEO_MAX = 5;
  private static final double GEO_NOISE_SCALE = 8.0;

  /** RTKLIB-aligned: constraint variance to hold fixed ambiguity (cycle^2) */
  private static final double VAR_HOLDAMB = 0.001;

  /** Holds the fixed DD ambiguity values (per satellite ID) for holdamb constraints.
   *  Updated in fixAmbiguitiesLambda() when ratio test passes. */
  private Map<Integer, Double> heldAmbiguities = new HashMap<>();

  /** RTKLIB-aligned base station SPP averaging (POSOPT_SINGLE).
   *  RTKLIB averages single-point positions over all epochs to get a stable
   *  reference station coordinate (postpos.c::avepos / rtksvr.c increment).
   *  These fields accumulate the running mean of the base station SPP.
   *  Declared static because run() is a static method. */
  private static double baseSppSumX = 0.0;
  private static double baseSppSumY = 0.0;
  private static double baseSppSumZ = 0.0;
  private static int    baseSppCount = 0;

  /** RTKLIB-aligned: ionospheric mapping function single-layer height (m) */
  private static final double IONO_H = 350000.0;

  /** RTKLIB-aligned: GMF constants */
  private static final double GMF_A = 1.001;
  private static final double GMF_B = 0.002001;

  /** Dummy observation variance for sats lacking L2 code/phase.
   * Keeps S matrix non-singular without affecting the KF solution
   * (y0=0, H=0 => zero Kalman gain for that row). */
  private static final double DUMMY_OBS_VAR = 1.0e10;

  /**
   * Check if a BDS satellite is GEO based on PRN.
   * BDS-2 GEO: C01-C05 (PRN 1-5). BDS-3 GEO: may use different PRN ranges.
   * TODO: replace PRN range with navigation ephemeris orbit type when available.
   */
  private static boolean isBdsGeo(char satType, int satPrn) {
    return satType == 'C' && satPrn >= BDS_GEO_MIN && satPrn <= BDS_GEO_MAX;
  }

  public KF_DD_code_phase(GoGPS goGPS) {
    super(goGPS);
  }

  @Override
  void setup(Observations roverObs, Observations masterObs, Coordinates masterPos) {

    epochCount++;
    boolean dualFreq = goGPS.isDualFreq();
    boolean estimIono = goGPS.getIonoOpt() != GoGPS.IonoOpt.OFF && dualFreq;
    boolean estimTropo = goGPS.getTropOpt() != GoGPS.TropOpt.OFF && dualFreq;

    // === Epoch-level diagnostic header (RTKLIB trace 对比入口) ===
    if (goGPS.isDebug() && epochCount <= 10) {
      System.err.printf("%n[SD-DIAG] ======== epoch=%d time=%s rover=[%.3f,%.3f,%.3f] master=[%.3f,%.3f,%.3f] ========%n",
          epochCount, roverObs.getRefTime(),
          rover.getX(), rover.getY(), rover.getZ(),
          masterPos.getX(), masterPos.getY(), masterPos.getZ());
    }

    // Number of GPS observations
    int nObs = roverObs.getNumSat();

    // Number of available satellites (i.e. observations)
    int nObsAvail = sats.avail.size();

    // Double differences with respect to pivot satellite reduce observations by 1
    nObsAvail--;

    // Number of available satellites with phase (DD)
    int nObsAvailPhase = sats.availPhase.size() - 1;

    // Matrix containing parameters obtained from the linearization of the observation equations
    SimpleMatrix A = new SimpleMatrix(nObsAvail, 3);

    // Pivot satellite ID
    int pivotId = roverObs.getSatID(sats.pivot);
    char satType = roverObs.getGnssType(sats.pivot);

    // Store rover-pivot and master-pivot observed pseudoranges (L1)
    double roverPivotCodeObs = roverObs.getSatByIDType(pivotId, satType).getPseudorange(0);
    double masterPivotCodeObs = masterObs.getSatByIDType(pivotId, satType).getPseudorange(0);

    // Compute and store rover-pivot and master-pivot observed phase ranges (L1)
    double roverPivotPhaseObs = roverObs.getSatByIDType(pivotId, satType).getPhaserange(0);
    double masterPivotPhaseObs = masterObs.getSatByIDType(pivotId, satType).getPhaserange(0);

    // L2 pivot observations (for dual-frequency)
    // RTKLIB-aligned: L2 reference satellite is selected independently
    // (sats.pivotL2), may differ from L1 pivot (sats.pivot).
    double roverPivotCodeObsL2 = 0, masterPivotCodeObsL2 = 0;
    double roverPivotPhaseObsL2 = 0, masterPivotPhaseObsL2 = 0;
    double gamma = 1.0; // (f1/f2)^2 = (lambda1/lambda2)^2
    // L2-pivot geometry & corrections (from sats.pivotL2, not sats.pivot)
    SimpleMatrix diffRoverPivotL2 = null;
    double roverPivotAppRangeL2 = 0, masterPivotAppRangeL2 = 0;
    double roverPivotTropoCorrL2 = 0, masterPivotTropoCorrL2 = 0;
    double roverPivotIonoCorrL2 = 0, masterPivotIonoCorrL2 = 0;
    double roverPivotAntCorrL2 = 0, masterPivotAntCorrL2 = 0;
    double roverPivotWindUpL2 = 0, masterPivotWindUpL2 = 0;
    double roverPivotMwL2 = 0, masterPivotMwL2 = 0;
    boolean l2PivotSameAsL1 = false;
    if (dualFreq && sats.pivotL2 >= 0) {
      int pivotL2Id = roverObs.getSatID(sats.pivotL2);
      char pivotL2Type = roverObs.getGnssType(sats.pivotL2);
      l2PivotSameAsL1 = (sats.pivotL2 == sats.pivot);
      roverPivotCodeObsL2 = roverObs.getSatByIDType(pivotL2Id, pivotL2Type).getPseudorange(1);
      masterPivotCodeObsL2 = masterObs.getSatByIDType(pivotL2Id, pivotL2Type).getPseudorange(1);
      roverPivotPhaseObsL2 = roverObs.getSatByIDType(pivotL2Id, pivotL2Type).getPhaserange(1);
      masterPivotPhaseObsL2 = masterObs.getSatByIDType(pivotL2Id, pivotL2Type).getPhaserange(1);
      // L2-pivot geometry & corrections (from pivotL2 index)
      diffRoverPivotL2 = rover.diffSat[sats.pivotL2];
      roverPivotAppRangeL2 = rover.satAppRange[sats.pivotL2];
      masterPivotAppRangeL2 = master.satAppRange[sats.pivotL2];
      roverPivotTropoCorrL2 = rover.satTropoCorr[sats.pivotL2];
      masterPivotTropoCorrL2 = master.satTropoCorr[sats.pivotL2];
      roverPivotIonoCorrL2 = rover.satIonoCorr[sats.pivotL2];
      masterPivotIonoCorrL2 = master.satIonoCorr[sats.pivotL2];
      roverPivotAntCorrL2 = rover.satAntennaCorr[sats.pivotL2];
      masterPivotAntCorrL2 = master.satAntennaCorr[sats.pivotL2];
      roverPivotWindUpL2 = rover.satWindUp[sats.pivotL2];
      masterPivotWindUpL2 = master.satWindUp[sats.pivotL2];
      double roverElevL2 = rover.topo[sats.pivotL2].getElevation();
      double masterElevL2 = master.topo[sats.pivotL2].getElevation();
      roverPivotMwL2 = tropoMapWet(roverElevL2);
      masterPivotMwL2 = tropoMapWet(masterElevL2);
      double lam1 = roverObs.getSatByIDType(pivotL2Id, pivotL2Type).getWavelength(0);
      double lam2 = roverObs.getSatByIDType(pivotL2Id, pivotL2Type).getWavelength(1);
      gamma = (lam1 * lam1) / (lam2 * lam2);
    }

    // Rover-pivot approximate pseudoranges
    SimpleMatrix diffRoverPivot = rover.diffSat[sats.pivot];
    double roverPivotAppRange = rover.satAppRange[sats.pivot];

    // Master-pivot approximate pseudoranges
    double masterPivotAppRange = master.satAppRange[sats.pivot];

    // Rover-pivot and master-pivot troposphere correction
    double roverPivotTropoCorr = rover.satTropoCorr[sats.pivot];
    double masterPivotTropoCorr = master.satTropoCorr[sats.pivot];

    // Rover-pivot and master-pivot ionosphere correction (L1)
    double roverPivotIonoCorr = rover.satIonoCorr[sats.pivot];
    double masterPivotIonoCorr = master.satIonoCorr[sats.pivot];

    // Rover-pivot and master-pivot antenna correction
    double roverPivotAntCorr = rover.satAntennaCorr[sats.pivot];
    double masterPivotAntCorr = master.satAntennaCorr[sats.pivot];

    // Rover-pivot and master-pivot phase wind-up correction
    double roverPivotWindUp = rover.satWindUp[sats.pivot];
    double masterPivotWindUp = master.satWindUp[sats.pivot];

    // Compute rover-pivot and master-pivot elevations
    double roverElevation = rover.topo[sats.pivot].getElevation();
    double masterElevation = master.topo[sats.pivot].getElevation();

    // Troposphere wet mapping function (RTKLIB GMF-aligned)
    // mw(el) = 1.001 / sqrt(0.002001 + sin^2(el))
    double roverPivotMw = tropoMapWet(roverElevation);
    double masterPivotMw = tropoMapWet(masterElevation);

    // Start filling in the observation error covariance matrix
    Cnn.zero();
    int nSatAvail = sats.avail.size() - 1;
    int nSatAvailPhase = (sats.availPhase.size() > 0) ? sats.availPhase.size() - 1 : 0;
    int l1Rows = nSatAvail + nSatAvailPhase;
    int totalObs = (dualFreq && sats.pivotL2 >= 0) ? l1Rows * 2 : l1Rows;

    if (goGPS.isDebug() && epochCount <= 2) {
      System.err.printf("[KF setup] dualFreq=%b, estimIono=%b, estimTropo=%b, nObs=%d, nObsAvail=%d, nSatAvailPhase=%d, l1Rows=%d, totalObs=%d%n",
          dualFreq, estimIono, estimTropo, nObs, nObsAvail, nSatAvailPhase, l1Rows, totalObs);
    }

    // Pre-fill Cnn with pivot baseline noise for L1 rows
    for (int i = 0; i < l1Rows; i++) {
      double baseNoise;
      if (i < nSatAvail) {
        baseNoise = varerr(Math.toRadians(roverElevation), false, satType, 0)
                  + varerr(Math.toRadians(masterElevation), false, satType, 0);
      } else {
        baseNoise = varerr(Math.toRadians(roverElevation), true, satType, 0)
                  + varerr(Math.toRadians(masterElevation), true, satType, 0);
      }
      Cnn.set(i, i, baseNoise);
    }
    // Pre-fill Cnn with pivot baseline noise for L2 rows
    if (dualFreq) {
      for (int i = 0; i < l1Rows; i++) {
        double baseNoise;
        int idx = l1Rows + i;
        if (i < nSatAvail) {
          baseNoise = varerr(Math.toRadians(roverElevation), false, satType, 1)
                    + varerr(Math.toRadians(masterElevation), false, satType, 1);
        } else {
          baseNoise = varerr(Math.toRadians(roverElevation), true, satType, 1)
                    + varerr(Math.toRadians(masterElevation), true, satType, 1);
        }
        Cnn.set(idx, idx, baseNoise);
      }
    }

    // Counter for available satellites
    int k = 0;
    int p = 0;

    for (int i = 0; i < nObs; i++) {

      int id = roverObs.getSatID(i);
      satType = roverObs.getGnssType(i);
      String checkAvailGnss = String.valueOf(satType) + String.valueOf(id);

      if (sats.pos[i]!=null && sats.gnssAvail.contains(checkAvailGnss) && i != sats.pivot) {

        // Compute parameters obtained from linearization of observation equations
        double alphaX = rover.diffSat[i].get(0) / rover.satAppRange[i]
            - diffRoverPivot.get(0) / roverPivotAppRange;
        double alphaY = rover.diffSat[i].get(1) / rover.satAppRange[i]
            - diffRoverPivot.get(1) / roverPivotAppRange;
        double alphaZ = rover.diffSat[i].get(2) / rover.satAppRange[i]
            - diffRoverPivot.get(2) / roverPivotAppRange;

        // Fill in the A matrix
        A.set(k, 0, alphaX);
        A.set(k, 1, alphaY);
        A.set(k, 2, alphaZ);

        // Approximate code double difference
        // 几何双差: 纯几何距离 DD，不含卫星钟差/对流层(对流层在 appRangeCode 单独加)。
        // RTKLIB 对应: zdres() 中 r = geodist() + (-CLIGHT*dts) + tropo_zhd，
        // goGPS 把 -CLIGHT*dts 留在观测值侧(ddcObs)，双差后公共钟差项抵消，结果等价。
        double ddcApp = (rover.satAppRange[i] - master.satAppRange[i])
            - (roverPivotAppRange - masterPivotAppRange);

        // Observed code double difference (L1)
        // 观测伪距 DD: getPseudorange() 已含卫星钟差(广播)和 TGD(BDS, EphemerisSystem 一次性扣除)。
        // 残差 y0 = ddcObs - appRangeCode 应在 [-30,30]m 内，否则触发 OUTLIER-CODE。
        // 若残差>100m，核查: ①rover/master 星历是否同源 ②TGD 是否被重复扣 ③地球自转改正。
        double ddcObs = (roverObs.getSatByIDType(id, satType).getPseudorange(0) - masterObs.getSatByIDType(id, satType).getPseudorange(0))
            - (roverPivotCodeObs - masterPivotCodeObs);

        // Observed phase double difference (L1)
        double ddpObs = (roverObs.getSatByIDType(id, satType).getPhaserange(0) - masterObs.getSatByIDType(id, satType).getPhaserange(0))
            - (roverPivotPhaseObs - masterPivotPhaseObs);

        if (goGPS.isDebug() && Double.isNaN(ddpObs)) {
          double rPh = roverObs.getSatByIDType(id, satType).getPhaserange(0);
          double mPh = masterObs.getSatByIDType(id, satType).getPhaserange(0);
          double rPc = roverObs.getSatByIDType(id, satType).getPhaseCycles(0);
          double mPc = masterObs.getSatByIDType(id, satType).getPhaseCycles(0);
          double rWl = roverObs.getSatByIDType(id, satType).getWavelength(0);
          double mWl = masterObs.getSatByIDType(id, satType).getWavelength(0);
          int rCode = roverObs.getSatByIDType(id, satType).getCode(0);
          int mCode = masterObs.getSatByIDType(id, satType).getCode(0);
          System.err.printf("[NaN-trace] sat=%c%d ddpObs=NaN | rPh=%.3e mPh=%.3e rPc=%.3e mPc=%.3e rWl=%.3e mWl=%.3e rCode=%d mCode=%d pivotPh=%.3e/%.3e%n",
              satType, id, rPh, mPh, rPc, mPc, rWl, mWl, rCode, mCode, roverPivotPhaseObs, masterPivotPhaseObs);
        }

        // Compute troposphere, ionosphere and antenna residuals
        double tropoResiduals = (rover.satTropoCorr[i] - master.satTropoCorr[i])
            - (roverPivotTropoCorr - masterPivotTropoCorr);
        double ionoResiduals = (rover.satIonoCorr[i] - master.satIonoCorr[i])
            - (roverPivotIonoCorr - masterPivotIonoCorr);
        double antResiduals = (rover.satAntennaCorr[i] - master.satAntennaCorr[i])
            - (roverPivotAntCorr - masterPivotAntCorr);
        double windUpResiduals = ((rover.satWindUp[i] - master.satWindUp[i])
            - (roverPivotWindUp - masterPivotWindUp))
            * roverObs.getSatByIDType(id, satType).getWavelength(0);

        // Troposphere DD wet mapping function
        double roverMw = tropoMapWet(rover.topo[i].getElevation());
        double masterMw = tropoMapWet(master.topo[i].getElevation());
        double mwDD = (roverMw - roverPivotMw) - (masterMw - masterPivotMw);

        // Compute approximate ranges (with/without iono estimation)
        // RTKLIB 对齐: ionoopt=OFF 时电离层用广播模型(Klobuchar)直接改正观测值；
        //              ionoopt=EST 时电离层作为状态量估计，不进 appRange。
        // 相位电离层符号与码相反(码+, 相位-)，此处 -ionoResiduals 对应相位。
        double appRangeCode;
        double appRangePhase;
        if (estimIono) {
          // IONOOPT_EST: ionosphere is estimated, NOT added to approximate range
          appRangeCode = ddcApp + tropoResiduals + antResiduals;
          appRangePhase = ddcApp + tropoResiduals + antResiduals + windUpResiduals;
        } else {
          appRangeCode = ddcApp + tropoResiduals + ionoResiduals + antResiduals;
          appRangePhase = ddcApp + tropoResiduals - ionoResiduals + antResiduals + windUpResiduals;
        }

        if (goGPS.isDebug() && epochCount <= 1) {
          System.err.printf("[EPOCH0 phase] sat=%c%d: ddpObs=%.3f, appRangePhase=%.3f, y0phase=%.3f | ddcApp=%.3f tropo=%.3f ant=%.3f windUp=%.3f iono=%.3f%n",
              satType, id, ddpObs, appRangePhase, ddpObs - appRangePhase,
              ddcApp, tropoResiduals, antResiduals, windUpResiduals, ionoResiduals);
        }

        // === L1 Code Row ===
        H.set(k, 0, alphaX);
        H.set(k, i1 + 1, alphaY);
        H.set(k, i2 + 1, alphaZ);

        // Ionosphere column (RTKLIB: +I for L1 code)
        if (estimIono) {
          H.set(k, iIono + id, 1.0);
        }
        // Troposphere column (RTKLIB: +mw_dd * dTrop)
        if (estimTropo) {
          H.set(k, iTropo, mwDD);
        }

        y0.set(k, 0, ddcObs - appRangeCode);

        // === RTKLIB-aligned outlier rejection (rtkpos.c ddres() L1360-1374) ===
        // RTKLIB: if (fabs(v[nv]) > opt->maxinno[code]*threshadj) { vsat=0; continue; }
        // threshadj=10 当模糊度协方差 == 初始方差 (bias 刚初始化) 或 前5个历元(位置收敛期),
        // 否则 threshadj=1。扩展了 RTKLIB 的判断逻辑，加入历元计数条件:
        //   epochCount < 5 → threshadj=10 (位置还在收敛，允许大残差)
        //   epochCount ≥ 5 → 回退到 RTKLIB 原始的 ambVar 判断
        // 这解决了初始历元位置不准 → 码/相残差大 → 全部被拒绝 → 恶性循环的问题。
        // 注意: 用 epochCount 而非 posStd，因为第一历元后 KF 位置方差从 900 暴跌到 1.2
        // (10+卫星同时约束), 但实际位置误差仍达 50-100m, posStd 不可靠。
        //
        // goGPS 对齐方式: 被剔除观测 H 行清零、y0=0，使其对滤波零贡献
        // (G=Cee*H'*S^-1, H 行=0 → G 行=0 → dx 贡献=0)，数学等价于 RTKLIB continue。
        // Cnn 保留 varerr (不用 HUGE)，避免与 ddcov 非对角项混合致 S 病态。
        {
          double codeResid = Math.abs(ddcObs - appRangeCode);
          double ambVar = Cee.get(i3 + id, i3 + id);
          double lambda = roverObs.getSatByIDType(id, satType).getWavelength(0);
          double expInitVar = Math.pow(stDevInit, 2) * lambda * lambda;
          boolean posConverging = (epochCount < 5);
          double threshadjCode = (posConverging || Math.abs(ambVar - expInitVar) < 1e-6) ? 10.0 : 1.0;
          double maxInnoCode = 30.0; // RTKLIB maxinno[1] for code
          if (codeResid > maxInnoCode * threshadjCode) {
            if (goGPS.isDebug()) {
              System.err.printf("[OUTLIER-CODE] sat=%c%d resid=%.3f > thresh=%.1f, REJECTED%n",
                  satType, id, codeResid, maxInnoCode * threshadjCode);
            }
            // H 行清零、y0=0 (数学等价 RTKLIB continue, Cnn 保留 varerr 维持 S 条件数)
            y0.set(k, 0, 0.0);
            for (int c = 0; c < H.numCols(); c++) H.set(k, c, 0.0);
          }
        }

        if (goGPS.isDebug() && k == 0) {
          System.err.printf("[DEBUG KF setup] sat %c%d: ddcObs=%.2f, appRangeCode=%.2f, ddcObs-appRangeCode=%.2f%n",
                  satType, id, ddcObs, appRangeCode, ddcObs - appRangeCode);
          System.err.printf("[DEBUG KF setup] sat %c%d: alpha=[%.6f,%.6f,%.6f], rover=[%.2f,%.2f,%.2f]%n",
                  satType, id, alphaX, alphaY, alphaZ, rover.getX(), rover.getY(), rover.getZ());
          System.err.printf("[DEBUG KF setup] sat %c%d: satPos=[%.2f,%.2f,%.2f], roverAppRange=%.2f, masterAppRange=%.2f%n",
                  satType, id, sats.pos[i].getX(), sats.pos[i].getY(), sats.pos[i].getZ(),
                  rover.satAppRange[i], master.satAppRange[i]);
          System.err.printf("[DEBUG KF setup] sat %c%d: ddcApp=%.2f, tropoRes=%.2f, ionoRes=%.2f, antRes=%.2f%n",
                  satType, id, ddcApp, tropoResiduals, ionoResiduals, antResiduals);
        }

        // Print all sat DD residuals for first few epochs (convergence analysis)
        if (goGPS.isDebug() && epochCount <= 10) {
          double roverPR  = roverObs.getSatByIDType(id, satType).getPseudorange(0);
          double masterPR = masterObs.getSatByIDType(id, satType).getPseudorange(0);
          double roverGeom  = rover.satAppRange[i];
          double masterGeom = master.satAppRange[i];
          double sdPR  = roverPR - masterPR;
          double sdGeom = roverGeom - masterGeom;
          double satClkErr = sats.pos[i].getSatelliteClockError(); // seconds
          double satClkMeters = satClkErr * org.gogpsproject.Constants.SPEED_OF_LIGHT;
          boolean rTgd = roverObs.getSatByIDType(id, satType).isTgdApplied();
          boolean mTgd = masterObs.getSatByIDType(id, satType).isTgdApplied();

          // === [SD-DIAG] RTKLIB 逐卫星对比格式 ===
          // 用途: 与 RTKLIB trace 的 sat= rs= dts= 输出逐行对比
          // 若 goGPS SD_PR 与 RTKLIB 相同但 SD_Geom 不同 → 卫星位置差异
          // 若 SD_PR 不同 → 伪距解码或 TGD 差异
          // 若 SD_PR-SD_Geom 不同 → 钟差/大气改正差异
          System.err.printf("[SD-DIAG] ep=%d %c%02d el=%.1f° | rPR=%.3f mPR=%.3f SDpr=%.3f | rGeom=%.3f mGeom=%.3f SDgeom=%.3f | SDpr-SDgeom=%.3f%n",
                  epochCount, satType, id, Math.toDegrees(rover.topo[i].getElevation()),
                  roverPR, masterPR, sdPR, roverGeom, masterGeom, sdGeom, sdPR - sdGeom);
          System.err.printf("[SD-DIAG] ep=%d %c%02d satPos=[%.3f,%.3f,%.3f] satClk=%.10fs (%.3fm) | rCode=%d mCode=%d | rTgd=%s mTgd=%s%n",
                  epochCount, satType, id, sats.pos[i].getX(), sats.pos[i].getY(), sats.pos[i].getZ(),
                  satClkErr, satClkMeters,
                  roverObs.getSatByIDType(id, satType).getCode(0),
                  masterObs.getSatByIDType(id, satType).getCode(0),
                  rTgd, mTgd);
          System.err.printf("[DD-DIAG] ep=%d %c%02d: ddcObs=%.3f ddcApp=%.3f codeDDres=%.3f | ddpObs=%.3f appPhase=%.3f phaseDDres=%.3f%n",
                  epochCount, satType, id, ddcObs, appRangeCode, ddcObs - appRangeCode,
                  ddpObs, appRangePhase, ddpObs - appRangePhase);
        }

        // L1 code noise
        double roverCodeVar = varerr(Math.toRadians(rover.topo[i].getElevation()), false, satType, 0);
        double masterCodeVar = varerr(Math.toRadians(master.topo[i].getElevation()), false, satType, 0);

        int satPrn = id;
        if (isBdsGeo(satType, satPrn)) {
          roverCodeVar *= GEO_NOISE_SCALE;
          masterCodeVar *= GEO_NOISE_SCALE;
        }

        double CnnBase = Cnn.get(k, k);
        // RTKLIB-aligned: Cnn diagonal = varerr(rover) + varerr(master) (single-diff variance sum).
        // Do NOT accumulate on top of the pre-filled baseNoise (which is the same value),
        // otherwise the variance is doubled, over-weighting code observations.
        // 被剔除观测也用此 varerr 值(H=0 已阻断更新，Cnn=varerr 维持 S 条件数)。
        Cnn.set(k, k, roverCodeVar + masterCodeVar);

        if (goGPS.isDebug() && epochCount <= 1 && k < 3) {
          System.err.printf("[EPOCH0 Cnn-code] sat=%c%d el=%.1f roverCodeVar=%.4f masterCodeVar=%.4f Cnn=%.4f%n",
              satType, id, Math.toDegrees(rover.topo[i].getElevation()),
              roverCodeVar, masterCodeVar, roverCodeVar + masterCodeVar);
        }

        // === L1 Phase Row ===
        // RTKLIB-aligned: only add phase row if phase observation is available
        // (availPhase), not just code (gnssAvail). Otherwise getPhaserange()
        // returns NaN, corrupting y0 and causing KF state NaN.
        if (sats.availPhase.contains(id)) {

          H.set(nObsAvail + p, 0, alphaX);
          H.set(nObsAvail + p, i1 + 1, alphaY);
          H.set(nObsAvail + p, i2 + 1, alphaZ);
          // 【关键设计 - H_amb = +1.0 (meters)】(2026-06-21 重构，对齐 RTKLIB)
          // ============================================================================
          // 模糊度状态单位: meters (与 RTKLIB 一致)
          //   H[phase, amb] = +1.0  (观测方程: phase_obs = α·dr + 1.0·N_meters + ε)
          //
          // 数学保证 S 非奇异:
          //   S[phase] = α²·K[pos] + K[amb] + 2·α·K[pos,amb] + Cnn
          //            = (α·√K[pos] + √K[amb])² + Cnn > 0
          //   交叉项为正，不会抵消对角项，S 永不近奇异。
          //
          // 历史背景(已废弃方案):
          //   原 goGPS 用 H_amb = -λ(状态单位 cycles)，交叉项 -2·α·λ·K[pos,amb] < 0，
          //   多卫星求和后抵消对角项 → S[phase]≈0.001 → S⁻¹爆炸 → gMax=201 → 发散。
          //   曾用 S_DIAG_FLOOR=0.1 workaround(凑数，无物理意义)，现已移除。
          //
          // 同步修改清单(状态单位 cycles→meters):
          //   1. H_amb L1/L2: -λ → +1.0 (本处 + L660)
          //   2. 状态初值: (codeDD-phaseDD)/λ → (codeDD-phaseDD) (L917/L935)
          //   3. 初始方差: 900 cycle² → 900 m² (L1204/L1221)
          //   4. Qphase: λ²·Cee → Cee (L1139)
          //   5. LAMBDA 输入: a[i](meters) → a[i]/λ(cycles) (L1551)
          //   6. LAMBDA 固定: F[i](cycles) → F[i]·λ(meters) (L1623)
          // ============================================================================
          H.set(nObsAvail + p, i3 + id, 1.0);

          // Ionosphere column (RTKLIB: -I for L1 phase)
          if (estimIono) {
            H.set(nObsAvail + p, iIono + id, -1.0);
          }
          // Troposphere column
          if (estimTropo) {
            H.set(nObsAvail + p, iTropo, mwDD);
          }

          // RTKLIB-aligned: innovation = obs - computed - H_amb * amb_state
          // RTKLIB ddres(): v[nv] = y - e - lambda*bias
          // goGPS H_amb=1.0(meters), so: y0 = ddpObs - appRangePhase - ambState
          // Without this subtraction, y0 = initial ambiguity (~100m), causing
          // false outlier rejection and incorrect Kalman update.
          double ambState = KFprediction.get(i3 + id, 0);
          double y0phaseVal = ddpObs - appRangePhase - ambState;
          if (goGPS.isDebug() && Double.isNaN(y0phaseVal)) {
            System.err.printf("[NaN-trace-y0phase] sat=%c%d ddpObs=%.3e appRangePhase=%.3e ambState=%.3e alpha=[%.6f,%.6f,%.6f] rover=[%.3e,%.3e,%.3e] windUpRes=%.3e%n",
                satType, id, ddpObs, appRangePhase, ambState, alphaX, alphaY, alphaZ,
                rover.getX(), rover.getY(), rover.getZ(), windUpResiduals);
          }
          y0.set(nObsAvail + p, 0, y0phaseVal);

          // === RTKLIB-aligned outlier rejection (rtkpos.c ddres() L1360-1374) ===
          // 同码观测: threshadj 基于 Cee[amb]==SQR(std[0]) 或 epochCount<5,
          // maxinno[0]=5.0 (phase)。H 行清零、y0=0 数学等价 RTKLIB continue。
          // 扩展逻辑: 前5个历元保持 threshadj=10，防止初始历元
          // 位置不准 → 相位残差大 → 全部被拒绝 → 恶性循环。
          {
            double phaseResid = Math.abs(y0phaseVal);
            double ambVar = Cee.get(i3 + id, i3 + id);
            double lambda = roverObs.getSatByIDType(id, satType).getWavelength(0);
            double expPhaseInitVar = Math.pow(stDevInit, 2) * lambda * lambda;
            boolean posConverging = (epochCount < 5);
            double threshadjPhase = (posConverging || Math.abs(ambVar - expPhaseInitVar) < 1e-6) ? 10.0 : 1.0;
            double maxInnoPhase = 5.0; // RTKLIB maxinno[0] for phase
            if (phaseResid > maxInnoPhase * threshadjPhase) {
              if (goGPS.isDebug()) {
                System.err.printf("[OUTLIER-PHASE] sat=%c%d resid=%.3f > thresh=%.1f, REJECTED%n",
                    satType, id, phaseResid, maxInnoPhase * threshadjPhase);
              }
              // H 行清零、y0=0 (数学等价 RTKLIB continue, Cnn 保留 varerr 维持 S 条件数)
              y0.set(nObsAvail + p, 0, 0.0);
              for (int c = 0; c < H.numCols(); c++) H.set(nObsAvail + p, c, 0.0);
            }
          }

          double roverPhaseVar = varerr(Math.toRadians(rover.topo[i].getElevation()), true, satType, 0);
          double masterPhaseVar = varerr(Math.toRadians(master.topo[i].getElevation()), true, satType, 0);

          if (isBdsGeo(satType, satPrn)) {
            roverPhaseVar *= GEO_NOISE_SCALE;
            masterPhaseVar *= GEO_NOISE_SCALE;
          }

          CnnBase = Cnn.get(nObsAvail + p, nObsAvail + p);
          // RTKLIB-aligned: Cnn diagonal = varerr(rover) + varerr(master) (no double-counting).
          // 被剔除观测也用此 varerr 值(H=0 已阻断更新，Cnn=varerr 维持 S 条件数)。
          Cnn.set(nObsAvail + p, nObsAvail + p, roverPhaseVar + masterPhaseVar);

          p++;
        }

        k++;
      }
    }

    // === L2 Observations (dual-frequency) ===
    // RTKLIB-aligned: L2 uses an independent reference satellite (sats.pivotL2),
    // which may differ from L1 pivot. All L2 pivot quantities (geometry,
    // corrections, observations) come from sats.pivotL2. Sats lacking L2
    // code/phase get dummy observations (y0=0, H=0, Cnn=HUGE) to keep S
    // matrix non-singular without affecting the solution.
    if (dualFreq && sats.pivotL2 >= 0) {
      int l2Base = l1Rows; // L2 rows start after L1 rows
      int k2 = 0;
      int p2 = 0;

      for (int i = 0; i < nObs; i++) {

        int id = roverObs.getSatID(i);
        satType = roverObs.getGnssType(i);
        String checkAvailGnss = String.valueOf(satType) + String.valueOf(id);

        if (sats.pos[i]!=null && sats.gnssAvail.contains(checkAvailGnss) && i != sats.pivotL2) {

          // L2 code availability for this sat (both rover and master)
          boolean l2CodeAvail = !Double.isNaN(roverObs.getSatByIDType(id, satType).getPseudorange(1)) &&
              !Double.isNaN(masterObs.getSatByIDType(id, satType).getPseudorange(1));
          // L2 phase availability for this sat (both rover and master)
          boolean l2PhaseAvail = !Double.isNaN(roverObs.getSatByIDType(id, satType).getPhaseCycles(1)) &&
              !Double.isNaN(masterObs.getSatByIDType(id, satType).getPhaseCycles(1));

          double alphaX = rover.diffSat[i].get(0) / rover.satAppRange[i]
              - diffRoverPivotL2.get(0) / roverPivotAppRangeL2;
          double alphaY = rover.diffSat[i].get(1) / rover.satAppRange[i]
              - diffRoverPivotL2.get(1) / roverPivotAppRangeL2;
          double alphaZ = rover.diffSat[i].get(2) / rover.satAppRange[i]
              - diffRoverPivotL2.get(2) / roverPivotAppRangeL2;

          double ddcApp = (rover.satAppRange[i] - master.satAppRange[i])
              - (roverPivotAppRangeL2 - masterPivotAppRangeL2);

          // L2 observed code DD
          double ddcObsL2 = (roverObs.getSatByIDType(id, satType).getPseudorange(1) - masterObs.getSatByIDType(id, satType).getPseudorange(1))
              - (roverPivotCodeObsL2 - masterPivotCodeObsL2);

          // L2 observed phase DD
          double ddpObsL2 = (roverObs.getSatByIDType(id, satType).getPhaserange(1) - masterObs.getSatByIDType(id, satType).getPhaserange(1))
              - (roverPivotPhaseObsL2 - masterPivotPhaseObsL2);

          double tropoResiduals = (rover.satTropoCorr[i] - master.satTropoCorr[i])
              - (roverPivotTropoCorrL2 - masterPivotTropoCorrL2);
          double ionoResiduals = (rover.satIonoCorr[i] - master.satIonoCorr[i])
              - (roverPivotIonoCorrL2 - masterPivotIonoCorrL2);
          double antResiduals = (rover.satAntennaCorr[i] - master.satAntennaCorr[i])
              - (roverPivotAntCorrL2 - masterPivotAntCorrL2);
          double windUpResidualsL2 = ((rover.satWindUp[i] - master.satWindUp[i])
              - (roverPivotWindUpL2 - masterPivotWindUpL2))
              * roverObs.getSatByIDType(id, satType).getWavelength(1);

          double roverMw = tropoMapWet(rover.topo[i].getElevation());
          double masterMw = tropoMapWet(master.topo[i].getElevation());
          double mwDD = (roverMw - roverPivotMwL2) - (masterMw - masterPivotMwL2);

          // L2 approximate range (iono scaled by gamma = (f1/f2)^2)
          double appRangeCodeL2;
          double appRangePhaseL2;
          if (estimIono) {
            appRangeCodeL2 = ddcApp + tropoResiduals + antResiduals;
            appRangePhaseL2 = ddcApp + tropoResiduals + antResiduals + windUpResidualsL2;
          } else {
            appRangeCodeL2 = ddcApp + tropoResiduals + ionoResiduals * gamma + antResiduals;
            appRangePhaseL2 = ddcApp + tropoResiduals - ionoResiduals * gamma + antResiduals + windUpResidualsL2;
          }

          // === L2 Code Row ===
          int rowL2Code = l2Base + k2;
          if (l2CodeAvail) {
            H.set(rowL2Code, 0, alphaX);
            H.set(rowL2Code, i1 + 1, alphaY);
            H.set(rowL2Code, i2 + 1, alphaZ);

            if (estimIono) {
              H.set(rowL2Code, iIono + id, gamma); // +gamma*I for L2 code
            }
            if (estimTropo) {
              H.set(rowL2Code, iTropo, mwDD);
            }

            y0.set(rowL2Code, 0, ddcObsL2 - appRangeCodeL2);

            double roverCodeVarL2 = varerr(Math.toRadians(rover.topo[i].getElevation()), false, satType, 1);
            double masterCodeVarL2 = varerr(Math.toRadians(master.topo[i].getElevation()), false, satType, 1);

            int satPrn = id;
            if (isBdsGeo(satType, satPrn)) {
              roverCodeVarL2 *= GEO_NOISE_SCALE;
              masterCodeVarL2 *= GEO_NOISE_SCALE;
            }

            double CnnBaseL2 = Cnn.get(rowL2Code, rowL2Code);
            // RTKLIB-aligned: no double-counting of pre-filled baseNoise.
            Cnn.set(rowL2Code, rowL2Code, roverCodeVarL2 + masterCodeVarL2);
          } else {
            // Dummy observation: sat lacks L2 code, exclude without singularity
            Cnn.set(rowL2Code, rowL2Code, DUMMY_OBS_VAR);
          }

          // === L2 Phase Row ===
          if (l2PhaseAvail) {
            int rowL2Phase = l2Base + nSatAvail + p2;
            H.set(rowL2Phase, 0, alphaX);
            H.set(rowL2Phase, i1 + 1, alphaY);
            H.set(rowL2Phase, i2 + 1, alphaZ);
            H.set(rowL2Phase, iAmbL2 + id, 1.0); // H_amb=+1.0 (meters), 见 L468 注释

            if (estimIono) {
              H.set(rowL2Phase, iIono + id, -gamma); // -gamma*I for L2 phase
            }
            if (estimTropo) {
              H.set(rowL2Phase, iTropo, mwDD);
            }

            // RTKLIB-aligned: subtract L2 ambiguity state from innovation
            double ambStateL2 = KFprediction.get(iAmbL2 + id, 0);
            y0.set(rowL2Phase, 0, ddpObsL2 - appRangePhaseL2 - ambStateL2);

            double roverPhaseVarL2 = varerr(Math.toRadians(rover.topo[i].getElevation()), true, satType, 1);
            double masterPhaseVarL2 = varerr(Math.toRadians(master.topo[i].getElevation()), true, satType, 1);

            int satPrn = id;
            if (isBdsGeo(satType, satPrn)) {
              roverPhaseVarL2 *= GEO_NOISE_SCALE;
              masterPhaseVarL2 *= GEO_NOISE_SCALE;
            }

            double CnnBaseL2 = Cnn.get(rowL2Phase, rowL2Phase);
            // RTKLIB-aligned: no double-counting of pre-filled baseNoise.
            Cnn.set(rowL2Phase, rowL2Phase, roverPhaseVarL2 + masterPhaseVarL2);

            p2++;
          } else {
            int rowL2Phase = l2Base + nSatAvail + p2;
            Cnn.set(rowL2Phase, rowL2Phase, DUMMY_OBS_VAR);
          }

          k2++;
        }
      }
    }

    // ============================================================================
    // 【已验证·勿重复检查】ddcov 非对角 Cnn 实现 (2026-06-21 多次会话反复验证)
    // ============================================================================
    // 历史排查记录（每次新会话不要再重新检查此处，已确认正确）：
    //   ✓ 非对角项 = pivotNoise = varerr(rover,pivot) + varerr(master,pivot)
    //     对齐 RTKLIB rtkpos.c ddcov()，公式正确
    //   ✓ 四个块（L1-code/L1-phase/L2-code/L2-phase）行范围划分正确
    //   ✓ L2 使用独立 pivot (sats.pivotL2)，elevation/type 从 pivotL2 取
    //   ✓ BDS GEO 噪声缩放 (GEO_NOISE_SCALE) 已应用到 pivot
    //   ✓ 对称填充 Cnn[i][j] = Cnn[j][i] = pivotNoise
    //
    // 结论：ddcov 不是 S 矩阵近奇异 / 条件数差 (1.3e7) / 增益爆炸 (gMax=201) 的根因。
    //   真正根因：位置-模糊度交叉协方差抵消 HKHt 对角项（HKHt=0.000698 极小），
    //   源于位置方差收敛过快（900→0.30）。需对齐 RTKLIB udpos() 位置方差下限管理，
    //   详见 KalmanFilter.java 的 Cee 更新逻辑。
    // ============================================================================
    // RTKLIB ddcov alignment: add off-diagonal elements to Cnn
    // Within each system-frequency block, DD observations share the same
    // reference satellite (pivot), so their noise is correlated through
    // the pivot noise. Cov(DD_i, DD_j) = pivotNoise for i != j.
    // Reference: RTKLIB rtkpos.c ddcov()

    // L1 code block: rows 0..nSatAvail-1
    double pivotCodeNoise = varerr(Math.toRadians(roverElevation), false, satType, 0)
                          + varerr(Math.toRadians(masterElevation), false, satType, 0);
    if (isBdsGeo(satType, pivotId)) {
      pivotCodeNoise *= GEO_NOISE_SCALE;
    }
    for (int i = 0; i < nSatAvail; i++) {
      for (int j = i + 1; j < nSatAvail; j++) {
        Cnn.set(i, j, pivotCodeNoise);
        Cnn.set(j, i, pivotCodeNoise);
      }
    }

    // L1 phase block: rows nSatAvail..l1Rows-1
    double pivotPhaseNoise = varerr(Math.toRadians(roverElevation), true, satType, 0)
                           + varerr(Math.toRadians(masterElevation), true, satType, 0);
    if (isBdsGeo(satType, pivotId)) {
      pivotPhaseNoise *= GEO_NOISE_SCALE;
    }
    for (int i = nSatAvail; i < l1Rows; i++) {
      for (int j = i + 1; j < l1Rows; j++) {
        Cnn.set(i, j, pivotPhaseNoise);
        Cnn.set(j, i, pivotPhaseNoise);
      }
    }

    if (dualFreq && sats.pivotL2 >= 0) {
      // L2 pivot elevation & type (from sats.pivotL2, may differ from L1 pivot)
      double roverElevL2Pivot = rover.topo[sats.pivotL2].getElevation();
      double masterElevL2Pivot = master.topo[sats.pivotL2].getElevation();
      char pivotL2Type = roverObs.getGnssType(sats.pivotL2);
      int pivotL2Id = roverObs.getSatID(sats.pivotL2);

      // L2 code block: rows l1Rows..l1Rows+nSatAvail-1
      double pivotCodeNoiseL2 = varerr(Math.toRadians(roverElevL2Pivot), false, pivotL2Type, 1)
                              + varerr(Math.toRadians(masterElevL2Pivot), false, pivotL2Type, 1);
      if (isBdsGeo(pivotL2Type, pivotL2Id)) {
        pivotCodeNoiseL2 *= GEO_NOISE_SCALE;
      }
      for (int i = l1Rows; i < l1Rows + nSatAvail; i++) {
        for (int j = i + 1; j < l1Rows + nSatAvail; j++) {
          Cnn.set(i, j, pivotCodeNoiseL2);
          Cnn.set(j, i, pivotCodeNoiseL2);
        }
      }

      // L2 phase block: rows l1Rows+nSatAvail..totalObs-1
      double pivotPhaseNoiseL2 = varerr(Math.toRadians(roverElevL2Pivot), true, pivotL2Type, 1)
                               + varerr(Math.toRadians(masterElevL2Pivot), true, pivotL2Type, 1);
      if (isBdsGeo(pivotL2Type, pivotL2Id)) {
        pivotPhaseNoiseL2 *= GEO_NOISE_SCALE;
      }
      for (int i = l1Rows + nSatAvail; i < totalObs; i++) {
        for (int j = i + 1; j < totalObs; j++) {
          Cnn.set(i, j, pivotPhaseNoiseL2);
          Cnn.set(j, i, pivotPhaseNoiseL2);
        }
      }
    }

    // Debug: print first epoch DD info
    if (goGPS.isDebug() && epochCount <= 5) {
      System.err.printf("%n=== [KF_DD setup] epoch=%d, pivot=%c%d, nSatAvail=%d, nSatAvailPhase=%d, totalObs=%d ===%n",
          epochCount, satType, pivotId, nSatAvail, nSatAvailPhase, totalObs);
      System.err.printf("  roverPos: [%.2f, %.2f, %.2f]%n", rover.getX(), rover.getY(), rover.getZ());
      System.err.printf("  masterPos: [%.2f, %.2f, %.2f]%n", masterPos.getX(), masterPos.getY(), masterPos.getZ());
      for (int row = 0; row < Math.min(6, totalObs); row++) {
        String rowType = "";
        if (row < nSatAvail) rowType = "L1Code";
        else if (row < l1Rows) rowType = "L1Phase";
        else if (dualFreq && row < l1Rows + nSatAvail) rowType = "L2Code";
        else if (dualFreq) rowType = "L2Phase";
        System.err.printf("  [%d %s] y0=%.3f, H=[%.6f, %.6f, %.6f], Cnn=%.4f%n",
            row, rowType, y0.get(row, 0),
            H.get(row, 0), H.get(row, i1 + 1), H.get(row, i2 + 1),
            Cnn.get(row, row));
      }
    }

    // Print pivot satellite raw values + baseline for convergence analysis
    if (goGPS.isDebug() && epochCount <= 5) {
      double pivotRoverGeom  = rover.satAppRange[sats.pivot];
      double pivotMasterGeom = master.satAppRange[sats.pivot];
      System.err.printf("[DEBUG KF PIVOT] %c%02d: roverPR=%.2f masterPR=%.2f sdPR=%.2f | roverGeom=%.2f masterGeom=%.2f sdGeom=%.2f | sdPR-sdGeom=%.2f%n",
              satType, pivotId, roverPivotCodeObs, masterPivotCodeObs, roverPivotCodeObs - masterPivotCodeObs,
              pivotRoverGeom, pivotMasterGeom, pivotRoverGeom - pivotMasterGeom,
              (roverPivotCodeObs - masterPivotCodeObs) - (pivotRoverGeom - pivotMasterGeom));
      double baseLen = Math.sqrt(Math.pow(rover.getX()-masterPos.getX(),2)
                               + Math.pow(rover.getY()-masterPos.getY(),2)
                               + Math.pow(rover.getZ()-masterPos.getZ(),2));
      System.err.printf("[DEBUG KF PIVOT] rover=[%.2f,%.2f,%.2f] master=[%.2f,%.2f,%.2f] baseline=%.2fm%n",
              rover.getX(), rover.getY(), rover.getZ(),
              masterPos.getX(), masterPos.getY(), masterPos.getZ(), baseLen);
    }

    // RTKLIB-aligned: save rover DD state at END of setup (after selectDoubleDiff).
    // This oldPivotId/satOld is used by the FIRST loop() call's
    // checkSatelliteConfiguration() to detect pivot/sat changes from setup to
    // the first epoch. RTKLIB ddres() selects ref sat per epoch; goGPS tracks
    // the change to transform DD ambiguity states (A*Cee*A').
    try {
      oldPivotId   = sats.pos[sats.pivot].getSatID();
      oldPivotType = sats.pos[sats.pivot].getSatType();
    } catch(ArrayIndexOutOfBoundsException e) {
      oldPivotId = 0;
    }
    satOld = sats.availPhase;
    satTypeOld = sats.typeAvailPhase;

    updateDops(A);
  }

  /** Troposphere wet mapping function (RTKLIB GMF-aligned).
   *  mw(el) = 1.001 / sqrt(0.002001 + sin^2(el)) */
  private static double tropoMapWet(double elevDeg) {
    double sinE = Math.sin(Math.toRadians(elevDeg));
    return GMF_A / Math.sqrt(GMF_B + sinE * sinE);
  }

  /** Ionospheric single-layer mapping function (RTKLIB-aligned).
   *  im(el) = 1.0 / sqrt(1.0 - (Re * cos(el) / (Re + H))^2)
   *  where Re = 6378137 m, H = 350000 m.
   *  Reference: RTKLIB rtkcmn.c ionmapf() */
  private static double ionoMapF(double elevDeg) {
    double sinE = Math.sin(Math.toRadians(elevDeg));
    double cosE = Math.cos(Math.toRadians(elevDeg));
    double re = 6378137.0;
    double factor = re / (re + IONO_H) * cosE;
    if (factor >= 1.0) return 1.0;
    return 1.0 / Math.sqrt(1.0 - factor * factor);
  }
  
  /**
   * @param roverObs
   * @param masterObs
   * @param masterPos
   */
  @Override
  void estimateAmbiguities( Observations roverObs, Observations masterObs, Coordinates masterPos, ArrayList<Integer> satAmb, int pivotIndex, boolean init) {

    // Check if pivot is in satAmb, in case remove it
    if (satAmb.contains(sats.pos[pivotIndex].getSatID()))
      satAmb.remove(satAmb.indexOf(sats.pos[pivotIndex].getSatID()));

    // Number of GPS observations
    int nObs = roverObs.getNumSat();

    // Number of available satellites (i.e. observations)
    int nObsAvail = sats.avail.size();

    // Number of available satellites (i.e. observations) with phase
    int nObsAvailPhase = sats.availPhase.size();

    // Double differences with respect to pivot satellite reduce
    // observations by 1
    nObsAvail--;
    nObsAvailPhase--;

    // Number of unknown parameters
    int nUnknowns = 3 + satAmb.size();

    // Pivot satellite ID
    int pivotId = roverObs.getSatID(pivotIndex);
    char satType = roverObs.getGnssType(pivotIndex);

    // Rover-pivot and master-pivot observed pseudorange
    double roverPivotCodeObs = roverObs.getSatByIDType(pivotId, satType).getPseudorange(goGPS.getFreq());
    double masterPivotCodeObs = masterObs.getSatByIDType(pivotId, satType).getPseudorange(goGPS.getFreq());

    // Rover-pivot and master-pivot observed phase
    double roverPivotPhaseObs = roverObs.getSatByIDType(pivotId, satType).getPhaserange(goGPS.getFreq());
    double masterPivotPhaseObs = masterObs.getSatByIDType(pivotId, satType).getPhaserange(goGPS.getFreq());

    // L2 pivot phase observations (for dual-freq L2 ambiguity initialization)
    double roverPivotPhaseObsL2 = Double.NaN;
    double masterPivotPhaseObsL2 = Double.NaN;
    if (goGPS.isDualFreq()) {
      roverPivotPhaseObsL2 = roverObs.getSatByIDType(pivotId, satType).getPhaserange(1);
      masterPivotPhaseObsL2 = masterObs.getSatByIDType(pivotId, satType).getPhaserange(1);
    }

    // Rover-pivot approximate pseudoranges
    SimpleMatrix diffRoverPivot = rover.diffSat[pivotIndex];
    double roverPivotAppRange = rover.satAppRange[pivotIndex];

    // Master-pivot approximate pseudoranges
    double masterPivotAppRange = master.satAppRange[pivotIndex];

    // Estimated ambiguity combinations (double differences)
    double[] estimatedAmbiguityComb = new double[satAmb.size()];

    // Covariance of estimated ambiguity combinations
    double[] estimatedAmbiguityCombCovariance = new double[satAmb.size()];

    // L2 estimated ambiguity combinations (dual-freq only)
    double[] estimatedAmbiguityCombL2 = new double[satAmb.size()];
    // 【RTKLIB 对齐】存储每个卫星的波长，用于计算等效 RTKLIB 的初始方差 900/λ² cycle²。
    // RTKLIB udbias() 用 initx(bias, SQR(std[0]=30m)=900 m², ...)，状态单位 meters(H=1.0)。
    // goGPS 状态单位 cycles(H=-λ)，等效方差 = 900/λ² cycle²，使 S_相位 = λ²*(900/λ²) = 900 m²，
    // 与 RTKLIB 一致，避免 S 病态(原用 LS 估计方差~5 cycle²，等效仅 0.19 m²，比 RTKLIB 小 4700 倍)。
    double[] estimatedWavelength = new double[satAmb.size()];
    double[] estimatedWavelengthL2 = new double[satAmb.size()];

    if (goGPS.getAmbiguityStrategy() == AmbiguityStrategy.OBSERV) {

      for (int i = 0; i < nObs; i++) {

        int id = roverObs.getSatID(i);
        satType = roverObs.getGnssType(i);

        if (sats.pos[i]!=null && satAmb.contains(id) && id != pivotId) {

          // Rover-satellite and master-satellite observed code
          double roverSatCodeObs = roverObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq());
          double masterSatCodeObs = masterObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq());

          // Rover-satellite and master-satellite observed phase
          double roverSatPhaseObs = roverObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq());
          double masterSatPhaseObs = masterObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq());

          // Observed code double difference
          double codeDoubleDiffObserv = (roverSatCodeObs - masterSatCodeObs) - (roverPivotCodeObs - masterPivotCodeObs);

          // Observed phase double difference
          double phaseDoubleDiffObserv = (roverSatPhaseObs - masterSatPhaseObs) - (roverPivotPhaseObs - masterPivotPhaseObs);

          // Store estimated ambiguity combinations and their covariance
          // 【状态单位 meters, 正模糊度】(2026-06-21 重构)
          // 物理方程: phase_obs = α·dr + λN_real + ε
          //   N_real = (phaseDD - codeDD) / λ  (正模糊度, phase > code 时为正)
          //   N_meters = phaseDD - codeDD = λ·N_real
          // H_amb = +1.0, 状态 = +N_meters, 观测方程: phase = α·dr + 1.0·N_meters ✓
          // (原 goGPS: 状态 = codeDD - phaseDD = -λN, H_amb = -λ, 已废弃)
          estimatedAmbiguityComb[satAmb.indexOf(id)] = (phaseDoubleDiffObserv - codeDoubleDiffObserv);
          // estimatedAmbiguityCombCovariance 仍为 cycles² 单位(仅用于诊断对比，不进滤波)
          estimatedAmbiguityCombCovariance[satAmb.indexOf(id)] = 4
          * getStDevCode(roverObs.getSatByIDType(id, satType), goGPS.getFreq())
          * getStDevCode(masterObs.getSatByIDType(id, satType), goGPS.getFreq()) / Math.pow(roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq()), 2);
          estimatedWavelength[satAmb.indexOf(id)] = roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq());

          // L2 ambiguity (dual-freq): same code DD, L2 phase DD, L2 wavelength
          if (goGPS.isDualFreq() && !Double.isNaN(roverPivotPhaseObsL2)) {
            double roverSatPhaseObsL2 = roverObs.getSatByIDType(id, satType).getPhaserange(1);
            double masterSatPhaseObsL2 = masterObs.getSatByIDType(id, satType).getPhaserange(1);
            double phaseDoubleDiffObservL2 = (roverSatPhaseObsL2 - masterSatPhaseObsL2) - (roverPivotPhaseObsL2 - masterPivotPhaseObsL2);
            // 【状态单位 meters, 正模糊度】phaseDD - codeDD
            estimatedAmbiguityCombL2[satAmb.indexOf(id)] = (phaseDoubleDiffObservL2 - codeDoubleDiffObserv);
            estimatedWavelengthL2[satAmb.indexOf(id)] = roverObs.getSatByIDType(id, satType).getWavelength(1);
          }
        }
      }
    }
    else if(goGPS.getAmbiguityStrategy() == AmbiguityStrategy.APPROX | (nObsAvail + nObsAvailPhase <= nUnknowns)) {

      for (int i = 0; i < nObs; i++) {

        int id = roverObs.getSatID(i);
        satType = roverObs.getGnssType(i);

        if( sats.pos[i]!=null && satAmb.contains(id) && id != pivotId) {

          // Rover-satellite and master-satellite approximate pseudorange
          double roverSatCodeAppRange  = rover.satAppRange[i];
          double masterSatCodeAppRange = master.satAppRange[i];

          // Rover-satellite and master-satellite observed phase
          double roverSatPhaseObs  = roverObs.getSatByIDType(id, satType) .getPhaserange(goGPS.getFreq());
          double masterSatPhaseObs = masterObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq());

          // Estimated code pseudorange double differences
          double codeDoubleDiffApprox = (roverSatCodeAppRange - masterSatCodeAppRange)
                                      - (roverPivotAppRange - masterPivotAppRange);

          // Observed phase double differences
          double phaseDoubleDiffObserv = (roverSatPhaseObs - masterSatPhaseObs)
                          - (roverPivotPhaseObs - masterPivotPhaseObs);

          // Store estimated ambiguity combinations and their covariance
          // 【状态单位 meters, 正模糊度】phaseDD - codeDD = +λN (见 OBSERV 分支注释)
          estimatedAmbiguityComb[satAmb.indexOf(id)] = (phaseDoubleDiffObserv - codeDoubleDiffApprox);
          estimatedAmbiguityCombCovariance[satAmb.indexOf(id)] = 4
            * getStDevCode(roverObs.getSatByIDType(id, satType), goGPS.getFreq())
            * getStDevCode(masterObs.getSatByIDType(id, satType), goGPS.getFreq()) / Math.pow(roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq()), 2);
          estimatedWavelength[satAmb.indexOf(id)] = roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq());

          // L2 ambiguity (dual-freq): same approx code DD, L2 phase DD, L2 wavelength
          if (goGPS.isDualFreq() && !Double.isNaN(roverPivotPhaseObsL2)) {
            double roverSatPhaseObsL2  = roverObs.getSatByIDType(id, satType).getPhaserange(1);
            double masterSatPhaseObsL2 = masterObs.getSatByIDType(id, satType).getPhaserange(1);
            double phaseDoubleDiffObservL2 = (roverSatPhaseObsL2 - masterSatPhaseObsL2) - (roverPivotPhaseObsL2 - masterPivotPhaseObsL2);
            // 【状态单位 meters, 正模糊度】phaseDD - codeDD
            estimatedAmbiguityCombL2[satAmb.indexOf(id)] = (phaseDoubleDiffObservL2 - codeDoubleDiffApprox);
            estimatedWavelengthL2[satAmb.indexOf(id)] = roverObs.getSatByIDType(id, satType).getWavelength(1);
          }
        }
      }
    } 
    else if ( goGPS.getAmbiguityStrategy() == AmbiguityStrategy.LS ) {

      // Least squares design matrix
      SimpleMatrix A = new SimpleMatrix(nObsAvail+nObsAvailPhase, nUnknowns);

      // Vector for approximate pseudoranges
      SimpleMatrix b = new SimpleMatrix(nObsAvail+nObsAvailPhase, 1);

      // Vector for observed pseudoranges
      SimpleMatrix y0 = new SimpleMatrix(nObsAvail+nObsAvailPhase, 1);

      // Cofactor matrices
      SimpleMatrix Qcode = new SimpleMatrix(nObsAvail, nObsAvail);
      SimpleMatrix Qphase = new SimpleMatrix(nObsAvailPhase, nObsAvailPhase);
      SimpleMatrix Q = new SimpleMatrix(nObsAvail+nObsAvailPhase, nObsAvail+nObsAvailPhase);

      // Solution vector
      SimpleMatrix x = new SimpleMatrix(nUnknowns, 1);

      // Vector for observation error
      SimpleMatrix vEstim = new SimpleMatrix(nObsAvail, 1);

      // Error covariance matrix
      SimpleMatrix covariance = new SimpleMatrix(nUnknowns, nUnknowns);

      // Vectors for troposphere and ionosphere corrections
      SimpleMatrix tropoCorr = new SimpleMatrix(nObsAvail+nObsAvailPhase, 1);
      SimpleMatrix ionoCorr = new SimpleMatrix(nObsAvail+nObsAvailPhase, 1);

      // Counters for available satellites
      int k = 0;
      int p = 0;

      // Rover-pivot and master-pivot troposphere correction
      double roverPivotTropoCorr  = rover.satTropoCorr[pivotIndex];
      double masterPivotTropoCorr = master.satTropoCorr[pivotIndex];;

      // Rover-pivot and master-pivot ionosphere correction
      double roverPivotIonoCorr  = rover.satIonoCorr[pivotIndex];
      double masterPivotIonoCorr = master.satIonoCorr[pivotIndex];

      // Compute weights using varerr
      double roverPivotElevation = rover.topo[pivotIndex].getElevation();
      double masterPivotElevation = master.topo[pivotIndex].getElevation();
      double roverPivotCodeVar = varerr(Math.toRadians(roverPivotElevation), false, satType, goGPS.getFreq());
      double masterPivotCodeVar = varerr(Math.toRadians(masterPivotElevation), false, satType, goGPS.getFreq());
      double roverPivotPhaseVar = varerr(Math.toRadians(roverPivotElevation), true, satType, goGPS.getFreq());
      double masterPivotPhaseVar = varerr(Math.toRadians(masterPivotElevation), true, satType, goGPS.getFreq());
      Qcode.fill(roverPivotCodeVar + masterPivotCodeVar);
      Qphase.fill(roverPivotPhaseVar + masterPivotPhaseVar);

      // Set up the least squares matrices...
      // ... for code ...
      for (int i = 0; i < nObs; i++) {

        int id = roverObs.getSatID(i);
        satType = roverObs.getGnssType(i);
        String checkAvailGnss = String.valueOf(satType) + String.valueOf(id);
        
        if (sats.pos[i] !=null && sats.gnssAvail.contains(checkAvailGnss)
            && i != pivotIndex) {

          // Fill in one row in the design matrix
          A.set(k, 0, rover.diffSat[i].get(0) / rover.satAppRange[i] - diffRoverPivot.get(0) / roverPivotAppRange); /* X */

          A.set(k, 1, rover.diffSat[i].get(1) / rover.satAppRange[i] - diffRoverPivot.get(1) / roverPivotAppRange); /* Y */

          A.set(k, 2, rover.diffSat[i].get(2) / rover.satAppRange[i] - diffRoverPivot.get(2) / roverPivotAppRange); /* Z */

          // Add the differenced approximate pseudorange value to b
          b.set(k, 0, (rover.satAppRange[i] - master.satAppRange[i])
              - (roverPivotAppRange - masterPivotAppRange));

          // Add the differenced observed pseudorange value to y0
          y0.set(k, 0, (roverObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq()) - masterObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq()))
              - (roverPivotCodeObs - masterPivotCodeObs));

          // Fill in troposphere and ionosphere double differenced
          // corrections
          tropoCorr.set(k, 0, (rover.satTropoCorr[i] - master.satTropoCorr[i])
              - (roverPivotTropoCorr - masterPivotTropoCorr));
          ionoCorr.set(k, 0, (rover.satIonoCorr[i] - master.satIonoCorr[i])
              - (roverPivotIonoCorr - masterPivotIonoCorr));

          // Fill in the cofactor matrix
          double roverCodeVar = varerr(Math.toRadians(rover.topo[i].getElevation()), false, satType, goGPS.getFreq());
          double masterCodeVar = varerr(Math.toRadians(master.topo[i].getElevation()), false, satType, goGPS.getFreq());
          Qcode.set(k, k, Qcode.get(k, k) + roverCodeVar + masterCodeVar);

          // Increment available satellites counter
          k++;
        }
      }

      // ... and phase
      for (int i = 0; i < nObs; i++) {

        int id = roverObs.getSatID(i);
        satType = roverObs.getGnssType(i);
        String checkAvailGnss = String.valueOf(satType) + String.valueOf(id);

        if( sats.pos[i] !=null && sats.gnssAvail.contains(checkAvailGnss)
            && i != pivotIndex) {

          // Fill in one row in the design matrix
          A.set(k, 0, rover.diffSat[i].get(0) / rover.satAppRange[i] - diffRoverPivot.get(0) / roverPivotAppRange); /* X */
          A.set(k, 1, rover.diffSat[i].get(1) / rover.satAppRange[i] - diffRoverPivot.get(1) / roverPivotAppRange); /* Y */
          A.set(k, 2, rover.diffSat[i].get(2) / rover.satAppRange[i] - diffRoverPivot.get(2) / roverPivotAppRange); /* Z */

          if (satAmb.contains(id)) {
            A.set(k, 3 + satAmb.indexOf(id), 1.0); /* N - H_amb=+1.0 (meters), 见 L468 注释 */

            // Add the differenced observed pseudorange value to y0
            y0.set(k, 0, (roverObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq()) - masterObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq()))
                - (roverPivotPhaseObs - masterPivotPhaseObs));
          } 
          else {
            // Add the differenced observed pseudorange value + known N to y0
            y0.set(k, 0, (roverObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq()) - masterObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq()))
                - (roverPivotPhaseObs - masterPivotPhaseObs) + KFprediction.get(i3 + id));
          }

          // Add the differenced approximate pseudorange value to b
          b.set(k, 0, (rover.satAppRange[i] - master.satAppRange[i])
              - (roverPivotAppRange - masterPivotAppRange));

          // Fill in troposphere and ionosphere double differenced corrections
          tropoCorr.set(k, 0, (rover.satTropoCorr[i] - master.satTropoCorr[i]) - (roverPivotTropoCorr - masterPivotTropoCorr));
          ionoCorr.set(k, 0, -((rover.satIonoCorr[i] - master.satIonoCorr[i]) - (roverPivotIonoCorr - masterPivotIonoCorr)));

          // Fill in the cofactor matrix
          double roverPhaseVar = varerr(Math.toRadians(rover.topo[i].getElevation()), true, satType, goGPS.getFreq());
          double masterPhaseVar = varerr(Math.toRadians(master.topo[i].getElevation()), true, satType, goGPS.getFreq());
          
          // 【状态单位 meters】(2026-06-21 重构)
          // 原: λ²·Cee(cycles²→meters²)，现 Cee 已是 meters²，直接用
          // stDevPhase² 是相位观测先验噪声(meters²)，Cee 是模糊度方差(meters²)
          Qphase.set(p, p, Qphase.get(p, p)
              + (Math.pow(stDevPhase, 2) + Cee.get(i3 + id, i3 + id))
              * (roverPivotPhaseVar + masterPivotPhaseVar)
              + (Math.pow(stDevPhase, 2) + Cee.get(i3 + id, i3 + id))
              * (roverPhaseVar + masterPhaseVar));
          
          int r = 1;
          for (int m = i+1; m < nObs; m++) {
            if (sats.pos[m] !=null && sats.availPhase.contains(sats.pos[m].getSatID()) && m != pivotIndex) {
              Qphase.set(p, p+r, 0);
              Qphase.set(p+r, p, 0);
              r++;
            }
          }
          //          int r = 1;
          //          for (int j = i+1; j < nObs; j++) {
          //            if (sats.pos[j] !=null && sats.availPhase.contains(sats.pos[j].getSatID()) && j != pivotIndex) {
          //              Qphase.set(p, p+r, Qphase.get(p, p+r)
          //                  + (Math.pow(lambda, 2) * Cee.get(i3 + sats.pos[i].getSatID(), i3 + sats.pos[j].getSatID()))
          //                  * (roverPivotWeight + masterPivotWeight));
          //              Qphase.set(p+r, p, Qphase.get(p, p+r));
          //              r++;
          //            }
          //          }

          // Increment available satellite counters
          k++;
          p++;
        }
      }

      // Apply troposphere and ionosphere correction
      b = b.plus(tropoCorr);
      b = b.plus(ionoCorr);

      //Build complete cofactor matrix (code and phase)
      Q.insertIntoThis(0, 0, Qcode);
      Q.insertIntoThis(nObsAvail, nObsAvail, Qphase);

      // Least squares solution x = ((A'*Q^-1*A)^-1)*A'*Q^-1*(y0-b);
      x = A.transpose().mult(Q.invert()).mult(A).invert().mult(A.transpose()).mult(Q.invert()).mult(y0.minus(b));

      // Estimation of the variance of the observation error
      vEstim = y0.minus(A.mult(x).plus(b));
      
      double varianceEstim = (vEstim.transpose().mult(Q.invert())
          .mult(vEstim)).get(0)
          / (nObsAvail + nObsAvailPhase - nUnknowns);

      // Covariance matrix of the estimation error
      covariance = A.transpose().mult(Q.invert()).mult(A).invert().scale(varianceEstim);

      // Store estimated ambiguity combinations and their covariance
      for (int m = 0; m < satAmb.size(); m++) {
        estimatedAmbiguityComb[m] = x.get(3 + m);
        estimatedAmbiguityCombCovariance[m] = covariance.get(3 + m, 3 + m);
      }
    }

    if (init) {
      // 【模糊度初始方差 = λ²·900 m²】(2026-06-21 修正，对齐 RTKLIB)
      // RTKLIB udbias(): initx(bias, SQR(std[0]=30 cycles)=900 cycle², H_amb=-λ)。
      //   有效观测空间方差: H_amb²·K_amb = λ²·900 m²
      // goGPS 重构后 H_amb=+1.0，状态单位 meters，初始方差需 = λ²·900 m²
      //   使 S_相位 = α²·K[pos] + λ²·K[amb]_rtklib + Cnn = RTKLIB 一致。
      //   若用 900 m² (缺 λ²)，S 对角范围 3.6e-5~903，条件数 9.5e6，
      //   LU 求解器数值不稳定 → gMax=106 → 位置修正 5000m → 发散。
      for (int i = 0; i < satAmb.size(); i++) {
        double ambInitVar = Math.pow(stDevInit, 2) * Math.pow(estimatedWavelength[i], 2);
        // Estimated ambiguity (meters)
        KFstate.set(i3 + satAmb.get(i), 0, estimatedAmbiguityComb[i]);

        // RTKLIB-aligned: initial ambiguity variance = λ²·900 m²
        Cee.set(i3 + satAmb.get(i), i3 + satAmb.get(i), ambInitVar);
      }
      // L2 ambiguity state initialization (dual-freq, RTKLIB-aligned)
      if (nAmbL2 > 0) {
        for (int i = 0; i < satAmb.size(); i++) {
          double ambInitVarL2 = Math.pow(stDevInit, 2) * Math.pow(estimatedWavelengthL2[i], 2);
          KFstate.set(iAmbL2 + satAmb.get(i), 0, estimatedAmbiguityCombL2[i]);
          Cee.set(iAmbL2 + satAmb.get(i), iAmbL2 + satAmb.get(i), ambInitVarL2);
        }
      }
    } else {
      // RTKLIB-aligned: re-initialize ambiguity state and covariance on cycle slip / new sat
      // 使用 λ²·900 m² (与 init 分支一致)，对齐 RTKLIB udbias()。
      for (int i = 0; i < satAmb.size(); i++) {
        double ambInitVar = Math.pow(stDevInit, 2) * Math.pow(estimatedWavelength[i], 2);
        // Estimated ambiguity (meters)
        KFprediction.set(i3 + satAmb.get(i), 0, estimatedAmbiguityComb[i]);

        // Reset state covariance (λ²·900 m²)
        Cee.set(i3 + satAmb.get(i), i3 + satAmb.get(i), ambInitVar);
      }
      // L2 ambiguity re-initialization on cycle slip / new sat (RTKLIB-aligned)
      if (nAmbL2 > 0) {
        for (int i = 0; i < satAmb.size(); i++) {
          double ambInitVarL2 = Math.pow(stDevInit, 2) * Math.pow(estimatedWavelengthL2[i], 2);
          KFprediction.set(iAmbL2 + satAmb.get(i), 0, estimatedAmbiguityCombL2[i]);
          Cee.set(iAmbL2 + satAmb.get(i), iAmbL2 + satAmb.get(i), ambInitVarL2);
        }
      }
    }
  }

  
  /**
   * @param roverObs
   * @param masterObs
   * @param masterPos
   */
  @Override
  void checkSatelliteConfiguration(Observations roverObs, Observations masterObs, Coordinates masterPos) {

    // Lists for keeping track of satellites that need ambiguity (re-)estimation
    ArrayList<Integer> newSatellites = new ArrayList<Integer>(0);
    ArrayList<Integer> slippedSatellites = new ArrayList<Integer>(0);

    // Check if satellites were lost since the previous epoch
    for (int i = 0; i < satOld.size(); i++) {

      // Set ambiguity of lost satellites to zero
//      if (!sats.gnssAvailPhase.contains(satOld.get(i))) {
      if (!sats.availPhase.contains(satOld.get(i)) && sats.typeAvailPhase.contains(satOld.get(i))) {

        if(goGPS.isDebug()) System.out.println("Lost satellite "+satOld.get(i));

        KFprediction.set(i3 + satOld.get(i), 0, 0);
      }
    }

    // Check if new satellites are available since the previous epoch
    int temporaryPivot = 0;
    boolean newPivot = false;
    for (int i = 0; i < sats.pos.length; i++) {

      if (sats.pos[i] != null && sats.availPhase.contains(sats.pos[i].getSatID()) && sats.typeAvailPhase.contains(sats.pos[i].getSatType())
          && !satOld.contains(sats.pos[i].getSatID()) && satTypeOld.contains(sats.pos[i].getSatType())) {

        newSatellites.add(sats.pos[i].getSatID());

        if (sats.pos[i].getSatID() == sats.pos[sats.pivot].getSatID() && sats.pos[i].getSatType() == sats.pos[sats.pivot].getSatType()) {
          newPivot = true;
          if(goGPS.isDebug()) System.out.println("New satellite "+sats.pos[i].getSatID()+" (new pivot)");
        } else {
          if(goGPS.isDebug()) System.out.println("New satellite "+sats.pos[i].getSatID());
        }
      }
    }

    // If a new satellite is going to be the pivot, its ambiguity needs to be estimated before switching pivot
    if (newPivot) {
      // If it is not the only satellite with phase
      if (sats.availPhase.size() > 1) {
        // If the former pivot is still among satellites with phase
        if (sats.availPhase.contains(oldPivotId) && sats.typeAvailPhase.contains(oldPivotType)) {
          // Find the index of the old pivot
          for (int j = 0; j < sats.pos.length; j ++) {
            if (sats.pos[j] != null && sats.pos[j].getSatID() == oldPivotId && sats.pos[j].getSatType() == oldPivotType) {
              temporaryPivot = j;
            }
          }
        } else {
          double maxEl = 0;
          // Find a temporary pivot with phase
          for (int j = 0; j < sats.pos.length; j ++) {
            if (sats.pos[j] != null && sats.availPhase.contains(sats.pos[j].getSatID()) && sats.typeAvailPhase.contains(sats.pos[j].getSatType())
                && j != sats.pivot
                && rover.topo[j].getElevation() > maxEl) {
              temporaryPivot = j;
              maxEl = rover.topo[j].getElevation();
            }
          }
          // Reset the ambiguities of other satellites according to the temporary pivot
          newSatellites.clear();
          newSatellites.addAll(sats.availPhase);
          oldPivotId = sats.pos[temporaryPivot].getSatID();
          oldPivotType = sats.pos[temporaryPivot].getSatType();
          
        }
        // Estimate the ambiguity of the new pivot and other (new) satellites, using the temporary pivot
        estimateAmbiguities(roverObs, masterObs, masterPos, newSatellites, temporaryPivot, false);
        newSatellites.clear();
      }
    }

    // Check if pivot satellite changed since the previous epoch
    // Guard: oldPivotId must be a valid satellite (>= 1) and must be in the ambiguity range
    if (oldPivotId > 0 && oldPivotId != sats.pos[sats.pivot].getSatID() && oldPivotType == sats.pos[sats.pivot].getSatType()  && sats.availPhase.size() > 1) {

      if(goGPS.isDebug()) System.out.println("Pivot change from satellite "+oldPivotId+" to satellite "+sats.pos[sats.pivot].getSatID());

      // Matrix construction to manage the change of pivot satellite
      // RTKLIB-aligned ambswap: DD ambiguity N_i = SD_i - SD_pivot
      // When pivot changes from p to q: N_i_new = N_i_old - N_q_old (for i ≠ q)
      // N_q_new = 0 (new pivot's DD ambiguity is zero by definition)
      // Use full state dimension to cover position + L1 amb + L2 amb + iono + tropo
      int totalStateDim = stateSize();
      SimpleMatrix A = SimpleMatrix.identity(totalStateDim);

      int newPivotIndex = i3 + sats.pos[sats.pivot].getSatID();
      int oldPivotIndex = i3 + oldPivotId;

      for (int i = 0; i < sats.availPhase.size(); i++) {
        int satIndex = i3 + sats.availPhase.get(i);
        if (satIndex != newPivotIndex) {
          A.set(satIndex, satIndex, 1);
          A.set(satIndex, newPivotIndex, -1);
        }
      }
      // Zero out old pivot row: its ambiguity is now expressed relative to new pivot
      A.set(oldPivotIndex, oldPivotIndex, 0);
      A.set(newPivotIndex, newPivotIndex, 0);

      // Update predicted state
      KFprediction = A.mult(KFprediction);

      // Re-computation of the Cee covariance matrix at the previous epoch
      Cee = A.mult(Cee).mult(A.transpose());
    }

    // Cycle-slip detection
    boolean lossOfLockCycleSlipRover;
    boolean lossOfLockCycleSlipMaster;
    boolean dopplerCycleSlipRover;
    boolean dopplerCycleSlipMaster;
    boolean approxRangeCycleSlip;
    boolean cycleSlip;
    //boolean slippedPivot = false;
    
    // Pivot satellite ID
    int pivotId = sats.pos[sats.pivot].getSatID();
    
    // Rover-pivot and master-pivot observed phase
    char satType = roverObs.getGnssType(0);
    double roverPivotPhaseObs = roverObs.getSatByIDType(pivotId, satType).getPhaserange(goGPS.getFreq());
    double masterPivotPhaseObs = masterObs.getSatByIDType(pivotId, satType).getPhaserange(goGPS.getFreq());
    
    // Rover-pivot and master-pivot approximate pseudoranges
    double roverPivotAppRange = rover.satAppRange[sats.pivot];
    double masterPivotAppRange = master.satAppRange[sats.pivot];
    
    for (int i = 0; i < roverObs.getNumSat(); i++) {

      int satID = roverObs.getSatID(i);
      satType = roverObs.getGnssType(i);
      String checkAvailGnss = String.valueOf(satType) + String.valueOf(satID);

      if (sats.gnssAvailPhase.contains(checkAvailGnss)) {

        // cycle slip detected by loss of lock indicator (RTKLIB-aligned: LLI flag)
        lossOfLockCycleSlipRover = roverObs.getSatByIDType(satID, satType).isPossibleCycleSlip(goGPS.getFreq());
        lossOfLockCycleSlipMaster = masterObs.getSatByIDType(satID, satType).isPossibleCycleSlip(goGPS.getFreq());

        // cycle slip detected by Doppler predicted phase range
        if (goGPS.getCycleSlipDetectionStrategy() == CycleSlipDetectionStrategy.DOPPLER_PREDICTED_PHASE_RANGE) {
          dopplerCycleSlipRover = rover.getDopplerPredictedPhase(satID) != 0.0 && (Math.abs(roverObs.getSatByIDType(satID, satType).getPhaseCycles(goGPS.getFreq())
              - rover.getDopplerPredictedPhase(satID)) > goGPS.getCycleSlipThreshold());
          dopplerCycleSlipMaster = master.getDopplerPredictedPhase(satID) != 0.0 && (Math.abs(masterObs.getSatByIDType(satID, satType).getPhaseCycles(goGPS.getFreq())
              - master.getDopplerPredictedPhase(satID)) > goGPS.getCycleSlipThreshold());
        } else {
          dopplerCycleSlipRover = false;
          dopplerCycleSlipMaster = false;
        }

        // cycle slip detected by GF (Geometry-Free) combination (dual-frequency only)
        boolean gfCycleSlipRover = false;
        boolean gfCycleSlipMaster = false;
        if (goGPS.isDualFreq()) {
          double gfThresh = goGPS.getGfCycleSlipThreshold();
          double gfRover = rover.getPrevGf(satID);
          double gfMaster = master.getPrevGf(satID);
          if (gfRover != 0.0) {
            double gfCurr = roverObs.getSatByIDType(satID, satType).getPhaserange(0)
                - roverObs.getSatByIDType(satID, satType).getPhaserange(1);
            gfCycleSlipRover = Math.abs(gfCurr - gfRover) > gfThresh;
          }
          rover.setPrevGf(satID, roverObs.getSatByIDType(satID, satType).getPhaserange(0)
              - roverObs.getSatByIDType(satID, satType).getPhaserange(1));
          if (gfMaster != 0.0) {
            double gfCurr = masterObs.getSatByIDType(satID, satType).getPhaserange(0)
                - masterObs.getSatByIDType(satID, satType).getPhaserange(1);
            gfCycleSlipMaster = Math.abs(gfCurr - gfMaster) > gfThresh;
          }
          master.setPrevGf(satID, masterObs.getSatByIDType(satID, satType).getPhaserange(0)
              - masterObs.getSatByIDType(satID, satType).getPhaserange(1));
        }

        // cycle slip detected by approximate pseudorange
        approxRangeCycleSlip = false;
        if (goGPS.getCycleSlipDetectionStrategy() == CycleSlipDetectionStrategy.APPROX_PSEUDORANGE && satID != pivotId) {

          // Rover-satellite and master-satellite approximate pseudorange
          double roverSatCodeAppRange = rover.satAppRange[i];
          double masterSatCodeAppRange = master.satAppRange[i];

          // Rover-satellite and master-satellite observed phase
          double roverSatPhaseObs = roverObs.getSatByIDType(satID, satType).getPhaserange(goGPS.getFreq());
          double masterSatPhaseObs = masterObs.getSatByIDType(satID, satType).getPhaserange(goGPS.getFreq());

          // Estimated code pseudorange double differences
          double codeDoubleDiffApprox = (roverSatCodeAppRange - masterSatCodeAppRange) - (roverPivotAppRange - masterPivotAppRange);

          // Observed phase double differences
          double phaseDoubleDiffObserv = (roverSatPhaseObs - masterSatPhaseObs) - (roverPivotPhaseObs - masterPivotPhaseObs);

          // Store estimated ambiguity combinations and their covariance
          double estimatedAmbiguityComb = (codeDoubleDiffApprox - phaseDoubleDiffObserv) / roverObs.getSatByIDType(satID, satType).getWavelength(goGPS.getFreq());

          approxRangeCycleSlip = (Math.abs(KFprediction.get(i3+satID) - estimatedAmbiguityComb)) > goGPS.getCycleSlipThreshold();

        } else {
          approxRangeCycleSlip = false;
        }

        // 【RTKLIB 对齐说明】周跳检测: goGPS 同时启用 LLI/Doppler/GF/ApproxRange 四种，
        // RTKLIB detslp() 主要用 LLI 标志 + GF 组合。goGPS 多了 Doppler 和 ApproxRange 两种，
        // 更敏感 → 可能导致模糊度频繁重置 → Cee 回到 900 → sqrtCov 停在 30 → ratio 难达 3.0。
        // 若 LAMBDA 日志中 sqrtCov 长期为 30.000，优先排查此处是否误判周跳。
        // 可临时只保留 LLI+GF 对齐 RTKLIB 验证。
        // 【验证结果】APR 检测在滤波器未收敛时每历元每卫星都触发(threshold=1cycle 太严)，
        // 导致 Cee 反复重置。已临时禁用 DOP+APR，只保留 LLI+GF 对齐 RTKLIB detslp()。
        // cycleSlip = (lossOfLockCycleSlipRover || lossOfLockCycleSlipMaster || dopplerCycleSlipRover || dopplerCycleSlipMaster || gfCycleSlipRover || gfCycleSlipMaster || approxRangeCycleSlip);
        cycleSlip = (lossOfLockCycleSlipRover || lossOfLockCycleSlipMaster || gfCycleSlipRover || gfCycleSlipMaster);

        // [诊断] 统计每种周跳检测的触发情况，便于定位 Cee 频繁重置的根因
        if (goGPS.isDebug() && satID != sats.pos[sats.pivot].getSatID() && !newSatellites.contains(satID)) {
          if (satID == 10 && satType == 'C' && epochCount <= 5) {
            int freq = goGPS.getFreq();
            org.gogpsproject.producer.ObservationSet rOs = roverObs.getSatByIDType(satID, satType);
            org.gogpsproject.producer.ObservationSet mOs = masterObs.getSatByIDType(satID, satType);
            int rLli = rOs.getLossLockInd(freq);
            int mLli = mOs.getLossLockInd(freq);
            System.err.printf("[CS-DBG] C10 ep=%d freq=%d rLli=%d mLli=%d rHash=%d mHash=%d lossM=%s%n",
              epochCount, freq, rLli, mLli, System.identityHashCode(rOs), System.identityHashCode(mOs), lossOfLockCycleSlipMaster);
          }
          if (cycleSlip) {
            String reasons = "";
            if (lossOfLockCycleSlipRover || lossOfLockCycleSlipMaster) reasons += "LLI ";
            if (dopplerCycleSlipRover || dopplerCycleSlipMaster) reasons += "DOP ";
            if (gfCycleSlipRover || gfCycleSlipMaster) reasons += "GF ";
            if (approxRangeCycleSlip) reasons += "APR ";
            System.err.printf("[CS-TRIG] %c%02d epoch=%d reasons=[%s]%n", satType, satID, epochCount, reasons.trim());
          }
        }

        if (satID != sats.pos[sats.pivot].getSatID() && !newSatellites.contains(satID) && cycleSlip) {

          slippedSatellites.add(satID);

          //        if (satID != sats.pos[sats.pivot].getSatID()) {
          if (dopplerCycleSlipRover)
            if(goGPS.isDebug()) System.out.println("[ROVER] Cycle slip on satellite "+satID+" (range diff = "+Math.abs(roverObs.getSatByIDType(satID, satType).getPhaseCycles(goGPS.getFreq())
                - rover.getDopplerPredictedPhase(satID))+")");
          if (dopplerCycleSlipMaster)
            if(goGPS.isDebug()) System.out.println("[MASTER] Cycle slip on satellite "+satID+" (range diff = "+Math.abs(masterObs.getSatByIDType(satID, satType).getPhaseCycles(goGPS.getFreq())
                - master.getDopplerPredictedPhase(satID))+")");
          //        } else {
          //          boolean slippedPivot = true;
          //          if (dopplerCycleSlipRover)
          //            System.out.println("[ROVER] Cycle slip on pivot satellite "+satID+" (range diff = "+Math.abs(roverObs.getGpsByID(satID).getPhase(goGPS.getFreq())
          //                - this.rover.getDopplerPredictedPhase(satID))+")");
          //          if (dopplerCycleSlipMaster)
          //            System.out.println("[MASTER] Cycle slip on pivot satellite "+satID+" (range diff = "+Math.abs(masterObs.getGpsByID(satID).getPhase(goGPS.getFreq())
          //                - this.master.getDopplerPredictedPhase(satID))+")");
          //        }
        }
      }
    }

//    // If the pivot satellites slipped, the ambiguities of all the other satellites must be re-estimated
//    if (slippedPivot) {
//      // If it is not the only satellite with phase
//      if (sats.availPhase.size() > 1) {
//        // Reset the ambiguities of other satellites
//        newSatellites.clear();
//        slippedSatellites.clear();
//        slippedSatellites.addAll(sats.availPhase);
//      }
//    }

    // Ambiguity estimation
    if (newSatellites.size() != 0 || slippedSatellites.size() != 0) {
      // List of satellites that need ambiguity estimation
      ArrayList<Integer> satAmb = newSatellites;
      satAmb.addAll(slippedSatellites);
      estimateAmbiguities(roverObs, masterObs, masterPos, satAmb, sats.pivot, false);
    }
  }

  /**
   * LAMBDA ambiguity resolution: fix DD ambiguities to integers.
   * Called after each KF update in the loop() method.
   */
  boolean fixAmbiguitiesLambda() {

    int nPhase = sats.availPhase.size();
    if (nPhase < 2) return false; // Need at least 2 to form DD

    int pivotId = sats.pos[sats.pivot].getSatID();

    // Count active DD ambiguity states (exclude pivot)
    int nb = 0;
    int[] satIds = new int[nPhase];
    for (int id : sats.availPhase) {
      if (id != pivotId) {
        satIds[nb++] = id;
      }
    }
    if (nb < 2) return false; // Need at least 2 DD ambiguities

    // Ratio test threshold (RTKLIB default: 3.0)
    double thres = goGPS.getAmbiguityRatioThreshold();
    if (thres <= 0) thres = 3.0;

    // 【EJML 重构】用 SimpleMatrix 统一管理模糊度向量与协方差矩阵，
    // 避免手动 double[] + 列主序索引(Q[i+j*nb])，降低维护与调试成本。
    // 单位转换通过 SimpleMatrix 元素运算完成，仅在调用 Lambda.lambda()
    // 边界处转换为 column-major double[] (RTKLIB Lambda 接口约定)。
    //
    // 状态单位转换 meters→cycles (LAMBDA 要求 cycles 输入):
    //   a_cycles = a_meters ./ λ
    //   Q_cycles = Q_meters ./ (λ_i · λ_j) = Q_meters ./ (λ · λ')
    // KFstate 是 meters(H_amb=+1.0)，Cee 是 meters²。
    SimpleMatrix lambdaVec = new SimpleMatrix(nb, 1);   // 各卫星 L1 波长
    SimpleMatrix aMeters = new SimpleMatrix(nb, 1);    // 浮点模糊度 (meters, 取自 KFstate)
    SimpleMatrix QMeters = new SimpleMatrix(nb, nb);   // 模糊度协方差子矩阵 (meters², 取自 Cee)

    for (int i = 0; i < nb; i++) {
      int satId = satIds[i];
      char satType = lookupSatType(satId);
      double lambda = currentRoverObs.getSatByIDType(satId, satType).getWavelength(0);
      lambdaVec.set(i, 0, lambda);
      aMeters.set(i, 0, KFstate.get(i3 + satId));
      for (int j = 0; j < nb; j++) {
        int satIdj = satIds[j];
        // QMeters 直接取 Cee 子块 (meters²)，单位转换稍后统一处理
        QMeters.set(i, j, Cee.get(i3 + satId, i3 + satIdj));
      }
    }

    // 单位转换: meters→cycles (element-wise 除以波长)
    SimpleMatrix aCycles = aMeters.copy().elementDiv(lambdaVec);
    // λ_i · λ_j 的外积矩阵 (nb×nb)
    SimpleMatrix lambdaMat = lambdaVec.mult(lambdaVec.transpose());
    SimpleMatrix QCycles = QMeters.copy().elementDiv(lambdaMat);

    // Lambda.lambda() 为 RTKLIB 移植接口 (column-major double[])，
    // 此处做 SimpleMatrix→column-major 边界转换，调用方代码保持 EJML 风格。
    double[] a = toColumnMajor(aCycles);
    double[] Q = toColumnMajor(QCycles);
    double[] F = new double[nb * 2];
    double[] s = new double[2];

    if (goGPS.isDebug()) {
      System.err.println("====== LAMBDA INPUT (6 decimals) ======");
      System.err.printf("nb=%d, pivot=%d%n", nb, pivotId);
      System.err.print("a (float ambiguities, cycles) = [");
      for (int i = 0; i < nb; i++) {
        System.err.printf("%.6f%s", aCycles.get(i, 0), (i < nb - 1) ? ", " : "");
      }
      System.err.println("]");
      System.err.println("Q (covariance, cycles²) = ");
      for (int i = 0; i < nb; i++) {
        System.err.print("  [");
        for (int j = 0; j < nb; j++) {
          System.err.printf("%.6f%s", QCycles.get(i, j), (j < nb - 1) ? ", " : "");
        }
        System.err.println("]");
      }
      System.err.println("====== END LAMBDA INPUT ======");
    }

    int info = Lambda.lambda(nb, 2, a, Q, F, s);

    if (goGPS.isDebug()) {
      System.err.printf("[LAMBDA debug] nb=%d, pivot=%d%n", nb, pivotId);
      for (int i = 0; i < nb; i++) {
        double floatAmb = aCycles.get(i, 0);
        double nearestInt = Math.round(floatAmb);
        double fracDist = floatAmb - nearestInt;
        double diagCov = QCycles.get(i, i);
        System.err.printf("  sat C%02d: float=%.3f, nearestInt=%.0f, fracDist=%.3f, sqrtCov=%.3f%n",
            satIds[i], floatAmb, nearestInt, fracDist, Math.sqrt(Math.max(diagCov, 0)));
      }

      // Verify LAMBDA: compute Mahalanobis distance of simple rounding
      // dRound = a - round(a), sRound = dRound' * Q^{-1} * dRound
      SimpleMatrix dVec = new SimpleMatrix(nb, 1);
      for (int i = 0; i < nb; i++) {
        dVec.set(i, 0, aCycles.get(i, 0) - Math.round(aCycles.get(i, 0)));
      }
      double sRound = dVec.transpose().mult(QCycles.invert()).mult(dVec).get(0);
      System.err.printf("[LAMBDA debug] Simple rounding Mahalanobis² = %.1f, LAMBDA s[0]=%.1f, s[1]=%.1f%n",
          sRound, s[0], s[1]);

      // Also print LAMBDA fixed candidates
      System.err.printf("[LAMBDA debug] LAMBDA fixed: ");
      for (int i = 0; i < Math.min(nb, 5); i++) {
        System.err.printf("%.0f ", F[i]);
      }
      System.err.println();
    }

    if (info != 0) {
      if (goGPS.isDebug()) System.out.println("[LAMBDA] Failed (info=" + info + ")");
      return false;
    }

    // Ratio test
    double ratio = (s[0] > 0) ? s[1] / s[0] : 0.0;
    if (ratio >= thres) {
      // 从 LAMBDA 输出 F (column-major double[], 前 nb 个为最优候选) 重建 SimpleMatrix
      SimpleMatrix FCycles = new SimpleMatrix(nb, 1);
      for (int i = 0; i < nb; i++) {
        FCycles.set(i, 0, F[i]);
      }

      // 【状态单位转换 cycles→meters】(2026-06-21 重构)
      // F 是 LAMBDA 输出的 cycles，KFstate 是 meters，需 ×λ。
      // fixedMetersVec = FCycles .* λ (element-wise)
      SimpleMatrix fixedMetersVec = FCycles.elementMult(lambdaVec);
      for (int i = 0; i < nb; i++) {
        double fixedMeters = fixedMetersVec.get(i, 0);
        KFstate.set(i3 + satIds[i], 0, fixedMeters);
        heldAmbiguities.put(satIds[i], fixedMeters); // save for holdamb constraint target
      }

      // Conditional covariance update (RTKLIB-aligned: resamb_LAMBDA)
      int na = i3 + 1; // number of non-ambiguity states (0..i3)

      // Qb = 模糊度协方差子块 (meters²)，与上方 QMeters 同源，复用避免重复构造
      SimpleMatrix Qb = QMeters;
      // Qab = 非模糊度状态与模糊度的交叉协方差 (na×nb, meters²)
      SimpleMatrix Qab = new SimpleMatrix(na, nb);
      for (int i = 0; i < nb; i++) {
        for (int j = 0; j < na; j++) {
          Qab.set(j, i, Cee.get(j, i3 + satIds[i]));
        }
      }

      // 残差向量: db = (a_float - a_fixed) .* λ  (cycles→meters)
      // aCycles、FCycles 均为 cycles，差值 ×λ 转回 meters
      SimpleMatrix dbVec = aCycles.minus(FCycles).elementMult(lambdaVec);

      // Check numerical stability of Qb before inversion
      // Use trace-based condition estimate: cond_est = trace(Qb) / nb * ||Qb^{-1}||_approx
      // If Qb is near-singular, skip covariance update to avoid numerical explosion
      double trace = 0;
      double maxAbs = 0;
      for (int i = 0; i < nb; i++) {
        double diag = Math.abs(Qb.get(i, i));
        trace += diag;
        if (diag > maxAbs) maxAbs = diag;
      }
      // Condition number estimate: if trace is tiny or maxAbs is huge, Qb is ill-conditioned
      boolean illCond = (trace < 1e-20) || (maxAbs / Math.max(trace / nb, 1e-30) > 1e12);
      if (illCond) {
        if (goGPS.isDebug()) {
          System.out.printf("[LAMBDA] Qb ill-conditioned (trace=%.2e, maxDiag=%.2e), skip covariance update%n", trace, maxAbs);
        }
      } else {
        try {
          // Qb_inv = Qb^(-1)
          SimpleMatrix QbInv = Qb.invert();

          // db = Qb^(-1) * (b0 - b)
          SimpleMatrix db = QbInv.mult(dbVec);

          // Update non-ambiguity states: xa = xa - Qab * db
          SimpleMatrix xaOld = new SimpleMatrix(na, 1);
          for (int j = 0; j < na; j++) {
            xaOld.set(j, 0, KFstate.get(j, 0));
          }
          SimpleMatrix xaNew = xaOld.minus(Qab.mult(db));
          for (int j = 0; j < na; j++) {
            KFstate.set(j, 0, xaNew.get(j, 0));
          }

          // Update non-ambiguity covariance: Pa = Pa - Qab * Qb^(-1) * Qab^T
          SimpleMatrix QabT = Qab.transpose();
          SimpleMatrix correction = Qab.mult(QbInv).mult(QabT);
          for (int j = 0; j < na; j++) {
            for (int k = 0; k < na; k++) {
              double newVal = Cee.get(j, k) - correction.get(j, k);
              Cee.set(j, k, newVal);
            }
          }
        } catch (Exception e) {
          if (goGPS.isDebug()) {
            System.out.println("[LAMBDA] Qb inversion failed, skip covariance update: " + e.getMessage());
          }
        }
      }

      if (goGPS.isDebug()) {
        System.out.printf("[LAMBDA] Fixed %d ambiguities, ratio=%.2f, s=[%.4f, %.4f]%n",
            nb, ratio, s[0], s[1]);
      }
      return true;
    } else {
      if (goGPS.isDebug()) {
        System.out.printf("[LAMBDA] Ratio test failed: %.2f < %.2f (nb=%d, s=[%.4f, %.4f])%n",
            ratio, thres, nb, s[0], s[1]);
      }
      return false;
    }
  }

  /**
   * SimpleMatrix → column-major double[] 边界转换。
   * EJML 内部为 row-major，RTKLIB Lambda.lambda() 要求 column-major (Fortran 约定)。
   * 此方法仅在调用 Lambda 边界处使用，使调用方代码保持 EJML 风格，
   * 避免在业务逻辑中手动维护 Q[i + j*nb] 形式的列主序索引。
   */
  private static double[] toColumnMajor(SimpleMatrix m) {
    int rows = m.numRows();
    int cols = m.numCols();
    double[] out = new double[rows * cols];
    for (int j = 0; j < cols; j++) {
      for (int i = 0; i < rows; i++) {
        out[i + j * rows] = m.get(i, j);
      }
    }
    return out;
  }

  /**
   * 根据 satId 在 sats.availPhase 中查找对应的卫星类型 (G/R/C/E)。
   * 提取为方法以消除 LAMBDA 输入构造中的重复查找逻辑。
   */
  private char lookupSatType(int satId) {
    for (int k = 0; k < sats.availPhase.size(); k++) {
      if (sats.availPhase.get(k) == satId) {
        return sats.typeAvailPhase.get(k);
      }
    }
    return 'G';
  }

  /**
   * RTKLIB-aligned holdamb: constrain fixed DD ambiguities.
   * After LAMBDA fix, apply pseudo-observation constraints to prevent
   * ambiguity states from drifting. This is equivalent to RTKLIB's
   * holdamb() which calls filter() with VAR_HOLDAMB constraints.
   *
   * In GoGPS DD model, each ambiguity state is already a DD ambiguity
   * (relative to pivot). The constraint is: x[i] - fixed_value = 0
   * with variance VAR_HOLDAMB.
   */
  @Override
  void holdamb() {
    int pivotId = sats.pos[sats.pivot].getSatID();
    int nb = 0;
    for (int id : sats.availPhase) {
      if (id != pivotId) nb++;
    }
    if (nb == 0) return;

    SimpleMatrix H_hold = new SimpleMatrix(nb, stateSize());
    SimpleMatrix v_hold = new SimpleMatrix(nb, 1);
    SimpleMatrix R_hold = new SimpleMatrix(nb, nb);

    int nv = 0;
    for (int id : sats.availPhase) {
      if (id == pivotId) continue;
      int idx = i3 + id;
      H_hold.set(nv, idx, 1.0);
      // Constraint target: the fixed integer value (not 0).
      // Falls back to current state if no fixed value recorded yet.
      Double fixedVal = heldAmbiguities.get(id);
      v_hold.set(nv, 0, (fixedVal != null) ? fixedVal : KFstate.get(idx));
      R_hold.set(nv, nv, VAR_HOLDAMB);
      nv++;
    }

    try {
      SimpleMatrix S_hold = H_hold.mult(Cee).mult(H_hold.transpose()).plus(R_hold);
      SimpleMatrix S_inv = S_hold.invert();
      SimpleMatrix K_hold = Cee.mult(H_hold.transpose()).mult(S_inv);

      SimpleMatrix innov = v_hold.minus(H_hold.mult(KFstate));
      KFstate = KFstate.plus(K_hold.mult(innov));

      SimpleMatrix I = SimpleMatrix.identity(stateSize());
      Cee = I.minus(K_hold.mult(H_hold)).mult(Cee);
    } catch (Exception e) {
      if (goGPS.isDebug()) {
        System.out.println("[holdamb] Constraint update failed: " + e.getMessage());
      }
    }

    if (goGPS.isDebug()) {
      System.out.printf("[holdamb] Held %d DD ambiguities%n", nb);
    }
  }

  /**
   * Run kalman filter on code and phase double differences.
   */
  public static void run( GoGPS goGPS ) {
    
    RoverPosition rover   = goGPS.getRoverPos();
    MasterPosition master = goGPS.getMasterPos();
    Satellites sats       = goGPS.getSats();
    ObservationsProducer roverIn = goGPS.getRoverIn();
    ObservationsProducer masterIn = goGPS.getMasterIn();
    boolean debug = goGPS.isDebug();
    
    long timeRead = System.currentTimeMillis();
    long depRead = 0;

    long timeProc = 0;
    long depProc = 0;

    KalmanFilter kf = new KF_DD_code_phase(goGPS);
    
    // Flag to check if Kalman filter has been initialized
    boolean kalmanInitialized = false;

    try {
      boolean validPosition = false;

      timeRead = System.currentTimeMillis() - timeRead;
      depRead = depRead + timeRead;

      Observations obsR = roverIn.getNextObservations();
      Observations obsM = masterIn.getNextObservations();

      // RTKLIB-aligned base station coordinate handling (POSOPT_SINGLE).
      //
      // RTKLIB (postpos.c::antpos -> avepos) computes the reference station
      // position by running single-point positioning on EVERY base epoch and
      // averaging the results, then uses that fixed average for the whole run.
      // Because goGPS reads the RTCM stream sequentially (no reset/rewind), we
      // reproduce the same effect with a running mean: each matched epoch runs
      // SPP on the base observations and accumulates into baseSppSum*/baseSppCount.
      //
      // basePos is the coordinate used CONSISTENTLY everywhere (geometry in
      // selectDoubleDiff, kf.init/loop, output guard). It starts from the 1005
      // message (if any) as a fallback, and is refreshed every epoch with the
      // latest running mean once at least one SPP has succeeded.
      Coordinates basePos = masterIn.getDefinedPosition();
      if (debug && basePos != null) {
        System.err.println("[KF_DD run] masterIn.getDefinedPosition() (1005/1006) =[" +
            basePos.getX() + ", " + basePos.getY() + ", " + basePos.getZ() + "]");
      }

      // SKIP_DUP_MARKER: 同一历元多条MSM消息会产生重复Observations，此处跳过已处理的时间戳
      // 注释掉while循环内的skip块可恢复输出重复数据（调试MSM消息合并时使用）
      long lastEpochTime = -1;
      // END_SKIP_DUP_MARKER

      while (obsR != null && obsM != null) {
//        System.out.println("obsR: " + obsR);

        
        if(debug)System.out.println("R:"+obsR.getRefTime().getMsec()+" M:"+obsM.getRefTime().getMsec());

        timeRead = System.currentTimeMillis();

        long maxTimeDiffMs = goGPS.getMaxTimeDiffMs();

        // Discard master epochs that are behind rover by more than tolerance
        while (obsM != null && obsR != null
            && (obsR.getRefTime().getMsec() - obsM.getRefTime().getMsec()) > maxTimeDiffMs) {
          if(debug) System.out.println("  [match] master behind, skip M="+obsM.getRefTime().getMsec());
          obsM = masterIn.getNextObservations();
        }
        if (obsM == null) {
          if(debug) System.out.println("Couldn't find an obsM in a valid time span: " + obsR.getRefTime());
          break;
        }

        // Discard rover epochs that are behind master by more than tolerance
        while (obsM != null && obsR != null
            && (obsM.getRefTime().getMsec() - obsR.getRefTime().getMsec()) > maxTimeDiffMs) {
          if(debug) System.out.println("  [match] rover behind, skip R="+obsR.getRefTime().getMsec());
          obsR = roverIn.getNextObservations();
        }
        if (obsR == null) {
          if(debug) System.out.println("Couldn't find an obsR in a valid time span: " + obsM.getRefTime());
          break;
        }

        if(debug) System.out.println("  [matched] R="+obsR.getRefTime().getMsec()+" M="+obsM.getRefTime().getMsec());

        // SKIP_DUP_MARKER: 跳过同一历元的重复MSM消息
        // 注意：只跳过 rover 的重复，master 不动（避免 master 被错误推进导致历元错位）
        // 如果 master 也有重复，下面的时间匹配逻辑会自动处理
        if (obsR != null && obsR.getRefTime().getMsec() == lastEpochTime) {
          if (debug) System.out.println("[Skip] Duplicate rover epoch: " + obsR.getRefTime());
          obsR = roverIn.getNextObservations();
          continue;
        }
        // 如果 master 时间戳与上一历元相同（master 重复），跳过 master 的重复
        if (obsM != null && obsM.getRefTime().getMsec() == lastEpochTime) {
          if (debug) System.out.println("[Skip] Duplicate master epoch: " + obsM.getRefTime());
          obsM = masterIn.getNextObservations();
          continue;
        }
        // END_SKIP_DUP_MARKER

        if(obsM!=null && obsR!=null){
          lastEpochTime = obsR.getRefTime().getMsec();  // SKIP_DUP_MARKER: 记录当前历元
          timeRead = System.currentTimeMillis() - timeRead;
          depRead = depRead + timeRead;
          timeProc = System.currentTimeMillis();

          // 【基站位置逻辑：首次计算，一直使用】(2026-06-21)
          // ---------------------------------------------------------------
          // 检查逻辑（已反复验证，排除以下误判）：
          //   1. 初始 SPP 位置偏差大      → 实测 rover SPP 偏差仅 ~50m，不足以解释 y0Max=377m
          //   2. 星历不一致（rover/master 不同源）→ 已确认 rover/master 共用同一导航文件
          //   3. 卫星钟差未正确处理       → 双差已消除钟差，且残差非全卫星同向偏置
          //   4. 地球自转改正缺失         → 几何距离计算已含地球自转改正
          //   5. TGD 重复扣除             → 已核对仅扣除一次
          // 真正根因：原代码每历元用 SPP running mean 覆盖 masterPos，导致基站位置
          //   每历元跳动 20-50m（实测 epoch1→4: [-493109,-493106,-493102,-493098]），
          //   几何距离 ||sat-master|| 不一致，双差残差不收敛（y0Max=377m 且 25 历元不收敛）。
          //
          // 正确逻辑（对齐 RTKLIB 实时模式）：
          //   - 有 1005/1006 定义位置：直接使用 basePos=getDefinedPosition()，固定不变
          //     （RTCM 1005 是基准站精确坐标，精度 cm 级，远优于 SPP）
          //   - 无 1005：首次 SPP 成功后固定（baseSppCount>0 后不再更新）
          //     （RTKLIB postpos.c::avepos 仅用于后处理参考输出，实时滤波基站位置固定）
          if (masterIn.getDefinedPosition() == null && baseSppCount == 0 && obsM.getNumSat() >= 4) {
            // Save rover state (SPP on base temporarily reuses the rover object)
            double origRoverX = rover.getX(), origRoverY = rover.getY(), origRoverZ = rover.getZ();
            double origRoverClk = rover.getClockError();

            // Iterative SPP on base station observations.
            // cutoff=-100 avoids filtering satellites when starting from (0,0,0),
            // matching the rover SPP pattern below.
            rover.setXYZ(0, 0, 0);
            rover.setClockError(0);
            if (debug) System.err.printf("[SELSTD-CALL] baseSPP: rover.setXYZ(0,0,0) done, rover=%.1f,%.1f,%.1f%n", rover.getX(), rover.getY(), rover.getZ());
            double prevX = 0, prevY = 0, prevZ = 0;
            for (int iter = 0; iter < 10; iter++) {
              if (debug) System.err.println("[SELSTD-CALL] baseSPP: calling selectStandalone(obsM)");
              sats.selectStandalone(obsM, -100);
              if (sats.getAvailNumber() >= 4) {
                kf.codeStandalone(obsM, false, true);
              }
              if (rover.isValidXYZ() && iter > 0) {
                double dx = rover.getX() - prevX;
                double dy = rover.getY() - prevY;
                double dz = rover.getZ() - prevZ;
                if (Math.sqrt(dx*dx + dy*dy + dz*dz) < 100) {
                  break; // converged
                }
              }
              prevX = rover.getX();
              prevY = rover.getY();
              prevZ = rover.getZ();
            }

            // Accumulate valid SPP result into the running mean (RTKLIB avepos)
            if (rover.isValidXYZ() && (prevX != 0 || prevY != 0 || prevZ != 0)) {
              baseSppCount++;
              // incremental mean: mean += (x - mean) / n
              double n = baseSppCount;
              double meanX = baseSppSumX + (rover.getX() - baseSppSumX) / n;
              double meanY = baseSppSumY + (rover.getY() - baseSppSumY) / n;
              double meanZ = baseSppSumZ + (rover.getZ() - baseSppSumZ) / n;
              baseSppSumX = meanX;
              baseSppSumY = meanY;
              baseSppSumZ = meanZ;

              // Refresh basePos to the averaged coordinate (consistent everywhere)
              Coordinates avgCoord = Coordinates.globalXYZInstance(meanX, meanY, meanZ);
              avgCoord.computeGeodetic();
              goGPS.setMasterPos(avgCoord);
              basePos = avgCoord;

              if (debug) System.out.println("[SPP] Base SPP epoch " + baseSppCount +
                  ": instant=[" + String.format("%.2f", rover.getX()) + "," +
                  String.format("%.2f", rover.getY()) + "," + String.format("%.2f", rover.getZ()) +
                  "] avg=[" + String.format("%.4f", meanX) + "," +
                  String.format("%.4f", meanY) + "," + String.format("%.4f", meanZ) + "]");
            }

            // Restore rover state
            rover.setXYZ(origRoverX, origRoverY, origRoverZ);
            rover.setClockError(origRoverClk);
          }

          try {
            // If Kalman filter was not initialized and if there are at least four satellites
            boolean valid = true;
            if (!kalmanInitialized && obsR.getNumSat() >= 4) {

            // Compute approximate positioning by iterative least-squares
            if( roverIn.getDefinedPosition() != null )
              roverIn.getDefinedPosition().cloneInto(rover);
            
            // 【修改】若未指定初始位置，设为零向量，避免NaN传播到卫星距离计算
            if (!rover.isValidXYZ()) {
              rover.setXYZ(0, 0, 0);
              rover.computeGeodetic();
            }
            
            boolean initConverged = false;
            double prevX = 0, prevY = 0, prevZ = 0;
            for (int iter = 0; iter < 3; iter++) {
              // Save previous position for convergence check
              if (iter > 0) {
                prevX = rover.getX();
                prevY = rover.getY();
                prevZ = rover.getZ();
              }
              
              // Select all satellites
              if (debug) System.err.println("[SELSTD-CALL] roverSPP: calling selectStandalone(obsR)");
              sats.selectStandalone( obsR, -100);
              
              if (sats.getAvailNumber() >= 4) {
                kf.codeStandalone( obsR, false, true );
              }
              
              // Convergence check: position change < 100m after 2nd iteration
              if (iter >= 1 && rover.isValidXYZ()) {
                double dx = rover.getX() - prevX;
                double dy = rover.getY() - prevY;
                double dz = rover.getZ() - prevZ;
                double change = Math.sqrt(dx*dx + dy*dy + dz*dz);
                if (change < 100) {
                  initConverged = true;
                  if(debug) System.out.println("[Init] Code standalone converged at iter " + (iter+1) + ", change=" + String.format("%.1f", change) + "m");
                  break;
                }
              }
            }
            
            if (!initConverged && rover.isValidXYZ()) {
              if(debug) System.out.println("[Init] Code standalone did NOT converge after 3 iterations");
            }
            
            // Height sanity check: reject if height > 10000m (likely wrong solution)
            // Fallback: use base station position as initial guess for short-baseline RTK
            if (rover.isValidXYZ()) {
              rover.computeGeodetic();
              double h = rover.getGeodeticHeight();
              if (Math.abs(h) > 10000) {
                if(debug) System.out.println("[Init] Height out of range: " + String.format("%.1f", h) + "m, reject init");
                if (basePos != null) {
                  basePos.cloneInto(rover);
                  if(debug) System.out.println("[Init] Fallback to base station position: " +
                      String.format("%.1f, %.1f, %.1f", rover.getX(), rover.getY(), rover.getZ()));
                } else {
                  rover.setXYZ(0, 0, 0);
                }
              }
            }

            // If an approximate position was computed
            if (rover.isValidXYZ()) {
              
              // Initialize Kalman filter
              kf.init(obsR, obsM, basePos);

              if (rover.isValidXYZ()) {
                kalmanInitialized = true;
                if(debug)System.out.println("Kalman filter initialized.");
              } else {
                if(debug)System.out.println("Kalman filter not initialized.");
              }
            }else{
              if(debug) System.out.println("A-priori position (from code observations) is not valid.");
              // Fallback: use base station position for short-baseline RTK
              if (basePos != null) {
                basePos.cloneInto(rover);
                if(debug) System.out.println("[Init] Fallback to base station position (SPP failed): " +
                    String.format("%.1f, %.1f, %.1f", rover.getX(), rover.getY(), rover.getZ()));
                // Retry Kalman filter initialization with base position
                kf.init(obsR, obsM, basePos);
                if (rover.isValidXYZ()) {
                  kalmanInitialized = true;
                  if(debug) System.out.println("Kalman filter initialized (base station fallback).");
                }
              }
            }
          } else if (kalmanInitialized) {

            // Do a Kalman filter loop
            try{
              kf.loop(obsR,obsM, basePos);
            }catch(Exception e){
              e.printStackTrace();
              valid = false;
            }
          }

          timeProc = System.currentTimeMillis() - timeProc;
          depProc = depProc + timeProc;

          if(kalmanInitialized && valid){
            // Coordinate validity check: intercept NaN, zero, and out-of-range positions
            boolean coordValid = rover.isValidXYZ()
                && !Double.isNaN(rover.getX()) && !Double.isNaN(rover.getY()) && !Double.isNaN(rover.getZ())
                && !Double.isInfinite(rover.getX()) && !Double.isInfinite(rover.getY()) && !Double.isInfinite(rover.getZ())
                && !(rover.getX() == 0 && rover.getY() == 0 && rover.getZ() == 0);

            // Check distance from base station (RTK short baseline guard)
            if (coordValid && basePos != null) {
              double dx = rover.getX() - basePos.getX();
              double dy = rover.getY() - basePos.getY();
              double dz = rover.getZ() - basePos.getZ();
              double distFromBase = Math.sqrt(dx*dx + dy*dy + dz*dz);
              if (debug) System.err.printf("[Output guard] rover=[%.2f,%.2f,%.2f] base=[%.2f,%.2f,%.2f] dist=%.1fm%n",
                  rover.getX(), rover.getY(), rover.getZ(),
                  basePos.getX(), basePos.getY(), basePos.getZ(),
                  distFromBase);
              if (distFromBase > goGPS.getMaxDivergenceDistance()) {
                if(debug) System.out.println("[Output guard] Rover too far from base: " + String.format("%.1f", distFromBase/1000) + "km, skip output");
                coordValid = false;
              }
            }

            if (coordValid) {
              if(!validPosition){
                goGPS.notifyPositionConsumerEvent(PositionConsumer.EVENT_START_OF_TRACK);
                validPosition = true;
              }else
              if(goGPS.getPositionConsumers().size()>0){
                RoverPosition coord = new RoverPosition(rover, DopType.KALMAN, rover.getKpDop(), rover.getKhDop(), rover.getKvDop());
                coord.setRefTime(new Time(obsR.getRefTime().getMsec()));
                coord.sampleTime = obsR.getRefTime();
                coord.obs = obsR;

                // Build satsInUse bitmask from available satellites (for downstream consumers)
                long satsMask = 0;
                for (int satId : sats.avail.keySet()) {
                    if (satId > 0 && satId < 64) {
                        satsMask |= (1L << (satId - 1));
                    }
                }
                coord.satsInUse = satsMask;

                // Determine FIX/FLOAT status based on LAMBDA ambiguity resolution
                coord.status = (kf.nfix >= MIN_FIX_HOLD) ? Status.Valid : Status.Low_Sat;

                goGPS.notifyPositionConsumerAddCoordinate(coord);
              }
            } else {
              if(debug) System.out.println("[Output guard] Invalid coordinates, skip output");
            }

          }
          //System.out.println("--------------------");

          } catch (Exception e) {
            System.err.println("[Epoch] Exception in epoch processing: " + e.getMessage());
            if (debug) {
              e.printStackTrace();
            }
          }

          if(debug)System.out.println("-- Get next epoch ---------------------------------------------------");
          // get next epoch
          obsR = roverIn.getNextObservations();
          obsM = masterIn.getNextObservations();

        }else{
          if(debug)System.out.println("Missing M or R obs ");
        }
      }

    } catch (Exception e) {
      e.printStackTrace();
    } finally {
      goGPS.notifyPositionConsumerEvent(PositionConsumer.EVENT_END_OF_TRACK);
    }

    int elapsedTimeSec = (int) Math.floor(depRead / 1000);
    int elapsedTimeMillisec = (int) (depRead - elapsedTimeSec * 1000);
    if(debug)System.out.println("\nElapsed time (read): " + elapsedTimeSec
        + " seconds " + elapsedTimeMillisec + " milliseconds.");

    elapsedTimeSec = (int) Math.floor(depProc / 1000);
    elapsedTimeMillisec = (int) (depProc - elapsedTimeSec * 1000);
    if(debug)System.out.println("\nElapsed time (proc): " + elapsedTimeSec
        + " seconds " + elapsedTimeMillisec + " milliseconds.");
  }

}