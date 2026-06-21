package org.gogpsproject.displacement.model;

/**
 * 测点空间类型枚举
 * <p>第六层空间一致性校验的测点分类，决定是否触发空间离群修正</p>
 */
public enum SpatialStationType {
    /** 普通测点（默认），参与空间离群判定与修正 */
    NORMAL,
    /** 特殊测点（如裂缝监测点），发生独立位移时触发特赦规则放行，不执行修正 */
    SPECIAL
}