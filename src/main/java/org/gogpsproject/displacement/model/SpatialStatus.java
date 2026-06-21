package org.gogpsproject.displacement.model;

/**
 * 空间校验状态枚举
 * <p>第六层空间联合校验的判定结果状态</p>
 */
public enum SpatialStatus {
    /** 空间校验正常 */
    NORMAL,
    /** 基准站疑似漂移 */
    BASE_STATION_SHIFT,
    /** 空间离群（单点干扰） */
    SPATIAL_OUTLIER,
    /** 局部真实形变 */
    LOCAL_DEFORMATION,
    /** 裂缝边缘形变 */
    FAULT_EDGE_MOVEMENT,
    /** 开关关闭/数据不足，跳过校验 */
    SKIP_SPATIAL_CHECK,
    /** 无分组配置 */
    NO_GROUP_CONFIG,
    /** 无空间属性配置 */
    NO_SPATIAL_META,
    /** 区域质量劣化，临时关闭模块 */
    AREA_DEGRADE_DISABLE,
    /** 达成全局共识，基线已重置（仅标记，由业务层执行重置） */
    BASE_STATION_RESET,
    /** 平票，按高权重判定 */
    TIE_VOTE_HIGH_WEIGHT,
    /** 平票，暂停预警 */
    TIE_VOTE_PAUSE_ALERT
}