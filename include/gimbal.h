#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 1. Data Structures
 * ========================================================================== */

/**
 * @brief 云台姿态/指令 (欧拉角)
 * @note  为了符合摄影和云台控制习惯，建议使用 **角度 (Degree)**。
 * 坐标系定义通常为：
 * Pitch: 低头为负，抬头为正
 * Yaw:   左转为正，右转为负 (右手定则)
 * Roll:  左倾为负，右倾为正
 */
typedef struct {
    float pitch;    // 俯仰轴
    float yaw;      // 偏航轴
    float roll;     // 横滚轴
} gimbal_euler_t;

/**
 * @brief 云台控制模式
 */
typedef enum {
    // --- 基础控制模式 ---
    GIMBAL_MODE_OFF = 0,     // 掉电/放松 (用于省电或手掰)

    // 角度控制 (Absolute Angle)
    // 也就是 "指向哪里"。适用于视觉追踪。
    GIMBAL_MODE_ANGLE_ABS,  // 绝对角度 (相对于世界或开机零点)
    GIMBAL_MODE_ANGLE_REL,  // 相对角度 (相对于当前角度的增量)
    GIMBAL_MODE_SPEED,       // 速度控制 (Velocity / Rate)
    GIMBAL_MODE_FOLLOW,     // 跟随模式 (Yaw 随底盘转动但有平滑，Pitch/Roll 锁定)
    GIMBAL_MODE_FPV,        // FPV 模式 (三轴都随底盘转动，但有平滑)
    GIMBAL_MODE_LOCK,       // 锁定模式 (不管底盘怎么动，云台始终指向一个绝对方向)

    // --- 维护模式 ---
    GIMBAL_MODE_CALIBRATE   // 触发校准
} gimbal_mode_t;

/**
 * @brief 限制参数 (用于保护硬件)
 */
typedef struct {
    gimbal_euler_t max_angle;  // 软限位最大值
    gimbal_euler_t min_angle;  // 软限位最小值
    gimbal_euler_t max_speed;  // 最大转速限制
} gimbal_limits_t;

/**
 * @brief 相机缩放方向
 * @note  T 系列驱动映射为可见光变倍命令 (45H)，C12 驱动映射为数码变焦命令 (DZM)
 */
typedef enum {
    GIMBAL_ZOOM_STOP = 0,
    GIMBAL_ZOOM_IN,
    GIMBAL_ZOOM_OUT,
} gimbal_zoom_dir_t;

/**
 * @brief UDP 驱动配置
 * @note  传给 gimbal_alloc_udp() 的 args；传 NULL 时由具体驱动使用默认配置。
 */
typedef struct {
    const char *bind_ip;
    uint16_t bind_port;
    const char *device_ip;
    uint16_t device_port;
    float resend_period_s;
} gimbal_udp_config_t;


/* ==========================================================================
 * 2. Opaque Handle
 * ========================================================================== */

struct gimbal_dev;

/* ==========================================================================
 * 3. API Functions
 * ========================================================================== */

/**
 * @brief 创建云台实例
 * @param driver_name e.g., "drv_udp_tz0xxx" / "drv_udp_c12"
 * @param args        UDP 驱动使用 gimbal_udp_config_t；传 NULL 则使用默认配置
 */
struct gimbal_dev *gimbal_alloc_udp(const char *driver_name, void *args);

/* --- 控制接口 --- */

int gimbal_set_mode(struct gimbal_dev *dev, gimbal_mode_t mode);
int gimbal_set_target(struct gimbal_dev *dev, const gimbal_euler_t *target);
int gimbal_set_limits(struct gimbal_dev *dev, const gimbal_limits_t *limits);

/**
 * @brief 设置相机缩放
 * @param dir         缩放方向：放大 / 缩小 / 停止
 * @param speed_level 缩放速度等级；0 表示使用设备默认速度
 */
int gimbal_set_zoom(struct gimbal_dev *dev, gimbal_zoom_dir_t dir,
    uint8_t speed_level);

/* --- 反馈接口 --- */
/**
 * @brief 获取当前姿态 (反馈)
 * @param out_angle 实际角度
 * @param out_speed 实际角速度 (可选，不支持则填0)
 */
int gimbal_get_state(struct gimbal_dev *dev, gimbal_euler_t *out_angle,
    gimbal_euler_t *out_speed);

/**
 * @brief 检查云台是否到位 (用于拍照触发)
 * @param threshold_deg 允许的误差角度
 */
bool gimbal_is_stable(struct gimbal_dev *dev, float threshold_deg);

/* --- 周期循环 --- */

void gimbal_tick(struct gimbal_dev *dev, float dt_s);
void gimbal_free(struct gimbal_dev *dev);

/* ==========================================================================
 * 4. Error Codes
 * ========================================================================== */

#define GIMBAL_OK 0
#define GIMBAL_ERR_ALLOC -1
#define GIMBAL_ERR_CONNECT -2
#define GIMBAL_ERR_TIMEOUT -3
#define GIMBAL_ERR_CONFIG -4
#define GIMBAL_ERR_PARAM -5
#define GIMBAL_ERR_NOSYS -6

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_H */
