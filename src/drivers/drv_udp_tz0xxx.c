/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <netinet/in.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "../gimbal_core.h"

#define T_GIMBAL_FRAME_HEAD_0 0xFB
#define T_GIMBAL_FRAME_HEAD_1 0x2C
#define T_GIMBAL_FRAME_HEAD_2 0xAA

#define T_GIMBAL_STATUS_HEAD_0 0xFC
#define T_GIMBAL_STATUS_HEAD_1 0x2C
#define T_GIMBAL_STATUS_HEAD_2 0x55

#define T_GIMBAL_CMD_MANUAL_ROUGH  0x0070
#define T_GIMBAL_CMD_CENTER        0x0071
#define T_GIMBAL_CMD_BODY_HOLD     0x0072
#define T_GIMBAL_CMD_DOWNLOOK      0x0073
#define T_GIMBAL_CMD_STOW          0x0074
#define T_GIMBAL_CMD_VISIBLE_ZOOM  0x0045
#define T_GIMBAL_CMD_SERVO_OFF     0x0075
#define T_GIMBAL_CMD_GYRO_CAL      0x0076
#define T_GIMBAL_CMD_SCAN          0x0079
#define T_GIMBAL_CMD_HEADING_LOCK  0x007A
#define T_GIMBAL_CMD_HEADING_FOLLOW 0x007B
#define T_GIMBAL_CMD_ATTITUDE_GUIDE 0x007C
#define T_GIMBAL_CMD_MANUAL_FINE   0x0082

#define T_GIMBAL_FEEDBACK_STATUS   0x0014

#define T_GIMBAL_MAX_ANGLE_YAW_DEG   179.0f
#define T_GIMBAL_MIN_ANGLE_PITCH_DEG -120.0f
#define T_GIMBAL_MAX_ANGLE_PITCH_DEG  90.0f
#define T_GIMBAL_MAX_RATE_DEG_S       50.0f
#define T_GIMBAL_RX_BUF_SIZE          1024U

#define DEBUG_ENABLE 0


struct t_series_udp_priv {
    int sockfd;
    struct sockaddr_in remote_addr;
    float resend_period_s;
    float resend_elapsed_s;
    uint8_t rx_buf[T_GIMBAL_RX_BUF_SIZE];
    size_t rx_len;
};

static int16_t le16_to_s16(const uint8_t *buf) {
    return (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
}

static uint16_t le16_to_u16(const uint8_t *buf) {
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

static void put_le16(uint8_t *buf, int16_t value) {
    uint16_t v = (uint16_t)value;
    buf[0] = (uint8_t)(v & 0xFFU);
    buf[1] = (uint8_t)((v >> 8) & 0xFFU);
}

static float clampf_local(float value, float min_value, float max_value) {
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

static uint8_t xor_checksum(const uint8_t *buf, size_t start, size_t end) {
    uint8_t x = 0U;
    size_t i;

    for (i = start; i <= end; ++i)
        x ^= buf[i];
    return x;
}

#if DEBUG_ENABLE
static const char *cmd_name(uint16_t cmd) {
    switch (cmd) {
    case T_GIMBAL_CMD_MANUAL_ROUGH:
        return "MANUAL_ROUGH";
    case T_GIMBAL_CMD_CENTER:
        return "CENTER";
    case T_GIMBAL_CMD_BODY_HOLD:
        return "BODY_HOLD";
    case T_GIMBAL_CMD_DOWNLOOK:
        return "DOWNLOOK";
    case T_GIMBAL_CMD_STOW:
        return "STOW";
    case T_GIMBAL_CMD_VISIBLE_ZOOM:
        return "VISIBLE_ZOOM";
    case T_GIMBAL_CMD_SERVO_OFF:
        return "SERVO_OFF";
    case T_GIMBAL_CMD_GYRO_CAL:
        return "GYRO_CAL";
    case T_GIMBAL_CMD_SCAN:
        return "SCAN";
    case T_GIMBAL_CMD_HEADING_LOCK:
        return "HEADING_LOCK";
    case T_GIMBAL_CMD_HEADING_FOLLOW:
        return "HEADING_FOLLOW";
    case T_GIMBAL_CMD_ATTITUDE_GUIDE:
        return "ATTITUDE_GUIDE";
    case T_GIMBAL_CMD_MANUAL_FINE:
        return "MANUAL_FINE";
    case T_GIMBAL_FEEDBACK_STATUS:
        return "FEEDBACK_STATUS";
    default:
        return "UNKNOWN";
    }
}


static void dump_frame(const char *prefix, uint16_t cmd,
    const uint8_t *buf, size_t len) {
    size_t i;

    printf("%s cmd=0x%04X(%s) len=%zu data=",
        prefix, cmd, cmd_name(cmd), len);
    for (i = 0; i < len; ++i)
        printf("%02X%s", buf[i], (i + 1U < len) ? " " : "");
    printf("\n");
}
#endif

static int send_command(struct gimbal_dev *dev, uint16_t cmd,
    const uint8_t data[4]) {
    struct t_series_udp_priv *priv = dev->priv_data;
    uint8_t frame[11];
    ssize_t sent;

    frame[0] = T_GIMBAL_FRAME_HEAD_0;
    frame[1] = T_GIMBAL_FRAME_HEAD_1;
    frame[2] = T_GIMBAL_FRAME_HEAD_2;
    frame[3] = 6U;
    frame[4] = (uint8_t)(cmd & 0xFFU);
    frame[5] = (uint8_t)((cmd >> 8) & 0xFFU);
    memcpy(&frame[6], data, 4U);
    frame[10] = xor_checksum(frame, 3U, 9U);

    sent = sendto(priv->sockfd, frame, sizeof(frame), 0,
        (const struct sockaddr *)&priv->remote_addr,
        sizeof(priv->remote_addr));
    if (sent != (ssize_t)sizeof(frame)) {
        printf("[GIMBAL-T-UDP] sendto failed: %s\n", strerror(errno));
        return GIMBAL_ERR_CONNECT;
    }
#if DEBUG_ENABLE
    dump_frame("[GIMBAL-T-UDP][TX]", cmd, frame, sizeof(frame));
#endif
    return GIMBAL_OK;
}

static int send_simple_command(struct gimbal_dev *dev, uint16_t cmd) {
    const uint8_t data[4] = {0U, 0U, 0U, 0U};

    return send_command(dev, cmd, data);
}

static int send_rate_command(struct gimbal_dev *dev,
    const gimbal_euler_t *target_rate) {
    uint8_t data[4] = {0U, 0U, 0U, 0U};
    float yaw_rate = clampf_local(target_rate->yaw,
        -T_GIMBAL_MAX_RATE_DEG_S,
        T_GIMBAL_MAX_RATE_DEG_S);
    float pitch_rate = clampf_local(target_rate->pitch,
        -T_GIMBAL_MAX_RATE_DEG_S,
        T_GIMBAL_MAX_RATE_DEG_S);
    int16_t yaw_q = (int16_t)lroundf(yaw_rate * 100.0f);
    int16_t pitch_q = (int16_t)lroundf(pitch_rate * 100.0f);

    put_le16(&data[0], yaw_q);
    put_le16(&data[2], pitch_q);

    return send_command(dev, T_GIMBAL_CMD_MANUAL_FINE, data);
}

static int send_attitude_command(struct gimbal_dev *dev,
    const gimbal_euler_t *target_angle) {
    uint8_t data[4] = {0U, 0U, 0U, 0U};
    float yaw = clampf_local(target_angle->yaw,
        -T_GIMBAL_MAX_ANGLE_YAW_DEG,
        T_GIMBAL_MAX_ANGLE_YAW_DEG);
    float pitch = clampf_local(target_angle->pitch,
        T_GIMBAL_MIN_ANGLE_PITCH_DEG,
        T_GIMBAL_MAX_ANGLE_PITCH_DEG);
    int16_t yaw_q = (int16_t)lroundf(yaw * 100.0f);
    int16_t pitch_q = (int16_t)lroundf(pitch * 100.0f);

    put_le16(&data[0], yaw_q);
    put_le16(&data[2], pitch_q);

    return send_command(dev, T_GIMBAL_CMD_ATTITUDE_GUIDE, data);
}

static int send_visible_zoom_command(struct gimbal_dev *dev,
    gimbal_zoom_dir_t dir,
    uint8_t speed_level) {
    uint8_t data[4] = {0U, 0U, 0U, 0U};

    switch (dir) {
    case GIMBAL_ZOOM_STOP:
        data[0] = 0U;
        data[1] = 0U;
        break;
    case GIMBAL_ZOOM_IN:
        data[0] = 1U;
        data[1] = speed_level;
        break;
    case GIMBAL_ZOOM_OUT:
        data[0] = 2U;
        data[1] = speed_level;
        break;
    default:
        return GIMBAL_ERR_PARAM;
    }

    return send_command(dev, T_GIMBAL_CMD_VISIBLE_ZOOM, data);
}

static int t_series_udp_set_mode(struct gimbal_dev *dev, gimbal_mode_t mode) {
    gimbal_mode_t old_mode;

    pthread_mutex_lock(&dev->cmd_lock);
    old_mode = dev->mode;
    pthread_mutex_unlock(&dev->cmd_lock);

    if (old_mode == GIMBAL_MODE_SPEED && mode != GIMBAL_MODE_SPEED) {
        gimbal_euler_t zero_rate = {0.0f, 0.0f, 0.0f};
        (void)send_rate_command(dev, &zero_rate);
    }

    switch (mode) {
    case GIMBAL_MODE_OFF:
        return send_simple_command(dev, T_GIMBAL_CMD_SERVO_OFF);
    case GIMBAL_MODE_FOLLOW:
        return send_simple_command(dev, T_GIMBAL_CMD_HEADING_FOLLOW);
    case GIMBAL_MODE_FPV:
        return send_simple_command(dev, T_GIMBAL_CMD_BODY_HOLD);
    case GIMBAL_MODE_LOCK:
        return send_simple_command(dev, T_GIMBAL_CMD_HEADING_LOCK);
    case GIMBAL_MODE_CALIBRATE: {
        const uint8_t data[4] = {1U, 0U, 0U, 0U};
        return send_command(dev, T_GIMBAL_CMD_GYRO_CAL, data);
    }
    case GIMBAL_MODE_ANGLE_ABS:
    case GIMBAL_MODE_ANGLE_REL:
    case GIMBAL_MODE_SPEED:
        return GIMBAL_OK;
    default:
        return GIMBAL_ERR_PARAM;
    }
}

static int t_series_udp_set_target(struct gimbal_dev *dev,
    const gimbal_euler_t *target) {
    gimbal_mode_t mode;
    struct t_series_udp_priv *priv = dev->priv_data;

    pthread_mutex_lock(&dev->cmd_lock);
    mode = dev->mode;
    pthread_mutex_unlock(&dev->cmd_lock);

    switch (mode) {
    case GIMBAL_MODE_ANGLE_ABS:
    case GIMBAL_MODE_ANGLE_REL:
        return send_attitude_command(dev, target);
    case GIMBAL_MODE_SPEED:
        priv->resend_elapsed_s = 0.0f;
        return send_rate_command(dev, target);
    default:
        return GIMBAL_ERR_PARAM;
    }
}

static int t_series_udp_set_limits(struct gimbal_dev *dev,
    const gimbal_limits_t *limits) {
    (void)dev;
    (void)limits;
    return GIMBAL_OK;
}

static int t_series_udp_set_zoom(struct gimbal_dev *dev,
    gimbal_zoom_dir_t dir, uint8_t speed_level) {
    return send_visible_zoom_command(dev, dir, speed_level);
}

static void parse_status_feedback(struct gimbal_dev *dev, const uint8_t *data,
    size_t data_len) {
    gimbal_euler_t prev_angle;
    gimbal_euler_t cur_angle;
    double now_s;
    double dt_s;

    if (data_len < 60U)
        return;

    cur_angle.yaw = le16_to_s16(&data[39]) / 100.0f;
    cur_angle.pitch = le16_to_s16(&data[41]) / 100.0f;
    cur_angle.roll = le16_to_s16(&data[43]) / 100.0f;

    now_s = gimbal_now_monotonic_s();

    pthread_mutex_lock(&dev->state_lock);
    prev_angle = dev->cur_angle;
    dt_s = dev->has_feedback ? (now_s - dev->last_feedback_time_s) : 0.0;

    dev->cur_angle = cur_angle;
    if (dt_s > 1e-3 && dt_s < 1.0) {
        dev->cur_speed.yaw =
            gimbal_delta_deg(cur_angle.yaw, prev_angle.yaw) / dt_s;
        dev->cur_speed.pitch =
            gimbal_delta_deg(cur_angle.pitch, prev_angle.pitch) / dt_s;
        dev->cur_speed.roll =
            gimbal_delta_deg(cur_angle.roll, prev_angle.roll) / dt_s;
    }

    dev->has_feedback = true;
    dev->last_feedback_time_s = now_s;
    dev->servo_status = data[6];
    dev->last_cmd_ack = data[46];
    dev->guide_status = data[47];
    printf(
        "[GIMBAL-T-UDP][RX-PARSED] yaw=%.2f pitch=%.2f roll=%.2f | "
        "speed_yaw=%.2f speed_pitch=%.2f speed_roll=%.2f | "
        "servo=0x%02X ack=0x%02X guide=0x%02X\n",
        dev->cur_angle.yaw, dev->cur_angle.pitch, dev->cur_angle.roll,
        dev->cur_speed.yaw, dev->cur_speed.pitch, dev->cur_speed.roll,
        dev->servo_status, dev->last_cmd_ack, dev->guide_status);
    pthread_mutex_unlock(&dev->state_lock);
}

static void process_rx_frame(struct gimbal_dev *dev, const uint8_t *buf,
    size_t len) {
    uint16_t cmd;
    uint8_t frame_len;
    size_t total_len;
    size_t data_len;

    if (len < 7U)
        return;
    if (buf[0] != T_GIMBAL_STATUS_HEAD_0 || buf[1] != T_GIMBAL_STATUS_HEAD_1 ||
        buf[2] != T_GIMBAL_STATUS_HEAD_2)
        return;

    frame_len = buf[3];
    total_len = frame_len;
    total_len += 5U;
    if (len < total_len || frame_len < 2U)
        return;

    if (xor_checksum(buf, 3U, total_len - 2U) != buf[total_len - 1U]) {
        printf("[GIMBAL-T-UDP] RX checksum mismatch\n");
        return;
    }

    cmd = le16_to_u16(&buf[4]);
    data_len = frame_len - 2U;

#if DEBUG_ENABLE
    dump_frame("[GIMBAL-T-UDP][RX]", cmd, buf, total_len);
#endif
    if (cmd == T_GIMBAL_FEEDBACK_STATUS)
        parse_status_feedback(dev, &buf[6], data_len);
}

static void append_rx_bytes(struct gimbal_dev *dev, const uint8_t *buf,
    size_t len) {
    struct t_series_udp_priv *priv = dev->priv_data;

    if (len == 0U)
        return;

    if (len > sizeof(priv->rx_buf)) {
        buf += len - sizeof(priv->rx_buf);
        len = sizeof(priv->rx_buf);
    }

    if (priv->rx_len + len > sizeof(priv->rx_buf)) {
        printf("[GIMBAL-T-UDP] RX buffer overflow, dropping buffered bytes\n");
        priv->rx_len = 0U;
    }

    memcpy(priv->rx_buf + priv->rx_len, buf, len);
    priv->rx_len += len;
}

static void consume_rx_stream(struct gimbal_dev *dev) {
    struct t_series_udp_priv *priv = dev->priv_data;
    size_t total_len;
    uint8_t frame_len;

    while (priv->rx_len > 0U) {
        if (priv->rx_buf[0] != T_GIMBAL_STATUS_HEAD_0) {
            memmove(priv->rx_buf, priv->rx_buf + 1, priv->rx_len - 1U);
            priv->rx_len -= 1U;
            continue;
        }

        if (priv->rx_len < 2U)
            break;
        if (priv->rx_buf[1] != T_GIMBAL_STATUS_HEAD_1) {
            memmove(priv->rx_buf, priv->rx_buf + 1, priv->rx_len - 1U);
            priv->rx_len -= 1U;
            continue;
        }

        if (priv->rx_len < 3U)
            break;
        if (priv->rx_buf[2] != T_GIMBAL_STATUS_HEAD_2) {
            memmove(priv->rx_buf, priv->rx_buf + 1, priv->rx_len - 1U);
            priv->rx_len -= 1U;
            continue;
        }

        if (priv->rx_len < 4U)
            break;

        frame_len = priv->rx_buf[3];
        total_len = frame_len;
        total_len += 5U;

        if (frame_len < 2U || total_len > sizeof(priv->rx_buf)) {
            memmove(priv->rx_buf, priv->rx_buf + 1, priv->rx_len - 1U);
            priv->rx_len -= 1U;
            continue;
        }

        if (priv->rx_len < total_len)
            break;

        process_rx_frame(dev, priv->rx_buf, total_len);
        memmove(priv->rx_buf, priv->rx_buf + total_len,
                priv->rx_len - total_len);
        priv->rx_len -= total_len;
    }
}

static void t_series_udp_tick(struct gimbal_dev *dev, float dt_s) {
    struct t_series_udp_priv *priv = dev->priv_data;
    uint8_t buf[256];
    ssize_t n;
    gimbal_mode_t mode;
    gimbal_euler_t target;

    do {
        n = recvfrom(priv->sockfd, buf, sizeof(buf), 0, NULL, NULL);
        if (n > 0) {
            size_t rx_bytes = n;

            append_rx_bytes(dev, buf, rx_bytes);
            consume_rx_stream(dev);
        }
    } while (n > 0);

    if (dt_s < 0.0f)
        dt_s = 0.0f;

    priv->resend_elapsed_s += dt_s;

    pthread_mutex_lock(&dev->cmd_lock);
    mode = dev->mode;
    target = dev->target;
    pthread_mutex_unlock(&dev->cmd_lock);

    if (mode == GIMBAL_MODE_SPEED
        && priv->resend_elapsed_s >= priv->resend_period_s) {
        priv->resend_elapsed_s = 0.0f;
        (void)send_rate_command(dev, &target);
    }
}

static void t_series_udp_free(struct gimbal_dev *dev) {
    struct t_series_udp_priv *priv;

    if (!dev)
        return;

    priv = dev->priv_data;
    if (priv && priv->sockfd >= 0) {
        close(priv->sockfd);
        priv->sockfd = -1;
    }

    gimbal_dev_free_default(dev);
}

static const struct gimbal_ops t_series_udp_ops = {
    .set_mode = t_series_udp_set_mode,
    .set_target = t_series_udp_set_target,
    .set_limits = t_series_udp_set_limits,
    .set_zoom = t_series_udp_set_zoom,
    .tick = t_series_udp_tick,
    .free = t_series_udp_free,
};

static int open_udp_socket(struct t_series_udp_priv *priv,
    const gimbal_udp_config_t *cfg) {
    struct sockaddr_in bind_addr;
    int fd;
    int flags;
    int optval = 1;
    const char *bind_ip = (cfg && cfg->bind_ip) ? cfg->bind_ip : "0.0.0.0";
    uint16_t bind_port =
        (cfg && cfg->bind_port != 0U) ? cfg->bind_port : 10000U;
    const char *device_ip =
        (cfg && cfg->device_ip) ? cfg->device_ip : "192.168.1.160";
    uint16_t device_port =
        (cfg && cfg->device_port != 0U) ? cfg->device_port : 10000U;

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
        return -1;

    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) < 0) {
        close(fd);
        return -1;
    }

    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(bind_port);
    if (inet_pton(AF_INET, bind_ip, &bind_addr.sin_addr) != 1) {
        close(fd);
        return -1;
    }

    if (bind(fd, (const struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        close(fd);
        return -1;
    }

    flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0)
        (void)fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    memset(&priv->remote_addr, 0, sizeof(priv->remote_addr));
    priv->remote_addr.sin_family = AF_INET;
    priv->remote_addr.sin_port = htons(device_port);
    if (inet_pton(AF_INET, device_ip, &priv->remote_addr.sin_addr) != 1) {
        close(fd);
        return -1;
    }

    priv->sockfd = fd;
    priv->resend_period_s =
        (cfg && cfg->resend_period_s > 0.0f) ? cfg->resend_period_s : 0.02f;
    priv->resend_elapsed_s = 0.0f;
    return 0;
}

static struct gimbal_dev *t_series_udp_create(const char *name, void *args) {
    struct gimbal_dev *dev;
    struct t_series_udp_priv *priv;
    const gimbal_udp_config_t *cfg = args;

    dev = gimbal_dev_alloc(name, sizeof(*priv));
    if (!dev)
        return NULL;

    dev->ops = &t_series_udp_ops;
    priv = dev->priv_data;
    priv->sockfd = -1;

    if (open_udp_socket(priv, cfg) != 0) {
        printf("[GIMBAL-T-UDP] Failed to open UDP socket: %s\n",
            strerror(errno));
        gimbal_dev_free_default(dev);
        return NULL;
    }

    printf("[GIMBAL-T-UDP] Initialized driver '%s'\n", name);
    return dev;
}

REGISTER_GIMBAL_DRIVER("drv_udp_tz0xxx", t_series_udp_create)
