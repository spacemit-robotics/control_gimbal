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

#define C12_DEFAULT_BIND_IP        "0.0.0.0"
#define C12_DEFAULT_DEVICE_IP      "192.168.144.108"
#define C12_DEFAULT_UDP_PORT       5000U
#define C12_DEFAULT_RESEND_PERIOD  0.02f
#define C12_DEFAULT_FEEDBACK_HZ    20U
#define C12_DEFAULT_ANGLE_SPEED    9.9f

#define C12_MAX_DATA_LEN           15U
#define C12_FRAME_OVERHEAD_LEN     12U
#define C12_MAX_FRAME_SIZE         32U
#define C12_MIN_TX_PACKET_SIZE     20U
#define C12_RX_BUF_SIZE            1024U

#define C12_MAX_ANGLE_DEG          90.0f
#define C12_MAX_SPEED_MODE_DEG_S   12.7f
#define C12_MAX_ANGLE_SPEED_Q      99U

#define DEBUG_ENABLE 0

struct c12_udp_priv {
    int sockfd;
    struct sockaddr_in remote_addr;
    float resend_period_s;
    float resend_elapsed_s;
    uint8_t rx_buf[C12_RX_BUF_SIZE];
    size_t rx_len;
};

static float clampf_local(float value, float min_value, float max_value) {
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

static int clampi_local(int value, int min_value, int max_value) {
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

static char hex_upper(unsigned int value) {
    static const char kHex[] = "0123456789ABCDEF";

    return kHex[value & 0x0FU];
}

static int hex_nibble(uint8_t c) {
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1;
}

static bool parse_hex_u8(const uint8_t *buf, uint8_t *out) {
    int hi = hex_nibble(buf[0]);
    int lo = hex_nibble(buf[1]);

    if (hi < 0 || lo < 0)
        return false;

    *out = (uint8_t)((hi << 4) | lo);
    return true;
}

static bool parse_hex_u16(const uint8_t *buf, uint16_t *out) {
    uint16_t value = 0U;
    size_t i;

    for (i = 0; i < 4U; ++i) {
        int nibble = hex_nibble(buf[i]);

        if (nibble < 0)
            return false;
        value = (uint16_t)((value << 4) | (uint16_t)nibble);
    }

    *out = value;
    return true;
}

static bool parse_hex_s16(const uint8_t *buf, int16_t *out) {
    uint16_t raw;

    if (!parse_hex_u16(buf, &raw))
        return false;

    *out = (int16_t)raw;
    return true;
}

static uint8_t top_checksum(const char *buf, size_t len) {
    uint8_t sum = 0U;
    size_t i;

    for (i = 0; i < len; ++i)
        sum = (uint8_t)(sum + (uint8_t)buf[i]);
    return sum;
}

static bool verify_checksum(const uint8_t *buf, size_t frame_len) {
    uint8_t expected;
    uint8_t sum = 0U;
    size_t i;

    if (frame_len < 2U || !parse_hex_u8(&buf[frame_len - 2U], &expected))
        return false;

    for (i = 0; i < frame_len - 2U; ++i)
        sum = (uint8_t)(sum + buf[i]);

    return sum == expected;
}

static int format_top_command(char *frame, size_t frame_size, char dst,
    char ctrl, const char id[3], const char *data) {
    size_t data_len = data ? strlen(data) : 0U;
    size_t prefix_len;
    size_t total_len;
    uint8_t crc;

    if (!frame || !id || data_len > C12_MAX_DATA_LEN)
        return GIMBAL_ERR_PARAM;

    prefix_len = 10U + data_len;
    total_len = prefix_len + 2U;
    if (frame_size < total_len + 1U)
        return GIMBAL_ERR_PARAM;

    frame[0] = '#';
    frame[1] = 'T';
    frame[2] = 'P';
    frame[3] = 'U';
    frame[4] = dst;
    frame[5] = hex_upper((unsigned int)data_len);
    frame[6] = ctrl;
    memcpy(&frame[7], id, 3U);
    if (data_len > 0U)
        memcpy(&frame[10], data, data_len);

    crc = top_checksum(frame, prefix_len);
    frame[prefix_len] = hex_upper((unsigned int)(crc >> 4));
    frame[prefix_len + 1U] = hex_upper((unsigned int)crc);
    frame[total_len] = '\0';
    return (int)total_len;
}

static int send_top_command(struct gimbal_dev *dev, char dst,
    const char id[3], const char *data) {
    struct c12_udp_priv *priv = dev->priv_data;
    char frame[C12_MAX_FRAME_SIZE];
    int frame_len;
    size_t send_len;
    ssize_t sent;
    int fd;
    int flags;
    int saved_errno;

    memset(frame, 0, sizeof(frame));
    frame_len = format_top_command(frame, sizeof(frame), dst, 'w', id, data);
    if (frame_len < 0)
        return frame_len;

    send_len = (frame_len < (int)C12_MIN_TX_PACKET_SIZE)
        ? C12_MIN_TX_PACKET_SIZE
        : (size_t)frame_len;

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        printf("[GIMBAL-C12-UDP] socket failed: %s\n", strerror(errno));
        return GIMBAL_ERR_CONNECT;
    }

    flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) {
        saved_errno = errno;
        close(fd);
        printf("[GIMBAL-C12-UDP] fcntl(F_GETFL) failed: %s\n",
            strerror(saved_errno));
        return GIMBAL_ERR_CONNECT;
    }

    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        saved_errno = errno;
        close(fd);
        printf("[GIMBAL-C12-UDP] fcntl(F_SETFL) failed: %s\n",
            strerror(saved_errno));
        return GIMBAL_ERR_CONNECT;
    }

    sent = sendto(fd, frame, send_len, 0,
        (const struct sockaddr *)&priv->remote_addr,
        sizeof(priv->remote_addr));
    if (sent != (ssize_t)send_len) {
        saved_errno = errno;
        close(fd);
        printf("[GIMBAL-C12-UDP] sendto failed: %s\n",
            strerror(saved_errno));
        return GIMBAL_ERR_CONNECT;
    }

    close(fd);

#if DEBUG_ENABLE
    size_t i;
    printf("[GIMBAL-C12-UDP][TX-SKYDROID] frame_len=%d udp_len=%zu "
        "cmd=%.*s payload=",
        frame_len, send_len, frame_len, frame);
    for (i = 0; i < send_len; ++i)
        printf("%02X%s", (uint8_t)frame[i],
            (i + 1U < send_len) ? " " : "");
    printf("\n");
#endif
    return GIMBAL_OK;
}

static int send_ptz_command(struct gimbal_dev *dev, uint8_t op) {
    char data[3];

    data[0] = hex_upper((unsigned int)(op >> 4));
    data[1] = hex_upper((unsigned int)op);
    data[2] = '\0';
    return send_top_command(dev, 'G', "PTZ", data);
}

static int send_feedback_enable(struct gimbal_dev *dev, uint8_t hz) {
    char data[3];

    if (hz > 100U)
        hz = 100U;
    data[0] = hex_upper((unsigned int)(hz >> 4));
    data[1] = hex_upper((unsigned int)hz);
    data[2] = '\0';
    return send_top_command(dev, 'G', "GAA", data);
}

static uint8_t angle_speed_to_q(float speed_deg_s) {
    int q;

    speed_deg_s = fabsf(speed_deg_s);
    if (speed_deg_s <= 0.0f)
        speed_deg_s = C12_DEFAULT_ANGLE_SPEED;

    q = (int)lroundf(speed_deg_s * 10.0f);
    q = clampi_local(q, 1, (int)C12_MAX_ANGLE_SPEED_Q);
    return (uint8_t)q;
}

static void get_angle_speed_q(struct gimbal_dev *dev, uint8_t *yaw_q,
    uint8_t *pitch_q) {
    float yaw_speed = C12_DEFAULT_ANGLE_SPEED;
    float pitch_speed = C12_DEFAULT_ANGLE_SPEED;

    pthread_mutex_lock(&dev->cmd_lock);
    if (dev->limits_valid) {
        if (fabsf(dev->limits.max_speed.yaw) > 0.0f)
            yaw_speed = fabsf(dev->limits.max_speed.yaw);
        if (fabsf(dev->limits.max_speed.pitch) > 0.0f)
            pitch_speed = fabsf(dev->limits.max_speed.pitch);
    }
    pthread_mutex_unlock(&dev->cmd_lock);

    *yaw_q = angle_speed_to_q(yaw_speed);
    *pitch_q = angle_speed_to_q(pitch_speed);
}

static int16_t angle_to_q(float angle_deg) {
    int q;

    angle_deg = clampf_local(angle_deg, -C12_MAX_ANGLE_DEG,
        C12_MAX_ANGLE_DEG);
    q = (int)lroundf(angle_deg * 100.0f);
    q = clampi_local(q, -9000, 9000);
    return (int16_t)q;
}

static int8_t speed_to_q(float speed_deg_s) {
    int q;

    speed_deg_s = clampf_local(speed_deg_s, -C12_MAX_SPEED_MODE_DEG_S,
        C12_MAX_SPEED_MODE_DEG_S);
    q = (int)lroundf(speed_deg_s * 10.0f);
    q = clampi_local(q, -127, 127);
    return (int8_t)q;
}

static int send_angle_command(struct gimbal_dev *dev,
    const gimbal_euler_t *target_angle) {
    char data[13];
    int16_t yaw_q = angle_to_q(target_angle->yaw);
    int16_t pitch_q = angle_to_q(target_angle->pitch);
    uint8_t yaw_speed_q;
    uint8_t pitch_speed_q;

    get_angle_speed_q(dev, &yaw_speed_q, &pitch_speed_q);
    snprintf(data, sizeof(data), "%04X%02X%04X%02X",
        (uint16_t)yaw_q, yaw_speed_q, (uint16_t)pitch_q, pitch_speed_q);
    return send_top_command(dev, 'G', "GAM", data);
}

static int send_speed_command(struct gimbal_dev *dev,
    const gimbal_euler_t *target_rate) {
    char data[5];
    int8_t yaw_q = speed_to_q(target_rate->yaw);
    int8_t pitch_q = speed_to_q(target_rate->pitch);

    snprintf(data, sizeof(data), "%02X%02X", (uint8_t)yaw_q,
        (uint8_t)pitch_q);
    return send_top_command(dev, 'G', "GSM", data);
}

static int send_digital_zoom_command(struct gimbal_dev *dev,
    gimbal_zoom_dir_t dir) {
    switch (dir) {
    case GIMBAL_ZOOM_STOP:
        return GIMBAL_OK;
    case GIMBAL_ZOOM_IN:
        return send_top_command(dev, 'D', "DZM", "0A");
    case GIMBAL_ZOOM_OUT:
        return send_top_command(dev, 'D', "DZM", "0B");
    default:
        return GIMBAL_ERR_PARAM;
    }
}

static int c12_udp_set_mode(struct gimbal_dev *dev, gimbal_mode_t mode) {
    gimbal_mode_t old_mode;

    pthread_mutex_lock(&dev->cmd_lock);
    old_mode = dev->mode;
    pthread_mutex_unlock(&dev->cmd_lock);

    if (old_mode == GIMBAL_MODE_SPEED && mode != GIMBAL_MODE_SPEED) {
        gimbal_euler_t zero_rate = {0.0f, 0.0f, 0.0f};
        (void)send_speed_command(dev, &zero_rate);
    }

    switch (mode) {
    case GIMBAL_MODE_OFF:
        return send_ptz_command(dev, 0x00U);
    case GIMBAL_MODE_FOLLOW:
    case GIMBAL_MODE_FPV:
        return send_ptz_command(dev, 0x06U);
    case GIMBAL_MODE_LOCK:
        return send_ptz_command(dev, 0x07U);
    case GIMBAL_MODE_CALIBRATE:
        return send_ptz_command(dev, 0x09U);
    case GIMBAL_MODE_ANGLE_ABS:
    case GIMBAL_MODE_ANGLE_REL:
    case GIMBAL_MODE_SPEED:
        return GIMBAL_OK;
    default:
        return GIMBAL_ERR_PARAM;
    }
}

static int c12_udp_set_target(struct gimbal_dev *dev,
    const gimbal_euler_t *target) {
    gimbal_mode_t mode;
    struct c12_udp_priv *priv = dev->priv_data;

    pthread_mutex_lock(&dev->cmd_lock);
    mode = dev->mode;
    pthread_mutex_unlock(&dev->cmd_lock);

    switch (mode) {
    case GIMBAL_MODE_ANGLE_ABS:
    case GIMBAL_MODE_ANGLE_REL:
        priv->resend_elapsed_s = 0.0f;
        return send_angle_command(dev, target);
    case GIMBAL_MODE_SPEED:
        priv->resend_elapsed_s = 0.0f;
        return send_speed_command(dev, target);
    default:
        return GIMBAL_ERR_PARAM;
    }
}

static int c12_udp_set_limits(struct gimbal_dev *dev,
    const gimbal_limits_t *limits) {
    (void)dev;
    (void)limits;
    return GIMBAL_OK;
}

static int c12_udp_set_zoom(struct gimbal_dev *dev,
    gimbal_zoom_dir_t dir, uint8_t speed_level) {
    (void)speed_level;
    return send_digital_zoom_command(dev, dir);
}

static void parse_gac_feedback(struct gimbal_dev *dev, const uint8_t *data,
    size_t data_len) {
    int16_t yaw_q;
    int16_t pitch_q;
    int16_t roll_q;
    gimbal_euler_t prev_angle;
    gimbal_euler_t cur_angle;
    double now_s;
    double dt_s;

    if (data_len < 12U)
        return;
    if (!parse_hex_s16(&data[0], &yaw_q) ||
        !parse_hex_s16(&data[4], &pitch_q) ||
        !parse_hex_s16(&data[8], &roll_q))
        return;

    cur_angle.yaw = yaw_q / 100.0f;
    cur_angle.pitch = pitch_q / 100.0f;
    cur_angle.roll = roll_q / 100.0f;
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
    pthread_mutex_unlock(&dev->state_lock);

#if DEBUG_ENABLE
    printf("[GIMBAL-C12-UDP][GAC] yaw=%.2f pitch=%.2f roll=%.2f\n",
        cur_angle.yaw, cur_angle.pitch, cur_angle.roll);
#endif
}

static bool is_top_header(const uint8_t *buf) {
    return buf[0] == '#' && (buf[1] == 'T' || buf[1] == 't') &&
        (buf[2] == 'P' || buf[2] == 'p');
}

static void process_rx_frame(struct gimbal_dev *dev, const uint8_t *buf,
    size_t len) {
    int data_len;
    size_t total_len;
    const uint8_t *data;

    if (len < C12_FRAME_OVERHEAD_LEN || !is_top_header(buf))
        return;

    data_len = hex_nibble(buf[5]);
    if (data_len < 0)
        return;

    total_len = C12_FRAME_OVERHEAD_LEN + (size_t)data_len;
    if (len < total_len)
        return;

    if (!verify_checksum(buf, total_len)) {
        printf("[GIMBAL-C12-UDP] RX checksum mismatch\n");
        return;
    }

#if DEBUG_ENABLE
    printf("[GIMBAL-C12-UDP][RX] %.*s\n", (int)total_len, buf);
#endif

    data = &buf[10];
    if (memcmp(&buf[7], "GAC", 3U) == 0)
        parse_gac_feedback(dev, data, (size_t)data_len);
}

static void append_rx_bytes(struct gimbal_dev *dev, const uint8_t *buf,
    size_t len) {
    struct c12_udp_priv *priv = dev->priv_data;
    size_t keep_len;

    if (len == 0U)
        return;

    if (len > sizeof(priv->rx_buf)) {
        buf += len - sizeof(priv->rx_buf);
        len = sizeof(priv->rx_buf);
    }

    if (priv->rx_len + len > sizeof(priv->rx_buf)) {
        keep_len = sizeof(priv->rx_buf) - len;
        if (keep_len > 0U) {
            memmove(priv->rx_buf,
                priv->rx_buf + priv->rx_len - keep_len,
                keep_len);
        }
        priv->rx_len = keep_len;
        printf("[GIMBAL-C12-UDP] RX buffer overflow, "
            "keeping latest bytes\n");
    }

    memcpy(priv->rx_buf + priv->rx_len, buf, len);
    priv->rx_len += len;
}

static void consume_rx_stream(struct gimbal_dev *dev) {
    struct c12_udp_priv *priv = dev->priv_data;
    int data_len;
    size_t total_len;

    while (priv->rx_len > 0U) {
        if (priv->rx_buf[0] != '#') {
            memmove(priv->rx_buf, priv->rx_buf + 1, priv->rx_len - 1U);
            priv->rx_len -= 1U;
            continue;
        }

        if (priv->rx_len < 6U)
            break;
        if (!is_top_header(priv->rx_buf)) {
            memmove(priv->rx_buf, priv->rx_buf + 1, priv->rx_len - 1U);
            priv->rx_len -= 1U;
            continue;
        }

        data_len = hex_nibble(priv->rx_buf[5]);
        if (data_len < 0 || data_len > (int)C12_MAX_DATA_LEN) {
            memmove(priv->rx_buf, priv->rx_buf + 1, priv->rx_len - 1U);
            priv->rx_len -= 1U;
            continue;
        }

        total_len = C12_FRAME_OVERHEAD_LEN + (size_t)data_len;
        if (priv->rx_len < total_len)
            break;

        process_rx_frame(dev, priv->rx_buf, total_len);
        memmove(priv->rx_buf, priv->rx_buf + total_len,
            priv->rx_len - total_len);
        priv->rx_len -= total_len;
    }
}

static void c12_udp_tick(struct gimbal_dev *dev, float dt_s) {
    struct c12_udp_priv *priv = dev->priv_data;
    uint8_t buf[256];
    ssize_t n;
    gimbal_mode_t mode;
    gimbal_euler_t target;

    do {
        n = recvfrom(priv->sockfd, buf, sizeof(buf), 0, NULL, NULL);
        if (n > 0) {
            append_rx_bytes(dev, buf, (size_t)n);
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

    if (mode == GIMBAL_MODE_SPEED &&
        priv->resend_elapsed_s >= priv->resend_period_s) {
        priv->resend_elapsed_s = 0.0f;
        (void)send_speed_command(dev, &target);
    }
}

static void c12_udp_free(struct gimbal_dev *dev) {
    struct c12_udp_priv *priv;

    if (!dev)
        return;

    priv = dev->priv_data;
    if (priv && priv->sockfd >= 0) {
        (void)send_feedback_enable(dev, 0U);
        close(priv->sockfd);
        priv->sockfd = -1;
    }

    gimbal_dev_free_default(dev);
}

static const struct gimbal_ops c12_udp_ops = {
    .set_mode = c12_udp_set_mode,
    .set_target = c12_udp_set_target,
    .set_limits = c12_udp_set_limits,
    .set_zoom = c12_udp_set_zoom,
    .tick = c12_udp_tick,
    .free = c12_udp_free,
};

static int open_udp_socket(struct c12_udp_priv *priv,
    const gimbal_udp_config_t *cfg) {
    struct sockaddr_in bind_addr;
    int fd;
    int flags;
    int optval = 1;
    const char *bind_ip = (cfg && cfg->bind_ip) ? cfg->bind_ip :
        C12_DEFAULT_BIND_IP;
    uint16_t bind_port = (cfg && cfg->bind_port != 0U) ? cfg->bind_port :
        C12_DEFAULT_UDP_PORT;
    const char *device_ip = (cfg && cfg->device_ip) ? cfg->device_ip :
        C12_DEFAULT_DEVICE_IP;
    uint16_t device_port = (cfg && cfg->device_port != 0U) ?
        cfg->device_port : C12_DEFAULT_UDP_PORT;

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
        return -1;

    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) <
        0) {
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
    priv->resend_period_s = (cfg && cfg->resend_period_s > 0.0f) ?
        cfg->resend_period_s : C12_DEFAULT_RESEND_PERIOD;
    priv->resend_elapsed_s = 0.0f;
    return 0;
}

static struct gimbal_dev *c12_udp_create(const char *name, void *args) {
    struct gimbal_dev *dev;
    struct c12_udp_priv *priv;
    const gimbal_udp_config_t *cfg = args;

    dev = gimbal_dev_alloc(name, sizeof(*priv));
    if (!dev)
        return NULL;

    dev->ops = &c12_udp_ops;
    priv = dev->priv_data;
    priv->sockfd = -1;

    if (open_udp_socket(priv, cfg) != 0) {
        printf("[GIMBAL-C12-UDP] Failed to open UDP socket: %s\n",
            strerror(errno));
        gimbal_dev_free_default(dev);
        return NULL;
    }

    (void)send_feedback_enable(dev, C12_DEFAULT_FEEDBACK_HZ);
    printf("[GIMBAL-C12-UDP] Initialized driver '%s'\n", name);
    return dev;
}

REGISTER_GIMBAL_DRIVER("drv_udp_c12", c12_udp_create)
