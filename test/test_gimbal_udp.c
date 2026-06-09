/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "gimbal.h"

#define DRIVER_TZ0XXX "drv_udp_tz0xxx"
#define DRIVER_C12    "drv_udp_c12"

static volatile bool g_running = true;

static void signal_handler(int sig) {
    (void)sig;
    g_running = false;
}

static bool is_c12_driver(const char *driver_name) {
    return driver_name && strcmp(driver_name, DRIVER_C12) == 0;
}

static const char *mode_name(gimbal_mode_t mode) {
    switch (mode) {
    case GIMBAL_MODE_OFF:
        return "OFF";
    case GIMBAL_MODE_ANGLE_ABS:
        return "ANGLE_ABS";
    case GIMBAL_MODE_ANGLE_REL:
        return "ANGLE_REL";
    case GIMBAL_MODE_SPEED:
        return "SPEED";
    case GIMBAL_MODE_FOLLOW:
        return "FOLLOW";
    case GIMBAL_MODE_FPV:
        return "FPV";
    case GIMBAL_MODE_LOCK:
        return "LOCK";
    case GIMBAL_MODE_CALIBRATE:
        return "CALIBRATE";
    default:
        return "UNKNOWN";
    }
}

static const char *zoom_name(gimbal_zoom_dir_t dir) {
    switch (dir) {
    case GIMBAL_ZOOM_STOP:
        return "ZOOM_STOP";
    case GIMBAL_ZOOM_IN:
        return "ZOOM_IN";
    case GIMBAL_ZOOM_OUT:
        return "ZOOM_OUT";
    default:
        return "ZOOM_UNKNOWN";
    }
}

static void usage(const char *prog) {
    printf("Usage: %s [OPTIONS]\n", prog);
    printf("\n");
    printf("Options:\n");
    printf("  --driver <name>     Driver: %s or %s (default: %s)\n",
        DRIVER_C12, DRIVER_TZ0XXX, DRIVER_C12);
    printf("  --ip <addr>         Gimbal IP (C12 default: 192.168.144.108; TZ0xxx default: 192.168.44.160)\n");
    printf("  --port <num>        Gimbal port (C12 default: 5000; TZ0xxx default: 4900)\n");
    printf("  --bind-ip <addr>    Local bind IP (default: 0.0.0.0)\n");
    printf("  --bind-port <num>   Local bind port (C12 default: 5000; TZ0xxx default: 4900)\n");
    printf("  -h, --help          Show this help message\n");
    printf("\n");
    printf("Demo sequence:\n");
    printf("  1. Center\n");
    printf("  2. Absolute guide\n");
    printf("  3. Relative guide\n");
    printf("  4. Speed mode yaw command for 2s\n");
    printf("  5. Zoom in / stop / zoom out / stop\n");
    printf("  6. Switch to heading lock\n");
    printf("\n");
    printf("Press Ctrl+C to stop early.\n");
}

static int send_mode(struct gimbal_dev *dev, gimbal_mode_t mode) {
    int ret;

    printf("[TX-API] set_mode(%s)\n", mode_name(mode));
    ret = gimbal_set_mode(dev, mode);
    printf("[TX-API] set_mode(%s) -> %d\n", mode_name(mode), ret);
    return ret;
}

static int send_target(struct gimbal_dev *dev, const char *label,
        const gimbal_euler_t *target) {
    int ret;

    printf("[TX-API] %s target: pitch=%.2f yaw=%.2f roll=%.2f\n",
        label, target->pitch, target->yaw, target->roll);
    ret = gimbal_set_target(dev, target);
    printf("[TX-API] %s -> %d\n", label, ret);
    return ret;
}

static int send_zoom(struct gimbal_dev *dev, gimbal_zoom_dir_t dir,
        uint8_t speed_level) {
    int ret;

    printf("[TX-API] set_zoom(%s, speed=%u)\n", zoom_name(dir), speed_level);
    ret = gimbal_set_zoom(dev, dir, speed_level);
    printf("[TX-API] set_zoom(%s, speed=%u) -> %d\n",
        zoom_name(dir), speed_level, ret);
    return ret;
}

static void tick_for(struct gimbal_dev *dev, float duration_s,
    const char *label) {
    int elapsed_ms = 0;
    int total_ms = (int)(duration_s * 1000.0f);

    printf("\n[%s] %.1f s\n", label, duration_s);
    while (g_running && elapsed_ms < total_ms) {
        gimbal_tick(dev, 0.02f);
        usleep(20000);
        elapsed_ms += 20;

        if (elapsed_ms % 200 == 0) {
            gimbal_euler_t angle = {0.0f, 0.0f, 0.0f};
            gimbal_euler_t speed = {0.0f, 0.0f, 0.0f};
            int ret = gimbal_get_state(dev, &angle, &speed);
            if (ret == GIMBAL_OK) {
                printf(
                    "[RX-STATE] pitch=%7.2f yaw=%7.2f roll=%7.2f | "
                    "speed: pitch=%6.2f yaw=%6.2f roll=%6.2f | stable=%s\n",
                    angle.pitch, angle.yaw, angle.roll,
                    speed.pitch, speed.yaw, speed.roll,
                    gimbal_is_stable(dev, 1.5f) ? "yes" : "no");
            } else {
                printf("[RX-STATE] waiting feedback...\n");
            }
        }
    }
}

int main(int argc, char *argv[]) {
    const char *driver_name = DRIVER_C12;
    const char *device_ip = NULL;
    const char *bind_ip = "0.0.0.0";
    uint16_t device_port = 0U;
    uint16_t bind_port = 0U;
    struct gimbal_dev *dev;
    gimbal_udp_config_t cfg;
    gimbal_limits_t limits;
    gimbal_euler_t target;
    float abs_pitch;
    float abs_yaw;
    float rel_pitch;
    float rel_yaw;
    float speed_yaw;
    int i;
    int zoom_steps;

    for (i = 1; i < argc; ++i) {
        if ((strcmp(argv[i], "--driver") == 0) && i + 1 < argc) {
            driver_name = argv[++i];
        } else if ((strcmp(argv[i], "--ip") == 0) && i + 1 < argc) {
            device_ip = argv[++i];
        } else if ((strcmp(argv[i], "--port") == 0) && i + 1 < argc) {
            device_port = (uint16_t)atoi(argv[++i]);
        } else if ((strcmp(argv[i], "--bind-ip") == 0) && i + 1 < argc) {
            bind_ip = argv[++i];
        } else if ((strcmp(argv[i], "--bind-port") == 0) && i + 1 < argc) {
            bind_port = (uint16_t)atoi(argv[++i]);
        } else if (strcmp(argv[i], "-h") == 0 ||
                strcmp(argv[i], "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Unknown argument: %s\n", argv[i]);
            usage(argv[0]);
            return 1;
        }
    }

    if (strcmp(driver_name, DRIVER_C12) != 0 &&
            strcmp(driver_name, DRIVER_TZ0XXX) != 0) {
        fprintf(stderr, "Unsupported driver: %s\n", driver_name);
        usage(argv[0]);
        return 1;
    }

    if (is_c12_driver(driver_name)) {
        if (!device_ip)
            device_ip = "192.168.144.108";
        if (device_port == 0U)
            device_port = 5000U;
        if (bind_port == 0U)
            bind_port = 5000U;
        abs_pitch = -75.0f;
        abs_yaw = 55.0f;
        rel_pitch = 60.0f;
        rel_yaw = -50.0f;
        speed_yaw = 10.0f;
    } else {
        if (!device_ip)
            device_ip = "192.168.44.160";
        if (device_port == 0U)
            device_port = 4900U;
        if (bind_port == 0U)
            bind_port = 4900U;
        abs_pitch = -70.0f;
        abs_yaw = 58.0f;
        rel_pitch = 80.0f;
        rel_yaw = -20.0f;
        speed_yaw = 12.0f;
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    cfg.bind_ip = bind_ip;
    cfg.bind_port = bind_port;
    cfg.device_ip = device_ip;
    cfg.device_port = device_port;
    cfg.resend_period_s = 0.02f;

    memset(&limits, 0, sizeof(limits));
    limits.max_angle.pitch = 90.0f;
    limits.max_angle.yaw = 90.0f;
    limits.max_angle.roll = 90.0f;
    limits.min_angle.pitch = -90.0f;
    limits.min_angle.yaw = -90.0f;
    limits.min_angle.roll = -90.0f;
    limits.max_speed.pitch = is_c12_driver(driver_name) ? 9.9f : 50.0f;
    limits.max_speed.yaw = is_c12_driver(driver_name) ? 9.9f : 50.0f;
    limits.max_speed.roll = 0.0f;

    printf("=== Gimbal UDP Demo ===\n");
    printf("Driver: %s\n", driver_name);
    printf("Remote: %s:%u\n", device_ip, device_port);
    printf("Local : %s:%u\n", bind_ip, bind_port);

    dev = gimbal_alloc_udp(driver_name, &cfg);
    if (!dev) {
        fprintf(stderr, "Failed to create gimbal device\n");
        return 1;
    }

    (void)gimbal_set_limits(dev, &limits);
    tick_for(dev, 1.0f, "Warmup / receive feedback");

    if (g_running) {
        printf("\n[1] Center\n");
        if (send_mode(dev, GIMBAL_MODE_LOCK) != GIMBAL_OK)
            fprintf(stderr, "  set LOCK mode failed\n");
        if (send_mode(dev, GIMBAL_MODE_ANGLE_ABS) != GIMBAL_OK)
            fprintf(stderr, "  set ANGLE_ABS mode failed\n");
        target.pitch = 0.0f;
        target.yaw = 0.0f;
        target.roll = 0.0f;
        if (send_target(dev, "center", &target) != GIMBAL_OK)
            fprintf(stderr, "  center command failed\n");
        tick_for(dev, 3.0f, "Centering");
    }

    if (g_running) {
        printf("\n[2] Absolute guide\n");
        if (send_mode(dev, GIMBAL_MODE_ANGLE_ABS) != GIMBAL_OK)
            fprintf(stderr, "  set ANGLE_ABS mode failed\n");
        target.pitch = abs_pitch;
        target.yaw = abs_yaw;
        target.roll = 0.0f;
        if (send_target(dev, "absolute", &target) != GIMBAL_OK)
            fprintf(stderr, "  absolute target failed\n");
        tick_for(dev, 4.0f, "Absolute target");
    }

    if (g_running) {
        printf("\n[3] Relative guide\n");
        if (send_mode(dev, GIMBAL_MODE_ANGLE_REL) != GIMBAL_OK)
            fprintf(stderr, "  set ANGLE_REL mode failed\n");
        target.pitch = rel_pitch;
        target.yaw = rel_yaw;
        target.roll = 0.0f;
        if (send_target(dev, "relative", &target) != GIMBAL_OK)
            fprintf(stderr, "  relative target failed\n");
        tick_for(dev, 4.0f, "Relative target");
    }

    if (g_running) {
        printf("\n[4] Speed mode yaw %.1f deg/s\n", speed_yaw);
        if (send_mode(dev, GIMBAL_MODE_SPEED) != GIMBAL_OK)
            fprintf(stderr, "  set SPEED mode failed\n");
        target.pitch = 0.0f;
        target.yaw = speed_yaw;
        target.roll = 0.0f;
        if (send_target(dev, "speed", &target) != GIMBAL_OK)
            fprintf(stderr, "  speed target failed\n");
        tick_for(dev, 2.0f, "Speed command");

        target.pitch = 0.0f;
        target.yaw = 0.0f;
        target.roll = 0.0f;
        if (send_target(dev, "speed-stop", &target) != GIMBAL_OK)
            fprintf(stderr, "  stop speed command failed\n");
        tick_for(dev, 1.5f, "Speed stop");

        target.pitch = 0.0f;
        target.yaw = -speed_yaw;
        target.roll = 0.0f;
        if (send_target(dev, "speed", &target) != GIMBAL_OK)
            fprintf(stderr, "  speed target failed\n");
        tick_for(dev, 2.0f, "Speed command");

        target.pitch = 0.0f;
        target.yaw = 0.0f;
        target.roll = 0.0f;
        if (send_target(dev, "speed-stop", &target) != GIMBAL_OK)
            fprintf(stderr, "  stop speed command failed\n");
        tick_for(dev, 2.0f, "Speed stop");
    }

    if (g_running) {
        printf("\n[1] Center\n");
        if (send_mode(dev, GIMBAL_MODE_LOCK) != GIMBAL_OK)
            fprintf(stderr, "  set LOCK mode failed\n");
        if (send_mode(dev, GIMBAL_MODE_ANGLE_ABS) != GIMBAL_OK)
            fprintf(stderr, "  set ANGLE_ABS mode failed\n");
        target.pitch = 0.0f;
        target.yaw = 0.0f;
        target.roll = 0.0f;
        if (send_target(dev, "center", &target) != GIMBAL_OK)
            fprintf(stderr, "  center command failed\n");
        tick_for(dev, 3.0f, "Centering");
    }


    if (g_running) {
        printf("\n[5] Zoom test\n");
        if (send_zoom(dev, GIMBAL_ZOOM_IN, 4) != GIMBAL_OK)
            fprintf(stderr, "  zoom in failed\n");
        tick_for(dev, 1.5f, "Zoom in");

        /*c12 only support stepwise digital zoom*/
        if (is_c12_driver(driver_name)) {
            for (zoom_steps = 0; zoom_steps < 10; zoom_steps++) {
                send_zoom(dev, GIMBAL_ZOOM_IN, 0);
                tick_for(dev, 0.1f, "Zoom in");
            }
        }

        if (send_zoom(dev, GIMBAL_ZOOM_STOP, 0) != GIMBAL_OK)
            fprintf(stderr, "  zoom stop failed\n");
        tick_for(dev, 1.0f, "Zoom stop");


        if (send_zoom(dev, GIMBAL_ZOOM_OUT, 4) != GIMBAL_OK)
            fprintf(stderr, "  zoom out failed\n");
        tick_for(dev, 1.5f, "Zoom out");

        /*c12 only support stepwise digital zoom*/
        if (is_c12_driver(driver_name)) {
            for (zoom_steps = 0; zoom_steps < 10; zoom_steps++) {
                send_zoom(dev, GIMBAL_ZOOM_OUT, 0);
                tick_for(dev, 0.1f, "Zoom out");
            }
        }

        if (send_zoom(dev, GIMBAL_ZOOM_STOP, 0) != GIMBAL_OK)
            fprintf(stderr, "  final zoom stop failed\n");
        tick_for(dev, 1.0f, "Final zoom stop");
    }

    if (g_running) {
        printf("\n[6] Heading lock\n");
        if (send_mode(dev, GIMBAL_MODE_LOCK) != GIMBAL_OK)
            fprintf(stderr, "  set LOCK mode failed\n");
        tick_for(dev, 2.0f, "Heading lock");
    }

    printf("\nCleaning up...\n");
    gimbal_free(dev);
    return 0;
}
