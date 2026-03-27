/*
 * Copyright (c) 2026, iHomeRobot Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @brief Buzzer driver - Active buzzer with waveform control
 */

#ifndef PERIPHERALS_BUZZER_H__
#define PERIPHERALS_BUZZER_H__

#include <rtthread.h>

/* ========== Buzzer Command Structure ========== */

/**
 * @brief Buzzer command definition
 * @param on_time   High level duration (ms)
 * @param off_time  Low level duration (ms)
 * @param repeat    Number of cycles (0 = infinite loop)
 */
typedef struct {
    uint16_t on_time;
    uint16_t off_time;
    uint16_t repeat;
} BuzzerCommandTypeDef;

typedef enum {
    BUZZER_IDLE,
    BUZZER_ON,
    BUZZER_OFF
} BuzzerState_t;

/* ========== Public API ========== */

/**
 * @brief Initialize buzzer system
 * @note Creates thread and message queue
 */
void buzzer_init(void);

/**
 * @brief Send a custom command to buzzer (preempts current pattern)
 * @param cmd Pointer to command structure
 */
void buzzer_start(const BuzzerCommandTypeDef *cmd);

/**
 * @brief Pre-defined: Single short beep (200ms)
 */
void buzzer_beep_once(void);
/**
 * @brief Pre-defined: Alarm mode (500ms on, 500ms off, infinite loop)
 */
void buzzer_beep_alarm(void);

/**
 * @brief Pre-defined: Error alarm (100ms on, 100ms off, infinite loop)
 */
void buzzer_beep_error(void);

/**
 * @brief Stop any ongoing buzzer pattern immediately
 */
void buzzer_stop(void);

#endif /* PERIPHERALS_BUZZER_H__ */
