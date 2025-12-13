/**
 * @file main.c
 * @brief ESP32 Robotic Arm Controller with 4 Servo Motors (MG90S)
 * 
 * This program controls a 3-DOF robotic arm using PWM signals via LEDC to drive MG90S servos.
 * It receives commands via UART (serial) to set servo angles.
 * 
 * Hardware Configuration:
 *   - Servo 1 (Base):     GPIO 5  (D5)
 *   - Servo 2 (Shoulder): GPIO 18 (D18)
 *   - Servo 3 (Elbow):    GPIO 19 (D19)
 *   - Servo 4 (Gripper):  GPIO 21 (D21)
 * 
 * Serial Command Format:
 *   - Instant movement:  "I:base,shoulder,elbow,gripper"  e.g., "I:90,90,90,90"
 *   - Smooth movement:   "S:base,shoulder,elbow,gripper"  e.g., "S:45,120,60,30"
 *   - Home position:     "H"
 *   - Query position:    "?"
 * 
 * @author Vinicius Alves - Projeto Spark
 * @date 2025
 */

/* ============================================================================
 * INCLUDES
 * ============================================================================ */

#include <stdio.h>              // Standard I/O functions (printf, etc.)
#include <string.h>             // String manipulation functions (strlen, strtok, etc.)
#include <stdlib.h>             // Standard library (atoi, etc.)
#include "freertos/FreeRTOS.h"  // FreeRTOS kernel
#include "freertos/task.h"      // FreeRTOS task management
#include "driver/ledc.h"        // LED Control (PWM) driver - used for servo control
#include "driver/uart.h"        // UART driver for serial communication
#include "esp_log.h"            // ESP-IDF logging library

/* ============================================================================
 * CONFIGURATION CONSTANTS
 * ============================================================================ */

/** @brief Tag used for ESP_LOG messages */
static const char *TAG = "ROBOTIC_ARM";

/* ----------------------------------------------------------------------------
 * UART Configuration
 * ---------------------------------------------------------------------------- */
#define UART_PORT_NUM      UART_NUM_0   // Using UART0 (USB serial)
#define UART_BAUD_RATE     115200       // Baud rate for serial communication
#define UART_TX_PIN        1            // Default TX pin for UART0
#define UART_RX_PIN        3            // Default RX pin for UART0
#define UART_BUF_SIZE      256          // UART receive buffer size

/* ----------------------------------------------------------------------------
 * PWM (LEDC) Configuration for Servos
 * 
 * MG90S Servo Specifications:
 *   - Operating frequency: 50Hz (20ms period)
 *   - Pulse width range: 500µs (0°) to 2500µs (180°)
 *   - Some servos use 1000µs to 2000µs for 0° to 180°
 * ---------------------------------------------------------------------------- */
#define SERVO_PWM_FREQ_HZ       50      // 50Hz = 20ms period (standard for servos)
#define SERVO_PWM_RESOLUTION    LEDC_TIMER_14_BIT  // 14-bit resolution (0-16383)
#define SERVO_PWM_SPEED_MODE    LEDC_LOW_SPEED_MODE // Low speed mode is sufficient

/**
 * @brief Timer assignments for each servo
 * 
 * Using separate timers for each servo to prevent interference when updating
 * duty cycles. This significantly reduces jitter.
 * ESP32 LEDC has 4 timers in low-speed mode (0-3)
 */
#define SERVO_TIMER_BASE        LEDC_TIMER_0
#define SERVO_TIMER_SHOULDER    LEDC_TIMER_1
#define SERVO_TIMER_ELBOW       LEDC_TIMER_2
#define SERVO_TIMER_GRIPPER     LEDC_TIMER_3

/**
 * @brief Pulse width in microseconds for 0° position
 */
#define SERVO_MIN_PULSEWIDTH_US 500

/**
 * @brief Pulse width in microseconds for 180° position
 */
#define SERVO_MAX_PULSEWIDTH_US 2500

/* ----------------------------------------------------------------------------
 * GPIO Pin Assignments for Servos
 * ---------------------------------------------------------------------------- */
#define GPIO_SERVO_BASE     5   // Servo 1: Base rotation
#define GPIO_SERVO_SHOULDER 18  // Servo 2: Shoulder joint
#define GPIO_SERVO_ELBOW    19  // Servo 3: Elbow joint
#define GPIO_SERVO_GRIPPER  21  // Servo 4: Gripper (open/close)

/* ----------------------------------------------------------------------------
 * Servo Index Definitions
 * ---------------------------------------------------------------------------- */
#define SERVO_BASE      0
#define SERVO_SHOULDER  1
#define SERVO_ELBOW     2
#define SERVO_GRIPPER   3
#define NUM_SERVOS      4

/* ----------------------------------------------------------------------------
 * Movement Configuration
 * ---------------------------------------------------------------------------- */
#define SMOOTH_MOVE_DELAY_MS    15  // Delay between steps in smooth movement (ms)
#define SMOOTH_MOVE_STEP_DEG    2   // Degrees per step in smooth movement

/* ============================================================================
 * DATA STRUCTURES
 * ============================================================================ */

/**
 * @brief Configuration structure for a single servo motor
 */
typedef struct {
    const char *name;       // Human-readable name for logging
    int gpio_pin;           // GPIO pin number connected to servo signal wire
    ledc_channel_t channel; // LEDC (PWM) channel assigned to this servo
    ledc_timer_t timer;     // LEDC timer assigned to this servo (separate timers reduce jitter)
    int min_angle;          // Minimum allowed angle (degrees) - SOFTWARE LIMIT
    int max_angle;          // Maximum allowed angle (degrees) - SOFTWARE LIMIT
    int current_angle;      // Current angle position (degrees)
    int initial_angle;      // Initial/home angle (degrees)
} servo_config_t;

/**
 * @brief Movement type enumeration
 */
typedef enum {
    MOVE_INSTANT,   // Jump directly to target angle
    MOVE_SMOOTH     // Gradually move to target angle
} move_type_t;

/* ============================================================================
 * GLOBAL VARIABLES
 * ============================================================================ */

/**
 * @brief Array of servo configurations
 */
static servo_config_t servos[NUM_SERVOS] = {
    // Servo 0: Base
    {
        .name = "Base",
        .gpio_pin = GPIO_SERVO_BASE,
        .channel = LEDC_CHANNEL_0,
        .timer = SERVO_TIMER_BASE,
        .min_angle = 0,
        .max_angle = 180,
        .current_angle = 90,
        .initial_angle = 90
    },
    // Servo 1: Shoulder
    {
        .name = "Shoulder",
        .gpio_pin = GPIO_SERVO_SHOULDER,
        .channel = LEDC_CHANNEL_1,
        .timer = SERVO_TIMER_SHOULDER,
        .min_angle = 0,
        .max_angle = 180,
        .current_angle = 90,
        .initial_angle = 90
    },
    // Servo 2: Elbow
    {
        .name = "Elbow",
        .gpio_pin = GPIO_SERVO_ELBOW,
        .channel = LEDC_CHANNEL_2,
        .timer = SERVO_TIMER_ELBOW,
        .min_angle = 0,
        .max_angle = 180,
        .current_angle = 90,
        .initial_angle = 90
    },
    // Servo 3: Gripper
    {
        .name = "Gripper",
        .gpio_pin = GPIO_SERVO_GRIPPER,
        .channel = LEDC_CHANNEL_3,
        .timer = SERVO_TIMER_GRIPPER,
        .min_angle = 0,
        .max_angle = 180,
        .current_angle = 90,
        .initial_angle = 90
    }
};

/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================ */

static void init_uart(void);
static void init_pwm_timer_for_servo(ledc_timer_t timer_num);
static void init_all_pwm_timers(void);
static void init_servo(int servo_index);
static void init_all_servos(void);
static uint32_t angle_to_duty(int angle);
static int constrain_angle(int servo_index, int angle);
static void set_servo_angle_instant(int servo_index, int angle);
static void set_servo_angle_smooth(int servo_index, int target_angle);
static void process_command(const char *command);
static void send_feedback(const char *message);
static void serial_receive_task(void *pvParameters);
static void move_to_home_position(void);

/* ============================================================================
 * INITIALIZATION FUNCTIONS
 * ============================================================================ */

/**
 * @brief Initialize UART for serial communication
 */
static void init_uart(void)
{
    ESP_LOGI(TAG, "Initializing UART%d at %d baud...", UART_PORT_NUM, UART_BAUD_RATE);
    
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, 
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "UART initialized successfully");
}

/**
 * @brief Initialize a PWM timer for servo control
 * 
 * @param timer_num Timer number to initialize
 */
static void init_pwm_timer_for_servo(ledc_timer_t timer_num)
{
    ledc_timer_config_t timer_config = {
        .speed_mode      = SERVO_PWM_SPEED_MODE,
        .duty_resolution = SERVO_PWM_RESOLUTION,
        .timer_num       = timer_num,
        .freq_hz         = SERVO_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));
}

/**
 * @brief Initialize all PWM timers (one per servo)
 */
static void init_all_pwm_timers(void)
{
    ESP_LOGI(TAG, "Initializing %d PWM timers...", NUM_SERVOS);
    
    init_pwm_timer_for_servo(SERVO_TIMER_BASE);
    init_pwm_timer_for_servo(SERVO_TIMER_SHOULDER);
    init_pwm_timer_for_servo(SERVO_TIMER_ELBOW);
    init_pwm_timer_for_servo(SERVO_TIMER_GRIPPER);
    
    ESP_LOGI(TAG, "All PWM timers initialized");
}

/**
 * @brief Initialize a single servo motor
 * 
 * @param servo_index Index of the servo to initialize (0-3)
 */
static void init_servo(int servo_index)
{
    if (servo_index < 0 || servo_index >= NUM_SERVOS) {
        ESP_LOGE(TAG, "Invalid servo index: %d", servo_index);
        return;
    }
    
    servo_config_t *servo = &servos[servo_index];
    
    ESP_LOGI(TAG, "Initializing servo %d (%s) on GPIO %d...", 
             servo_index, servo->name, servo->gpio_pin);
    
    ledc_channel_config_t channel_config = {
        .gpio_num   = servo->gpio_pin,
        .speed_mode = SERVO_PWM_SPEED_MODE,
        .channel    = servo->channel,
        .timer_sel  = servo->timer,
        .duty       = 0,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
    
    // Set initial position
    uint32_t duty = angle_to_duty(servo->initial_angle);
    ESP_ERROR_CHECK(ledc_set_duty(SERVO_PWM_SPEED_MODE, servo->channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(SERVO_PWM_SPEED_MODE, servo->channel));
    
    ESP_LOGI(TAG, "Servo %d (%s) initialized", servo_index, servo->name);
}

/**
 * @brief Initialize all servo motors
 */
static void init_all_servos(void)
{
    ESP_LOGI(TAG, "Initializing all %d servos...", NUM_SERVOS);
    
    for (int i = 0; i < NUM_SERVOS; i++) {
        init_servo(i);
    }
    
    ESP_LOGI(TAG, "All servos initialized successfully");
}

/* ============================================================================
 * ANGLE/PWM CONVERSION FUNCTIONS
 * ============================================================================ */

/**
 * @brief Convert angle (degrees) to PWM duty cycle
 * 
 * @param angle Angle in degrees (0-180)
 * @return Duty cycle value
 */
static uint32_t angle_to_duty(int angle)
{
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    // Calculate pulse width in microseconds
    uint32_t pulse_width_us = SERVO_MIN_PULSEWIDTH_US + 
        (angle * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US)) / 180;
    
    // Convert to duty cycle
    // PWM period = 20ms = 20000µs
    // Duty = (pulse_width / period) * max_duty
    uint32_t max_duty = (1 << LEDC_TIMER_14_BIT) - 1;  // 16383 for 14-bit
    uint32_t duty = (pulse_width_us * max_duty) / 20000;
    
    return duty;
}

/**
 * @brief Constrain angle to servo's allowed limits
 * 
 * @param servo_index Index of the servo (0-3)
 * @param angle Requested angle in degrees
 * @return Constrained angle
 */
static int constrain_angle(int servo_index, int angle)
{
    if (servo_index < 0 || servo_index >= NUM_SERVOS) {
        return angle;
    }
    
    servo_config_t *servo = &servos[servo_index];
    
    if (angle < servo->min_angle) {
        return servo->min_angle;
    }
    if (angle > servo->max_angle) {
        return servo->max_angle;
    }
    return angle;
}

/* ============================================================================
 * SERVO CONTROL FUNCTIONS
 * ============================================================================ */

/**
 * @brief Set servo to a specific angle instantly
 * 
 * @param servo_index Index of the servo (0-3)
 * @param angle Target angle in degrees
 */
static void set_servo_angle_instant(int servo_index, int angle)
{
    if (servo_index < 0 || servo_index >= NUM_SERVOS) {
        ESP_LOGE(TAG, "Invalid servo index: %d", servo_index);
        return;
    }
    
    servo_config_t *servo = &servos[servo_index];
    
    // Constrain angle to allowed limits
    angle = constrain_angle(servo_index, angle);
    
    // Skip update if angle hasn't changed (prevents jitter)
    if (angle == servo->current_angle) {
        return;
    }
    
    // Convert angle to duty cycle
    uint32_t duty = angle_to_duty(angle);
    
    // Set the PWM duty cycle
    ESP_ERROR_CHECK(ledc_set_duty(SERVO_PWM_SPEED_MODE, servo->channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(SERVO_PWM_SPEED_MODE, servo->channel));
    
    // Update current angle
    servo->current_angle = angle;
    
    ESP_LOGD(TAG, "%s: Set to %d° (duty: %lu)", servo->name, angle, duty);
}

/**
 * @brief Set servo to a specific angle with smooth movement
 * 
 * @param servo_index Index of the servo (0-3)
 * @param target_angle Target angle in degrees
 */
static void set_servo_angle_smooth(int servo_index, int target_angle)
{
    if (servo_index < 0 || servo_index >= NUM_SERVOS) {
        ESP_LOGE(TAG, "Invalid servo index: %d", servo_index);
        return;
    }
    
    servo_config_t *servo = &servos[servo_index];
    
    // Constrain target angle
    target_angle = constrain_angle(servo_index, target_angle);

    // Skip if already at target
    if (target_angle == servo->current_angle) {
        return;
    }
    
    int current = servo->current_angle;
    int step = (target_angle > current) ? SMOOTH_MOVE_STEP_DEG : -SMOOTH_MOVE_STEP_DEG;
    
    ESP_LOGI(TAG, "%s: Smooth move from %d° to %d°", servo->name, current, target_angle);
    
    // Gradually move towards target
    while (current != target_angle) {
        if (abs(target_angle - current) <= SMOOTH_MOVE_STEP_DEG) {
            current = target_angle;
        } else {
            current += step;
        }
        
        uint32_t duty = angle_to_duty(current);
        ESP_ERROR_CHECK(ledc_set_duty(SERVO_PWM_SPEED_MODE, servo->channel, duty));
        ESP_ERROR_CHECK(ledc_update_duty(SERVO_PWM_SPEED_MODE, servo->channel));
        
        servo->current_angle = current;
        vTaskDelay(pdMS_TO_TICKS(SMOOTH_MOVE_DELAY_MS));
    }
    
    ESP_LOGD(TAG, "%s: Smooth move complete at %d°", servo->name, target_angle);
}

/**
 * @brief Move all servos to their home positions
 */
static void move_to_home_position(void)
{
    ESP_LOGI(TAG, "Moving all servos to home positions...");
    
    for (int i = 0; i < NUM_SERVOS; i++) {
        set_servo_angle_instant(i, servos[i].initial_angle);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    ESP_LOGI(TAG, "All servos at home position");
}

/* ============================================================================
 * SERIAL COMMUNICATION FUNCTIONS
 * ============================================================================ */

/**
 * @brief Send feedback message via UART
 * 
 * @param message Message string to send
 */
static void send_feedback(const char *message)
{
    uart_write_bytes(UART_PORT_NUM, message, strlen(message));
    uart_write_bytes(UART_PORT_NUM, "\r\n", 2);
}

/**
 * @brief Process a received command string
 * 
 * @param command Null-terminated command string
 */
static void process_command(const char *command)
{
    char response[128];
    
    if (strlen(command) < 1) {
        return;
    }
    
    ESP_LOGI(TAG, "Received command: %s", command);
    
    // Home command
    if (command[0] == 'H' || command[0] == 'h') {
        move_to_home_position();
        snprintf(response, sizeof(response), "OK:HOME,%d,%d,%d,%d",
                 servos[0].current_angle, servos[1].current_angle,
                 servos[2].current_angle, servos[3].current_angle);
        send_feedback(response);
        return;
    }
    
    // Query command
    if (command[0] == '?') {
        snprintf(response, sizeof(response), "POS:%d,%d,%d,%d",
                 servos[0].current_angle, servos[1].current_angle,
                 servos[2].current_angle, servos[3].current_angle);
        send_feedback(response);
        return;
    }
    
    // Movement commands
    if (strlen(command) < 3 || command[1] != ':') {
        send_feedback("ERR:Invalid format. Use I:a,b,c,d or S:a,b,c,d");
        return;
    }
    
    move_type_t move_type;
    if (command[0] == 'I' || command[0] == 'i') {
        move_type = MOVE_INSTANT;
    } else if (command[0] == 'S' || command[0] == 's') {
        move_type = MOVE_SMOOTH;
    } else {
        send_feedback("ERR:Unknown command. Use I (instant) or S (smooth)");
        return;
    }
    
    // Parse angles
    char cmd_copy[64];
    strncpy(cmd_copy, command + 2, sizeof(cmd_copy) - 1);
    cmd_copy[sizeof(cmd_copy) - 1] = '\0';
    
    int angles[NUM_SERVOS];
    int parsed_count = 0;
    
    char *token = strtok(cmd_copy, ",");
    while (token != NULL && parsed_count < NUM_SERVOS) {
        angles[parsed_count] = atoi(token);
        parsed_count++;
        token = strtok(NULL, ",");
    }
    
    if (parsed_count != NUM_SERVOS) {
        snprintf(response, sizeof(response), "ERR:Expected %d angles, got %d", 
                 NUM_SERVOS, parsed_count);
        send_feedback(response);
        return;
    }
    
    // Execute movement
    if (move_type == MOVE_INSTANT) {
        for (int i = 0; i < NUM_SERVOS; i++) {
            set_servo_angle_instant(i, angles[i]);
        }
    } else {
        for (int i = 0; i < NUM_SERVOS; i++) {
            set_servo_angle_smooth(i, angles[i]);
        }
    }
    
    // Send confirmation
    snprintf(response, sizeof(response), "OK:%d,%d,%d,%d",
             servos[0].current_angle, servos[1].current_angle,
             servos[2].current_angle, servos[3].current_angle);
    send_feedback(response);
}

/**
 * @brief FreeRTOS task for receiving serial commands
 * 
 * @param pvParameters Task parameters (not used)
 */
static void serial_receive_task(void *pvParameters)
{
    uint8_t data[UART_BUF_SIZE];
    char command_buffer[UART_BUF_SIZE];
    int cmd_index = 0;
    
    ESP_LOGI(TAG, "Serial receive task started");
    send_feedback("READY:Robotic Arm Controller v1.0 (LEDC)");
    send_feedback("HELP:Commands: I:a,b,c,d (instant), S:a,b,c,d (smooth), H (home), ? (query)");
    
    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE - 1, 
                                  pdMS_TO_TICKS(100));
        
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = (char)data[i];
                
                if (c == '\n' || c == '\r') {
                    if (cmd_index > 0) {
                        command_buffer[cmd_index] = '\0';
                        process_command(command_buffer);
                        cmd_index = 0;
                    }
                } else {
                    if (cmd_index < UART_BUF_SIZE - 1) {
                        command_buffer[cmd_index++] = c;
                    } else {
                        ESP_LOGW(TAG, "Command buffer overflow, resetting");
                        cmd_index = 0;
                    }
                }
            }
        }
    }
}

/* ============================================================================
 * MAIN APPLICATION ENTRY POINT
 * ============================================================================ */

/**
 * @brief Main application entry point
 */
void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "   ESP32 Robotic Arm Controller");
    ESP_LOGI(TAG, "   Using LEDC PWM");
    ESP_LOGI(TAG, "========================================");
    
    // Step 1: Initialize UART
    ESP_LOGI(TAG, "Step 1: Initializing UART...");
    init_uart();
    
    // Step 2: Initialize PWM timers (one per servo)
    ESP_LOGI(TAG, "Step 2: Initializing PWM timers...");
    init_all_pwm_timers();
    
    // Step 3: Initialize servos
    ESP_LOGI(TAG, "Step 3: Initializing servos...");
    init_all_servos();
    
    // Step 4: Move to home position
    ESP_LOGI(TAG, "Step 4: Moving to home position...");
    move_to_home_position();
    
    // Step 5: Create serial receive task
    ESP_LOGI(TAG, "Step 5: Starting serial receive task...");
    xTaskCreate(serial_receive_task,
                "serial_rx_task",
                4096,
                NULL,
                5,
                NULL);
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "   Initialization complete!");
    ESP_LOGI(TAG, "   Waiting for serial commands...");
    ESP_LOGI(TAG, "========================================");
}
