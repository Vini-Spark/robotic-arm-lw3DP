/**
 * @file main.c
 * @brief ESP32 Robotic Arm Controller using MCPWM for Servo Control
 * 
 * This program controls 4 MG90S servo motors for a simple robotic arm.
 * The servos are controlled via PWM (Pulse Width Modulation) using the
 * ESP32's MCPWM (Motor Control PWM) peripheral, which is specifically
 * designed for motor control applications and provides better stability
 * than the LEDC peripheral for servo control.
 * 
 * Communication with a host computer (Python application) is done via 
 * UART (serial port).
 * 
 * Hardware Configuration:
 *   - Servo 1 (Base):      GPIO 5
 *   - Servo 2 (Shoulder):  GPIO 18
 *   - Servo 3 (Elbow):     GPIO 19
 *   - Servo 4 (Gripper):   GPIO 21
 * 
 * Servo Motor Specifications (MG90S):
 *   - Operating voltage: 4.8V - 6V
 *   - PWM frequency: 50Hz (20ms period)
 *   - Pulse width range: 500µs (0°) to 2500µs (180°)
 *   - Rotation range: 0° to 180°
 * 
 * Communication Protocol:
 *   - Baud rate: 115200
 *   - Command format: "TYPE:value1,value2,value3,value4\n"
 *   - Types: I (instant), S (smooth), H (home), ? (query)
 * 
 * ============================================================================
 * WHY MCPWM INSTEAD OF LEDC?
 * ============================================================================
 * 
 * The ESP32 has two distinct PWM peripherals: LEDC and MCPWM.
 * 
 * LEDC (LED Control):
 *   - Originally designed for LED dimming applications
 *   - Simpler API, good for single-channel applications
 *   - Can cause timing interference when multiple channels update together
 *   - Updates to one channel can momentarily affect others
 * 
 * MCPWM (Motor Control PWM):
 *   - Specifically designed for motor control (DC motors, servos, brushless)
 *   - Hardware-level isolation between channels
 *   - Synchronization capabilities
 *   - Dead-time insertion for H-bridge drivers
 *   - Fault detection and protection
 *   - Better suited for multi-servo applications (less jitter)
 * 
 * For robotic arms with multiple servos, MCPWM provides more stable operation.
 * 
 * ============================================================================
 * MCPWM ARCHITECTURE (ESP-IDF v5.x New Driver)
 * ============================================================================
 * 
 * The MCPWM peripheral has a hierarchical structure:
 * 
 *   ┌────────────────────────────────────────────────────────────────┐
 *   │                    MCPWM GROUP (Unit)                          │
 *   │   ESP32 has 2 groups: MCPWM0 and MCPWM1                        │
 *   │                                                                │
 *   │   ┌─────────────────────────────────────────────────────────┐  │
 *   │   │                     TIMER                               │  │
 *   │   │   - Sets the PWM period (frequency)                     │  │
 *   │   │   - Multiple operators can share one timer              │  │
 *   │   │   - We use 50Hz (20ms) for servo control                │  │
 *   │   │                                                         │  │
 *   │   │   ┌──────────────────────────────────────────────────┐  │  │
 *   │   │   │                  OPERATOR                        │  │  │
 *   │   │   │   - Contains comparator(s) and generator(s)      │  │  │
 *   │   │   │   - One operator per PWM output typically        │  │  │
 *   │   │   │                                                  │  │  │
 *   │   │   │   ┌────────────────────────────────────────────┐ │  │  │
 *   │   │   │   │              COMPARATOR                    │ │  │  │
 *   │   │   │   │   - Sets the compare value (pulse width)   │ │  │  │
 *   │   │   │   │   - Compare value = pulse width in µs      │ │  │  │
 *   │   │   │   │   - Range: 500 (0°) to 2500 (180°)         │ │  │  │
 *   │   │   │   └────────────────────────────────────────────┘ │  │  │
 *   │   │   │                                                  │  │  │
 *   │   │   │   ┌────────────────────────────────────────────┐ │  │  │
 *   │   │   │   │              GENERATOR                     │ │  │  │
 *   │   │   │   │   - Produces actual PWM signal on GPIO     │ │  │  │
 *   │   │   │   │   - HIGH when counter < compare value      │ │  │  │
 *   │   │   │   │   - LOW when counter >= compare value      │ │  │  │
 *   │   │   │   └────────────────────────────────────────────┘ │  │  │
 *   │   │   └──────────────────────────────────────────────────┘  │  │
 *   │   └─────────────────────────────────────────────────────────┘  │
 *   └────────────────────────────────────────────────────────────────┘
 * 
 * For our 4-servo arm:
 *   - We use 1 MCPWM group (MCPWM0)
 *   - 1 Timer (all servos share the same 50Hz frequency)
 *   - 4 Operators (one per servo)
 *   - 4 Comparators (one per servo, sets pulse width)
 *   - 4 Generators (one per servo, outputs PWM on GPIO)
 * 
 * This architecture provides excellent isolation between servos because
 * each servo has its own operator/comparator/generator chain.
 * 
 * @author Vinicius Alves - Projeto Spark
 * @date 2025
 */

/* ============================================================================
 * INCLUDES
 * ============================================================================ */

#include <stdio.h>              // Standard I/O functions (printf, etc.)
#include <string.h>             // String manipulation functions (strlen, strtok, etc.)
#include <stdlib.h>             // Standard library (atoi, abs, etc.)
#include "freertos/FreeRTOS.h"  // FreeRTOS kernel
#include "freertos/task.h"      // FreeRTOS task management
#include "driver/mcpwm_prelude.h"  // MCPWM driver (ESP-IDF v5.x new API)
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
 * MCPWM Configuration for Servos
 * 
 * MCPWM Timer Resolution and Period:
 * ----------------------------------
 * The timer resolution determines the precision of pulse width control.
 * 
 * We use 1MHz resolution (1µs per tick) because:
 *   - Servo pulse widths are specified in microseconds (500-2500µs)
 *   - 1MHz means compare_value = pulse_width_in_µs directly
 *   - No complex calculations needed
 * 
 * Timer period = 20000 ticks = 20ms = 50Hz (standard servo frequency)
 * 
 * Compare Value Calculation:
 * --------------------------
 * The comparator sets when the PWM output transitions from HIGH to LOW.
 * 
 *   Signal:  ──────┐            ┌─────────────────────────────
 *                  │            │
 *                  └────────────┘
 *            <---->              <--------------------------->
 *            Pulse               Rest of period
 *            Width               (LOW)
 *           (HIGH)
 * 
 *   Compare value = Pulse width in microseconds (at 1MHz resolution)
 * 
 * Examples:
 *   - 0° position:   500µs pulse  → compare value = 500
 *   - 90° position:  1500µs pulse → compare value = 1500
 *   - 180° position: 2500µs pulse → compare value = 2500
 * 
 * ---------------------------------------------------------------------------- */
#define SERVO_MCPWM_TIMER_RESOLUTION_HZ  1000000  // 1MHz = 1µs per tick
#define SERVO_MCPWM_TIMER_PERIOD_TICKS   20000    // 20ms period = 50Hz

/**
 * @brief Pulse width in microseconds for servo positions
 * 
 * MG90S Servo Specifications:
 *   - Operating frequency: 50Hz (20ms period)
 *   - Pulse width range: 500µs (0°) to 2500µs (180°)
 *   - Some servos use 1000µs to 2000µs for 0° to 180°
 * 
 * Adjust these values if your servos don't reach full range.
 */
#define SERVO_MIN_PULSEWIDTH_US  500     // Pulse width for 0° position
#define SERVO_MAX_PULSEWIDTH_US  2500    // Pulse width for 180° position

/* ----------------------------------------------------------------------------
 * GPIO Pin Assignments for Servos
 * 
 * Note: Avoid using these GPIO pins:
 *   - GPIO 0, 2, 12: Boot strapping pins (affect boot mode)
 *   - GPIO 6-11: Connected to internal flash memory
 *   - GPIO 34-39: Input only (no PWM output capability)
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
#define SMOOTH_MOVE_STEP_DEG    1   // Degrees per step in smooth movement

/* ============================================================================
 * DATA STRUCTURES
 * ============================================================================ */

/**
 * @brief Configuration structure for a single servo motor (MCPWM version)
 * 
 * Each servo needs:
 *   - An operator (container for comparator and generator)
 *   - A comparator (sets the pulse width via compare value)
 *   - A generator (outputs PWM signal to GPIO)
 * 
 * All servos share the same timer (they all run at 50Hz).
 * 
 * The MCPWM handles (mcpwm_oper_handle_t, mcpwm_cmpr_handle_t, mcpwm_gen_handle_t)
 * are opaque pointers to driver-managed resources. They must be stored and used
 * for all subsequent operations on that component.
 */
typedef struct {
    const char *name;           // Human-readable name for logging
    int gpio_pin;               // GPIO pin number connected to servo signal wire
    int min_angle;              // Minimum allowed angle (degrees) - SOFTWARE LIMIT
    int max_angle;              // Maximum allowed angle (degrees) - SOFTWARE LIMIT
    int current_angle;          // Current angle position (degrees)
    int initial_angle;          // Initial/home angle (degrees)
    
    // MCPWM component handles
    mcpwm_oper_handle_t oper;   // Operator handle (container for comparator/generator)
    mcpwm_cmpr_handle_t cmpr;   // Comparator handle (sets pulse width)
    mcpwm_gen_handle_t gen;     // Generator handle (outputs PWM signal)
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
 * @brief MCPWM Timer handle
 * 
 * All servos share a single timer running at 50Hz (20ms period).
 * This is efficient because all standard hobby servos use the same frequency.
 * The timer only sets the period - individual pulse widths are controlled
 * by each servo's comparator.
 */
static mcpwm_timer_handle_t servo_timer = NULL;

/**
 * @brief Array of servo configurations
 * 
 * IMPORTANT: Modify the min_angle, max_angle, and initial_angle values
 * after testing each servo to find the ideal limits for your robotic arm.
 * 
 * Current defaults:
 *   - All servos: 0° to 180° range
 *   - All servos start at 90° (center position)
 * 
 * Note: The MCPWM handles (oper, cmpr, gen) are initialized to NULL here
 * and will be populated during init_servo() function.
 */
static servo_config_t servos[NUM_SERVOS] = {
    // Servo 0: Base
    {
        .name = "Base",
        .gpio_pin = GPIO_SERVO_BASE,
        .min_angle = 0,         // TODO: Adjust after testing
        .max_angle = 180,       // TODO: Adjust after testing
        .current_angle = 90,
        .initial_angle = 90,    // TODO: Adjust to ideal home position
        .oper = NULL,
        .cmpr = NULL,
        .gen = NULL
    },
    // Servo 1: Shoulder
    {
        .name = "Shoulder",
        .gpio_pin = GPIO_SERVO_SHOULDER,
        .min_angle = 0,         // TODO: Adjust after testing
        .max_angle = 180,       // TODO: Adjust after testing
        .current_angle = 90,
        .initial_angle = 90,    // TODO: Adjust to ideal home position
        .oper = NULL,
        .cmpr = NULL,
        .gen = NULL
    },
    // Servo 2: Elbow
    {
        .name = "Elbow",
        .gpio_pin = GPIO_SERVO_ELBOW,
        .min_angle = 0,         // TODO: Adjust after testing
        .max_angle = 180,       // TODO: Adjust after testing
        .current_angle = 90,
        .initial_angle = 90,    // TODO: Adjust to ideal home position
        .oper = NULL,
        .cmpr = NULL,
        .gen = NULL
    },
    // Servo 3: Gripper
    {
        .name = "Gripper",
        .gpio_pin = GPIO_SERVO_GRIPPER,
        .min_angle = 53,         
        .max_angle = 170,       
        .current_angle = 90,
        .initial_angle = 90,    // TODO: Adjust to ideal home position
        .oper = NULL,
        .cmpr = NULL,
        .gen = NULL
    }
};

/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================ */

// Initialization functions
static void init_uart(void);
static void init_mcpwm_timer(void);
static void init_servo(int servo_index);
static void init_all_servos(void);

// Angle/PWM conversion functions
static uint32_t angle_to_compare(int angle);
static int constrain_angle(int servo_index, int angle);

// Servo control functions
static void set_servo_angle_instant(int servo_index, int angle);
static void set_servo_angle_smooth(int servo_index, int target_angle);
static void move_to_home_position(void);

// Serial communication functions
static void send_feedback(const char *message);
static void process_command(const char *command);
static void serial_receive_task(void *pvParameters);

/* ============================================================================
 * INITIALIZATION FUNCTIONS
 * ============================================================================ */

/**
 * @brief Initialize UART for serial communication
 * 
 * Configures UART0 with the specified baud rate for communication
 * with the Python application. UART0 is typically connected to the
 * USB-to-serial converter on most ESP32 development boards.
 */
static void init_uart(void)
{
    ESP_LOGI(TAG, "Initializing UART%d at %d baud...", UART_PORT_NUM, UART_BAUD_RATE);
    
    // UART configuration structure
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,            // Set baud rate (115200)
        .data_bits = UART_DATA_8_BITS,          // 8 data bits
        .parity    = UART_PARITY_DISABLE,       // No parity
        .stop_bits = UART_STOP_BITS_1,          // 1 stop bit
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  // No hardware flow control
        .source_clk = UART_SCLK_DEFAULT,        // Use default clock source
    };

    // Install UART driver with RX buffer, no TX buffer, no event queue
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    
    // Apply UART configuration
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    
    // Set UART pins (using default pins for UART0)
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, 
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "UART initialized successfully");
}

/**
 * @brief Initialize the MCPWM timer for servo control
 * 
 * Creates and configures a single MCPWM timer that will be shared by all servos.
 * The timer runs at 50Hz (20ms period) which is standard for hobby servos.
 * 
 * MCPWM Timer Configuration:
 * --------------------------
 * - Group ID: 0 (ESP32 has groups 0 and 1, we use group 0)
 * - Resolution: 1MHz (1 microsecond per tick)
 * - Period: 20000 ticks = 20ms = 50Hz
 * - Count mode: Up (counter counts from 0 to period, then resets)
 * 
 * Why share one timer?
 * --------------------
 * All standard hobby servos use 50Hz PWM. By sharing a timer:
 * - We save hardware resources (ESP32 has limited MCPWM timers)
 * - All servos stay perfectly synchronized
 * - Less configuration code
 * 
 * Each servo still has independent pulse width control via its own comparator.
 */
static void init_mcpwm_timer(void)
{
    ESP_LOGI(TAG, "Initializing MCPWM timer...");
    
    /**
     * MCPWM Timer Configuration Structure
     * 
     * group_id: Which MCPWM unit to use (0 or 1)
     * clk_src: Clock source (default uses APB clock)
     * resolution_hz: Timer tick frequency (1MHz = 1µs per tick)
     * period_ticks: Number of ticks per PWM period (20000 = 20ms)
     * count_mode: How the counter operates
     *   - MCPWM_TIMER_COUNT_MODE_UP: Count from 0 to period, then reset
     *   - MCPWM_TIMER_COUNT_MODE_DOWN: Count from period to 0
     *   - MCPWM_TIMER_COUNT_MODE_UP_DOWN: Count up then down (for center-aligned PWM)
     */
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,                              // Use MCPWM group 0
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,     // Use default clock source
        .resolution_hz = SERVO_MCPWM_TIMER_RESOLUTION_HZ,  // 1MHz resolution
        .period_ticks = SERVO_MCPWM_TIMER_PERIOD_TICKS,    // 20ms period
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,    // Count up mode
    };
    
    // Create the timer with the configuration
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &servo_timer));
    
    // Enable the timer (start counting)
    ESP_ERROR_CHECK(mcpwm_timer_enable(servo_timer));
    
    // Start the timer in continuous mode (keeps running forever)
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(servo_timer, MCPWM_TIMER_START_NO_STOP));
    
    ESP_LOGI(TAG, "MCPWM timer initialized: %d Hz, %d ticks period",
             SERVO_MCPWM_TIMER_RESOLUTION_HZ, SERVO_MCPWM_TIMER_PERIOD_TICKS);
}

/**
 * @brief Initialize a single servo motor using MCPWM
 * 
 * For each servo, we create:
 *   1. An OPERATOR - Container that manages comparators and generators
 *   2. A COMPARATOR - Sets the compare value (determines pulse width)
 *   3. A GENERATOR - Produces the actual PWM signal on the GPIO pin
 * 
 * MCPWM Component Hierarchy:
 * --------------------------
 *   Timer (shared) → Operator → Comparator → Generator → GPIO
 * 
 * The operator connects to the timer and contains the comparator and generator.
 * When the timer's counter reaches the comparator's value, the generator
 * changes the output state.
 * 
 * PWM Signal Generation:
 * ----------------------
 * We configure the generator to:
 *   - Go HIGH when timer counter = 0 (start of period)
 *   - Go LOW when timer counter = compare value (end of pulse)
 * 
 * This creates a PWM signal with:
 *   - Period = timer period (20ms)
 *   - Pulse width = compare value in microseconds
 * 
 * @param servo_index Index of the servo to initialize (0-3)
 */
static void init_servo(int servo_index)
{
    // Validate servo index
    if (servo_index < 0 || servo_index >= NUM_SERVOS) {
        ESP_LOGE(TAG, "Invalid servo index: %d", servo_index);
        return;
    }
    
    servo_config_t *servo = &servos[servo_index];
    ESP_LOGI(TAG, "Initializing servo %d (%s) on GPIO %d...", 
             servo_index, servo->name, servo->gpio_pin);
    
    /* ------------------------------------------------------------------------
     * Step 1: Create MCPWM Operator
     * ------------------------------------------------------------------------
     * The operator is the "middle manager" between the timer and the 
     * comparator/generator. It connects to the timer and contains
     * the comparator and generator.
     * 
     * group_id must match the timer's group_id.
     */
    mcpwm_operator_config_t oper_config = {
        .group_id = 0,  // Same group as the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &servo->oper));
    
    // Connect the operator to the timer
    // This is essential - without this, the operator won't receive timing signals
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(servo->oper, servo_timer));
    
    /* ------------------------------------------------------------------------
     * Step 2: Create MCPWM Comparator
     * ------------------------------------------------------------------------
     * The comparator holds the "compare value" which determines when the
     * PWM output transitions from HIGH to LOW.
     * 
     * When timer_counter == compare_value, an event is triggered that
     * the generator can respond to.
     * 
     * For servos:
     *   - compare_value = pulse width in microseconds (at 1MHz resolution)
     *   - Range: 500 (0°) to 2500 (180°)
     * 
     * update_cmp_on_tez: Update compare value on Timer Equal Zero event
     *   - This means changes take effect at the start of the next PWM period
     *   - Prevents glitches in the middle of a pulse
     */
    mcpwm_comparator_config_t cmpr_config = {
        .flags.update_cmp_on_tez = true,  // Update on Timer Equal Zero
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(servo->oper, &cmpr_config, &servo->cmpr));
    
    // Set initial compare value for 90° position (center)
    uint32_t initial_compare = angle_to_compare(servo->initial_angle);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(servo->cmpr, initial_compare));
    
    /* ------------------------------------------------------------------------
     * Step 3: Create MCPWM Generator
     * ------------------------------------------------------------------------
     * The generator is what actually produces the PWM signal on the GPIO pin.
     * It responds to events (timer events, comparator events) by changing
     * the output state (HIGH or LOW).
     * 
     * gen_gpio_num: Which GPIO pin to output the PWM signal on
     */
    mcpwm_generator_config_t gen_config = {
        .gen_gpio_num = servo->gpio_pin,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(servo->oper, &gen_config, &servo->gen));
    
    /* ------------------------------------------------------------------------
     * Step 4: Configure Generator Actions
     * ------------------------------------------------------------------------
     * This is where we define how the PWM signal behaves.
     * 
     * We set two actions:
     *   1. On Timer Empty Event (counter = 0): Set output HIGH
     *   2. On Comparator Match Event: Set output LOW
     * 
     * This creates a PWM signal that:
     *   - Starts HIGH at the beginning of each period
     *   - Goes LOW when the counter reaches the compare value
     *   - Stays LOW until the next period starts
     * 
     * Visual representation:
     * 
     *   Output: ──────────┐                    ┌──────────┐
     *                     │                    │          │
     *                     └────────────────────┘          └───...
     *           ^         ^                    ^
     *           │         │                    │
     *           │         Compare Event        Timer Empty Event
     *           │         (go LOW)             (go HIGH)
     *           │
     *           Timer Empty Event
     *           (go HIGH)
     *           
     *           <-------->                     <---------->
     *           Pulse Width                    Pulse Width
     *           = Compare Value                = Compare Value
     */
    
    // Action 1: Set output HIGH on timer empty event (start of period)
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        servo->gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,      // When counting up
            MCPWM_TIMER_EVENT_EMPTY,       // On timer = 0 (empty/zero)
            MCPWM_GEN_ACTION_HIGH          // Set output HIGH
        )
    ));
    
    // Action 2: Set output LOW on comparator match event
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        servo->gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,      // When counting up
            servo->cmpr,                    // Use this servo's comparator
            MCPWM_GEN_ACTION_LOW           // Set output LOW
        )
    ));
    
    ESP_LOGI(TAG, "Servo %d (%s) initialized: GPIO=%d, initial_angle=%d°", 
             servo_index, servo->name, servo->gpio_pin, servo->initial_angle);
}

/**
 * @brief Initialize all servo motors
 * 
 * Calls init_servo() for each servo in the array.
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
 * @brief Convert angle (degrees) to MCPWM compare value
 * 
 * MCPWM uses compare values instead of duty cycles. At 1MHz timer resolution,
 * the compare value equals the pulse width in microseconds directly.
 * 
 * Linear interpolation formula:
 *   compare_value = MIN_PULSE + (angle / MAX_ANGLE) * (MAX_PULSE - MIN_PULSE)
 * 
 * Examples:
 *   - 0°:   500 + (0/180) * 2000 = 500µs
 *   - 90°:  500 + (90/180) * 2000 = 1500µs
 *   - 180°: 500 + (180/180) * 2000 = 2500µs
 * 
 * Why this is simpler than LEDC:
 * ------------------------------
 * With LEDC, we had to calculate duty cycle as a fraction of the resolution
 * (e.g., 14-bit = 16384 steps). With MCPWM at 1MHz resolution, the math
 * is straightforward: compare value = pulse width in microseconds.
 * 
 * @param angle Angle in degrees (0-180)
 * @return Compare value for the MCPWM comparator
 */
static uint32_t angle_to_compare(int angle)
{
    // Clamp angle to valid range
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    // Linear interpolation from angle to pulse width
    // pulse_width = MIN_PULSE + (angle / 180) * (MAX_PULSE - MIN_PULSE)
    uint32_t compare_value = SERVO_MIN_PULSEWIDTH_US + 
        (angle * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US)) / 180;
    
    return compare_value;
}

/**
 * @brief Constrain angle to servo's allowed limits
 * 
 * Each servo may have different mechanical limits. This function
 * ensures the requested angle is within the servo's safe operating range.
 * 
 * @param servo_index Index of the servo (0-3)
 * @param angle Requested angle in degrees
 * @return Constrained angle within the servo's min/max limits
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
 * Updates the MCPWM comparator value to move the servo to the target angle
 * as fast as the servo motor can move.
 * 
 * MCPWM Compare Value Update:
 * ---------------------------
 * The mcpwm_comparator_set_compare_value() function updates the compare
 * value immediately in the hardware. However, because we configured
 * update_cmp_on_tez = true, the new value takes effect at the start
 * of the next PWM period (when timer counter reaches zero).
 * 
 * This prevents glitches that could occur if we changed the compare
 * value in the middle of a pulse.
 * 
 * Skip-if-unchanged optimization:
 * -------------------------------
 * If the requested angle equals the current angle, we skip the update.
 * This prevents unnecessary hardware writes and reduces any potential
 * for timing-related jitter.
 * 
 * @param servo_index Index of the servo (0-3)
 * @param angle Target angle in degrees
 */
static void set_servo_angle_instant(int servo_index, int angle)
{
    // Validate servo index
    if (servo_index < 0 || servo_index >= NUM_SERVOS) {
        ESP_LOGE(TAG, "Invalid servo index: %d", servo_index);
        return;
    }
    
    servo_config_t *servo = &servos[servo_index];
    
    // Constrain angle to allowed limits
    angle = constrain_angle(servo_index, angle);
    
    // IMPORTANT: Skip update if angle hasn't changed
    // This prevents unnecessary hardware writes and potential jitter
    if (angle == servo->current_angle) {
        return;
    }
    
    // Convert angle to MCPWM compare value (pulse width in µs)
    uint32_t compare_value = angle_to_compare(angle);
    
    // Update the comparator value
    // This is atomic - no need for semaphores or disabling interrupts
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(servo->cmpr, compare_value));
    
    // Update current angle in our tracking structure
    servo->current_angle = angle;
    
    ESP_LOGD(TAG, "%s: Set to %d° (compare: %lu µs)", servo->name, angle, compare_value);
}

/**
 * @brief Set servo to a specific angle with smooth movement
 * 
 * Gradually moves the servo from its current position to the target angle
 * in small increments. This prevents sudden movements that could:
 *   - Damage the robotic arm mechanism
 *   - Drop objects being held
 *   - Cause excessive current draw (power supply stress)
 * 
 * Movement Algorithm:
 * -------------------
 * 1. Calculate step direction (positive or negative)
 * 2. Move one step at a time (SMOOTH_MOVE_STEP_DEG degrees)
 * 3. Wait SMOOTH_MOVE_DELAY_MS milliseconds between steps
 * 4. Repeat until target is reached
 * 
 * Note: Movement speed = SMOOTH_MOVE_STEP_DEG / SMOOTH_MOVE_DELAY_MS degrees per ms
 * Default: 1° per 15ms = 66.7° per second
 * 
 * @param servo_index Index of the servo (0-3)
 * @param target_angle Target angle in degrees
 */
static void set_servo_angle_smooth(int servo_index, int target_angle)
{
    // Validate servo index
    if (servo_index < 0 || servo_index >= NUM_SERVOS) {
        ESP_LOGE(TAG, "Invalid servo index: %d", servo_index);
        return;
    }
    
    servo_config_t *servo = &servos[servo_index];
    
    // Constrain target angle to allowed limits
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
        // Calculate next position
        if (abs(target_angle - current) <= SMOOTH_MOVE_STEP_DEG) {
            current = target_angle;  // Final step - go directly to target
        } else {
            current += step;
        }
        
        // Set servo to new position
        uint32_t compare_value = angle_to_compare(current);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(servo->cmpr, compare_value));
        servo->current_angle = current;
        
        // Wait before next step
        vTaskDelay(pdMS_TO_TICKS(SMOOTH_MOVE_DELAY_MS));
    }
    
    ESP_LOGD(TAG, "%s: Smooth move complete, now at %d°", servo->name, target_angle);
}

/**
 * @brief Move all servos to their home (initial) positions
 * 
 * Uses instant movement to move all servos to their configured initial angles.
 * A small delay between servos reduces power surge from multiple motors
 * starting simultaneously.
 */
static void move_to_home_position(void)
{
    ESP_LOGI(TAG, "Moving all servos to home positions...");
    
    for (int i = 0; i < NUM_SERVOS; i++) {
        set_servo_angle_instant(i, servos[i].initial_angle);
        // Small delay between servos to reduce power surge
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
 * Sends a message back to the Python application to confirm
 * command execution or report errors.
 * 
 * @param message Message string to send
 */
static void send_feedback(const char *message)
{
    uart_write_bytes(UART_PORT_NUM, message, strlen(message));
    uart_write_bytes(UART_PORT_NUM, "\r\n", 2);  // Add line ending
}

/**
 * @brief Process a received command string
 * 
 * Parses the command and executes the appropriate action.
 * 
 * Command formats:
 *   - "I:base,shoulder,elbow,gripper" - Instant movement
 *   - "S:base,shoulder,elbow,gripper" - Smooth movement
 *   - "H" - Move to home position
 *   - "?" - Query current positions
 * 
 * Example commands:
 *   - "I:90,90,90,90"  - Move all servos to 90° instantly
 *   - "S:45,120,60,30" - Move all servos smoothly to specified angles
 *   - "H"              - Move all servos to home position
 *   - "?"              - Query and return current positions
 * 
 * Response formats:
 *   - "OK:a,b,c,d"     - Success, returns current positions
 *   - "OK:HOME,a,b,c,d" - Home command success
 *   - "POS:a,b,c,d"    - Query response
 *   - "ERR:message"    - Error with description
 * 
 * @param command Null-terminated command string
 */
static void process_command(const char *command)
{
    char response[128];
    
    // Skip empty commands
    if (strlen(command) < 1) {
        return;
    }
    
    ESP_LOGI(TAG, "Received command: %s", command);
    
    // Check command type
    if (command[0] == 'H' || command[0] == 'h') {
        // Home command - move all servos to initial positions
        move_to_home_position();
        snprintf(response, sizeof(response), "OK:HOME,%d,%d,%d,%d",
                 servos[0].current_angle, servos[1].current_angle,
                 servos[2].current_angle, servos[3].current_angle);
        send_feedback(response);
        return;
    }
    
    if (command[0] == '?') {
        // Query command - return current positions
        snprintf(response, sizeof(response), "POS:%d,%d,%d,%d",
                 servos[0].current_angle, servos[1].current_angle,
                 servos[2].current_angle, servos[3].current_angle);
        send_feedback(response);
        return;
    }
    
    // Check for movement commands (I: or S:)
    if (strlen(command) < 3 || command[1] != ':') {
        send_feedback("ERR:Invalid format. Use I:a,b,c,d or S:a,b,c,d");
        return;
    }
    
    // Determine movement type
    move_type_t move_type;
    if (command[0] == 'I' || command[0] == 'i') {
        move_type = MOVE_INSTANT;
    } else if (command[0] == 'S' || command[0] == 's') {
        move_type = MOVE_SMOOTH;
    } else {
        send_feedback("ERR:Unknown command. Use I (instant) or S (smooth)");
        return;
    }
    
    // Parse the angle values
    // Create a copy of the command for tokenization (strtok modifies the string)
    char cmd_copy[64];
    strncpy(cmd_copy, command + 2, sizeof(cmd_copy) - 1);  // Skip "X:" prefix
    cmd_copy[sizeof(cmd_copy) - 1] = '\0';
    
    int angles[NUM_SERVOS];
    int parsed_count = 0;
    
    // Tokenize by comma
    char *token = strtok(cmd_copy, ",");
    while (token != NULL && parsed_count < NUM_SERVOS) {
        angles[parsed_count] = atoi(token);
        parsed_count++;
        token = strtok(NULL, ",");
    }
    
    // Verify we got all 4 angles
    if (parsed_count != NUM_SERVOS) {
        snprintf(response, sizeof(response), "ERR:Expected %d angles, got %d", 
                 NUM_SERVOS, parsed_count);
        send_feedback(response);
        return;
    }
    
    // Execute the movement
    if (move_type == MOVE_INSTANT) {
        // Instant movement - set all servos immediately
        for (int i = 0; i < NUM_SERVOS; i++) {
            set_servo_angle_instant(i, angles[i]);
        }
    } else {
        // Smooth movement - move servos sequentially
        // Note: For truly simultaneous smooth movement, we would need
        // separate tasks or a more complex algorithm
        for (int i = 0; i < NUM_SERVOS; i++) {
            set_servo_angle_smooth(i, angles[i]);
        }
    }
    
    // Send confirmation with actual positions (after constraining)
    snprintf(response, sizeof(response), "OK:%d,%d,%d,%d",
             servos[0].current_angle, servos[1].current_angle,
             servos[2].current_angle, servos[3].current_angle);
    send_feedback(response);
}

/**
 * @brief FreeRTOS task for receiving serial commands
 * 
 * This task continuously monitors the UART for incoming data,
 * assembles complete commands (terminated by newline), and
 * processes them.
 * 
 * The task runs in an infinite loop and uses non-blocking reads
 * with a timeout to allow other tasks to run.
 * 
 * @param pvParameters Task parameters (not used)
 */
static void serial_receive_task(void *pvParameters)
{
    uint8_t data[UART_BUF_SIZE];
    char command_buffer[UART_BUF_SIZE];
    int cmd_index = 0;
    
    ESP_LOGI(TAG, "Serial receive task started");
    send_feedback("READY:Robotic Arm Controller v2.0 (MCPWM)");
    send_feedback("HELP:Commands: I:a,b,c,d (instant), S:a,b,c,d (smooth), H (home), ? (query)");
    
    while (1) {
        // Read data from UART with a timeout
        int len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE - 1, 
                                  pdMS_TO_TICKS(100));
        
        if (len > 0) {
            // Process each received byte
            for (int i = 0; i < len; i++) {
                char c = (char)data[i];
                
                // Check for end of command (newline or carriage return)
                if (c == '\n' || c == '\r') {
                    if (cmd_index > 0) {
                        // Null-terminate and process the command
                        command_buffer[cmd_index] = '\0';
                        process_command(command_buffer);
                        cmd_index = 0;  // Reset buffer
                    }
                } else {
                    // Add character to buffer if there's space
                    if (cmd_index < UART_BUF_SIZE - 1) {
                        command_buffer[cmd_index++] = c;
                    } else {
                        // Buffer overflow - reset
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
 * 
 * This function is called by the ESP-IDF framework after the system
 * initialization is complete. It initializes all peripherals and
 * creates the serial receive task.
 * 
 * Initialization sequence:
 *   1. UART - For serial communication with host
 *   2. MCPWM Timer - Shared by all servos (50Hz)
 *   3. Servos - Each with operator, comparator, and generator
 *   4. Home position - Move all servos to initial angles
 *   5. Serial task - Start listening for commands
 */
void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "   ESP32 Robotic Arm Controller");
    ESP_LOGI(TAG, "   Using MCPWM for Servo Control");
    ESP_LOGI(TAG, "========================================");
    
    // Step 1: Initialize UART for serial communication
    ESP_LOGI(TAG, "Step 1: Initializing UART...");
    init_uart();
    
    // Step 2: Initialize MCPWM timer (shared by all servos)
    ESP_LOGI(TAG, "Step 2: Initializing MCPWM timer...");
    init_mcpwm_timer();
    
    // Step 3: Initialize all servo motors (operator, comparator, generator each)
    ESP_LOGI(TAG, "Step 3: Initializing servos...");
    init_all_servos();
    
    // Step 4: Move servos to home position
    ESP_LOGI(TAG, "Step 4: Moving to home position...");
    move_to_home_position();
    
    // Step 5: Create the serial receive task
    ESP_LOGI(TAG, "Step 5: Starting serial receive task...");
    xTaskCreate(serial_receive_task,    // Task function
                "serial_rx_task",       // Task name (for debugging)
                4096,                   // Stack size (bytes)
                NULL,                   // Task parameters
                5,                      // Task priority
                NULL);                  // Task handle (not needed)
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "   Initialization complete!");
    ESP_LOGI(TAG, "   Waiting for serial commands...");
    ESP_LOGI(TAG, "========================================");
    
    // Main task can now idle or be used for other purposes
    // The serial_receive_task handles all the command processing
}
