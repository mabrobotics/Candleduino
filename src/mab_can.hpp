#ifndef MAB_CAN_H
#define MAB_CAN_H

/** @brief Minimum valid CAN device ID. */
#define CAN_MIN_ID ((uint32_t)10)
/** @brief Maximum valid CAN device ID. */
#define CAN_MAX_ID ((uint32_t)2000)

/** @brief CAN frame type for writing a MD device register and getting default response. */
#define FRAME_WRITE_REGISTER_DEFAULT_RESPONSE 0x40
/** @brief CANFD frame type for reading from a MD device register. */
#define FRAME_READ_REGISTER_FD 0x41
/** @brief CANFD frame type for writing to a MD device register. */
#define FRAME_WRITE_REGISTER_FD 0x42
/** @brief CAN frame type for reading from a MD device register. */
#define FRAME_READ_REGISTER 0x43
/** @brief CAN frame type for writing to a MD device register. */
#define FRAME_WRITE_REGISTER 0x44
/** @brief Default response frame ID for MAB CAN protocol. */
#define RESPONSE_DEFAULT 0xA0

/** @brief Blink register ID*/
#define REG_BLINK 0x08B
/** @brief Register for setting the state of the device. */
#define REG_CONTROL_WORD 0x142
/** @brief Control word for enabling the device. */
#define CONTROL_WORD_ENABLE 0x27
/** @brief Control word for disabling the device. */
#define CONTROL_WORD_DISABLE 0x40

/** @brief Reset register ID*/
#define REG_RESET 0x088
/** @brief Clear errors register ID*/
#define REG_CLEAR_ERRORS 0x08A
/** @brief RunSave register ID*/
#define REG_SAVE 0x080
/** @brief RunZero register ID*/
#define REG_ZERO 0x08C
/** @brief Motion mode command register ID. */
#define REG_MOTION_MODE_COMMAND 0x140
/** @brief Motion mode status register ID. */
#define REG_MOTION_MODE_STATUS 0x141

/** @brief Quick status register (used for high-level health/status info). */
#define REG_QUICK_STATUS 0x805
/** @brief [float] DC bus voltage monitoring register ID. */
#define REG_DC_VOLTAGE 0x811
/** @brief [float] Mosfet temperature register ID*/
#define REG_MOSFET_TEMPERATURE 0x806
/** @brief [float] Motor temperature register ID*/
#define REG_MOTOR_TEMPERATURE 0x807

/** @brief Current limit register ID*/
#define REG_CURRENT_LIMIT 0x016
/** @brief Torque bandwidth register ID*/
#define REG_TORQUE_BANDWIDTH 0x018

/** @brief [float32] PID position kp register ID*/
#define REG_POSITION_KP 0x030
/** @brief [float32] PID position ki register ID*/
#define REG_POSITION_KI 0x031
/** @brief [float32] PID position kd register ID*/
#define REG_POSITION_KD 0x032
/** @brief [float32] PID position windup register ID*/
#define REG_POSITION_WINDUP 0x034

/** @brief [float] PID velocity kp register ID*/
#define REG_VELOCITY_KP 0x040
/** @brief [float] PID velocity ki register ID*/
#define REG_VELOCITY_KI 0x041
/** @brief [float] PID velocity kd register ID*/
#define REG_VELOCITY_KD 0x042
/** @brief [float] PID velocity windup register ID*/
#define REG_VELOCITY_WINDUP 0x044

/** @brief Impedance mode kp register ID*/
#define REG_IMPEDANCE_KP 0x050
/** @brief Impedance mode kd register ID*/
#define REG_IMPEDANCE_KD 0x051

/** @brief [float] Target velocity register ID. */
#define REG_TARGET_POS 0x150
/** @brief [float] Target velocity register ID. */
#define REG_TARGET_VEL 0x151
/** @brief [float] Target velocity register ID. */
#define REG_TARGET_TORQUE 0x152

/** @brief [float] Main encoder velocity register ID. */
#define REG_MAIN_ENCODER_VEL 0x062
/** @brief [float] Main encoder position register ID. */
#define REG_MAIN_ENCODER_POS 0x063
/** @brief [float] Output encoder velocity register ID. */
#define REG_OUTPUT_ENCODER_VEL 0x023
/** @brief [float] Output encoder position register ID. */
#define REG_OUTPUT_ENCODER_POS 0x024

/** @brief CAN ID register ID*/
#define REG_CAN_ID 0x001
/** @brief CANFD BAUDRATE register ID*/
#define REG_FD_CAN_BAUDRATE 0x002
/** @brief Motor pole pairs register ID. */
#define REG_MOTOR_POLE_PAIRS 0x011

/**
 * @brief Enumeration of possible error codes returned by MAB CAN functions.
 *
 * Functions in this library return values of this enum to indicate the result of an operation.
 * `OK` indicates success, while other values indicate the type of error encountered.
 */
typedef enum
{
    /** Unknown or unspecified error occurred */
    UNKNOWN_ERROR,
    /** Operation completed successfully */
    OK,
    /** Data type provided is not supported or invalid */
    ERROR_INVALID_TYPE,
    /** A required pointer argument was NULL */
    ERROR_NULL_PTR,
    /** Provided buffer is too small to hold data */
    ERROR_BUF_SIZE,
    /** Frame format or content is invalid */
    ERROR_INVALID_FRAME,
    /** Specified register ID is invalid or unsupported */
    ERROR_INVALID_REG,
    /** Operation failed during execution (e.g. mismatch, timeout) */
    ERROR_OPERATION_FAILED
} errorMab_E;

/**
 * @brief Enumeration of supported motion control modes.
 *
 * These modes define how the motor controller interprets and responds to control commands, such
 * as position, velocity, torque, or impedance targets.
 */
typedef enum
{
    /** Idle mode, no control output */
    IDLE = 0,
    /** Position PID control */
    POSITION_PID = 1,
    /** Velocity PID control */
    VELOCITY_PID = 2,
    /** Direct raw torque control */
    RAW_TORQUE = 3,
    /** Impedance control, similar to spring-damper system */
    IMPEDANCE = 4,
    /** Position PID control with a trapezoidal profile (constant acceleration) */
    POSITION_PROFILE = 7,
    /** Velocity PID control with a trapezoidal profile (constant acceleration) */
    VELOCITY_PROFILE = 8
} motionModeMab_E;

#endif // MAB_CAN_H
