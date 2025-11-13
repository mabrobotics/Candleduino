#ifndef MAB_CAN_H
#define MAB_CAN_H

#include <cstdarg>

#define BLINK 0x08B

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

/** @brief Minimum valid CAN device ID. */
#define CAN_MIN_ID ((uint32_t)10)
/** @brief Maximum valid CAN device ID. */
#define CAN_MAX_ID ((uint32_t)2000)

/** @brief Output encoder velocity register ID. */
#define REG_OUTPUT_ENCODER_VEL 0x023
/** @brief Output encoder position register ID. */
#define REG_OUTPUT_ENCODER_POS 0x024

/** @brief Main encoder velocity register ID. */
#define REG_MAIN_ENCODER_VEL 0x062
/** @brief Main encoder position register ID. */
#define REG_MAIN_ENCODER_POS 0x063
/** @brief Motor torque measurement register ID. */
#define REG_MOTOR_TORQUE 0x064
/** @brief Motor pole pairs register ID. */
#define REG_MOTOR_POLE_PAIRS 0x011

/** @brief Velocity setpoint for motion profile. */
#define REG_PROFILE_VEL 0x120
/** @brief Acceleration setpoint for motion profile. */
#define REG_PROFILE_ACC 0x121
/** @brief Deceleration setpoint for motion profile. */
#define REG_PROFILE_DEC 0x122

/** @brief Motion mode command register ID. */
#define REG_MOTION_MODE_COMMAND 0x140
/** @brief Motion mode status register ID. */
#define REG_MOTION_MODE_STATUS 0x141
/** @brief Register for setting the state of the device. */
#define REG_CONTROL_WORD 0x142

/** @brief Target velocity register ID. */
#define REG_TARGET_VEL 0x151

/** @brief Quick status register (used for high-level health/status info). */
#define REG_QUICK_STATUS 0x805
/** @brief DC bus voltage monitoring register ID. */
#define REG_DC_VOLTAGE 0x811

/** @brief Control word for enabling the device. */
#define CONTROL_WORD_ENABLE 0x27
/** @brief Control word for disabling the device. */
#define CONTROL_WORD_DISABLE 0x40
/** @brief Control word for shutting down the device. */
#define CONTROL_WORD_SHUTDOWN 0x06

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

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

/**
 * @brief Prepares a data buffer for a MAB CAN read register frame.
 *
 * This function serializes the given register ID and data into the provided buffer so that it
 * can be transmitted over the CAN bus to read a value to a device register.
 *
 * @param buffer      Pointer to the buffer where the frame data will be read.
 * @param bufferSize  Size of the buffer in bytes.
 * @param registerId  ID of the register to write to.
 * @param data        Pointer to the data to read into the register.
 * @return            Error code indicating success or failure.
 */
template <typename T>
errorMab_E parseResponse(const void *buffer,
                         const size_t bufferSize,
                         const uint8_t frameId,
                         const uint16_t registerId,
                         T *data)
{
    if (buffer == NULL || data == NULL)
        return ERROR_NULL_PTR;

    uint8_t *buf = (uint8_t *)buffer;

    if ((uint8_t)buf[0] != frameId)
        return ERROR_INVALID_FRAME;

    if (((uint16_t)buf[2] | ((uint16_t)buf[3] << 8)) != registerId)
        return ERROR_INVALID_REG;

    if (bufferSize < 2 + sizeof(uint16_t) + sizeof(T))
        return ERROR_BUF_SIZE;
    memcpy((T *)data, buf + 2 + sizeof(uint16_t), sizeof(T));

    return OK;
}

/**
 * @brief Prepares a data buffer for a MAB CAN write register frame.
 *
 * This function serializes the given register ID and data into the provided buffer so that it
 * can be transmitted over the CAN bus to write a value to a device register.
 *
 * @param buffer      Pointer to the buffer where the frame data will be written.
 * @param bufferSize  Size of the buffer in bytes.
 * @param registerId  ID of the register to write to.
 * @param data        Pointer to the data to write into the register.
 * @param useDefaultResponse If true, the function will use the
 * FRAME_WRITE_REGISTER_DEFAULT_RESPONSE type for writing a MD device register and getting
 * default response.
 * @return            Error code indicating success or failure.
 */
template <typename T>
errorMab_E writeRegisterPrepareDataFrame(void *buffer,
                                         const size_t bufferSize,
                                         const uint16_t registerId,
                                         const T *data,
                                         const bool defaultResponse)
{
    if (buffer == NULL || data == NULL)
        return ERROR_NULL_PTR;

    memset(buffer, 0, bufferSize);

    uint8_t *buf = (uint8_t *)buffer;

    if (defaultResponse)
        buf[0] = FRAME_WRITE_REGISTER_DEFAULT_RESPONSE;
    else
        buf[0] = FRAME_WRITE_REGISTER;
    buf[1] = 0x00;

    memcpy(buf + 2, &registerId, sizeof(uint16_t));

    if (bufferSize < 2 + sizeof(uint16_t) + sizeof(T))
        return ERROR_BUF_SIZE;
    memcpy(buf + 2 + sizeof(uint16_t), (T *)data, sizeof(T));

    return OK;
}

template <typename... T>
errorMab_E writeRegisterPrepareDataFrame(void *buffer,
                                         T *...message)
{

    // if (buffer == NULL || data == NULL)
    //     return ERROR_NULL_PTR;

    // memset(buffer, 0, bufferSize);

    // uint8_t *buf = (uint8_t *)buffer;

    // if (defaultResponse)
    //     buf[0] = FRAME_WRITE_REGISTER_DEFAULT_RESPONSE;
    // else
    //     buf[0] = FRAME_WRITE_REGISTER;
    // buf[1] = 0x00;
    return 0;
}

template <typename T>
errorMab_E readRegisterPrepareDataFrame(void *buffer,
                                        const size_t bufferSize,
                                        const uint16_t registerId)
{
    if (buffer == NULL)
        return ERROR_NULL_PTR;

    memset(buffer, 0, bufferSize);

    uint8_t *buf = (uint8_t *)buffer;

    buf[0] = FRAME_READ_REGISTER;
    buf[1] = 0x00;

    memcpy(buf + 2, &registerId, sizeof(uint16_t));

    if (bufferSize < 2 + sizeof(uint16_t) + sizeof(T))
        return ERROR_BUF_SIZE;
    buf[2 + sizeof(uint16_t)] = (T)0;

    return OK;
}

#endif // MAB_CAN_H
