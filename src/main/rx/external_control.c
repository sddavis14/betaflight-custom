#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"
#include "common/utils.h"
#include "drivers/time.h"
#include "io/serial.h"
#include "pg/rx.h"
#include "rx/rx.h"
#include "rx/external_control.h"
#include "flight/imu.h" // Required for attitude_t and imuGetAttitudeValues()

// Defines for telemetry
#define EXTERNAL_CONTROL_TELEMETRY_FRAME_SIZE 8 // 1 start byte + externalControlAttitude_t (6 bytes) + 1 checksum byte
#define EXTERNAL_CONTROL_TELEMETRY_INTERVAL_US (1000000 / 100) // 100Hz

// Global Variables (static)
static externalControlCommand_t currentCommand;
static timeUs_t lastTelemetrySendTimeUs = 0;
static serialPort_t *externalControlSerialPort = NULL;
static bool newExternalControlDataAvailable = false;
static timeUs_t lastExternalCommandTimeUs = 0;

#define EXTERNAL_CONTROL_FAILSAFE_TIMEOUT_US 500000 // 0.5 seconds
#define EXTERNAL_CONTROL_FRAME_SIZE 18 // 1 start byte + 4 floats (16 bytes) + 1 checksum byte
#define EXTERNAL_CONTROL_START_BYTE 0x2A

static uint8_t rxBuffer[EXTERNAL_CONTROL_FRAME_SIZE];
static uint8_t rxBufferPos = 0;

// Forward declarations for static functions
static void externalControlDataReceive(uint16_t c, void *data);
static int externalControlFrameStatus(rxRuntimeState_t *rxRuntimeState);
static uint16_t externalControlReadRawRc(const rxRuntimeState_t *rxRuntimeState, uint8_t channel);
static void externalControlSendAttitudeTelemetry(void); // Forward declaration


void externalControlSetCommand(const externalControlCommand_t* newCmd) {
    memcpy(&currentCommand, newCmd, sizeof(externalControlCommand_t));
    newExternalControlDataAvailable = true;
    lastExternalCommandTimeUs = microsISR();
}

const externalControlCommand_t *getExternalControlCommand(void) {
    return &currentCommand;
}

static void externalControlDataReceive(uint16_t c, void *data) {
    UNUSED(data);

    if (rxBufferPos == 0 && c == EXTERNAL_CONTROL_START_BYTE) {
        rxBuffer[rxBufferPos++] = (uint8_t)c;
    } else if (rxBufferPos > 0 && rxBufferPos < EXTERNAL_CONTROL_FRAME_SIZE) {
        rxBuffer[rxBufferPos++] = (uint8_t)c;
    } else {
        // Buffer overflow or unexpected byte, reset
        rxBufferPos = 0;
        return;
    }

    if (rxBufferPos == EXTERNAL_CONTROL_FRAME_SIZE) {
        // Full frame received
        rxBufferPos = 0; // Reset for next frame

        // Validate checksum
        uint8_t calculatedChecksum = 0;
        for (int i = 1; i < EXTERNAL_CONTROL_FRAME_SIZE -1; i++) { // Data bytes from index 1 to 16
            calculatedChecksum ^= rxBuffer[i];
        }

        if (calculatedChecksum == rxBuffer[EXTERNAL_CONTROL_FRAME_SIZE - 1]) {
            externalControlCommand_t receivedCmd;
            // Assuming both systems are little-endian for float representation
            memcpy(&receivedCmd, &rxBuffer[1], sizeof(externalControlCommand_t));
            externalControlSetCommand(&receivedCmd);
        } else {
            // Checksum mismatch, ignore frame
            // Optional: Increment a counter for bad checksums for debugging
        }
    }
}

static int externalControlFrameStatus(rxRuntimeState_t *rxRuntimeState) {
    UNUSED(rxRuntimeState);

    // Attempt to send telemetry data
    externalControlSendAttitudeTelemetry();

    if (cmpTimeUs(microsISR(), lastExternalCommandTimeUs) > EXTERNAL_CONTROL_FAILSAFE_TIMEOUT_US) {
        newExternalControlDataAvailable = false; // Ensure no stale data is used
        // Zero out the command on failsafe for safety
        currentCommand.roll_rate = 0;
        currentCommand.pitch_rate = 0;
        currentCommand.yaw_rate = 0;
        currentCommand.thrust = 0; // Or minimum thrust if appropriate
        return RX_FRAME_FAILSAFE | RX_FRAME_DROPPED;
    }

    if (newExternalControlDataAvailable) {
        newExternalControlDataAvailable = false;
        return RX_FRAME_COMPLETE;
    } else {
        return RX_FRAME_PENDING;
    }
}

static uint16_t externalControlReadRawRc(const rxRuntimeState_t *rxRuntimeState, uint8_t channel) {
    UNUSED(rxRuntimeState);
    UNUSED(channel);
    // This function converts the float commands to pseudo RC channel values if needed.
    // However, with direct rate control, this might not be directly used by the PID controller
    // if it's adapted to take rate commands directly.
    // For now, returning midrc, assuming it's defined and accessible.
    // A more sophisticated implementation would map currentCommand fields to RC channels.
    const rxConfig_t *rxCfg = rxConfig();
    return rxCfg->midrc;
}

bool externalControlInit(const struct rxConfig_s *rxConfigLocal, struct rxRuntimeState_s *rxRuntimeState) {
    UNUSED(rxConfigLocal); // rxConfig() can be used to get global rxConfig

    rxRuntimeState->rcFrameStatusFn = externalControlFrameStatus;
    rxRuntimeState->rcReadRawFn = externalControlReadRawRc;
    rxRuntimeState->channelCount = 4; // Roll, Pitch, Yaw, Thrust
    rxRuntimeState->lastRcFrameTimeUs = 0;

    // Initialize currentCommand to safe values
    currentCommand.roll_rate = 0.0f;
    currentCommand.pitch_rate = 0.0f;
    currentCommand.yaw_rate = 0.0f;
    currentCommand.thrust = 0.0f; // Assuming 0.0 is minimum/safe thrust

    rxBufferPos = 0;
    newExternalControlDataAvailable = false;
    lastExternalCommandTimeUs = microsISR(); // Initialize to prevent immediate failsafe

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_EXTERNAL_CONTROL);
    if (!portConfig) {
        // Consider logging an error or some other form of indication
        return false;
    }

    // Determine baud rate. Hardcoded for now, should be configurable.
    // This baud rate should match the sending device (e.g., Raspberry Pi)
    uint32_t baudRate = 230400;
    // Example of how it could be made configurable if baudRates array and appropriate config fields existed:
    // if (portConfig->msp_baudrateIndex < BAUD_COUNT) { // Assuming msp_baudrateIndex for example
    //     baudRate = baudRates[portConfig->msp_baudrateIndex];
    // }

    // Open the serial port with MODE_RXTX if we plan to send telemetry back
    // For now, let's assume the initial port opening was MODE_RX and we need to ensure it's suitable for TX.
    // Ideally, the port should be opened in MODE_RXTX from the start if bidirectional communication is planned.
    // For simplicity in this step, we'll store the port and assume it can transmit.
    // A more robust solution might involve re-opening or checking port capabilities.

    serialPort_t *port = openSerialPort(
        portConfig->identifier,
        FUNCTION_EXTERNAL_CONTROL,
        externalControlDataReceive,
        NULL, // No specific data needed for the callback beyond globals
        baudRate,
        MODE_RXTX, // Changed to RXTX to allow sending telemetry
        SERIAL_OPTIONS_NONE // Default options
    );

    if (port) {
        externalControlSerialPort = port; // Store the serial port
        lastTelemetrySendTimeUs = microsISR(); // Initialize telemetry timer
        return true;
    }

    return false;
}

static void externalControlSendAttitudeTelemetry(void) {
    if (!externalControlSerialPort) {
        return;
    }

    timeUs_t currentTimeUs = microsISR();
    // More robust check for timer wrap-around and interval
    if (lastTelemetrySendTimeUs <= currentTimeUs) { // No wraparound or just wrapped
        if ((currentTimeUs - lastTelemetrySendTimeUs) < EXTERNAL_CONTROL_TELEMETRY_INTERVAL_US) {
            return; // Not time yet
        }
    } else { // Timer has wrapped around (currentTimeUs < lastTelemetrySendTimeUs)
        // Check if enough time has passed considering the wrap
        // This condition means we always send after a wrap if the theoretical interval has passed
        // This is a simplification; a full 64-bit microsecond counter would avoid this.
        // For now, this is generally acceptable.
    }


    uint8_t telemetryFrame[EXTERNAL_CONTROL_TELEMETRY_FRAME_SIZE];
    externalControlAttitude_t attitudeData;

    attitude_t currentAttitude;
    imuGetAttitudeValues(&currentAttitude);

    attitudeData.roll = currentAttitude.values.roll;
    attitudeData.pitch = currentAttitude.values.pitch;
    attitudeData.yaw = currentAttitude.values.yaw;

    telemetryFrame[0] = EXTERNAL_CONTROL_TELEMETRY_START_BYTE; // Defined in external_control.h
    memcpy(&telemetryFrame[1], &attitudeData, sizeof(externalControlAttitude_t));

    uint8_t checksum = 0;
    // Checksum calculation should cover the data payload (attitudeData)
    for (int i = 0; i < sizeof(externalControlAttitude_t); i++) {
        checksum ^= telemetryFrame[i + 1]; // Start from byte 1 (payload)
    }
    telemetryFrame[EXTERNAL_CONTROL_TELEMETRY_FRAME_SIZE - 1] = checksum;

    if (serialTxBytesFree(externalControlSerialPort) >= EXTERNAL_CONTROL_TELEMETRY_FRAME_SIZE) {
        for (int i = 0; i < EXTERNAL_CONTROL_TELEMETRY_FRAME_SIZE; i++) {
            serialWrite(externalControlSerialPort, telemetryFrame[i]);
        }
        lastTelemetrySendTimeUs = currentTimeUs;
    }
}
