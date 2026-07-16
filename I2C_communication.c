/*
 * Copyright (c) 2024, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "I2C_communication.h"

/*
 * ============================================================================
 * Preserved: original global variables for interrupt-based state machine.
 * These are NOT used by the blocking polling functions below,
 * but are kept so the existing ISR and any future interrupt-driven
 * code can still reference them.
 * ============================================================================
 */

/* Data sent to the Target */
uint8_t gTxPacket[I2C_TX_MAX_PACKET_SIZE] = {0x3E, 0x01, 0x00, 0x40, 0x04,
    0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
/* Counters for TX length and bytes sent */
uint32_t gTxLen, gTxCount;

/* Data received from Target */
uint8_t gRxPacket[I2C_RX_MAX_PACKET_SIZE];
/* Counters for RX length and bytes received */
uint32_t gRxLen, gRxCount;
/* I2C status (used by ISR) */
I2cControllerStatus_t gI2cControllerStatus;

/*
 * ============================================================================
 * New: last-result tracking for blocking register-level I2C operations.
 * ============================================================================
 */
volatile I2C_Result_t gI2cLastResult = I2C_RESULT_OK;

/*
 * ============================================================================
 * Static helper: check for controller error or arbitration lost.
 * ============================================================================
 */
static I2C_Result_t I2C_CheckControllerError(void)
{
    uint32_t status;

    status = DL_I2C_getControllerStatus(I2C_1_INST);

    if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U)
    {
        return I2C_RESULT_CONTROLLER_ERROR;
    }

    if ((status &
         DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST) != 0U)
    {
        return I2C_RESULT_CONTROLLER_ERROR;
    }

    return I2C_RESULT_OK;
}

/*
 * ============================================================================
 * Static helper: wait for controller to become idle, with timeout.
 * ============================================================================
 */
static I2C_Result_t I2C_WaitIdle(void)
{
    uint32_t timeout = I2C_POLL_TIMEOUT_COUNT;
    I2C_Result_t result;

    while ((DL_I2C_getControllerStatus(I2C_1_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U)
    {
        result = I2C_CheckControllerError();

        if (result != I2C_RESULT_OK)
        {
            return result;
        }

        if (timeout == 0U)
        {
            return I2C_RESULT_TIMEOUT_IDLE;
        }

        timeout--;
    }

    return I2C_RESULT_OK;
}

/*
 * ============================================================================
 * Static helper: wait for bus to become free, with timeout.
 * ============================================================================
 */
static I2C_Result_t I2C_WaitBusFree(void)
{
    uint32_t timeout = I2C_POLL_TIMEOUT_COUNT;
    I2C_Result_t result;

    while ((DL_I2C_getControllerStatus(I2C_1_INST) &
            DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0U)
    {
        result = I2C_CheckControllerError();

        if (result != I2C_RESULT_OK)
        {
            return result;
        }

        if (timeout == 0U)
        {
            return I2C_RESULT_TIMEOUT_BUS;
        }

        timeout--;
    }

    return I2C_RESULT_OK;
}

/*
 * ============================================================================
 * Static helper: wait for RX FIFO to have data, with timeout.
 * ============================================================================
 */
static I2C_Result_t I2C_WaitRxData(void)
{
    uint32_t timeout = I2C_POLL_TIMEOUT_COUNT;
    I2C_Result_t result;

    while (DL_I2C_isControllerRXFIFOEmpty(I2C_1_INST))
    {
        result = I2C_CheckControllerError();

        if (result != I2C_RESULT_OK)
        {
            return result;
        }

        if (timeout == 0U)
        {
            return I2C_RESULT_TIMEOUT_RX;
        }

        timeout--;
    }

    return I2C_RESULT_OK;
}

/*
 * ============================================================================
 * Static helper: abort current transfer and flush FIFOs on error/timeout.
 * Minimal recovery — does not reset the entire I2C peripheral.
 * ============================================================================
 */
static void I2C_AbortTransfer(void)
{
    DL_I2C_resetControllerTransfer(I2C_1_INST);
    DL_I2C_flushControllerTXFIFO(I2C_1_INST);
    DL_I2C_flushControllerRXFIFO(I2C_1_INST);
}

/*
 * ============================================================================
 * Query the last blocking communication result (for debugging).
 * ============================================================================
 */
I2C_Result_t I2C_GetLastResult(void)
{
    return (I2C_Result_t)gI2cLastResult;
}

//************Array copy **********************
/* Preserved: utility function, unchanged from original. */
void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count)
{
    uint8_t copyIndex = 0;
    for (copyIndex = 0; copyIndex < count; copyIndex++) {
        dest[copyIndex] = source[copyIndex];
    }
}

//************I2C write register **********************
I2C_Result_t I2C_WriteReg(
    uint8_t addr,
    uint8_t reg_addr,
    const uint8_t *reg_data,
    uint8_t count)
{
    uint8_t txBuffer[I2C_TX_MAX_PACKET_SIZE];
    uint8_t i;
    uint16_t filled;
    I2C_Result_t result;

    /* Parameter validation */
    if ((reg_data == (const uint8_t *)0) ||
        (count == 0U) ||
        (((uint16_t)count + 1U) >
         I2C_TX_MAX_PACKET_SIZE))
    {
        gI2cLastResult = I2C_RESULT_INVALID_PARAM;
        return gI2cLastResult;
    }

    /* Assemble: register address + data */
    txBuffer[0] = reg_addr;

    for (i = 0U; i < count; i++)
    {
        txBuffer[i + 1U] = reg_data[i];
    }

    /* Wait for controller to be idle before starting */
    result = I2C_WaitIdle();

    if (result != I2C_RESULT_OK)
    {
        I2C_AbortTransfer();
        gI2cLastResult = result;
        return result;
    }

    DL_I2C_flushControllerTXFIFO(I2C_1_INST);

    /* Fill TX FIFO with register address + data */
    filled = DL_I2C_fillControllerTXFIFO(
        I2C_1_INST,
        txBuffer,
        (uint16_t)count + 1U);

    if (filled != ((uint16_t)count + 1U))
    {
        I2C_AbortTransfer();
        gI2cLastResult = I2C_RESULT_TX_FIFO_ERROR;
        return gI2cLastResult;
    }

    /* Start the controller transfer (TX direction) */
    DL_I2C_startControllerTransfer(
        I2C_1_INST,
        addr,
        DL_I2C_CONTROLLER_DIRECTION_TX,
        (uint16_t)count + 1U);

    /* Wait for transfer to complete */
    result = I2C_WaitBusFree();

    if (result == I2C_RESULT_OK)
    {
        result = I2C_WaitIdle();
    }

    if (result != I2C_RESULT_OK)
    {
        I2C_AbortTransfer();
    }

    DL_I2C_flushControllerTXFIFO(I2C_1_INST);

    gI2cLastResult = result;
    return result;
}

//************I2C read register **********************
I2C_Result_t I2C_ReadReg(
    uint8_t addr,
    uint8_t reg_addr,
    uint8_t *reg_data,
    uint8_t count)
{
    uint8_t i;
    uint16_t filled;
    I2C_Result_t result;

    /* Parameter validation */
    if ((reg_data == (uint8_t *)0) ||
        (count == 0U) ||
        (count > I2C_RX_MAX_PACKET_SIZE))
    {
        gI2cLastResult = I2C_RESULT_INVALID_PARAM;
        return gI2cLastResult;
    }

    /* Wait for controller to be idle before starting */
    result = I2C_WaitIdle();

    if (result != I2C_RESULT_OK)
    {
        I2C_AbortTransfer();
        gI2cLastResult = result;
        return result;
    }

    DL_I2C_flushControllerTXFIFO(I2C_1_INST);
    DL_I2C_flushControllerRXFIFO(I2C_1_INST);

    /*
     * CRITICAL FIX:
     * The register address is exactly 1 byte.
     * The fill length MUST be 1U, NOT count.
     * Using count here was a bug that caused out-of-bounds read
     * from the local stack variable 'reg_addr'.
     */
    filled = DL_I2C_fillControllerTXFIFO(
        I2C_1_INST,
        &reg_addr,
        1U);

    if (filled != 1U)
    {
        I2C_AbortTransfer();
        gI2cLastResult = I2C_RESULT_TX_FIFO_ERROR;
        return gI2cLastResult;
    }

    /* Send register address (TX direction, 1 byte) */
    DL_I2C_startControllerTransfer(
        I2C_1_INST,
        addr,
        DL_I2C_CONTROLLER_DIRECTION_TX,
        1U);

    result = I2C_WaitBusFree();

    if (result == I2C_RESULT_OK)
    {
        result = I2C_WaitIdle();
    }

    if (result != I2C_RESULT_OK)
    {
        I2C_AbortTransfer();
        gI2cLastResult = result;
        return result;
    }

    DL_I2C_flushControllerTXFIFO(I2C_1_INST);

    /* Initiate read (RX direction, count bytes) */
    DL_I2C_startControllerTransfer(
        I2C_1_INST,
        addr,
        DL_I2C_CONTROLLER_DIRECTION_RX,
        count);

    /* Read each byte with per-byte timeout */
    for (i = 0U; i < count; i++)
    {
        result = I2C_WaitRxData();

        if (result != I2C_RESULT_OK)
        {
            I2C_AbortTransfer();
            gI2cLastResult = result;
            return result;
        }

        reg_data[i] =
            DL_I2C_receiveControllerData(I2C_1_INST);
    }

    /*
     * After all bytes received, confirm bus and controller return to idle.
     */
    result = I2C_WaitBusFree();

    if (result == I2C_RESULT_OK)
    {
        result = I2C_WaitIdle();
    }

    if (result != I2C_RESULT_OK)
    {
        I2C_AbortTransfer();
    }

    gI2cLastResult = result;
    return result;
}

/*
 * ============================================================================
 * Preserved: original I2C interrupt handler.
 * This ISR is kept but is NOT used by the blocking polling functions above.
 * The blocking functions do NOT enable I2C interrupts and do NOT depend
 * on this ISR to complete transfers.
 *
 * If SysConfig has I2C interrupts enabled, they should be disabled
 * (or kept disabled) for the blocking polling approach used here.
 *
 * Note: added break after NACK handler to prevent fall-through.
 * ============================================================================
 */
void I2C_1_INST_IRQHandler(void)
{
    switch (DL_I2C_getPendingInterrupt(I2C_1_INST)) {
        case DL_I2C_IIDX_CONTROLLER_RX_DONE:
            gI2cControllerStatus = I2C_STATUS_RX_COMPLETE;
            break;
        case DL_I2C_IIDX_CONTROLLER_TX_DONE:
            DL_I2C_disableInterrupt(
                I2C_1_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
            gI2cControllerStatus = I2C_STATUS_TX_COMPLETE;
            break;
        case DL_I2C_IIDX_CONTROLLER_RXFIFO_TRIGGER:
            gI2cControllerStatus = I2C_STATUS_RX_INPROGRESS;
            /* Receive all bytes from target */
            while (DL_I2C_isControllerRXFIFOEmpty(I2C_1_INST) != true) {
                if (gRxCount < gRxLen) {
                    gRxPacket[gRxCount++] =
                        DL_I2C_receiveControllerData(I2C_1_INST);
                } else {
                    /* Ignore and remove from FIFO if the buffer is full */
                    DL_I2C_receiveControllerData(I2C_1_INST);
                }
            }
            break;
        case DL_I2C_IIDX_CONTROLLER_TXFIFO_TRIGGER:
            gI2cControllerStatus = I2C_STATUS_TX_INPROGRESS;
            /* Fill TX FIFO with next bytes to send */
            if (gTxCount < gTxLen) {
                gTxCount += DL_I2C_fillControllerTXFIFO(
                    I2C_1_INST, &gTxPacket[gTxCount], gTxLen - gTxCount);
            }
            break;
        case DL_I2C_IIDX_CONTROLLER_ARBITRATION_LOST:
        case DL_I2C_IIDX_CONTROLLER_NACK:
            if ((gI2cControllerStatus == I2C_STATUS_RX_STARTED) ||
                (gI2cControllerStatus == I2C_STATUS_TX_STARTED)) {
                /* NACK interrupt if I2C Target is disconnected */
                gI2cControllerStatus = I2C_STATUS_ERROR;
            }
            break; /* Fixed: prevent fall-through to RXFIFO_FULL */
        case DL_I2C_IIDX_CONTROLLER_RXFIFO_FULL:
        case DL_I2C_IIDX_CONTROLLER_TXFIFO_EMPTY:
        case DL_I2C_IIDX_CONTROLLER_START:
        case DL_I2C_IIDX_CONTROLLER_STOP:
        case DL_I2C_IIDX_CONTROLLER_EVENT1_DMA_DONE:
        case DL_I2C_IIDX_CONTROLLER_EVENT2_DMA_DONE:
        default:
            break;
    }
}
