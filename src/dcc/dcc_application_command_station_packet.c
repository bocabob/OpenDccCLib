/** \copyright
 * Copyright (c) 2026, Jim Kueneman
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file dcc_application_command_station_packet.c
 * @brief Application-layer API for building DCC command station packets.
 *
 * @details Implements the packet-building functions declared in
 * dcc_application_command_station_packet.h. Pure computational — no hardware
 * dependencies, no state. Takes command parameters and fills a dcc_packet_t
 * with the correct byte layout and XOR error detection byte per NMRA S-9.2.
 *
 * @author Jim Kueneman
 * @date 23 Jun 2026
 */

#include "dcc_application_command_station_packet.h"

#ifdef DCC_COMPILE_COMMAND_STATION

// =============================================================================
// Static helpers
// =============================================================================

    /**
     * @brief Compute XOR of all bytes and append it to the packet.
     * @param packet Packet with data[0..byte_count-1] already filled.
     */
static void _append_xor(dcc_packet_t *packet) {

    uint8_t xor_byte = 0;
    uint8_t byte_index;

    for (byte_index = 0; byte_index < packet->byte_count; byte_index++) {

        xor_byte ^= packet->data[byte_index];

    }

    packet->data[packet->byte_count] = xor_byte;
    packet->byte_count++;

}

    /**
     * @brief Encode address bytes into packet. Returns the index of the next
     *        free byte in data[].
     * @param packet Packet to fill.
     * @param address DCC address.
     * @param address_type Short or long.
     * @return Next byte index (1 for short, 2 for long), or 0 on error.
     */
static uint8_t _encode_address(dcc_packet_t *packet, dcc_address_t address, dcc_address_type_enum address_type) {

    if (address_type == DCC_ADDRESS_SHORT || address_type == DCC_ADDRESS_BROADCAST) {

        packet->data[0] = (uint8_t)(address & 0x7F);
        return 1;

    }

    /* Long address: high byte has bits 7-6 set to 11 (0xC0 | high 6 bits) */
    /* Wire encoding: CV 17 = 0xC0 | ((addr >> 8) & 0x3F), CV 18 = addr & 0xFF */
    packet->data[0] = 0xC0 | (uint8_t)((address >> 8) & 0x3F);
    packet->data[1] = (uint8_t)(address & 0xFF);
    return 2;

}

    /**
     * @brief Validate address type is suitable for multi-function decoder commands
     *        (speed, function, CV ops-mode, consist, etc.).
     * @param address_type Address type to check.
     * @return true if valid for loco/multi-function commands.
     */
static bool _is_loco_address_type(dcc_address_type_enum address_type) {

    return (address_type == DCC_ADDRESS_SHORT ||
            address_type == DCC_ADDRESS_LONG ||
            address_type == DCC_ADDRESS_BROADCAST);

}

    /**
     * @brief Common implementation for feature expansion function group packets
     *        (F13-F20 through F61-F68). Two-byte instruction: expansion code
     *        followed by function state bits.
     * @param packet Pointer to packet struct to fill.
     * @param address DCC address.
     * @param address_type Short or long address.
     * @param instruction_byte Feature expansion instruction byte (e.g., 0xDE).
     * @param func_bits 8-bit function state bitmask.
     * @return true if packet was built successfully, false if invalid parameters.
     */
static bool _func_expansion(dcc_packet_t *packet, dcc_address_t address, dcc_address_type_enum address_type, uint8_t instruction_byte, uint8_t func_bits) {

    uint8_t byte_index;

    if (!_is_loco_address_type(address_type)) {

        return false;

    }

    byte_index = _encode_address(packet, address, address_type);

    packet->data[byte_index] = instruction_byte;
    byte_index++;

    packet->data[byte_index] = func_bits;
    byte_index++;

    packet->byte_count = byte_index;
    _append_xor(packet);

    packet->preamble_bits = DCC_PREAMBLE_BITS_OPS;
    packet->repeat_count = DCC_REPEAT_ONE_SHOT_DEFAULT;

    return true;

}

    /**
     * @brief Common implementation for ops-mode CV access (write/verify).
     * @param packet Pointer to packet struct to fill.
     * @param address DCC address.
     * @param address_type Short or long address.
     * @param cv_number CV number (1-1024, 1-based).
     * @param value Data byte.
     * @param cv_instruction_prefix CV instruction prefix (DCC_CV_LONG_WRITE or
     *        DCC_CV_LONG_VERIFY).
     * @return true if packet was built successfully, false if invalid parameters.
     */
static bool _cv_ops_common(dcc_packet_t *packet, dcc_address_t address, dcc_address_type_enum address_type, uint16_t cv_number, uint8_t value, uint8_t cv_instruction_prefix) {

    uint8_t byte_index;
    uint16_t wire_cv;

    if (!_is_loco_address_type(address_type)) {

        return false;

    }

    if (cv_number < 1 || cv_number > 1024) {

        return false;

    }

    byte_index = _encode_address(packet, address, address_type);

    /* CV number is 1-based in API, 0-based on wire */
    wire_cv = cv_number - 1;

    /* Instruction byte: 1110CCDD where CC=operation, DD=CV address high 2 bits */
    packet->data[byte_index] = cv_instruction_prefix | (uint8_t)((wire_cv >> 8) & 0x03);
    byte_index++;

    /* CV address low 8 bits */
    packet->data[byte_index] = (uint8_t)(wire_cv & 0xFF);
    byte_index++;

    /* Data byte */
    packet->data[byte_index] = value;
    byte_index++;

    packet->byte_count = byte_index;
    _append_xor(packet);

    packet->preamble_bits = DCC_PREAMBLE_BITS_OPS;
    /* One-shot send (not auto-refresh): the scheduler's _select_one_shot()
     * skips any non-auto-refresh slot whose repeat_count is 0 -- that is its
     * "nothing left to send" state (what the count is decremented to after a
     * real send), not a valid starting value. A CV write / POM verify packet
     * built with repeat_count = 0 was accepted into a scheduler slot but
     * never selected for transmission. Matches the accessory-stop builders
     * (DccApplicationCommandStationPacket_load_accessory_*_stop), which
     * correctly use repeat_count = 1.
     *
     * repeat_count = 1 is right for verify, but S-9.2.1 p.9 "Type=11 WRITE
     * BYTE" requires two identical packets before a decoder modifies a CV
     * ("These two packets need not be back to back on the track. However
     * any other packet to the same decoder will invalidate the write
     * operation."); VERIFY BYTE (p.8) acts on the first packet it receives.
     * The scheduler decrements repeat_count after each send and drops the
     * slot at 0 (dcc_scheduler.c), so a write has to start at 2. */
    packet->repeat_count = (cv_instruction_prefix == DCC_CV_LONG_WRITE) ? DCC_REPEAT_CV_WRITE : DCC_REPEAT_CV_VERIFY;

    return true;

}

// =============================================================================
// Idle / Reset / Emergency Stop
// =============================================================================

void DccApplicationCommandStationPacket_load_idle(dcc_packet_t *packet) {

    packet->data[0] = DCC_IDLE_ADDR_BYTE;
    packet->data[1] = DCC_IDLE_DATA_BYTE;
    packet->data[2] = DCC_IDLE_XOR_BYTE;
    packet->byte_count = 3;
    packet->preamble_bits = DCC_PREAMBLE_BITS_OPS;
    packet->repeat_count = DCC_REPEAT_ONE_SHOT_DEFAULT;

}

void DccApplicationCommandStationPacket_load_reset(dcc_packet_t *packet) {

    packet->data[0] = DCC_RESET_BYTE;
    packet->data[1] = DCC_RESET_BYTE;
    packet->data[2] = DCC_RESET_BYTE;
    packet->byte_count = 3;
    packet->preamble_bits = DCC_PREAMBLE_BITS_OPS;
    packet->repeat_count = DCC_REPEAT_ONE_SHOT_DEFAULT;

}

void DccApplicationCommandStationPacket_load_estop_all(dcc_packet_t *packet, bool isPanic) {

    /* S-9.2 baseline broadcast stop: 00000000 0 01DC000S, error byte = copy of
     * byte two. Uses the baseline 01x speed/direction instruction class that
     * every decoder must obey -- NOT the 128-step form (which baseline-only
     * decoders ignore). Byte two = 01DC000S with D=0, C=1 (ignore direction).
     * isPanic -> S: 1 = stop delivering energy (emergency); 0 = bring the
     * locomotive to a controlled stop. */
    packet->data[0] = DCC_ADDRESS_BROADCAST_VALUE;          /* 00000000      */
    packet->data[1] = 0x50 | (isPanic ? 0x01u : 0x00u);    /* 01 0 1 000 S  */
    packet->byte_count = 2;
    _append_xor(packet);   /* error byte = data[0] ^ data[1] = copy of byte two */

    packet->preamble_bits = DCC_PREAMBLE_BITS_OPS;
    packet->repeat_count = DCC_REPEAT_ONE_SHOT_DEFAULT;

}

// =============================================================================
// Speed Commands
// =============================================================================

bool DccApplicationCommandStationPacket_load_speed_128(dcc_packet_t *packet, dcc_address_t address, dcc_address_type_enum address_type, uint8_t speed, bool direction) {

    uint8_t byte_index;

    if (!_is_loco_address_type(address_type)) {

        return false;

    }

    if (speed > 127) {

        return false;

    }

    byte_index = _encode_address(packet, address, address_type);

    /* Advanced operations instruction byte: 00111111 */
    packet->data[byte_index] = DCC_ADV_OPS_128_SPEED;
    byte_index++;

    /* Speed byte: DSSSSSSS — D=direction (bit 7), S=speed (bits 6-0) */
    packet->data[byte_index] = (direction ? 0x80 : 0x00) | (speed & 0x7F);
    byte_index++;

    packet->byte_count = byte_index;
    _append_xor(packet);

    packet->preamble_bits = DCC_PREAMBLE_BITS_OPS;
    packet->repeat_count = DCC_REPEAT_ONE_SHOT_DEFAULT;

    return true;

}

    /**
     * @brief 28-step speed-to-instruction lookup table per NMRA S-9.2 Figure 2.
     *
     * Index = API speed value (0-29).
     * Value = CSSSS bits to OR into the instruction byte (bit4=C, bits3-0=SSSS).
     *
     * API speed 0     = Stop      (encoded 0: C=0 SSSS=0000)
     * API speed 1     = E-Stop    (encoded 2: C=0 SSSS=0001)
     * API speed 2-29  = Step 1-28 (encoded 4-31)
     */
static const uint8_t _speed_28_encode[30] = {

    0x00,                                                   /* speed  0: Stop   */
    0x01,                                                   /* speed  1: E-Stop */
    0x02, 0x12, 0x03, 0x13, 0x04, 0x14, 0x05, 0x15,       /* speed  2-9       */
    0x06, 0x16, 0x07, 0x17, 0x08, 0x18, 0x09, 0x19,       /* speed 10-17      */
    0x0A, 0x1A, 0x0B, 0x1B, 0x0C, 0x1C, 0x0D, 0x1D,       /* speed 18-25      */
    0x0E, 0x1E, 0x0F, 0x1F                                 /* speed 26-29      */

};

bool DccApplicationCommandStationPacket_load_speed_28(dcc_packet_t *packet, dcc_address_t address, dcc_address_type_enum address_type, uint8_t speed, bool direction) {

    uint8_t byte_index;
    uint8_t instruction;

    if (!_is_loco_address_type(address_type)) {

        return false;

    }

    if (speed > 29) {

        return false;

    }

    byte_index = _encode_address(packet, address, address_type);

    instruction = direction ? DCC_INST_SPEED_FORWARD : DCC_INST_SPEED_REVERSE;
    instruction |= _speed_28_encode[speed];

    packet->data[byte_index] = instruction;
    byte_index++;

    packet->byte_count = byte_index;
    _append_xor(packet);

    packet->preamble_bits = DCC_PREAMBLE_BITS_OPS;
    packet->repeat_count = DCC_REPEAT_ONE_SHOT_DEFAULT;

    return true;

}

bool DccApplicationCommandStationPacket_load_speed_14(dcc_packet_t *packet, dcc_address_t address, dcc_address_type_enum address_type, uint8_t speed, bool direction, bool headlight) {

    uint8_t byte_index;
    uint8_t instruction;

    if (!_is_loco_address_type(address_type)) {

        return false;

    }

    if (speed > 15) {

        return false;

    }

    byte_index = _encode_address(packet, address, address_type);

    /* Instruction: 01DCSSSS — D=direction, C=FL headlight, SSSS=speed */
    instruction = direction ? DCC_INST_SPEED_FORWARD : DCC_INST_SPEED_REVERSE;
    instruction |= (headlight ? 0x10 : 0x00) | (speed & 0x0F);

    packet->data[byte_index] = instruction;
    byte_index++;

    packet->byte_count = byte_index;
    _append_xor(packet);

    packet->preamble_bits = DCC_PREAMBLE_BITS_OPS;
    packet->repeat_count = DCC_REPEAT_ONE_SHOT_DEFAULT;

    return true;

}

// =============================================================================
// Function Commands
// =============================================================================

bool DccApplicationCommandStationPacket_load_func_group_1(dcc_packet_t *packet, dcc_address_t address, dcc_address_type_enum address_type, uint8_t func_bits) {

    uint8_t byte_index;

    if (!_is_loco_address_type(address_type)) {

        return false;

    }

    byte_index = _encode_address(packet, address, address_type);

    /* Instruction: 100DDDDD — D bits = FL|F4|F3|F2|F1 */
    packet->data[byte_index] = DCC_INST_FUNC_GROUP_1 | (func_bits & 0x1F);
    byte_index++;

    packet->byte_count = byte_index;
    _append_xor(packet);

    packet->preamble_bits = DCC_PREAMBLE_BITS_OPS;
    packet->repeat_count = DCC_REPEAT_ONE_SHOT_DEFAULT;

    return true;

}

bool DccApplicationCommandStationPacket_load_func_group_2a(dcc_packet_t *packet, dcc_address_t address, dcc_address_type_enum address_type, uint8_t func_bits) {

    uint8_t byte_index;

    if (!_is_loco_address_type(address_type)) {

        return false;

    }

    byte_index = _encode_address(packet, address, address_type);

    /* Instruction: 1011DDDD — D bits = F8|F7|F6|F5 */
    packet->data[byte_index] = DCC_INST_FUNC_GROUP_2A | (func_bits & 0x0F);
    byte_index++;

    packet->byte_count = byte_index;
    _append_xor(packet);

    packet->preamble_bits = DCC_PREAMBLE_BITS_OPS;
    packet->repeat_count = DCC_REPEAT_ONE_SHOT_DEFAULT;

    return true;

}

bool DccApplicationCommandStationPacket_load_func_group_2b(dcc_packet_t *packet, dcc_address_t address, dcc_address_type_enum address_type, uint8_t func_bits) {

    uint8_t byte_index;

    if (!_is_loco_address_type(address_type)) {

        return false;

    }

    byte_index = _encode_address(packet, address, address_type);

    /* Instruction: 1010DDDD — D bits = F12|F11|F10|F9 */
    packet->data[byte_index] = DCC_INST_FUNC_GROUP_2B | (func_bits & 0x0F);
    byte_index++;

    packet->byte_count = byte_index;
    _append_xor(packet);

    packet->preamble_bits = DCC_PREAMBLE_BITS_OPS;
    packet->repeat_count = DCC_REPEAT_ONE_SHOT_DEFAULT;

    return true;

}

bool DccApplicationCommandStationPacket_load_func_f13_f20(dcc_packet_t *packet, dcc_address_t address, dcc_address_type_enum address_type, uint8_t func_bits) {

    return _func_expansion(packet, address, address_type, DCC_FEAT_F13_F20, func_bits);

}

bool DccApplicationCommandStationPacket_load_func_f21_f28(dcc_packet_t *packet, dcc_address_t address, dcc_address_type_enum address_type, uint8_t func_bits) {

    return _func_expansion(packet, address, address_type, DCC_FEAT_F21_F28, func_bits);

}

bool DccApplicationCommandStationPacket_load_func_f29_f36(dcc_packet_t *packet, dcc_address_t address, dcc_address_type_enum address_type, uint8_t func_bits) {

    return _func_expansion(packet, address, address_type, DCC_FEAT_F29_F36, func_bits);

}

bool DccApplicationCommandStationPacket_load_func_f37_f44(dcc_packet_t *packet, dcc_address_t address, dcc_address_type_enum address_type, uint8_t func_bits) {

    return _func_expansion(packet, address, address_type, DCC_FEAT_F37_F44, func_bits);

}

bool DccApplicationCommandStationPacket_load_func_f45_f52(dcc_packet_t *packet, dcc_address_t address, dcc_address_type_enum address_type, uint8_t func_bits) {

    return _func_expansion(packet, address, address_type, DCC_FEAT_F45_F52, func_bits);

}

bool DccApplicationCommandStationPacket_load_func_f53_f60(dcc_packet_t *packet, dcc_address_t address, dcc_address_type_enum address_type, uint8_t func_bits) {

    return _func_expansion(packet, address, address_type, DCC_FEAT_F53_F60, func_bits);

}

bool DccApplicationCommandStationPacket_load_func_f61_f68(dcc_packet_t *packet, dcc_address_t address, dcc_address_type_enum address_type, uint8_t func_bits) {

    return _func_expansion(packet, address, address_type, DCC_FEAT_F61_F68, func_bits);

}

// =============================================================================
// Accessory Commands
// =============================================================================

bool DccApplicationCommandStationPacket_load_accessory_basic(dcc_packet_t *packet, uint16_t board_address, uint8_t output_pair, bool activate) {

    if (board_address > 511) {

        return false;

    }

    if (output_pair > 7) {

        return false;

    }

    /* Byte 1: 10AAAAAA — lower 6 bits of board address */
    packet->data[0] = DCC_ACCESSORY_BASIC_PREFIX | (uint8_t)(board_address & 0x3F);

    /* Byte 2: 1AAACDDDD — upper 3 address bits (INVERTED), C=activate, DDD=output */
    packet->data[1] = 0x80
                      | (uint8_t)((~(board_address >> 6) & 0x07) << 4)
                      | (activate ? 0x08 : 0x00)
                      | (output_pair & 0x07);

    packet->byte_count = 2;
    _append_xor(packet);

    packet->preamble_bits = DCC_PREAMBLE_BITS_OPS;
    packet->repeat_count = DCC_REPEAT_ONE_SHOT_DEFAULT;

    return true;

}

bool DccApplicationCommandStationPacket_load_accessory_extended(dcc_packet_t *packet, uint16_t address, uint8_t aspect) {

    if (address > 2047) {

        return false;

    }

    /* Byte 1: 10AAAAAA — lower 6 bits of address */
    packet->data[0] = DCC_ACCESSORY_BASIC_PREFIX | (uint8_t)(address & 0x3F);

    /* Byte 2: 0AAA0AA1 — upper 3 bits inverted (bits 6-4), next 2 bits (bits 2-1) */
    packet->data[1] = DCC_ACCESSORY_EXTENDED_PREFIX
                      | (uint8_t)((~(address >> 6) & 0x07) << 4)
                      | (uint8_t)(((address >> 9) & 0x03) << 1);

    /* Byte 3: signal aspect */
    packet->data[2] = aspect;

    packet->byte_count = 3;
    _append_xor(packet);

    packet->preamble_bits = DCC_PREAMBLE_BITS_OPS;
    packet->repeat_count = DCC_REPEAT_ONE_SHOT_DEFAULT;

    return true;

}

bool DccApplicationCommandStationPacket_load_accessory_nop(dcc_packet_t *packet, uint16_t address, bool is_extended) {

    if (address > 2047) {

        return false;

    }

    /* Byte 1: 10AAAAAA — lower 6 bits of address */
    packet->data[0] = DCC_ACCESSORY_BASIC_PREFIX | (uint8_t)(address & 0x3F);

    /* Byte 2: 0AAA1AAT — upper 3 bits inverted (bits 6-4), bit 3 = 1 (NOP marker),
     * next 2 address bits (bits 2-1), T (bit 0): 0 = basic, 1 = extended */
    packet->data[1] = (uint8_t)((~(address >> 6) & 0x07) << 4)
                      | 0x08
                      | (uint8_t)(((address >> 9) & 0x03) << 1)
                      | (is_extended ? 0x01 : 0x00);

    packet->byte_count = 2;
    _append_xor(packet);

    packet->preamble_bits = DCC_PREAMBLE_BITS_OPS;
    packet->repeat_count = DCC_REPEAT_ACCESSORY_NOP;

    return true;

}

    /**
     * @verbatim
     * @param packet        Pointer to a dcc_packet_t struct to fill.
     * @param board_address 9-bit board address (0-511).
     * @param output_pair   Output selection (0-7).
     * @endverbatim
     */
bool DccApplicationCommandStationPacket_load_accessory_basic_stop(dcc_packet_t *packet, uint16_t board_address, uint8_t output_pair) {

    if (board_address > 511) {

        return false;

    }

    if (output_pair > 7) {

        return false;

    }

    /* Byte 1: 10AAAAAA — lower 6 bits of board address */
    packet->data[0] = DCC_ACCESSORY_BASIC_PREFIX | (uint8_t)(board_address & 0x3F);

    /* Byte 2: 1AAAC0DDD — upper 3 address bits (INVERTED), C=0 (deactivate), DDD=output */
    packet->data[1] = 0x80
                      | (uint8_t)((~(board_address >> 6) & 0x07) << 4)
                      | (output_pair & 0x07);

    packet->byte_count = 2;
    _append_xor(packet);

    packet->preamble_bits = DCC_PREAMBLE_BITS_OPS;
    packet->repeat_count = DCC_REPEAT_ACCESSORY_STOP;

    return true;

}

    /**
     * @verbatim
     * @param packet  Pointer to a dcc_packet_t struct to fill.
     * @param address 11-bit address (0-2047).
     * @endverbatim
     */
bool DccApplicationCommandStationPacket_load_accessory_extended_stop(dcc_packet_t *packet, uint16_t address) {

    if (address > 2047) {

        return false;

    }

    /* Byte 1: 10AAAAAA — lower 6 bits of address */
    packet->data[0] = DCC_ACCESSORY_BASIC_PREFIX | (uint8_t)(address & 0x3F);

    /* Byte 2: 0AAA0AA1 — upper 3 bits inverted (bits 6-4), next 2 bits (bits 2-1) */
    packet->data[1] = DCC_ACCESSORY_EXTENDED_PREFIX
                      | (uint8_t)((~(address >> 6) & 0x07) << 4)
                      | (uint8_t)(((address >> 9) & 0x03) << 1);

    /* Byte 3: aspect 0 (all stop) */
    packet->data[2] = 0x00;

    packet->byte_count = 3;
    _append_xor(packet);

    packet->preamble_bits = DCC_PREAMBLE_BITS_OPS;
    packet->repeat_count = DCC_REPEAT_ACCESSORY_STOP;

    return true;

}

// =============================================================================
// Accessory CV Programming (Ops-Mode) — static helpers
// =============================================================================

    /**
     * @brief Common implementation for basic accessory CV ops-mode packets.
     *
     * Builds the 5-byte payload (+ XOR) for basic accessory CV access.
     * Byte 1 format for CV ops-mode: 1AAA1AA0 — bit3=1 forced, bits 2-1
     * are A1/A0 (output_pair), bit0=0 forced.  This differs from the
     * operating-command byte 1 where bit3=C and bits 2-0=DDD.
     *
     * @param packet Pointer to packet struct to fill.
     * @param board_address 9-bit board address (0-511).
     * @param output_pair Output pair (0-3).
     * @param cv_instruction_prefix CV instruction prefix byte (DCC_CV_LONG_WRITE,
     *        DCC_CV_LONG_VERIFY, or DCC_CV_LONG_BIT).
     * @param wire_cv 0-based CV number (cv_number - 1).
     * @param data_byte Data or bit-manipulation byte.
     * @param is_write true for a WRITE BYTE/WRITE BIT operation, false for
     *        VERIFY BYTE/VERIFY BIT (the caller already knows which; for the
     *        BIT case this is the same flag baked into data_byte's own C bit).
     * @return true if packet was built successfully.
     */
static bool _acc_basic_cv_common(dcc_packet_t *packet, uint16_t board_address, uint8_t output_pair, uint8_t cv_instruction_prefix, uint16_t wire_cv, uint8_t data_byte, bool is_write) {

    /* Byte 0: 10AAAAAA — lower 6 bits of board address */
    packet->data[0] = DCC_ACCESSORY_BASIC_PREFIX | (uint8_t)(board_address & 0x3F);

    /* Byte 1: 1AAA1AA0 — upper 3 addr bits inverted, bit3=1, A1A0, bit0=0 */
    packet->data[1] = 0x80
                      | (uint8_t)((~(board_address >> 6) & 0x07) << 4)
                      | 0x08
                      | (uint8_t)((output_pair & 0x03) << 1);

    /* Byte 2: 1110CCDD — CV instruction prefix + CV address high 2 bits */
    packet->data[2] = cv_instruction_prefix | (uint8_t)((wire_cv >> 8) & 0x03);

    /* Byte 3: CV address low 8 bits */
    packet->data[3] = (uint8_t)(wire_cv & 0xFF);

    /* Byte 4: data byte */
    packet->data[4] = data_byte;

    packet->byte_count = 5;
    _append_xor(packet);

    packet->preamble_bits = DCC_PREAMBLE_BITS_OPS;
    /* One-shot send: same bug as _cv_ops_common() (see its comment) -- these
     * accessory CV ops-mode builders have no caller anywhere in this library
     * yet, so a repeat_count = 0 that silently drops every packet before
     * transmission was never caught. Fixed alongside the same
     * write/verify split S-9.2.1 p.9 requires for the loco POM builders --
     * two identical packets for a write, one for a verify. */
    packet->repeat_count = is_write ? DCC_REPEAT_CV_WRITE : DCC_REPEAT_CV_VERIFY;

    return true;

}

    /**
     * @brief Common implementation for extended accessory CV ops-mode packets.
     *
     * Builds the 5-byte payload (+ XOR) for extended accessory CV access.
     * Byte 0-1 encoding matches the extended operating command format.
     *
     * @param packet Pointer to packet struct to fill.
     * @param address 11-bit address (0-2047).
     * @param cv_instruction_prefix CV instruction prefix byte.
     * @param wire_cv 0-based CV number (cv_number - 1).
     * @param data_byte Data or bit-manipulation byte.
     * @param is_write true for a WRITE BYTE/WRITE BIT operation, false for
     *        VERIFY BYTE/VERIFY BIT.
     * @return true if packet was built successfully.
     */
static bool _acc_extended_cv_common(dcc_packet_t *packet, uint16_t address, uint8_t cv_instruction_prefix, uint16_t wire_cv, uint8_t data_byte, bool is_write) {

    /* Byte 0: 10AAAAAA — lower 6 bits of address */
    packet->data[0] = DCC_ACCESSORY_BASIC_PREFIX | (uint8_t)(address & 0x3F);

    /* Byte 1: 0AAA0AA1 — upper 3 bits inverted, next 2 bits */
    packet->data[1] = DCC_ACCESSORY_EXTENDED_PREFIX
                      | (uint8_t)((~(address >> 6) & 0x07) << 4)
                      | (uint8_t)(((address >> 9) & 0x03) << 1);

    /* Byte 2: 1110CCDD — CV instruction prefix + CV address high 2 bits */
    packet->data[2] = cv_instruction_prefix | (uint8_t)((wire_cv >> 8) & 0x03);

    /* Byte 3: CV address low 8 bits */
    packet->data[3] = (uint8_t)(wire_cv & 0xFF);

    /* Byte 4: data byte */
    packet->data[4] = data_byte;

    packet->byte_count = 5;
    _append_xor(packet);

    packet->preamble_bits = DCC_PREAMBLE_BITS_OPS;
    /* Same never-sent bug and the same write/verify split as
     * _acc_basic_cv_common() above -- see its comment. */
    packet->repeat_count = is_write ? DCC_REPEAT_CV_WRITE : DCC_REPEAT_CV_VERIFY;

    return true;

}

// =============================================================================
// Accessory CV Programming (Ops-Mode) — public functions
// =============================================================================

bool DccApplicationCommandStationPacket_load_accessory_basic_cv_write(dcc_packet_t *packet, uint16_t board_address, uint8_t output_pair, uint16_t cv_number, uint8_t value) {

    if (board_address > 511 || output_pair > 3 || cv_number < 1 || cv_number > 1024) {

        return false;

    }

    return _acc_basic_cv_common(packet, board_address, output_pair, DCC_CV_LONG_WRITE, cv_number - 1, value, true);

}

bool DccApplicationCommandStationPacket_load_accessory_basic_cv_verify(dcc_packet_t *packet, uint16_t board_address, uint8_t output_pair, uint16_t cv_number, uint8_t value) {

    if (board_address > 511 || output_pair > 3 || cv_number < 1 || cv_number > 1024) {

        return false;

    }

    return _acc_basic_cv_common(packet, board_address, output_pair, DCC_CV_LONG_VERIFY, cv_number - 1, value, false);

}

bool DccApplicationCommandStationPacket_load_accessory_basic_cv_bit(dcc_packet_t *packet, uint16_t board_address, uint8_t output_pair, uint16_t cv_number, uint8_t bit_position, bool bit_value, bool write) {

    uint8_t bit_byte;

    if (board_address > 511 || output_pair > 3 || cv_number < 1 || cv_number > 1024 || bit_position > 7) {

        return false;

    }

    /* Bit manipulation byte: 111CDBBB — C=write, D=bit value, BBB=bit position */
    bit_byte = 0xE0
               | (write ? 0x10 : 0x00)
               | (bit_value ? 0x08 : 0x00)
               | (bit_position & 0x07);

    return _acc_basic_cv_common(packet, board_address, output_pair, DCC_CV_LONG_BIT, cv_number - 1, bit_byte, write);

}

bool DccApplicationCommandStationPacket_load_accessory_extended_cv_write(dcc_packet_t *packet, uint16_t address, uint16_t cv_number, uint8_t value) {

    if (address > 2047 || cv_number < 1 || cv_number > 1024) {

        return false;

    }

    return _acc_extended_cv_common(packet, address, DCC_CV_LONG_WRITE, cv_number - 1, value, true);

}

bool DccApplicationCommandStationPacket_load_accessory_extended_cv_verify(dcc_packet_t *packet, uint16_t address, uint16_t cv_number, uint8_t value) {

    if (address > 2047 || cv_number < 1 || cv_number > 1024) {

        return false;

    }

    return _acc_extended_cv_common(packet, address, DCC_CV_LONG_VERIFY, cv_number - 1, value, false);

}

bool DccApplicationCommandStationPacket_load_accessory_extended_cv_bit(dcc_packet_t *packet, uint16_t address, uint16_t cv_number, uint8_t bit_position, bool bit_value, bool write) {

    uint8_t bit_byte;

    if (address > 2047 || cv_number < 1 || cv_number > 1024 || bit_position > 7) {

        return false;

    }

    /* Bit manipulation byte: 111CDBBB — C=write, D=bit value, BBB=bit position */
    bit_byte = 0xE0
               | (write ? 0x10 : 0x00)
               | (bit_value ? 0x08 : 0x00)
               | (bit_position & 0x07);

    return _acc_extended_cv_common(packet, address, DCC_CV_LONG_BIT, cv_number - 1, bit_byte, write);

}

// =============================================================================
// CV Programming (POM — Programming on the Main)
// =============================================================================

bool DccApplicationCommandStationPacket_load_cv_write_pom(dcc_packet_t *packet, dcc_address_t address, dcc_address_type_enum address_type, uint16_t cv_number, uint8_t value) {

    return _cv_ops_common(packet, address, address_type, cv_number, value, DCC_CV_LONG_WRITE);

}

bool DccApplicationCommandStationPacket_load_cv_verify_pom(dcc_packet_t *packet, dcc_address_t address, dcc_address_type_enum address_type, uint16_t cv_number, uint8_t value) {

    return _cv_ops_common(packet, address, address_type, cv_number, value, DCC_CV_LONG_VERIFY);

}

bool DccApplicationCommandStationPacket_load_cv_bit_pom(dcc_packet_t *packet, dcc_address_t address, dcc_address_type_enum address_type, uint16_t cv_number, uint8_t bit_position, bool bit_value, bool write) {

    uint8_t byte_index;
    uint16_t wire_cv;

    if (!_is_loco_address_type(address_type)) {

        return false;

    }

    if (cv_number < 1 || cv_number > 1024) {

        return false;

    }

    if (bit_position > 7) {

        return false;

    }

    byte_index = _encode_address(packet, address, address_type);

    wire_cv = cv_number - 1;

    /* Instruction byte: 1110 10DD — bit manipulation + CV address high 2 bits */
    packet->data[byte_index] = DCC_CV_LONG_BIT | (uint8_t)((wire_cv >> 8) & 0x03);
    byte_index++;

    /* CV address low 8 bits */
    packet->data[byte_index] = (uint8_t)(wire_cv & 0xFF);
    byte_index++;

    /* Bit manipulation byte: 111CDBBB — C=write, D=bit value, BBB=bit position */
    packet->data[byte_index] = 0xE0
                               | (write ? 0x10 : 0x00)
                               | (bit_value ? 0x08 : 0x00)
                               | (bit_position & 0x07);
    byte_index++;

    packet->byte_count = byte_index;
    _append_xor(packet);

    packet->preamble_bits = DCC_PREAMBLE_BITS_OPS;
    /* One-shot send: see _cv_ops_common()'s comment above -- this builder does
     * not route through it (bit manipulation has its own instruction byte
     * layout) and was missed when that fix landed there. Same bug, same fix:
     * a CV bit write/verify built with repeat_count = 0 is accepted into a
     * scheduler slot but never selected for transmission.
     *
     * And the same write/verify split as _cv_ops_common()'s repeat_count
     * comment: S-9.2.1 p.9 requires two identical packets for WRITE BIT,
     * same as WRITE BYTE ("a configuration variable access acknowledgment
     * will be generated in response to the second identical WRITE BIT
     * instruction"); VERIFY BIT, like VERIFY BYTE, acts on the first. */
    packet->repeat_count = write ? DCC_REPEAT_CV_WRITE : DCC_REPEAT_CV_VERIFY;

    return true;

}

// =============================================================================
// Consist Control
// =============================================================================

bool DccApplicationCommandStationPacket_load_consist_set(dcc_packet_t *packet, dcc_address_t address, dcc_address_type_enum address_type, uint8_t consist_address, bool direction_normal) {

    uint8_t byte_index;

    if (!_is_loco_address_type(address_type)) {

        return false;

    }

    if (consist_address > 127) {

        return false;

    }

    byte_index = _encode_address(packet, address, address_type);

    /* Instruction: 0001001D — D=0 normal, D=1 reversed */
    packet->data[byte_index] = direction_normal
                               ? DCC_CONSIST_SET_NORMAL
                               : DCC_CONSIST_SET_REVERSED;
    byte_index++;

    /* Consist address (7-bit) */
    packet->data[byte_index] = consist_address & 0x7F;
    byte_index++;

    packet->byte_count = byte_index;
    _append_xor(packet);

    packet->preamble_bits = DCC_PREAMBLE_BITS_OPS;
    packet->repeat_count = DCC_REPEAT_ONE_SHOT_DEFAULT;

    return true;

}

bool DccApplicationCommandStationPacket_load_consist_clear(dcc_packet_t *packet, dcc_address_t address, dcc_address_type_enum address_type) {

    /* Clear consist = set consist address 0 with normal direction */
    return DccApplicationCommandStationPacket_load_consist_set(packet, address, address_type, 0, true);

}

// =============================================================================
// Binary State / Analog Function
// =============================================================================

bool DccApplicationCommandStationPacket_load_binary_state_short(dcc_packet_t *packet, dcc_address_t address, dcc_address_type_enum address_type, uint8_t state_number, bool active) {

    uint8_t byte_index;

    if (!_is_loco_address_type(address_type)) {

        return false;

    }

    if (state_number > 127) {

        return false;

    }

    byte_index = _encode_address(packet, address, address_type);

    /* Feature expansion instruction byte */
    packet->data[byte_index] = DCC_FEAT_BINARY_STATE_SHORT;
    byte_index++;

    /* Data byte: DLLLLLLL — D=active, L=state number */
    packet->data[byte_index] = (active ? 0x80 : 0x00) | (state_number & 0x7F);
    byte_index++;

    packet->byte_count = byte_index;
    _append_xor(packet);

    packet->preamble_bits = DCC_PREAMBLE_BITS_OPS;
    packet->repeat_count = DCC_REPEAT_ONE_SHOT_DEFAULT;

    return true;

}

bool DccApplicationCommandStationPacket_load_binary_state_long(dcc_packet_t *packet, dcc_address_t address, dcc_address_type_enum address_type, uint16_t state_number, bool active) {

    uint8_t byte_index;

    if (!_is_loco_address_type(address_type)) {

        return false;

    }

    if (state_number > 32767) {

        return false;

    }

    byte_index = _encode_address(packet, address, address_type);

    /* Feature expansion instruction byte */
    packet->data[byte_index] = DCC_FEAT_BINARY_STATE_LONG;
    byte_index++;

    /* Low byte: DLLLLLLL — D=active, L=state number low 7 bits */
    packet->data[byte_index] = (active ? 0x80 : 0x00) | (uint8_t)(state_number & 0x7F);
    byte_index++;

    /* High byte: HHHHHHHH — state number high 8 bits */
    packet->data[byte_index] = (uint8_t)((state_number >> 7) & 0xFF);
    byte_index++;

    packet->byte_count = byte_index;
    _append_xor(packet);

    packet->preamble_bits = DCC_PREAMBLE_BITS_OPS;
    packet->repeat_count = DCC_REPEAT_ONE_SHOT_DEFAULT;

    return true;

}

bool DccApplicationCommandStationPacket_load_analog_function(dcc_packet_t *packet, dcc_address_t address, dcc_address_type_enum address_type, uint8_t output_number, uint8_t value) {

    uint8_t byte_index;

    if (!_is_loco_address_type(address_type)) {

        return false;

    }

    byte_index = _encode_address(packet, address, address_type);

    /* Advanced operations instruction byte */
    packet->data[byte_index] = DCC_ADV_OPS_ANALOG_FUNCTION;
    byte_index++;

    /* Output number */
    packet->data[byte_index] = output_number;
    byte_index++;

    /* Value */
    packet->data[byte_index] = value;
    byte_index++;

    packet->byte_count = byte_index;
    _append_xor(packet);

    packet->preamble_bits = DCC_PREAMBLE_BITS_OPS;
    packet->repeat_count = DCC_REPEAT_ONE_SHOT_DEFAULT;

    return true;

}

void DccApplicationCommandStationPacket_load_system_time(dcc_packet_t *packet, uint16_t milliseconds) {

    /* S-9.2.1 §2.3.6.3 System Time: broadcast to address 0, feature-expansion
     * sub-instruction 110-00010. Three-byte instruction carrying a 16-bit
     * milliseconds-since-startup count, most significant byte first. */
    packet->data[0] = DCC_ADDRESS_BROADCAST_VALUE;              /* 00000000  */
    packet->data[1] = DCC_FEAT_SYSTEM_TIME;                     /* 110-00010 */
    packet->data[2] = (uint8_t)((milliseconds >> 8) & 0xFF);    /* MSB       */
    packet->data[3] = (uint8_t)(milliseconds & 0xFF);           /* LSB       */
    packet->byte_count = 4;
    _append_xor(packet);

    packet->preamble_bits = DCC_PREAMBLE_BITS_OPS;
    packet->repeat_count = DCC_REPEAT_TIME;

}

bool DccApplicationCommandStationPacket_load_model_time(dcc_packet_t *packet, uint8_t minutes, dcc_day_of_week_enum day_of_week, uint8_t hours, bool update, uint8_t accel_factor) {

    if (minutes > 59 || hours > 23 || accel_factor > 63 || day_of_week > DCC_DAY_OF_WEEK_NOT_SUPPORTED) {

        return false;

    }

    /* S-9.2.1 §2.3.6.2 Time command: broadcast addr 0, feature-expansion
     * 110-00001 (0xC1). CC=00 selects the Time sub-format:
     *   00MMMMMM  WWWHHHHH  U0BBBBBB  */
    packet->data[0] = DCC_ADDRESS_BROADCAST_VALUE;                          /* 00000000 */
    packet->data[1] = DCC_FEAT_TIME_DATE;                                   /* 110-00001 */
    packet->data[2] = (uint8_t)(minutes & 0x3F);                            /* CC=00, MMMMMM */
    packet->data[3] = (uint8_t)(((day_of_week & 0x07) << 5) | (hours & 0x1F));
    packet->data[4] = (uint8_t)((update ? 0x80 : 0x00) | (accel_factor & 0x3F));
    packet->byte_count = 5;
    _append_xor(packet);

    packet->preamble_bits = DCC_PREAMBLE_BITS_OPS;
    packet->repeat_count = DCC_REPEAT_TIME;

    return true;

}

bool DccApplicationCommandStationPacket_load_model_date(dcc_packet_t *packet, uint8_t day, uint8_t month, uint16_t year) {

    if (day < 1 || day > 31 || month < 1 || month > 12 || year > 4095) {

        return false;

    }

    /* S-9.2.1 §2.3.6.2 Date command: broadcast addr 0, feature-expansion
     * 110-00001 (0xC1). CC=01 selects the Date sub-format:
     *   010TTTTT  MMMMYYYY  YYYYYYYY  (year is 12 bits, MSB nibble packed
     *   with the month, LSB byte last). */
    packet->data[0] = DCC_ADDRESS_BROADCAST_VALUE;                          /* 00000000 */
    packet->data[1] = DCC_FEAT_TIME_DATE;                                   /* 110-00001 */
    packet->data[2] = (uint8_t)(0x40 | (day & 0x1F));                       /* 010TTTTT (CC=01) */
    packet->data[3] = (uint8_t)(((month & 0x0F) << 4) | ((year >> 8) & 0x0F));
    packet->data[4] = (uint8_t)(year & 0xFF);
    packet->byte_count = 5;
    _append_xor(packet);

    packet->preamble_bits = DCC_PREAMBLE_BITS_OPS;
    packet->repeat_count = DCC_REPEAT_MODEL_DATE;

    return true;

}

#endif /* DCC_COMPILE_COMMAND_STATION */
