/** @file
    RST Temperature and Humidity sensor (based on Hideki).

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

*/
/**
RST Temperature and Humidity sensor.

The received bits are inverted.

Every 8 bits are stuffed with a (even) parity bit.
The payload (excluding the header) has an byte parity (XOR) check.
The payload (excluding the header) has CRC-8, poly 0x07 init 0x00 check.
The payload bytes are reflected (LSB first / LSB last) after the CRC check.

Temp:

    11111001 0  11110101 0  01110011 1 01111010 1  11001100 0  01000011 1  01000110 1  00111111 0  00001001 0  00010111 0
    SYNC+HEAD P   RC cha P     LEN   P     Nr.? P   .1° 1°  P   10°  BV P   1%  10% P     ?     P     XOR   P     CRC   P

*/

#include "decoder.h"

#define RST_MAX_BYTES_PER_ROW 10

enum sensortypes { RST_UNKNOWN, RST_TEMP };

static int rst_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    int ret = 0;
    for (int row = 0; row < bitbuffer->num_rows; row++) {
        int sensortype;
        // Expect 10 unstuffed bytes, allow up to 4 missing bits
        int unstuffed_len = (bitbuffer->bits_per_row[row] + 4) / 9;
        if (unstuffed_len == 10)
            sensortype = RST_TEMP;
        else {
            ret = DECODE_ABORT_LENGTH;
            continue;
        }
        unstuffed_len -= 1; // exclude sync

        uint8_t *b = bitbuffer->bb[row];
        // Expect a start (not inverted) of 00000110 1, but allow missing bits
        int sync = b[0] << 1 | b[1] >> 7;
        int startpos = -1;
        for (int i = 0; i < 4; ++i) {
            if (sync == 0x0d) {
                startpos = 9 - i;
                break;
            }
            sync >>= 1;
        }
        if (startpos < 0) {
            ret = DECODE_ABORT_EARLY;
            continue;
        }

        // Invert all bits
        bitbuffer_invert(bitbuffer);

        uint8_t packet[RST_MAX_BYTES_PER_ROW];
        // Strip (unstuff) and check parity bit
        // TODO: refactor to util function
        int unstuff_error = 0;
        for (int i = 0; i < unstuffed_len; ++i) {
            unsigned int offset = startpos + i * 9;
            packet[i] = (b[offset / 8] << (offset % 8)) | (b[offset / 8 + 1] >> (8 - offset % 8));
            // check parity
            uint8_t parity = (b[offset / 8 + 1] >> (7 - offset % 8)) & 1;
            if (parity != parity8(packet[i])) {
                decoder_logf(decoder, 1, __func__, "Parity error at %d", i);
                ret = DECODE_FAIL_MIC;
                unstuff_error = i;
                break;
            }
        }
        if (unstuff_error) {
            continue;
        }

        // XOR check all bytes
        int chk = xor_bytes(packet, unstuffed_len - 1);
        if (chk) {
            decoder_log(decoder, 1, __func__, "XOR error");
            ret = DECODE_FAIL_MIC;
            continue;
        }

        // CRC-8 poly=0x07 init=0x00
        if (crc8(packet, unstuffed_len, 0x07, 0x00)) {
            decoder_log(decoder, 1, __func__, "CRC error");
            ret = DECODE_FAIL_MIC;
            continue;
        }

        // Reflect LSB first to LSB last
        reflect_bytes(packet, unstuffed_len);

        int pkt_len  = (packet[1] >> 1) & 0x1f;
        //int pkt_seq  = packet[2] >> 6;
        //int pkt_type = packet[2] & 0x1f;
        // 0x0C Anemometer
        // 0x0D UV sensor
        // 0x0E Rain level meter
        // 0x1E Thermo/hygro-sensor

        if (pkt_len + 2 != unstuffed_len) {
            decoder_log(decoder, 1, __func__, "LEN error");
            ret = DECODE_ABORT_LENGTH;
            continue;
        }

        int channel = (packet[0] >> 5) & 0x0F;
        if (channel >= 5) channel -= 1;
        int rc = packet[0] & 0x0F;
        int temp = (packet[4] & 0x0F) * 100 + ((packet[3] & 0xF0) >> 4) * 10 + (packet[3] & 0x0F);
        if (((packet[4]>>7) & 1) == 0) {
            temp = -temp;
        }
        int battery_ok = (packet[4] >> 6) & 1;

        if (sensortype == RST_TEMP) {
            /* clang-format off */
            data_t *data = data_make(
                    "model",            "",                 DATA_STRING, "RST-Temperature",
                    "id",               "Rolling Code",     DATA_INT,    rc,
                    "channel",          "Channel",          DATA_INT,    channel,
                    "battery_ok",       "Battery",          DATA_INT,    battery_ok,
                    "temperature_C",    "Temperature",      DATA_FORMAT, "%.01f C", DATA_DOUBLE, temp * 0.1f,
                    "mic",              "Integrity",        DATA_STRING, "CRC",
                    NULL);
            /* clang-format on */
            decoder_output_data(decoder, data);
            return 1;
        }

        // unknown sensor type
        return DECODE_FAIL_SANITY;
    }
    return ret;
}

static char const *const output_fields[] = {
        "model",
        "id",
        "channel",
        "battery_ok",
        "temperature_C",
        "humidity",
        "wind_avg_mi_h",
        "wind_max_mi_h",
        "wind_approach",
        "wind_dir_deg",
        "rain_mm",
        "mic",
        NULL,
};

r_device const rst = {
        .name        = "RST Sweden Temperature and Humidity Sensor",
        .modulation  = OOK_PULSE_DMC,
        .short_width = 520,  // half-bit width 520 us
        .long_width  = 1040, // bit width 1040 us
        .reset_limit = 4000,
        .tolerance   = 240,
        .decode_fn   = &rst_decode,
        .fields      = output_fields,
};
