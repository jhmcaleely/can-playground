#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>

const char test_bits[] = "CAN 0 ar:id:011-0011-0111 rtr:1 control:r1r1dlc0000 crc11111-11111-11111-1 ack00 eof111-1111";
const char format_bits[] = "CAN b ar:id:bbb-bbbb-bbbb rtr:b control:rbrbdlcbbbb crcbbbbb-bbbbb-bbbbb-b ackbb eofbbb-bbbb";

const char can_data_frame_header_bits_str[] = "CAN 0 ar:id:011-0011-0111 rtr:1 control:r1r1dlc0000";
const char can_bits[] = "CAN 0 ar:id:011-0011-0111 rtr:1 control:r1r1dlc0000 crc11111-11111-11111-1 ack00 eof111-1111";

const char format_bits_cbus5[] = "CAN b ar:id:bbb-bbbb-bbbb rtr:b control:rbrbdlcbbbb mergbbb b-bbbb bbbb-bbbb bbbb-bbbb bbbb-bbbb bbbb-bbbb crcbbbbb-bbbbb-bbbbb-b ackbb eofbbb-bbbb";


uint8_t get_bit_char(const char* bits, size_t index) {
    unsigned char bitchr = bits[index];
    if (bitchr == '0') {
        return 0;
    } else if (bitchr == '1') {
        return 1;
    }
    return 0xff;
}

size_t count_bits(const char* bits) {
    size_t count = 0;
    for (size_t i = 0; bits[i] != '\0'; i++) {
        if (bits[i] == '0' || bits[i] == '1') {
            count++;
        }
    }
    return count;
}

size_t count_format_bits(const char* bits) {
    size_t count = 0;
    for (size_t i = 0; bits[i] != '\0'; i++) {
        if (bits[i] == 'b' || bits[i] == 'B') {
            count++;
        }
    }
    return count;
}


bool get_next_bit(const char* bits, size_t* cursor, uint8_t* bit) {
    char next = bits[*cursor];
    while (next != '\0' && next != '0' && next != '1') {
        (*cursor)++;
        next = bits[*cursor];
    }
    if (next == '\0') {
        return false;
    } else if (next == '0') {
        *bit = 0;
    } else if (next == '1') {
        *bit = 1;
    }
    (*cursor)++;
    return true;
}

void put_bit(size_t index, uint8_t bit, uint8_t* bytes) {

    size_t byte_index = index / 8;
    size_t bit_index = index % 8;

    if (bit == 0x1) {
        uint8_t mask = (0x1 << (7 - bit_index));
        bytes[byte_index] = bytes[byte_index] | mask;
    } else if (bit == 0x0) {
        uint8_t mask = (0x1 << (7 - bit_index));
        bytes[byte_index] = bytes[byte_index] & ~mask;
    }
}

uint8_t get_bit(size_t index, const uint8_t* bytes) {
    size_t byte_index = index / 8;
    size_t bit_index = index % 8;
    uint8_t mask = 0x1 << (7 - bit_index);
    uint8_t bit = bytes[byte_index] & mask;
    return bit >> (7 - bit_index);
}


bool get_next_format_char(const char* format, size_t* cursor,char* bit_char) {
    char next = format[*cursor];
    if (next == '\0') {
        return false;
    }
    else {
        *bit_char = next;
        (*cursor)++;
        return true;
    }
}

bool format_bits_string(const char* format, const char* bits, char* output) {
    size_t format_cursor = 0;
    size_t bit_cursor = 0;
    char format_char;
    uint8_t bit;
    size_t output_index = 0;

    while (get_next_bit(bits, &bit_cursor, &bit)) {
        while (get_next_format_char(format, &format_cursor, &format_char) && (format_char != 'b' && format_char != 'B')) {
            output[output_index++] = format_char;
        }
        if (format_char == 'b' || format_char == 'B') {
            output[output_index++] = bit ? '1' : '0';
        } else {
            return false;
        }
    }
    output[output_index] = '\0';
    return true;
}

bool format_bits_bytes(const char* format, const uint8_t* bytes, size_t bit_count, char* output) {
    size_t format_cursor = 0;
    size_t bit_cursor = 0;
    char format_char;
    uint8_t bit;
    size_t output_index = 0;

    for (bit_cursor = 0; bit_cursor < bit_count; bit_cursor++) {
        bit = get_bit(bit_cursor, bytes);
        while (get_next_format_char(format, &format_cursor, &format_char) && (format_char != 'b' && format_char != 'B')) {
            output[output_index++] = format_char;
        }
        if (format_char == 'b' || format_char == 'B') {
            output[output_index++] = bit ? '1' : '0';
        } else {
            return false;
        }
    }
    output[output_index] = '\0';
    return true;
}

void bits_to_str(const uint8_t* bytes, size_t bit_count, char* output) {
    for (size_t i = 0; i < bit_count; i++) {
        uint8_t bit = get_bit(i, bytes);
        output[i] = bit ? '1' : '0';
    }
    output[bit_count] = '\0';
}

void test() {

    size_t bit_count = count_bits(test_bits);
    printf("%s (bit count: %zu)\n", test_bits, bit_count);

    size_t byte_count = (bit_count + 7) / 8;

    uint8_t* bytes = (uint8_t*)calloc(byte_count, sizeof(uint8_t));

    size_t bit_cursor = 0;
    uint8_t bit;
    size_t i = 0;
    while (get_next_bit(test_bits, &bit_cursor, &bit)) {
        put_bit(i, bit, bytes);
        i++;
    }
    printf("bytes: ");
    for (size_t b = 0; b < byte_count; b++) {
        printf("0x%02x ", bytes[b]);
    }
    printf("\n");

    char* reconstructed_bits = (char*)malloc(i + 1);

    for (size_t j = 0; j < i; j++) {
        uint8_t bit = get_bit(j, bytes);
        reconstructed_bits[j] = bit ? '1' : '0';
    }
    reconstructed_bits[i] = '\0';
    printf("reconstructed bits: %s\n", reconstructed_bits);

    char* formatted_output = (char*)malloc(strlen(format_bits + 1));
    format_bits_string(format_bits, reconstructed_bits, formatted_output);
    printf("formatted output: %s\n", formatted_output);

    char* formatted_output_bytes = (char*)malloc(strlen(format_bits) + 1);
    format_bits_bytes(format_bits, bytes, bit_count, formatted_output_bytes);
    printf("formatted output from bytes: %s\n", formatted_output_bytes);

    strcmp(test_bits, formatted_output) == 0 ? printf("reconstruction successful\n") : printf("reconstruction failed\n");
    strcmp(test_bits, formatted_output_bytes) == 0 ? printf("reconstruction from bytes successful\n") : printf("reconstruction from bytes failed\n");

    free(formatted_output);
    free(formatted_output_bytes);
    free(reconstructed_bits);
    free(bytes);
}

size_t bytes_required(size_t bit_count) {
    return (bit_count + 7) / 8;
}

void put_bits(size_t bit_index, size_t bit_count, uint32_t value, uint8_t* data) {
    for (size_t i = 0; i < bit_count; i++) {
        size_t byte_index = (bit_index + i) / 8;
        size_t bit_offset = 7 - ((bit_index + i) % 8);
        data[byte_index] &= ~(1 << bit_offset);
        data[byte_index] |= ((value >> (bit_count - 1 - i)) & 1) << bit_offset;
    }
}

// https://www.port.de/fileadmin/user_upload/Dateien_IST_fuer_Migration/CAN20A.pdf
// appears to be canonical result for 'CAN Specification 2.0A'
uint16_t can_crc15_next(uint16_t crc, uint8_t data, int len) {

    for (int i = 0; i < len; i++) {


        uint16_t nxtbit = data & 0x80;
        data <<= 1;

        nxtbit <<= 7;

        uint8_t crcnext = (nxtbit ^ (crc & 0x4000)) ? 1 : 0 ;

        crc <<= 1;
        crc &= 0x7FFF;
        if (crcnext) {
            crc ^= 0x4599;
        }
        crc &= 0x7FFF;
    }

    return crc;
}

uint16_t compute_crc(uint8_t* data, size_t bit_count) {

    size_t byte_count = bytes_required(bit_count);
    uint16_t crc = 0;

    // Implementation for computing CRC
    for (size_t i = 0; i < byte_count; i++) {
        int bits_in_byte = (i == byte_count - 1) ? (bit_count % 8) : 8;
        if (bits_in_byte == 0) bits_in_byte = 8;
        crc = can_crc15_next(crc, data[i], bits_in_byte);
    }
    return crc;
}

uint8_t can_recessive = 1;
uint8_t can_dominant = 0;
size_t can_max_bytes = 14; // Maximum CAN data frame size in bytes

const size_t can_identifier_bits = 11; // Standard CAN identifier size in bits
const size_t can_arbitration_bits = can_identifier_bits + 1; // Standard CAN arbitration field size in bits (identifier + RTR bit)
const size_t can_dlc_bits = 4; // Standard CAN DLC field size in bits
const size_t can_control_field_bits = 2 + can_dlc_bits;
const size_t can_data_frame_header_bits = 1 + can_arbitration_bits + can_control_field_bits; // can data frame header size in bits
const size_t can_data_frame_crc_bits = 15; // CRC size in bits
const size_t can_data_frame_trailer_bits = 10; // CRC delimiter, ACK slot, ACK delimiter, and end of frame size in bits
const size_t can_max_data_field_bits = 8 * 8;
const size_t can_max_frame_bits = can_data_frame_header_bits + can_max_data_field_bits + can_data_frame_crc_bits + can_data_frame_trailer_bits;
const size_t can_max_frame_buffer_bytes = (can_max_frame_bits + 7) / 8;
const size_t can_max_stuff_bits = (can_data_frame_header_bits + can_max_data_field_bits + can_data_frame_crc_bits) / 5;
const size_t can_max_transmission_bits = can_max_stuff_bits + can_max_frame_bits;
const size_t can_max_transmission_buffer_bytes = (can_max_transmission_bits + 7) / 8;

size_t assemble_cbus_can_frame(uint8_t* cbus_data, uint8_t* can_data) {
    // Implementation for assembling a CAN frame from CBUS data
    size_t cbus_data_length = (cbus_data[0] & 0xe0) >> 5;
    printf("CBUS data length: %zu\n", cbus_data_length);
    size_t can_data_length = cbus_data_length + 1; // for the opcode.

    assert(can_max_bytes == can_max_frame_buffer_bytes);
    printf("CAN max frame buffer bytes: %zu\n", can_max_frame_buffer_bytes);
    printf("CAN max transmission buffer bytes: %zu\n", can_max_transmission_buffer_bytes);
    printf("CAN max stuff bits: %zu\n", can_max_stuff_bits);
    printf("CAN max transmission bits: %zu\n", can_max_transmission_bits);

    printf("CAN data length: %zu\n", can_data_length);

    uint8_t cbus_MjPri = 0x2;
    uint8_t cbus_MinPri = 0x3;
    uint8_t cbus_ID = 10;

    size_t max_cbus_bytes = 8; // Maximum CBUS data bytes
    size_t max_cbus_bits = max_cbus_bytes * 8;
    size_t max_can_bits = count_bits(can_bits) + max_cbus_bits;
    size_t max_can_bytes = bytes_required(max_can_bits);
    assert(max_can_bytes == can_max_bytes);

    size_t bits_to_stuff = 0;
    size_t bit_index = 0;
    put_bit(bit_index++, 0, can_data);
    put_bits(bit_index, 2, cbus_MjPri, can_data);
    bit_index += 2;
    put_bits(bit_index, 2, cbus_MinPri, can_data);
    bit_index += 2;
    put_bits(bit_index, 7, cbus_ID, can_data);
    bit_index += 7;
    put_bit(bit_index++, can_dominant, can_data); // rtr, data frame.
    put_bit(bit_index++, can_dominant, can_data); // control r0 (reserved, send dominant)
    put_bit(bit_index++, can_dominant, can_data); // control r1 (reserved, send dominant)
    put_bits(bit_index, 4, can_data_length, can_data); // control r2
    bit_index += 4;
    printf("Bit index after header: %zu\n", bit_index);
    for (size_t i = 0; i < can_data_length; i++) {
        put_bits(bit_index, 8, cbus_data[i], can_data);
        bit_index += 8;
    }
    uint16_t crc = compute_crc(can_data, bit_index);
    put_bits(bit_index, 15, crc, can_data);
    bit_index += 15;
    bits_to_stuff = bit_index;
    put_bit(bit_index++, can_recessive, can_data); // crc delimiter
    put_bit(bit_index++, can_recessive, can_data); // ack slot (recessive as transmitter)
    put_bit(bit_index++, can_recessive, can_data); // ack delimiter
    put_bits(bit_index, 7, 0x7F, can_data); // end of frame
    bit_index += 7;

    printf("Total bits assembled: %zu\n", bit_index);

    size_t send_bits = count_bits(can_bits) + can_data_length * 8;
    printf("Total bits to send: %zu\n", send_bits);

    printf("count of format bits: %zu\n", count_format_bits(format_bits_cbus5));

    char* formatted_output_bytes = malloc(1000);
    format_bits_bytes(format_bits_cbus5, can_data, bit_index, formatted_output_bytes);

    printf("Formatted output bytes: %s\n", formatted_output_bytes);
    free((void*)formatted_output_bytes);

    printf("Bits to stuff: %zu\n", bits_to_stuff);
    return bits_to_stuff;
}

size_t stuff_frame(uint8_t* can_data, size_t bits_to_stuff, uint8_t* stuffed_can_data) {
    size_t transmit_bits = 0;
    uint8_t previous_bit = can_recessive;
    size_t consecutive_count = 1;
    for (size_t i = 0; i < bits_to_stuff; i++) {
        size_t byte_index = i / 8;
        size_t bit_index = i % 8;
        uint8_t bit = get_bit(i, can_data);
        put_bit(transmit_bits++, bit, stuffed_can_data);
        if (bit == previous_bit) {
            consecutive_count++;
            if (consecutive_count == 5) {
                put_bit(transmit_bits++, !previous_bit, stuffed_can_data);
                consecutive_count = 0;
                continue;
            }
        } else {
            consecutive_count = 1;
        }
        previous_bit = bit;
    }
    return transmit_bits;
}

void cbus_op_code(uint8_t* cbus_data, uint8_t opcode) {
    cbus_data[0] = opcode;
}

void cbus_16bit_param(uint8_t* cbus_data, uint16_t param) {
    cbus_data[0] = (param >> 8) & 0xFF;
    cbus_data[1] = param & 0xFF;
}

void cbus_test() {
    uint8_t cbus_data[8];
    cbus_op_code(&cbus_data[0], 0x90);
    cbus_16bit_param(&cbus_data[1], 256);
    cbus_16bit_param(&cbus_data[3], 8);

    printf("can_headerBits: %zu\n", count_bits(can_data_frame_header_bits_str));

    uint8_t can_data[can_max_bytes];
    size_t bits_to_stuff = assemble_cbus_can_frame(cbus_data, can_data);
    size_t max_stuff_bits = bits_to_stuff / 5; // Assuming 1 stuffed bit for every 5 bits
    printf("Maximum stuff bits: %zu\n", max_stuff_bits);

    size_t stuffed_bytes = bytes_required(bits_to_stuff + max_stuff_bits);
    printf("Stuffed bytes required: %zu\n", stuffed_bytes);

    size_t max_transmitted_bits = bits_to_stuff + max_stuff_bits + 10; // Adding extra bits for CRC delimiter, ACK slot, ACK delimiter, and end of frame
    size_t transmitted_bytes = bytes_required(max_transmitted_bits);

    uint8_t* stuffed_can_data = malloc(transmitted_bytes);
    size_t transmit_bits = stuff_frame(can_data, bits_to_stuff, stuffed_can_data);

    put_bit(transmit_bits++, can_recessive, stuffed_can_data); // crc delimiter
    put_bit(transmit_bits++, can_recessive, stuffed_can_data); // ack slot (recessive as transmitter)
    put_bit(transmit_bits++, can_recessive, stuffed_can_data); // ack delimiter
    put_bits(transmit_bits, 7, 0x7F, stuffed_can_data); // end of frame
    transmit_bits += 7;

    printf("Total transmit bits: %zu\n", transmit_bits);

    char bit_str[transmit_bits + 1];
    char can_str[transmit_bits + 1];
    bits_to_str(can_data, bits_to_stuff, can_str);
    printf("Original CAN data: %s\n", can_str);
    bits_to_str(stuffed_can_data, transmit_bits, bit_str);
    printf("Stuffed CAN data:  %s\n", bit_str);

    free(stuffed_can_data);
}

int main(void) {
    cbus_test();
    test();
}