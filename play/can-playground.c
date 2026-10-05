#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

const char test_bits[] = "CAN 0 ar:id:011-0011-0111 rtr:1 control:r1r1dlc0000 crc11111-11111-11111-1 ack00 eof111-1111";
const char format_bits[] = "CAN b ar:id:bbb-bbbb-bbbb rtr:b control:rbrbdlcbbbb crcbbbbb-bbbbb-bbbbb-b ackbb eofbbb-bbbb";

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

int main(void) {
    test();
}