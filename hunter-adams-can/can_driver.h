/**
 * V. Hunter Adams (vha3@cornell.edu)
 * 
 * CAN driver code
 * 
 */

// Includes
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "can.pio.h"
#include "can_parameters.h"


//                         CLOCK AND CHECKSUM PARAMETERS
//
// Clock settings
#define SYSTEM_CLOCK_RATE  128000
#define CLKDIV          32
// Checksum polynomial and initial value
#define CRC16_POLY              0x8005
#define CRC_INIT                0xFFFF


//          OTHER BUFFERS FOR STORING STUFFED/UNSTUFFED PACKETS FOR TX/RX
//
// Assembled packet for transmission (unstuffed then stuffed)
unsigned short tx_packet_unstuffed[MAX_PACKET_LEN>>1] = {0} ;
unsigned short tx_packet_stuffed[MAX_STUFFED_PACKET_LEN>>1] = {0} ;
unsigned short * tx_packet_stuffed_pointer = &tx_packet_stuffed[0] ;

// Buffer for received packets (stuffed then unstuffed)
unsigned char rx_packet_stuffed[MAX_STUFFED_PACKET_LEN] = {0} ;
unsigned char * rx_packet_stuffed_pointer = &rx_packet_stuffed[0] ;

// For re-initializing these buffers
unsigned char zero_packet[MAX_STUFFED_PACKET_LEN] = {0} ;


uint16_t can_crc15_next(uint16_t crc, uint8_t data, int len);
uint8_t can_dominant_bit = 0;
uint8_t can_recessive_bit = 1;


//                             INFRASTRUCTURE GLOBALS
//
// Our PIO blocks
PIO pio_0 = pio0 ;
PIO pio_1 = pio1 ;
// Our state machines
int can_tx_sm         = 0 ;
int can_idle_check_sm = 1 ;
int can_rx_sm         = 0 ;
// Select dma channels
int dma_chan_0  = 0 ;
int dma_chan_1  = 1 ;
int dma_chan_2  = 2 ;
int dma_chan_3  = 3 ;
int dma_chan_4  = 4 ;
// Dummy DMA source/destination for chained channel
unsigned int dummy_source = 0 ;
unsigned int dummy_dest   = 0 ;


//               FUNCTION WHICH COMPUTES CHECKSUM (TX & RX)
//
// Function which computes the checksum over a series of bytes
unsigned short culCalcCRC(char crcData, unsigned short crcReg) {
    int i;
    for (i = 0; i < 8; i++) {
        if (((crcReg & 0x8000) >> 8) ^ (crcData & 0x80)) {
            crcReg = (crcReg << 1) ^ CRC16_POLY;
        } else {
          crcReg = (crcReg << 1);
        }
        crcData <<= 1;
    }
    return crcReg;
}


//                 FUNCTIONS USED FOR PACKET TRANSMISSION
//
// Helper function which access a particular bit in an unsigned short.
// It returns that bit (0 or 1) as an unsigned short.
unsigned short getBitShort(unsigned short * shorty, unsigned char bitnum) {
    switch(bitnum) {
        case 0:
            return ((*shorty & 0x8000) >> 15) ;
        case 1:
            return ((*shorty & 0x4000) >> 14) ;
        case 2:
            return ((*shorty & 0x2000) >> 13) ;
        case 3:
            return ((*shorty & 0x1000) >> 12) ;
        case 4:
            return ((*shorty & 0x0800) >> 11) ;
        case 5:
            return ((*shorty & 0x0400) >> 10) ;
        case 6:
            return ((*shorty & 0x0200) >> 9) ;
        case 7:
            return ((*shorty & 0x0100) >> 8) ;
        case 8:
            return ((*shorty & 0x0080) >> 7) ;
        case 9:
            return ((*shorty & 0x0040) >> 6) ;
        case 10:
            return ((*shorty & 0x0020) >> 5) ;
        case 11:
            return ((*shorty & 0x0010) >> 4) ;
        case 12:
            return ((*shorty & 0x0008) >> 3) ;
        case 13:
            return ((*shorty & 0x0004) >> 2) ;
        case 14:
            return ((*shorty & 0x0002) >> 1) ;
        case 15:
            return ((*shorty & 0x0001) >> 0) ;
        default:
            return 0xFFFF ;
    }
}
// Helper function which modifies a single bit in an unsigned short. The first argument
// is a pointer to the short that will be modified. The second is the bit number (0-15) to
// modify. And the third is the value (0 or 1) to which we would like it modified.
void modifyBitShort(unsigned short * shorty, unsigned char bitnum, unsigned short value) {
    switch(bitnum) {
        case 15:
            *shorty |= ((value << 0) & 0x0001) ;
            break ;
        case 14:
            *shorty |= ((value << 1) & 0x0002) ;
            break ;
        case 13:
            *shorty |= ((value << 2) & 0x0004) ;
            break ;
        case 12:
            *shorty |= ((value << 3) & 0x0008) ;
            break ;
        case 11:
            *shorty |= ((value << 4) & 0x0010) ;
            break ;
        case 10:
            *shorty |= ((value << 5) & 0x0020) ;
            break ;
        case 9:
            *shorty |= ((value << 6) & 0x0040) ;
            break ;
        case 8:
            *shorty |= ((value << 7) & 0x0080) ;
            break ;
        case 7:
            *shorty |= ((value << 8) & 0x0100) ;
            break ;
        case 6:
            *shorty |= ((value << 9) & 0x0200) ;
            break ;
        case 5:
            *shorty |= ((value << 10) & 0x0400) ;
            break ;
        case 4:
            *shorty |= ((value << 11) & 0x0800) ;
            break ;
        case 3:
            *shorty |= ((value << 12) & 0x1000) ;
            break ;
        case 2:
            *shorty |= ((value << 13) & 0x2000) ;
            break ;
        case 1:
            *shorty |= ((value << 14) & 0x4000) ;
            break ;
        case 0:
            *shorty |= ((value << 15) & 0x8000) ;
            break ;
        default:
            printf("Invalid argument to modifyBitChar   \n") ;
            break ;
    }
}
// Assumes that the final element in the unstuffed buffer is 0xFFFF
void bitStuff(unsigned short * unstuffed, unsigned short * stuffed) {
    // Clear the buffer
    memcpy(&stuffed[0], &zero_packet[0], MAX_STUFFED_PACKET_LEN) ;

    // Variables for monitoring position in each buffer
    int stuffed_index   = 0 ;
    int unstuffed_index = 0 ;
    int stuffed_bit     = 0 ;
    int unstuffed_bit   = 0 ;

    // Accumulated bit run length
    int bit_run_len = 1 ;

    // Memory of old bit value
    unsigned short new_val = 0 ;
    unsigned short old_val = 2 ;

    // Until we find the end of frame . . .
    while ((*(unstuffed + unstuffed_index) != 0xFFFF) && (unstuffed_index < MAX_PACKET_LEN)) {
        new_val = getBitShort((unstuffed + unstuffed_index), unstuffed_bit) ;
        bit_run_len = (new_val==old_val)?(bit_run_len+1):1 ;
        old_val = new_val ;

        if (bit_run_len < 5) {
            modifyBitShort(stuffed+stuffed_index, stuffed_bit, new_val) ;
            stuffed_bit = (stuffed_bit<15)?(stuffed_bit+1):0 ;
            stuffed_index = (stuffed_bit==0)?(stuffed_index+1):stuffed_index ;
            unstuffed_bit = (unstuffed_bit<15)?(unstuffed_bit+1):0 ;
            unstuffed_index = (unstuffed_bit==0)?(unstuffed_index+1):unstuffed_index ;

        }
        else {
            modifyBitShort(stuffed+stuffed_index, stuffed_bit, new_val) ;
            stuffed_bit = (stuffed_bit<15)?(stuffed_bit+1):0 ;
            stuffed_index = (stuffed_bit==0)?(stuffed_index+1):stuffed_index ;

            modifyBitShort(stuffed+stuffed_index, stuffed_bit, !new_val) ;
            stuffed_bit = (stuffed_bit<15)?(stuffed_bit+1):0 ;
            stuffed_index = (stuffed_bit==0)?(stuffed_index+1):stuffed_index ;
            unstuffed_bit = (unstuffed_bit<15)?(unstuffed_bit+1):0 ;
            unstuffed_index = (unstuffed_bit==0)?(unstuffed_index+1):unstuffed_index ;

            bit_run_len = 1 ;
            old_val = !new_val ;
        }
    }
    // Pack out rest of that index with zeroes
    while(stuffed_bit <= 15) {
        modifyBitShort(stuffed+stuffed_index, stuffed_bit, 0) ;
        stuffed_bit += 1 ;
    }

    // Postpend a short of all ones
    stuffed_index += 1 ;
    *(stuffed + stuffed_index) = *(unstuffed + unstuffed_index) ;
}

unsigned char get_bit(unsigned char* data, size_t bit_index) {
    size_t byte_index = bit_index / 8;
    size_t bit_offset = bit_index % 8;
    return (data[byte_index] >> (7 - bit_offset)) & 0x1;
}

void put_bit(unsigned char* data, size_t bit_index, unsigned char value) {
    size_t byte_index = bit_index / 8;
    size_t bit_offset = bit_index % 8;
    if (value) {
        data[byte_index] |= (1 << (7 - bit_offset));
    } else {
        data[byte_index] &= ~(1 << (7 - bit_offset));
    }
}

void print_bits(unsigned char* data, size_t bit_len, unsigned char* buffer_description) {
    size_t whole_bytes = bit_len / 8;
    size_t remaining_bits = bit_len % 8;
    printf("TX: %s bit length: %zu\n", buffer_description, bit_len);
    for (int i = 0; i < whole_bytes; i++) {
        printf("TX: %s byte %d: %02x\n", buffer_description, i, data[i]);
        printf("TX: %s byte %d bits: ", buffer_description, i);
        for (int j = 0; j < 8; j++) {
            printf("%d", get_bit(data, i * 8 + j));
        }
        printf("\n");
    }
    printf("TX: %s byte %d: %02x (bits %d)\n", buffer_description, whole_bytes, data[whole_bytes], remaining_bits);
    printf("TX: %s byte bits: ", buffer_description);
    for (int i = 0; i < remaining_bits; i++) {
        printf("%d", get_bit(data, whole_bytes * 8 + i));
    }
    printf("\n");
}

size_t my_bit_stuff(unsigned char* stuffed, unsigned char* unstuffed, size_t bit_len, size_t unstuffed_len) {

    //print_bits(unstuffed, bit_len, "Unstuffed");

    size_t stuffed_bit_len = 0;
    unsigned char prev_bit = 0;
    size_t bit_run = 1;
    for (int i = 0; i < bit_len; i++) {
        unsigned char next_bit = get_bit(unstuffed, i);
        if (next_bit == prev_bit) {
            bit_run++;
        } else {
            bit_run = 1;
        }
        //printf("TX: (%i) bit run = %zu\n", i, bit_run);

        put_bit(stuffed, stuffed_bit_len, next_bit);
        stuffed_bit_len++;

        if (bit_run == 5) {
            //printf("TX: stuff a bit\n");
            bit_run = 0;
            put_bit(stuffed, stuffed_bit_len, !next_bit); // Stuff the opposite bit
            stuffed_bit_len++;
        }
        prev_bit = next_bit;
    }
    //print_bits(stuffed, stuffed_bit_len, "Stuffed");
    return stuffed_bit_len;
}

size_t my_bit_unstuff(unsigned char* unstuffed, unsigned char* stuffed, size_t stuffed_bit_len) {

    print_bits(stuffed, stuffed_bit_len, "RX: Stuffed");

    size_t unstuffed_bit_len = 0;
    size_t unstuffed_target = stuffed_bit_len;
    unsigned char prev_bit = 0;
    size_t bit_run = 1;
    for (int i = 0; i < unstuffed_target; i++) {
        unsigned char next_bit = get_bit(stuffed, i);
        if (next_bit == prev_bit) {
            bit_run++;
        } else {
            bit_run = 1;
        }
        printf("RX: Bit %d: %d (bit run %zu)\n", i, next_bit, bit_run);

        if (bit_run == 5) {
            // Skip the stuffed bit
            if (get_bit(stuffed, i+1) == next_bit) {
                printf("RX: Unexpected bit stuffing at bit %d\n", i+1);
            }
            i++;
            bit_run = 1;
            printf("RX: Skipped stuffed bit at bit %d\n", i);
        }
        put_bit(unstuffed, unstuffed_bit_len, next_bit);
        printf("RX: Unstuffed bit %zu: %d\n", unstuffed_bit_len, next_bit);
        unstuffed_bit_len++;

        if (unstuffed_bit_len == 19) {
            unsigned char dlc_value = get_bit(unstuffed, 17);
            dlc_value |= get_bit(unstuffed, 16) << 1;
            dlc_value |= get_bit(unstuffed, 15) << 2;
            dlc_value |= get_bit(unstuffed, 14) << 3;

            printf("RX: DLC value: %d\n", dlc_value);
            size_t unstuffed_frame_len = 18 + dlc_value * 8 + 15;

            unstuffed_target = unstuffed_frame_len;

        }

        prev_bit = next_bit;
    }
    return unstuffed_bit_len;
}

// Assemble the unstuffed packet for transmit using the global values for
// arbitration, reserve byte, payload length, and the payload. This function
// automatically computes and appends the checksum, then appends the EOF.
void sendCBUSPacket() {

    unsigned char tx_buffer_byte[100];
    memset(tx_buffer_byte, 0, sizeof(tx_buffer_byte));

    const unsigned char can_cbus_mjPrj = 0x2;
    const unsigned char can_cbus_minPrj = 0x3;
    const unsigned char can_cbus_id = 0x2;

    unsigned short can_id = (can_cbus_mjPrj & 0x3) << 9;
    can_id |= (can_cbus_minPrj & 0x3) << 7;
    can_id |= (can_cbus_id & 0x7F);

    const unsigned char can_rtr = can_dominant_bit;

    unsigned short can_arbitration = (can_id & 0x7ff) << 1;
    can_arbitration |= (can_rtr & 0x1) ;

    tx_buffer_byte[0] = (can_arbitration & 0xFF0) >> 4;
    tx_buffer_byte[1] = (can_arbitration & 0x00F) << 4;

    printf("TX: CAN arbitration: %04x\n", can_arbitration);
    printf("TX: TX buffer bytes: %02x %02x\n", tx_buffer_byte[0], tx_buffer_byte[1]);

    const unsigned char can_control_r1 = 0x0;
    const unsigned char can_control_r0 = 0x0;
    const unsigned char can_dlc = 5;

    unsigned char can_ctrl = (can_control_r1 & 0x1) << 5;
    can_ctrl |= (can_control_r0 & 0x1) << 4;
    can_ctrl |= (can_dlc & 0xF);

    tx_buffer_byte[1] |= (can_ctrl & 0x3C) >> 2;
    tx_buffer_byte[2] = (can_ctrl & 0x03) << 6;

    printf("TX: CAN control: %02x, %d\n", can_ctrl, can_dlc);
    printf("TX: TX buffer after CAN control: %02x %02x %02x\n", tx_buffer_byte[0], tx_buffer_byte[1], tx_buffer_byte[2]);

    const unsigned char cbus_payload_len = 5;
    const unsigned char tx_cursor = 2;
    for (int i = 0; i < cbus_payload_len; i++) {
        tx_buffer_byte[tx_cursor + i] |= (cbus_payload[i] & 0xFC) >> 2;
        tx_buffer_byte[tx_cursor + i + 1] = (cbus_payload[i] & 0x03) << 6;
    }

    uint16_t checksum = 0;
    checksum = can_crc15_next(checksum, 0, 1);
    int i;
    for (i = 0; i < can_dlc + 2; i++) {
        checksum = can_crc15_next(checksum, tx_buffer_byte[i], 8);
    }
    checksum = can_crc15_next(checksum, tx_buffer_byte[i], 2);
    printf("TX: Computed checksum: %04x\n", checksum);

    uint16_t checksum_field = (checksum << 1) | (can_recessive_bit & 0x1);
    printf("TX: Checksum field: %04x\n", checksum_field);
    uint8_t checksum_high = (checksum_field & 0xFF00) >> 8;
    uint8_t checksum_low = (checksum_field & 0x00FF);

    tx_buffer_byte[can_dlc + 2] |= (checksum_high & 0xFC) >> 2;
    tx_buffer_byte[can_dlc + 3] = (checksum_high & 0x03) << 6;
    tx_buffer_byte[can_dlc + 3] |= (checksum_low & 0xFC) >> 2;
    tx_buffer_byte[can_dlc + 4] = (checksum_low & 0x03) << 6;

    unsigned char tx_stuffed[MAX_STUFFED_PACKET_LEN];
    size_t stuffed_bits = my_bit_stuff(tx_stuffed, tx_buffer_byte, (can_dlc + 4) * 8 + 1, can_dlc + 4);
    put_bit(tx_stuffed, stuffed_bits + 1, can_recessive_bit & 0x1); // crc_delim

    // ack
    put_bit(tx_stuffed, stuffed_bits + 2, can_dominant_bit & 0x1);
    put_bit(tx_stuffed, stuffed_bits + 3, can_recessive_bit & 0x1);

    // eof
    put_bit(tx_stuffed, stuffed_bits + 4, can_recessive_bit & 0x1);
    put_bit(tx_stuffed, stuffed_bits + 5, can_recessive_bit & 0x1);
    put_bit(tx_stuffed, stuffed_bits + 6, can_recessive_bit & 0x1);
    put_bit(tx_stuffed, stuffed_bits + 7, can_recessive_bit & 0x1);
    put_bit(tx_stuffed, stuffed_bits + 8, can_recessive_bit & 0x1);
    put_bit(tx_stuffed, stuffed_bits + 9, can_recessive_bit & 0x1);
    put_bit(tx_stuffed, stuffed_bits + 10, can_recessive_bit & 0x1);

//    print_bits(tx_stuffed, stuffed_bits + 10, "stuffed frame");


    unsigned char ack = can_recessive_bit & 0x1 << 1;
    ack |= can_recessive_bit & 0x1;

    // ACK
    tx_buffer_byte[can_dlc + 4] |= ack << 4;

    // End of packet marker
    tx_buffer_byte[can_dlc + 4] |= 0xF; // End of packet marker
    tx_buffer_byte[can_dlc + 5] = 0xe0;
    tx_buffer_byte[can_dlc + 5] = 0xFF; // our internal marker
    tx_buffer_byte[can_dlc + 6] = 0xff;
    tx_buffer_byte[can_dlc + 7] = 0xff;

    unsigned int bytes_loaded = (stuffed_bits + 10) / 8 + 1;


    // Print the populated packet
    printf("TX: Populated packet: ") ;
    for(i=0; i<bytes_loaded; i++){
         printf("%02x", tx_stuffed[i]) ;
    }
    printf("\n") ;


    printf("TX: stuffed packet: ") ;
    unsigned int cursor = 0;
    for (i = 0; i < bytes_loaded; i += 2) {
        unsigned short tx_blob = tx_stuffed[i];
        tx_blob <<= 8;
        tx_blob |= tx_stuffed[i+1];
        tx_packet_stuffed[cursor] = tx_blob;
        printf("%04x", tx_packet_stuffed[cursor]);
        cursor++;
    }
    printf("\n") ;

    // Bit stuff the packet
   // bitStuff(tx_packet_unstuffed, tx_packet_stuffed) ;

    // Print packet after stuffing
    //printf("TX Stuffed packet:\n") ;
    //for (i=0; i<((payload_len>>1)+6); i++){
    //    printf("%04x", tx_packet_stuffed[i]) ;
    //}
    //printf("\n") ;

    printf("TX: stuffed bytes:\n");
    for (i = 0; i < bytes_loaded; i++) {
        printf("%02x", tx_stuffed[i]) ;
    }
    printf("\n") ;
    // BEGIN TRANSMISSION
    dma_start_channel_mask((1u << dma_chan_0)) ;
}


// Assemble the unstuffed packet for transmit using the global values for
// arbitration, reserve byte, payload length, and the payload. This function
// automatically computes and appends the checksum, then appends the EOF.
void sendPacket() {
    // Incrementer
    int i ;
    // Load arbitration
    tx_packet_unstuffed[0] = arbitration ;
    // Load reserve byte and payload length
    tx_packet_unstuffed[1] =  (((((unsigned short)reserve_byte)<<8) & 0xFF00) |
                             (((unsigned short)payload_len) & 0x00FF));
    // Load payload
    memcpy(&tx_packet_unstuffed[2], &payload[0], payload_len) ;
    // Compute checksum
    unsigned short checksum = CRC_INIT; // Init value for CRC calculation
    while (checksum == 0xFFFF) {
        tx_packet_unstuffed[1] ^= 0x8000 ;
        for (i = 0; i < ((payload_len>>1)+2); i++) {
          checksum = culCalcCRC((tx_packet_unstuffed[i]>>8)&0xFF, checksum);
          checksum = culCalcCRC((tx_packet_unstuffed[i])&0xFF, checksum);
        }
    }

    // Load checksum
    tx_packet_unstuffed[i] = checksum ;
    // Load EOF
    tx_packet_unstuffed[i+1] = 0xFFFF ;

    // // Print the populated packet
    // printf("\nPopulated packet\n") ;
    // for(i=0; i<((payload_len>>1)+6); i++){
    //     printf("%04x", tx_packet_unstuffed[i]) ;
    // }
    // printf("\n") ;

    // Bit stuff the packet
    bitStuff(tx_packet_unstuffed, tx_packet_stuffed) ;

    // // Print packet after stuffing
    // printf("Stuffed packet:\n") ;
    // for (i=0; i<((payload_len>>1)+6); i++){
    //     printf("%04x", tx_packet_stuffed[i]) ;
    // }
    // printf("\n") ;

    // BEGIN TRANSMISSION
    dma_start_channel_mask((1u << dma_chan_0)) ;
}


//                   FUNCTIONS USED FOR PACKET RECEPTION
//
// Helper function which access a particular bit in an unsigned char.
// It returns that bit (0 or 1) as an unsigned char.
unsigned char getBitChar(unsigned char * byte, unsigned char bitnum) {
    switch(bitnum) {
        case 0:
            return ((*byte & 0x80) >> 7) ;
        case 1:
            return ((*byte & 0x40) >> 6) ;
        case 2:
            return ((*byte & 0x20) >> 5) ;
        case 3:
            return ((*byte & 0x10) >> 4) ;
        case 4:
            return ((*byte & 0x08) >> 3) ;
        case 5:
            return ((*byte & 0x04) >> 2) ;
        case 6:
            return ((*byte & 0x02) >> 1) ;
        case 7:
            return ((*byte & 0x01) >> 0) ;
        default:
            return 0xFF ;
    }
}
// Helper function which modifies a single bit in an unsigned char. The first argument
// is a pointer to the char that will be modified. The second is the bit number (0-7) to
// modify. And the third is the value (0 or 1) to which we would like it modified.
void modifyBitChar(unsigned char * byte, unsigned char bitnum, unsigned char value) {
    switch(bitnum) {
        case 7:
            *byte |= ((value << 0) & 0x01) ;
            break ;
        case 6:
            *byte |= ((value << 1) & 0x02) ;
            break ;
        case 5:
            *byte |= ((value << 2) & 0x04) ;
            break ;
        case 4:
            *byte |= ((value << 3) & 0x08) ;
            break ;
        case 3:
            *byte |= ((value << 4) & 0x10) ;
            break ;
        case 2:
            *byte |= ((value << 5) & 0x20) ;
            break ;
        case 1:
            *byte |= ((value << 6) & 0x40) ;
            break ;
        case 0:
            *byte |= ((value << 7) & 0x80) ;
            break ;
        default:
            printf("Invalid argument to modifyBitChar   \n") ;
            break ;
    }
}
// Function which takes a pointer to a stuffed character array,
// and a pointer to an array where we would like the unstuffed
// data to be stored. Function unstuffs the first array and stores
// the result in the second.
//
// Why not do an in-place replacement? I think that it will be nice
// to start gathering the next stuffed buffer while doing work on the
// last one.
void unBitStuff(unsigned char * stuffed, unsigned char * unstuffed) {
    // Clear the buffer
    memcpy(&unstuffed[0], &zero_packet[0], MAX_STUFFED_PACKET_LEN) ;

    // Variables for monitoring position in each buffer
    int stuffed_index   = 0 ;
    int unstuffed_index = 0 ;
    int stuffed_bit     = 0 ;
    int unstuffed_bit   = 0 ;

    // Accumulated bit run length
    int bit_run_len     = 0 ;

    // Memory of old bit value
    unsigned char new_val = 0 ;
    unsigned char old_val = 2 ;

    // Until we find the end of frame . . .
    while ((*(stuffed + stuffed_index) != 0xFF) && (stuffed_index < (MAX_STUFFED_PACKET_LEN))) {
        // Get a new bit, update the bit run length, and update the bit memory
        new_val = getBitChar((stuffed+stuffed_index), stuffed_bit) ;
        bit_run_len = (new_val==old_val)?(bit_run_len+1):1 ;
        old_val = new_val ;

        // If our bit run length is less than 5, update the unstuffed buffer
        // and increment position in each buffer.
        if (bit_run_len < 5) {

            modifyBitChar(unstuffed+unstuffed_index, unstuffed_bit, new_val) ;

            unstuffed_bit = (unstuffed_bit<7)?(unstuffed_bit+1):0 ;
            unstuffed_index = (unstuffed_bit==0)?(unstuffed_index+1):unstuffed_index ;

            stuffed_bit = (stuffed_bit<7)?(stuffed_bit+1):0 ;
            stuffed_index = (stuffed_bit==0)?(stuffed_index+1):stuffed_index ;

        }

        // If our bit run length is 5, update the unstuffed buffer. Then
        // increment the unstuffed buffer by ONE and the stuffed buffer by TWO
        // in order to skip over the stuff bit.
        else {
            modifyBitChar(unstuffed+unstuffed_index, unstuffed_bit, new_val) ;
            unstuffed_bit = (unstuffed_bit<7)?(unstuffed_bit+1):0 ;
            unstuffed_index = (unstuffed_bit==0)?(unstuffed_index+1):unstuffed_index ;

            stuffed_bit = (stuffed_bit<7)?(stuffed_bit+1):0 ;
            stuffed_index = (stuffed_bit==0)?(stuffed_index+1):stuffed_index ;
            stuffed_bit = (stuffed_bit<7)?(stuffed_bit+1):0 ;
            stuffed_index = (stuffed_bit==0)?(stuffed_index+1):stuffed_index ;

            // Reset bit run length
            bit_run_len = 1 ;
            // We jumped over a stuffed bit, opposite polarity to
            // the last bit that we measured
            old_val = !new_val ;
        }
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

// Function assumes that a stuffed packet lives in rx_packet_stuffed.
// It unpacks that packet, checks the arbitration bits, and checks the
// checksum. If it is a valid packet (correct arbitration and checksum)
// then the function returns 1. Else it returns 0. Valid packet will
// remain in rx_packet_unstuffed for user to access.
unsigned char attemptPacketReceive() {
    // // Print packet received stuffed packet
    int i ;
    // printf("Received (stuffed) packet:\n") ;
    // for (i=0; i<22; i++){
    //     printf("%02x", rx_packet_stuffed[i]) ;
    // }
    // printf("\n") ;

    // Unstuff the received packet
    unBitStuff(rx_packet_stuffed, rx_packet_unstuffed) ;

    my_bit_unstuff(rx_packet_unstuffed, rx_packet_stuffed, 18+15);

    // Print the packet after unstuffing
    printf("RX: Packet after unstuffing: ") ;
    for (i=0; i<22; i++){
        printf("%02x", rx_packet_unstuffed[i]) ;
    }
    printf("\n") ;

    // print MERG CAN/CBUS decode
    uint16_t can_arbitration = rx_packet_unstuffed[0] << 8 | rx_packet_unstuffed[1];
    can_arbitration >>= 4;
    printf("RX: Decoded CAN arbitration: 0x%03x\n", can_arbitration);

    uint16_t can_identifier = can_arbitration >> 1;
    uint8_t can_rtr = can_arbitration & 0x1;

    printf("RX: Decoded CAN identifier: 0x%03x\n", can_identifier);
    printf("RX: Decoded CAN RTR: %d\n", can_rtr);

    uint8_t can_control_r1 = (rx_packet_unstuffed[1] & 0x20) >> 5;
    uint8_t can_control_r0 = (rx_packet_unstuffed[1] & 0x10) >> 4;
    if (can_control_r1 != can_dominant_bit || can_control_r0 != can_dominant_bit) {
        printf("RX: Unexpected CAN Control bits! (R1: %d, R0: %d)\n", can_control_r1, can_control_r0);
        return 0;
    }

    uint8_t dlcHn = rx_packet_unstuffed[1] & 0x03;
    uint8_t dlcLn = (rx_packet_unstuffed[2] & 0xC0) >> 6;
    uint8_t dlc = (dlcHn << 2) | dlcLn;
    printf("RX: Decoded CAN Control DLC: %d\n", dlc);

    uint8_t can_data_n_crcfield[10];
    for (i = 0; i < dlc + 2; i++) {
        can_data_n_crcfield[i] = (rx_packet_unstuffed[i+2] & 0x3F) << 2;
        can_data_n_crcfield[i] |= (rx_packet_unstuffed[i+3] & 0xC0) >> 6;
    }

    uint8_t crchi = can_data_n_crcfield[i-2];
    uint8_t crclo = can_data_n_crcfield[i-1];
    uint16_t crc = (crchi << 8) | crclo;
    uint8_t crc_delim = crc & 0x1;
    if (crc_delim != can_recessive_bit) {
        printf("RX: Unexpected CRC delimiter bit!\n");
        return 0;
    }

    crc  = crc >> 1;
    printf("RX: Received checksum: %04x\n", crc);

    uint16_t checsum = 0;
    checsum = can_crc15_next(checsum, 0, 1);
    for (i = 0; i < dlc + 2; i++) {
        checsum = can_crc15_next(checsum, rx_packet_unstuffed[i], 8);
    }
    checsum = can_crc15_next(checsum, rx_packet_unstuffed[i], 2);
    printf("RX: Computed checksum: %04x\n", checsum);

    if (checsum != crc) {
        printf("RX: Checksum mismatch!\n");
        return 0;
    }
    
    uint8_t* can_data_field = NULL;
    if (dlc > 0) {
        can_data_field = &can_data_n_crcfield[0];
        printf("RX: CAN Data field: ");
        for (i = 0; i < dlc; i++) {
            printf("%02x ", can_data_field[i]);
        }
        printf("\n");
    }

    // MERG CAN ID for CBUS
    uint8_t cbus_MjPri = (can_arbitration & 0xC00) >> 10; 
    uint8_t cbus_MinPri = (can_arbitration & 0x300) >> 8;
    uint8_t cbus_id = (can_arbitration & 0xFE) >> 1;
    printf("RX: Decoded CBUS CAN: MjPrj: 0x%01X MinPrj: 0x%01X ID: 0x%02X (%d)\n", cbus_MjPri, cbus_MinPri, cbus_id, cbus_id);

    uint8_t* cbus_message = &can_data_field[0];
    uint8_t cbus_len = (cbus_message[0] & 0xe0) >> 5;
    if (cbus_len != (dlc - 1)) {
        printf("RX: Invalid CBUS length: expected %d, got %d\n", dlc - 1, cbus_len);
    }
    printf("RX: Decoded CBUS length: %d\n", cbus_len);

    uint8_t cbus_op = cbus_message[0];
    uint16_t cbus_node = (cbus_message[1] << 8) | cbus_message[2];
    uint16_t cbus_event = (cbus_message[3] << 8) | cbus_message[4];

    printf("RX: Decoded CBUS operation: OP 0x%02x, Node: %d, Event: %d\n", cbus_op, cbus_node, cbus_event);


    return 1;
}




//                        DRIVER INTERRUPT SERVICE ROUTINES
//
// In the event of an overrun on the RX DMA channel (happens when a new node
// joins the network), this ISR resets the DMA channel
void dma_handler() {
    // Clear the interrupt request
    dma_hw->ints0 = 1u << dma_chan_1;
    // Reset the DMA channel write address, and start the channel
    dma_channel_set_write_addr(dma_chan_1, rx_packet_stuffed_pointer, true) ;
}


//                  CAN SETUP FUNCTIONS. CALL ON CORE WHERE YOU WANT TO RUN
//
void setupIdleCheck() {

    // Load PIO program onto PIO0
    uint can_idle_offset = pio_add_program(pio_0, &idle_check_program) ;

    // Initialize the PIO program
    idle_check_program_init(pio_0, can_idle_check_sm, can_idle_offset, CAN_TX+1, CLKDIV) ;

    // Zero the irq 1
    pio_interrupt_clear(pio_0, 1) ;

    // Start the PIO program
    pio_sm_set_enabled(pio_0, can_idle_check_sm, true) ;

    // Channel Two (sends data to TX PIO machine)
    dma_channel_config c2 = dma_channel_get_default_config(dma_chan_2);
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_32);
    channel_config_set_read_increment(&c2, false);
    channel_config_set_write_increment(&c2, false);
    channel_config_set_dreq(&c2, DREQ_PIO0_TX1) ;
    channel_config_set_chain_to(&c2, dma_chan_3);

    dma_channel_configure(
        dma_chan_2,                     // Channel to be configured
        &c2,                            // The configuration we just created
        &pio_0->txf[can_idle_check_sm], // write address (idle check PIO TX FIFO)
        &tx_idle_time,                  // read address (variable containing idle time)
        1,                              // Number of transfers (aborts early!)
        false                           // Don't start immediately.
    );

    // Channel Three (resets channel 2)
    dma_channel_config c3 = dma_channel_get_default_config(dma_chan_3);
    channel_config_set_transfer_data_size(&c3, DMA_SIZE_32);
    channel_config_set_read_increment(&c3, false);
    channel_config_set_write_increment(&c3, false);
    channel_config_set_chain_to(&c3, dma_chan_2);

    dma_channel_configure(
        dma_chan_3,                     // Channel to be configured
        &c3,                            // The configuration we just created
        &dummy_dest,                    // write address (dummy)
        &dummy_source,                  // read address (dummy)
        1,                              // Number of transfers
        true                            // Start immediately.
    );
}
//
// Function which sets up CAN TX machine. Adds and initializes the PIO program
// to PIO 0, associates the interrupt handler with irq 0, and configures the DMA
// channel for moving data to the transmit PIO.
void setupCANTX(irq_handler_t handler) {

    // Power off transciever (avoids transients on bus)
    gpio_init(TRANSCIEVER_SLNT) ;
    gpio_set_dir(TRANSCIEVER_SLNT, GPIO_OUT) ;
    gpio_put(TRANSCIEVER_SLNT, 1) ;

    // Setup the idle checking system
    setupIdleCheck() ;

    // Load PIO programs onto PIO0
    uint can_tx_offset = pio_add_program(pio_0, &can_tx_program) ;

    // Initialize the PIO program
    can_tx_program_init(pio_0, can_tx_sm, can_tx_offset, CAN_TX, CLKDIV) ;

    // Setup interrupts for TX machine
    pio_interrupt_clear(pio_0, 0) ;
    pio_set_irq0_source_enabled(pio_0, pis_interrupt0, true) ;
    irq_set_exclusive_handler(PIO0_IRQ_0, handler) ;
    irq_set_enabled(PIO0_IRQ_0, true) ;

    // Channel Zero (sends data to TX PIO machine)
    dma_channel_config c0 = dma_channel_get_default_config(dma_chan_0);
    channel_config_set_transfer_data_size(&c0, DMA_SIZE_16);
    channel_config_set_read_increment(&c0, true);
    channel_config_set_write_increment(&c0, false);
    channel_config_set_dreq(&c0, DREQ_PIO0_TX0) ;

    dma_channel_configure(
        dma_chan_0,                     // Channel to be configured
        &c0,                            // The configuration we just created
        &pio_0->txf[can_tx_sm],         // write address (transmit PIO TX FIFO)
        tx_packet_stuffed_pointer,      // read address (start of stuffed packet)
        sizeof(tx_packet_stuffed)>>1,   // Number of transfers (aborts early!)
        false                           // Don't start immediately.
    );

    // Start the TX PIO program (sets output high, among other things)
    pio_sm_set_enabled(pio_0, can_tx_sm, true) ;

    // Brief delay to allow GPIO to stabilize
    sleep_ms(1) ;

    // Power on transciever
    gpio_put(TRANSCIEVER_SLNT, 0) ;
}

// Function which sets up CAN RX machine. Adds and initializes the PIO program
// to PIO 1, associates the interrupt handler with irq 0, and configures the DMA
// channel for moving data from the receive PIO.
void setupCANRX(irq_handler_t handler) {

    // Load pio program onto PIO 1
    uint can_rx_offset = pio_add_program(pio_1, &can_rx_program) ;

    // Initialize the PIO programs
    can_rx_program_init(pio_1, can_rx_sm, can_rx_offset, CAN_TX+1, CLKDIV) ;

    // Setup interrupts for RX machine
    pio_interrupt_clear(pio_1, 0) ;
    pio_set_irq0_source_enabled(pio_1, pis_interrupt0, true) ;
    irq_set_exclusive_handler(PIO1_IRQ_0, handler) ;
    irq_set_enabled(PIO1_IRQ_0, true) ;

    // Channel One (gets data from RX PIO machine)
    dma_channel_config c1 = dma_channel_get_default_config(dma_chan_1);
    channel_config_set_transfer_data_size(&c1, DMA_SIZE_8);
    channel_config_set_read_increment(&c1, false);
    channel_config_set_write_increment(&c1, true);
    channel_config_set_dreq(&c1, DREQ_PIO1_RX0) ;

    dma_channel_configure(
        dma_chan_1,                 // Channel to be configured
        &c1,                        // The configuration we just created
        rx_packet_stuffed_pointer,  // write address (receive buffer)
        &pio_1->rxf[can_rx_sm],     // read address (receive PIO RX FIFO)
        sizeof(rx_packet_stuffed),  // Number of transfers (aborts early!!)
        false                       // Don't start immediately.
    );
  
    // Tell DMA to rasie IRQ line 0 when channel 1 finished a block
    dma_channel_set_irq0_enabled(dma_chan_1, true);

    // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // Start the RX PIO machine
    pio_sm_set_enabled(pio_1, can_rx_sm, true) ;

    // Start the RX DMA channel
    dma_start_channel_mask((1u << dma_chan_1)) ;

}

//                              API HELPER FUNCTIONS
//
// Call in the tx_handler interrupt service routine to reset the transmitter
static inline void resetTransmitter() {
    // Abort the DMA channel sending data to the TX PIO (EOF found)
    dma_channel_abort(dma_chan_0) ;
    // Drain the TX FIFO
    pio_sm_drain_tx_fifo(pio_0, can_tx_sm) ;
    // Unstall the PIO state machine
    pio_interrupt_clear(pio_0, 0) ;
    // Reset the DMA channel read address, don't start channel yet
    dma_channel_set_read_addr(dma_chan_0, tx_packet_stuffed_pointer, false) ;
}

// Call in the rx_handler interrupt service routing to reset the receiver
static inline void resetReceiver() {
    // Full message received, abort DMA channel 2
    // disable the channel on IRQ0
    dma_channel_set_irq0_enabled(dma_chan_1, false);
    // abort the channel
    dma_channel_abort(dma_chan_1);
    // clear the spurious IRQ (if there was one)
    dma_channel_acknowledge_irq0(dma_chan_1);
    // re-enable the channel on IRQ0
    dma_channel_set_irq0_enabled(dma_chan_1, true);
    // Reset the DMA channel write address, and start the channel
    dma_channel_set_write_addr(dma_chan_1, rx_packet_stuffed_pointer, true) ;
}

// At end of receive ISR, clear interrupt to accept new packets
static inline void acceptNewPacket() {
    pio_interrupt_clear(pio_1, 0) ;
}




