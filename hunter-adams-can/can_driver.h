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

struct can_data_frame {
    uint16_t id;
    unsigned char data_field[8];
    uint8_t data_len;
};

bool decode_can_packet(const unsigned char* rx_packet_stuffed, unsigned char* rx_packet_unstuffed, struct can_data_frame* decoded_packet);


//                 FUNCTIONS USED FOR PACKET TRANSMISSION
//
unsigned char get_bit(const unsigned char* data, size_t bit_index) {
    size_t byte_index = bit_index / 8;
    size_t bit_offset = bit_index % 8;
    return (data[byte_index] >> (7 - bit_offset)) & 0x1;
}

uint32_t get_bits(const unsigned char* data, size_t bit_index, size_t bit_len) {
    assert(bit_len <= 32);
    uint32_t value = 0;
    for (size_t i = 0; i < bit_len; i++) {
        value <<= 1;
        value |= get_bit(data, bit_index + i);
    }
    return value;
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

void put_bits(unsigned char* data, size_t bit_index, uint32_t value, size_t bit_len) {
    for (size_t i = 0; i < bit_len; i++) {
        put_bit(data, bit_index + i, (value >> (bit_len - 1 - i)) & 0x1);
    }
}

void print_bits(const unsigned char* data, size_t bit_len, const unsigned char* buffer_description) {
    size_t whole_bytes = bit_len / 8;
    size_t remaining_bits = bit_len % 8;
    printf("TX: %s bit length: %zu\n", buffer_description, bit_len);
    for (int i = 0; i < whole_bytes; i++) {
        printf("TX: %s byte %2d: %02x  bits (%3d, %3d): ", buffer_description, i, data[i], i * 8, i * 8 + 7);
        for (int j = 0; j < 8; j++) {
            printf("%d", get_bit(data, i * 8 + j));
        }
        printf("\n");
    }
    if (remaining_bits > 0) {
        printf("TX: %s byte %2d: %02x bits (%3d, %3d): ", buffer_description, whole_bytes, data[whole_bytes], whole_bytes * 8, whole_bytes * 8 + remaining_bits - 1);
        for (int i = 0; i < remaining_bits; i++) {
            printf("%d", get_bit(data, whole_bytes * 8 + i));
        }
        printf("\n");
    }
}

size_t unstuff_frame(unsigned char* unstuffed, const unsigned char* stuffed, size_t stuffed_bit_len) {

    size_t unstuffed_bit_len = 0;
    size_t unstuffed_target = stuffed_bit_len;
    unsigned char prev_bit = 0;
    size_t bit_run = 1;
    size_t stuffed_cursor = 0;
    size_t stuffing_removed = 0;
    uint8_t received_dlc = 0;

    while (stuffed_cursor < unstuffed_target) {
        unsigned char next_bit = get_bit(stuffed, stuffed_cursor);
        put_bit(unstuffed, unstuffed_bit_len, next_bit);
        unstuffed_bit_len++;

        if (next_bit == prev_bit) {
            bit_run++;
        } else {
            bit_run = 1;
        }

        if (bit_run == 5) {
            // Skip the stuffed bit
            if (get_bit(stuffed, stuffed_cursor+1) == next_bit) {
                // todo: signal an error or handle unexpected bit stuffing
                printf("RX: Unexpected bit stuffing at bit %zu\n", stuffed_cursor+1);
            }
            stuffed_cursor++;
            stuffing_removed++;
            bit_run = 0;
        }

        if (unstuffed_bit_len == 15) {
            received_dlc |= (next_bit << 3);
        }
        if (unstuffed_bit_len == 16) {
            received_dlc |= (next_bit << 2);
        }
        if (unstuffed_bit_len == 17) {
            received_dlc |= (next_bit << 1);
        }
        if (unstuffed_bit_len == 18) {
            received_dlc |= next_bit;
        }

        if (unstuffed_bit_len == 18) {
            printf("RX: Received DLC: %d\n", received_dlc);
            size_t unstuffed_frame_len = 18 + received_dlc * 8 + 15;

            unstuffed_target = unstuffed_frame_len;
        }

        prev_bit = next_bit;
        stuffed_cursor++;
    }

    assert(unstuffed_bit_len == stuffed_cursor - stuffing_removed);

    // it seems the pio buffer doesn't have sensible values for most of this
    uint8_t crc_delim = get_bits(stuffed, stuffed_cursor, 1);
    stuffed_cursor += 1;
    put_bits(unstuffed, unstuffed_bit_len, crc_delim, 1);
    unstuffed_bit_len += 1;

    uint8_t ack_bits = get_bits(stuffed, stuffed_cursor, 2);
    stuffed_cursor += 2;
    put_bits(unstuffed, unstuffed_bit_len, ack_bits, 2);
    unstuffed_bit_len += 2;
    
    uint8_t eof_bits = get_bits(stuffed, stuffed_cursor, 7);
    stuffed_cursor += 7;
    put_bits(unstuffed, unstuffed_bit_len, eof_bits, 7);
    unstuffed_bit_len += 7;

    return unstuffed_bit_len;
}

size_t stuff_frame(unsigned char* stuffed, unsigned char* unstuffed, size_t unstuffed_bit_len) {
    size_t transmit_bits = 0;
    uint8_t prev_bit = can_recessive_bit;
    size_t bit_run = 1;

    for (size_t i = 0; i < unstuffed_bit_len; i++) {
        uint8_t next_bit = get_bit(unstuffed, i);
        if (next_bit == prev_bit) {
            bit_run++;
        } else {
            bit_run = 1;
        }
        put_bit(stuffed, transmit_bits, next_bit);
        transmit_bits++;
        if (bit_run == 5) {
            // Insert a stuffed bit
            put_bit(stuffed, transmit_bits, !next_bit);
            transmit_bits++;
            bit_run = 0;
        }
        prev_bit = next_bit;
    }
    return transmit_bits;
}

// Assemble the unstuffed packet for transmit using the global values for
// arbitration, reserve byte, payload length, and the payload. This function
// automatically computes and appends the checksum, then appends the EOF.
void sendCBUSPacket() {

    unsigned char tx_decode_buffer[MAX_STUFFED_PACKET_LEN];

    unsigned char tx_buffer_byte[100];
    memset(tx_buffer_byte, 0, sizeof(tx_buffer_byte));
    size_t tx_cursor = 0;

    const unsigned char can_cbus_mjPrj = 0x2;
    const unsigned char can_cbus_minPrj = 0x3;
    const unsigned char can_cbus_id = 0x2;

    unsigned short can_id = (can_cbus_mjPrj & 0x3) << 9;
    can_id |= (can_cbus_minPrj & 0x3) << 7;
    can_id |= (can_cbus_id & 0x7F);

    put_bits(tx_buffer_byte, tx_cursor, can_id, 11);
    tx_cursor += 11;

    const unsigned char can_rtr = can_dominant_bit;

    put_bit(tx_buffer_byte, tx_cursor, can_rtr);
    tx_cursor++;

    // control r1
    put_bit(tx_buffer_byte, tx_cursor, can_dominant_bit);
    tx_cursor++;

    // control r0
    put_bit(tx_buffer_byte, tx_cursor, can_dominant_bit);
    tx_cursor++;

    const unsigned char can_dlc = 5;
    put_bits(tx_buffer_byte, tx_cursor, can_dlc, 4);
    tx_cursor += 4;

    const unsigned char cbus_payload_len = 5;
    for (int i = 0; i < cbus_payload_len; i++) {
        put_bits(tx_buffer_byte, tx_cursor + i * 8, cbus_payload[i], 8);
    }
    tx_cursor += cbus_payload_len * 8;

    uint16_t checksum = 0;
    checksum = can_crc15_next(checksum, 0, 1);
    int i;
    for (i = 0; i < can_dlc + 2; i++) {
        checksum = can_crc15_next(checksum, tx_buffer_byte[i], 8);
    }
    checksum = can_crc15_next(checksum, tx_buffer_byte[i], 2);
    printf("TX: Computed checksum: %04x\n", checksum);

    put_bits(tx_buffer_byte, tx_cursor, checksum, 15);
    tx_cursor += 15;

    unsigned char tx_stuffed[MAX_STUFFED_PACKET_LEN];
    size_t stuffed_bits = stuff_frame(tx_stuffed, tx_buffer_byte, tx_cursor);


    // crc delimiter
    put_bit(tx_stuffed, stuffed_bits, can_recessive_bit);
    stuffed_bits++;

    // ack slot
    put_bit(tx_stuffed, stuffed_bits++, can_recessive_bit);
    
    
    // ack delimiter
    put_bit(tx_stuffed, stuffed_bits++, can_recessive_bit);

    // eof
    put_bits(tx_stuffed, stuffed_bits, 0x7f, 7);
    stuffed_bits += 7;



    print_bits(tx_stuffed, stuffed_bits, "stuffed frame");

    decode_can_packet(tx_stuffed, tx_decode_buffer, NULL);

    // BEGIN TRANSMISSION
    dma_start_channel_mask((1u << dma_chan_0)) ;
}


//                   FUNCTIONS USED FOR PACKET RECEPTION
//

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


void decode_merg_can_packet(const struct can_data_frame* packet) {
    
    // MERG CAN ID for CBUS
    uint8_t cbus_MjPri = (packet->id & 0x600) >> 9; 
    uint8_t cbus_MinPri = (packet->id & 0x180) >> 7;
    uint8_t cbus_id = (packet->id & 0x7f);
    printf("RX: Decoded CBUS CAN: MjPrj: 0x%01X MinPrj: 0x%01X ID: 0x%02X (%d)\n", cbus_MjPri, cbus_MinPri, cbus_id, cbus_id);

    uint8_t cbus_len = (packet->data_field[0] & 0xe0) >> 5;
    if (cbus_len != (packet->data_len - 1)) {
        printf("RX: Invalid CBUS length: expected %d, got %d\n", packet->data_len - 1, cbus_len);
    }
    printf("RX: Decoded CBUS length: %d\n", cbus_len);

    uint8_t cbus_op = packet->data_field[0];
    uint16_t cbus_node = (packet->data_field[1] << 8) | packet->data_field[2];
    uint16_t cbus_event = (packet->data_field[3] << 8) | packet->data_field[4];

    printf("RX: Decoded CBUS operation: OP 0x%02x, Node: %d, Event: %d\n", cbus_op, cbus_node, cbus_event);
}

bool decode_can_packet(const unsigned char* rx_packet_stuffed, unsigned char* rx_packet_unstuffed, struct can_data_frame* decoded_packet) {
    
    // Print packet received stuffed packet
    print_bits(rx_packet_stuffed, 22*8, "RX: Stuffed packet: ");

    size_t unstuffed_bit_len = unstuff_frame(rx_packet_unstuffed, rx_packet_stuffed, 22*8);

    // Print the packet after unstuffing
    print_bits(rx_packet_unstuffed, unstuffed_bit_len, "RX: Unstuffed packet: ");

    uint16_t can_arbitration = get_bits(rx_packet_unstuffed, 0, 12);
    printf("RX: Decoded CAN arbitration: 0x%03x\n", can_arbitration);

    uint16_t can_identifier = get_bits(rx_packet_unstuffed, 0, 11);
    uint8_t can_rtr = get_bits(rx_packet_unstuffed, 11, 1);

    printf("RX: Decoded CAN identifier: 0x%03x\n", can_identifier);
    printf("RX: Decoded CAN RTR: %d\n", can_rtr);

    uint8_t can_control_r1 = get_bits(rx_packet_unstuffed, 12, 1);
    uint8_t can_control_r0 = get_bits(rx_packet_unstuffed, 13, 1);
    if (can_control_r1 != can_dominant_bit || can_control_r0 != can_dominant_bit) {
        printf("RX: Unexpected CAN Control bits! (R1: %d, R0: %d)\n", can_control_r1, can_control_r0);
    }

    uint8_t dlc = get_bits(rx_packet_unstuffed, 14, 4);
    printf("RX: Decoded CAN Control DLC: %d\n", dlc);

    unsigned char can_data_field[8];
    uint8_t can_data_len = dlc;
    for (size_t i = 0; i < can_data_len; i++) {
        can_data_field[i] = get_bits(rx_packet_unstuffed, 18 + i*8, 8);
        printf("RX: CAN Data byte %d: 0x%02x\n", i, can_data_field[i]);

    }

    uint16_t can_crc = get_bits(rx_packet_unstuffed, 18 + can_data_len*8, 15);
    printf("RX: Decoded CAN CRC: 0x%04x\n", can_crc);

    uint8_t can_crc_delim = get_bits(rx_packet_unstuffed, 18 + can_data_len*8 + 15, 1);
    if (can_crc_delim != can_recessive_bit) {
        printf("RX: Unexpected CAN CRC delimiter bit!\n");
        return false;
    }

    uint8_t ack_slot = get_bits(rx_packet_unstuffed, 18 + can_data_len*8 + 15 + 1, 1);
    printf("RX: Decoded CAN ACK slot: %d\n", ack_slot);
    
    uint8_t ack_delim = get_bits(rx_packet_unstuffed, 18 + can_data_len*8 + 15 + 1 + 1, 1);
    if (ack_delim != can_recessive_bit) {
        printf("RX: Unexpected CAN ACK delimiter bit!\n");
    }

    uint8_t eof = get_bits(rx_packet_unstuffed, 18 + can_data_len*8 + 15 + 1 + 1 + 1, 7);
    if (eof != 0x7F) {
        printf("RX: Unexpected CAN EOF bits!\n");
    }

    printf("RX: Received checksum: %04x\n", can_crc);

    uint16_t checsum = 0;
    checsum = can_crc15_next(checsum, 0, 1);
    size_t crc_cursor = 0;
    for (; crc_cursor < dlc + 2; crc_cursor++) {
        checsum = can_crc15_next(checsum, rx_packet_unstuffed[crc_cursor], 8);
    }
    checsum = can_crc15_next(checsum, rx_packet_unstuffed[crc_cursor], 2);
    printf("RX: Computed checksum: %04x\n", checsum);

    if (checsum != can_crc) {
        printf("RX: Checksum mismatch!\n");
        return false;
    }

    if (decoded_packet) {
        decoded_packet->id = can_identifier; 
        for (size_t i = 0; i < can_data_len; i++) {
            decoded_packet->data_field[i] = can_data_field[i];
        }
        decoded_packet->data_len = can_data_len;
    }
    

    return true;
}

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

// Function assumes that a stuffed packet lives in rx_packet_stuffed.
// It unpacks that packet, checks the arbitration bits, and checks the
// checksum. If it is a valid packet (correct arbitration and checksum)
// then the function returns 1. Else it returns 0. Valid packet will
// remain in rx_packet_unstuffed for user to access.
unsigned char attemptPacketReceive() {
    // Print packet received stuffed packet
    print_bits(rx_packet_stuffed, 22*8, "RX: Stuffed packet: ");

    struct can_data_frame decoded_packet;

    if (decode_can_packet(rx_packet_stuffed, rx_packet_unstuffed, &decoded_packet)) {
        decode_merg_can_packet(&decoded_packet);

        return 1;
    }
    return 0;
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




