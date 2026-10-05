/**
 * V. Hunter Adams (vha3@cornell.edu)
 * 
 * CAN parameters
 * 
 */

//                             INTERFACE PARAMETERS
//
// GPIO pins
#define CAN_TX           2
#define LED_PIN          25
#define TRANSCIEVER_SLNT 4


//                                CAN PARAMETERS
//
// Size of TX buffer
#define MAX_PAYLOAD_SIZE        16
#define MAX_PACKET_LEN          MAX_PAYLOAD_SIZE+8
#define MAX_STUFFED_PACKET_LEN  MAX_PACKET_LEN+(MAX_PACKET_LEN>>1)
// My own identity, and a broadcast value
#define MY_ARBITRATION_VALUE    0xb041
#define NETWORK_BROADCAST       0x5555
// Time to wait (in bit times) for bus to be idle before tx. Dynamically modifiable.
unsigned int tx_idle_time = 500 ;


//            PACKET INFORMATION WHICH WILL BE MODIFIED AT RUNTIME
//
// Start of frame (1 bit), arbitration (12 bits), stuffer buffer (3 bits).
// This specifies the destination for the packet. 
unsigned short arbitration = 0x4234 ;
// Reserve byte (data request? routing?)
unsigned char reserve_byte = 0x55 ;
// Payload length in bytes (should be even)
unsigned char payload_len  = 10 ;
// The payload (modified at runtime)
unsigned short payload[MAX_PAYLOAD_SIZE] = {0x1335, 0x5678, 0x9012,
                                            0x3456, 0x7890};

#define max_merg_payload 8
unsigned char cbus_payload[max_merg_payload] = {0} ;


//                           BUFFER FOR RECEIVED DATA
//
unsigned char rx_packet_unstuffed[MAX_PACKET_LEN] = {0} ;