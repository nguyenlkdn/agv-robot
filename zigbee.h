/*
 * zigbee.h
 *
 *  Created on: Oct 4, 2017
 *      Author: sdev
 */

#ifndef ZIGBEE_H_
#define ZIGBEE_H_
#include <stdint.h>
#include <stdbool.h>
#define SLAVER_1 1
//#define MATER
/*
@deftypefn Extension {unsigned int} crc32 (const unsigned char *@var{buf}, @
  int @var{len}, unsigned int @var{init})
Compute the 32-bit CRC of @var{buf} which has length @var{len}.  The
starting value is @var{init}; this may be used to compute the CRC of
data split across multiple buffers by passing the return value of each
call as the @var{init} parameter of the next.
This is used by the @command{gdb} remote protocol for the @samp{qCRC}
command.  In order to get the same results as gdb for a block of data,
you must pass the first CRC parameter as @code{0xffffffff}.
This CRC can be specified as:
  Width  : 32
  Poly   : 0x04c11db7
  Init   : parameter, typically 0xffffffff
  RefIn  : false
  RefOut : false
  XorOut : 0
This differs from the "standard" CRC-32 algorithm in that the values
are not reflected, and there is no final XOR value.  These differences
make it easy to compose the values of multiple blocks.
@end deftypefn
*/

/*
 * Zigbee Defination
 */
#define ZIGBEE_HEADER_SIZE 3
#define HEADER1 '1'
#define HEADER2 '2'
#define HEADER3 '3'

#define ZIGBEE_ADDR_SIZE 2
#define ZIGBEE_HADDR 'A'
#define ZIGBEE_LADDR 'B'

#define ZIGBEE_LENGTH 1
#define ZIGBEE_TYPE 1

#define ZIGBEE_CRC 4

#define ZIGBEE_DATA 5

#define BUFFER_SIZE (ZIGBEE_HEADER_SIZE + ZIGBEE_ADDR_SIZE + ZIGBEE_LENGTH + ZIGBEE_CRC + ZIGBEE_TYPE + ZIGBEE_DATA)

#define LENGTH_LOCATION (ZIGBEE_HEADER_SIZE + ZIGBEE_ADDR_SIZE)
#define HEADER_LOCATION (0)
#define ADDR_LOCATION (ZIGBEE_HEADER_SIZE)
#define CRC_LOCATION (ZIGBEE_HEADER_SIZE + ZIGBEE_ADDR_SIZE + ZIGBEE_LENGTH)
#define MESTYPE_LOCATION = (ZIGBEE_HEADER_SIZE + ZIGBEE_ADDR_SIZE + ZIGBEE_LENGTH + ZIGBEE_CRC)
#define DATA_LOCATION (ZIGBEE_HEADER_SIZE + ZIGBEE_ADDR_SIZE + ZIGBEE_LENGTH + ZIGBEE_TYPE + ZIGBEE_CRC)
extern char ZIGBEE_ID;
enum messtype{
    setlocation = 0,
    getlocation = 1,
    getadc = 2,
    getcarstatus = 3,
    getzigbeestatus = 4,
    getrespond = 5
};
/*
 * End of Zigbee defination
 */

void zigbeeInit(uint8_t addr, uint32_t uart);

uint32_t xcrc32 (const int8_t *buf);
int8_t zigbeeCheckHeader(int8_t zigbeepack[BUFFER_SIZE]);
int8_t zigbeeCheckAddress(int8_t zigbeepack[BUFFER_SIZE]);
uint32_t zigbeeGetCRC(int8_t zigbeepack[BUFFER_SIZE]);
void zigbeeSetCRC(int8_t zigbeepack[BUFFER_SIZE], uint32_t crc32);
uint8_t zigbeeSend(int8_t zigbeepack[BUFFER_SIZE]);
uint32_t zigbeeCheckCRC(int8_t zigbeepack[BUFFER_SIZE]);
int8_t zigbeeGetLength(int8_t zigbeepack[BUFFER_SIZE]);
uint8_t zigbeeGetType(int8_t zigbeepack[BUFFER_SIZE]);
void zigbeePrintBuffer(int8_t zigbeepack[BUFFER_SIZE]);

/*
 * External APIs
 */
void MasterSetLocation(uint8_t slave, uint8_t location);
uint8_t MasterGetLocation(uint8_t slave);

#endif /* ZIGBEE_H_ */
