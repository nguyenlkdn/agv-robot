/*
 * modbus-rtu.h
 *
 *  Created on: Oct 21, 2017
 *      Author: TDNC
 */

#ifndef MODBUS_RTU_H_
#define MODBUS_RTU_H_
#include <stdint.h>
#include <stdbool.h>

//#define MODBUS_DEBUG    1

/*
 * MODBUS-RTU Defination
 */
#define ERROR_TIMEOUT   1
#ifdef ADDR16
    #define ADDR_LEN    2
#else
    #define ADDR_LEN    1
#endif

#define FUNC_LEN        1
#define REGADDR_LEN     2
#define REGNO_LEN       2
#define CRC_LEN         2
#define SENT_NUM        1

#define ADDR_POS        0
#define FUNC_POS        (ADDR_POS + ADDR_LEN)
#define REGADDR_POS     (FUNC_POS + FUNC_LEN)
#define REGNO_POS       (REGADDR_POS + REGADDR_LEN)
#define CRC_POS         (REGNO_POS + REGNO_LEN)
#define SENT_POS        (REGNO_POS + REGNO_LEN)
#define DATA_POS        (SENT_POS + SENT_NUM)

#define MODBUS_SIZE     (ADDR_LEN + FUNC_LEN + REGADDR_LEN + REGNO_LEN + SENT_NUM + CRC_LEN)
#define DATA_SIZE       2
#define MODBUS_TIMEOUT  1000000
#define REG_LOCATION    0
#define REG_LEN         2
typedef struct {
    uint8_t *buffer;
    uint8_t index;
} modbusbuffer;
typedef struct {
  uint8_t status;
  uint16_t addr;
  uint8_t func;
  uint16_t start;
  uint16_t nofreg;
  uint16_t index;
  uint8_t *data;
  uint32_t uartbase;
  uint8_t respond;
  uint16_t crc;
  modbusbuffer readingbuffer;
} modbus;
typedef struct {
    uint32_t uartbase;
    uint32_t baurate;
    uint32_t portbase;
} uartparam;
extern modbus *modbusrunning;

/*
 * General Internal APIs
 */
void        modbusExport(modbus *modbuspack);
void        modbusInit(modbus *modbuspack, uint16_t addr, uint16_t func,
                uint16_t start, uint16_t numofreg, uint32_t uartbase);
void        modbusInitProtocol(modbus *modbuspack);

void        modbusReInit(modbus *modbuspack);
void        modbusDeInit(modbus *modbuspack);
uint16_t    crc16Gen(uint8_t *data, uint8_t length);
uint16_t    crc16Get(modbus *modbuspack);
void        crc16Set(modbus *modbuspack, uint16_t crc16);
uint8_t     crc16Check(modbus *modbuspack, uint16_t crc);
uint16_t    combineBytes(uint8_t high, uint8_t low);

void        addrSet(modbus *modbuspack, uint16_t addr);
uint16_t    addrGet(modbus *modbuspack);

uint16_t    startaddrGet(modbus *modbuspack);
void        startaddrSet(modbus *modbuspack, uint16_t start);

uint16_t    numofregGet(modbus *modbuspack);
void        numofregSet(modbus *modbuspack, uint16_t numofreg);

void        funcSet(modbus *modbuspack, uint8_t function);
uint8_t     funcGet(modbus *modbuspack);

void        dataLenghSet(modbus *modbuspack, uint8_t lengh);
uint8_t     dataLenghGet(modbus *modbuspack);
uint8_t     dataAllocate(modbus *modbuspack, uint8_t size);

uint8_t     CRCCompare(modbus *pack1, modbus *pack2);

void        dataSet(modbus *modbuspack, uint8_t *data);
uint8_t     *dataGet(modbus *modbuspack);
uint8_t     dataAttach(modbus *modbuspack, uint16_t data);
uint16_t    dataDetach(modbus *modbuspack, uint16_t idex);
uint8_t     dataAttachIndex(modbus *modbuspack, uint16_t data, uint8_t index);
uint8_t     modbustoarray(modbus *modbuspack, uint8_t *array, uint8_t length);
uint16_t    arraytomodbus(modbus *modbuspack, uint8_t *array, uint8_t length);
uint8_t     modbusarrayProcessing(uint8_t *array, uint8_t length, uint8_t addr);

uint16_t    arrayf03tomodbus(modbus *modbuspack, uint8_t *array, uint8_t length);
uint8_t     modbustoarraywithoutdata(modbus *modbuspack, uint8_t *array, uint8_t length);
uint8_t     modbustostring(modbus *modbuspack, uint8_t *array, uint8_t length);
uint8_t     modbussendstring(uint32_t ui32Base, uint8_t *array, uint16_t length);
/*
 * External APIs
 */
uint8_t     modbusSendF16(modbus *modbuspack);
uint8_t     modbusSendF03(modbus *modbuspack);
uint8_t     modbusRespondF03(modbus *modbuspack);
uint8_t     modbusF03Respond(modbus *modbuspack, uint16_t addr, uint16_t reg, uint16_t *data);
uint8_t     modbusWriteSingle(modbus *modbuspack, uint16_t addr, uint16_t reg, uint16_t data);
uint8_t     modbusWriteMulti(modbus *modbuspack, uint16_t addr, uint16_t reg, uint16_t *data, uint32_t leng);
uint8_t     modbusWriteMultiBlocking(modbus *modbuspack, uint16_t addr, uint16_t reg, uint16_t *data, uint32_t leng, uint32_t timeout);
uint8_t     modbusWriteSingleBlocking(modbus *modbuspack, uint16_t addr, uint16_t reg, uint16_t data, uint32_t timeout);
uint8_t     modbusReadSingle(modbus *modbuspack, uint16_t addr, uint16_t reg);
uint8_t     modbusReadMulti(modbus *modbuspack, uint16_t addr, uint16_t reg, uint16_t numofreg);
uint8_t     modbusReadSingleBlocking(modbus *modbuspack, uint16_t addr, uint16_t reg, uint16_t *data, uint32_t timeout);
uint8_t     modbusReadMultiBlocking(modbus *modbuspack, uint16_t addr, uint16_t reg, uint16_t numofreg, uint32_t timeout);
uint16_t    masterGetLocation(modbus *modbuspack);
#endif /* MODBUS_RTU_H_ */
