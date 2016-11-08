/**
 * @file    mfrc522.c
 * @brief   MFRC522 Driver code.
 *
 * @addtogroup MFRC522
 * @{
 */

#include "hal.h"
#include "mfrc522.h"
#include <string.h>

#if HAL_USE_MFRC522 || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/* MFRC522 Commands */
#define PCD_IDLE                        0x00   //NO action; Cancel the current command
#define PCD_AUTHENT                     0x0E   //Authentication Key
#define PCD_RECEIVE                     0x08   //Receive Data
#define PCD_TRANSMIT                    0x04   //Transmit data
#define PCD_TRANSCEIVE                  0x0C   //Transmit and receive data,
#define PCD_RESETPHASE                  0x0F   //Reset
#define PCD_CALCCRC                     0x03   //CRC Calculate

/* Mifare_One card command word */
#define PICC_REQIDL                     0x26   // find the antenna area does not enter hibernation
#define PICC_REQALL                     0x52   // find all the cards antenna area
#define PICC_ANTICOLL_CL1               0x93   // anti-collision CL1
#define PICC_ANTICOLL_CL2               0x95   // anti-collision CL2
#define PICC_ANTICOLL_CL3               0x97   // anti-collision CL3
#define PICC_SElECTTAG                  0x93   // election card
#define PICC_AUTHENT1A                  0x60   // authentication key A
#define PICC_AUTHENT1B                  0x61   // authentication key B
#define PICC_READ                       0x30   // Read Block
#define PICC_WRITE                      0xA0   // write block
#define PICC_DECREMENT                  0xC0   // debit
#define PICC_INCREMENT                  0xC1   // recharge
#define PICC_RESTORE                    0xC2   // transfer block data to the buffer
#define PICC_TRANSFER                   0xB0   // save the data in the buffer
#define PICC_HALT                       0x50   // Sleep

/* MFRC522 Registers */
//Page 0: Command and Status
#define MifareREG_RESERVED00          0x00
#define MifareREG_COMMAND             0x01
#define MifareREG_COMM_IE_N           0x02
#define MifareREG_DIV1_EN             0x03
#define MifareREG_COMM_IRQ            0x04
#define MifareREG_DIV_IRQ             0x05
#define MifareREG_ERROR               0x06
#define MifareREG_STATUS1             0x07
#define MifareREG_STATUS2             0x08
#define MifareREG_FIFO_DATA           0x09
#define MifareREG_FIFO_LEVEL          0x0A
#define MifareREG_WATER_LEVEL         0x0B
#define MifareREG_CONTROL             0x0C
#define MifareREG_BIT_FRAMING         0x0D
#define MifareREG_COLL                0x0E
#define MifareREG_RESERVED01          0x0F
//Page 1: Command
#define MifareREG_RESERVED10          0x10
#define MifareREG_MODE                0x11
#define MifareREG_TX_MODE             0x12
#define MifareREG_RX_MODE             0x13
#define MifareREG_TX_CONTROL          0x14
#define MifareREG_TX_AUTO             0x15
#define MifareREG_TX_SELL             0x16
#define MifareREG_RX_SELL             0x17
#define MifareREG_RX_THRESHOLD        0x18
#define MifareREG_DEMOD               0x19
#define MifareREG_RESERVED11          0x1A
#define MifareREG_RESERVED12          0x1B
#define MifareREG_MIFARE              0x1C
#define MifareREG_RESERVED13          0x1D
#define MifareREG_RESERVED14          0x1E
#define MifareREG_SERIALSPEED         0x1F
//Page 2: CFG
#define MifareREG_RESERVED20          0x20
#define MifareREG_CRC_RESULT_M        0x21
#define MifareREG_CRC_RESULT_L        0x22
#define MifareREG_RESERVED21          0x23
#define MifareREG_MOD_WIDTH           0x24
#define MifareREG_RESERVED22          0x25
#define MifareREG_RF_CFG              0x26
#define MifareREG_GS_N                0x27
#define MifareREG_CWGS_PREG           0x28
#define MifareREG__MODGS_PREG         0x29
#define MifareREG_T_MODE              0x2A
#define MifareREG_T_PRESCALER         0x2B
#define MifareREG_T_RELOAD_H          0x2C
#define MifareREG_T_RELOAD_L          0x2D
#define MifareREG_T_COUNTER_VALUE_H   0x2E
#define MifareREG_T_COUNTER_VALUE_L   0x2F
//Page 3:TestRegister
#define MifareREG_RESERVED30          0x30
#define MifareREG_TEST_SEL1           0x31
#define MifareREG_TEST_SEL2           0x32
#define MifareREG_TEST_PIN_EN         0x33
#define MifareREG_TEST_PIN_VALUE      0x34
#define MifareREG_TEST_BUS            0x35
#define MifareREG_AUTO_TEST           0x36
#define MifareREG_VERSION             0x37
#define MifareREG_ANALOG_TEST         0x38
#define MifareREG_TEST_ADC1           0x39
#define MifareREG_TEST_ADC2           0x3A
#define MifareREG_TEST_ADC0           0x3B
#define MifareREG_RESERVED31          0x3C
#define MifareREG_RESERVED32          0x3D
#define MifareREG_RESERVED33          0x3E
#define MifareREG_RESERVED34          0x3F
//Dummy byte
#define MifareDUMMY                   0x00

#define MifareMAX_LEN                 16

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/
static void MFRC522Reset(MFRC522Driver* mfrc522p) {
    MFRC522WriteRegister(mfrc522p, MifareREG_COMMAND, PCD_RESETPHASE);
}

static void MFRC522SetBitMask(MFRC522Driver* mfrc522p, uint8_t reg, uint8_t mask) {
    MFRC522WriteRegister(mfrc522p, reg, MFRC522ReadRegister(mfrc522p, reg) | mask);
}

static void MFRC522ClearBitMask(MFRC522Driver* mfrc522p, uint8_t reg, uint8_t mask){
    MFRC522WriteRegister(mfrc522p, reg, MFRC522ReadRegister(mfrc522p, reg) & (~mask));
}

static void MFRC522AntennaOn(MFRC522Driver* mfrc522p) {
    uint8_t temp;

    temp = MFRC522ReadRegister(mfrc522p, MifareREG_TX_CONTROL);
    if (!(temp & 0x03)) {
        MFRC522SetBitMask(mfrc522p, MifareREG_TX_CONTROL, 0x03);
    }
}

static void MFRC522AntennaOff(MFRC522Driver* mfrc522p) {
    MFRC522ClearBitMask(mfrc522p, MifareREG_TX_CONTROL, 0x03);
}

static MIFARE_Status_t MifareToPICC(MFRC522Driver* mfrc522p, uint8_t command, uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint8_t backDataLen, uint16_t* backLen)
{
    MIFARE_Status_t status = MIFARE_ERR;
    uint8_t irqEn = 0x00;
    uint8_t waitIRq = 0x00;
    uint8_t lastBits;
    uint8_t n;
    uint16_t i;

    switch (command) {
        case PCD_AUTHENT: {
            irqEn = 0x12;
            waitIRq = 0x10;
            break;
        }
        case PCD_TRANSCEIVE: {
            irqEn = 0x77;
            waitIRq = 0x30;
            break;
        }
        default:
            break;
    }

    MFRC522WriteRegister(mfrc522p, MifareREG_COMM_IE_N, irqEn | 0x80);
    MFRC522ClearBitMask(mfrc522p, MifareREG_COMM_IRQ, 0x80);
    MFRC522SetBitMask(mfrc522p, MifareREG_FIFO_LEVEL, 0x80);

    MFRC522WriteRegister(mfrc522p, MifareREG_COMMAND, PCD_IDLE);

    //Writing data to the FIFO
    for (i = 0; i < sendLen; i++) {
        MFRC522WriteRegister(mfrc522p, MifareREG_FIFO_DATA, sendData[i]);
    }

    //Execute the command
    MFRC522WriteRegister(mfrc522p, MifareREG_COMMAND, command);
    if (command == PCD_TRANSCEIVE) {
        MFRC522SetBitMask(mfrc522p, MifareREG_BIT_FRAMING, 0x80);      //StartSend=1,transmission of data starts
    }

    //Waiting to receive data to complete
    i = 2000;   //i according to the clock frequency adjustment, the operator M1 card maximum waiting time 25ms???
    do {
        //CommIrqReg[7..0]
        //Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
        n = MFRC522ReadRegister(mfrc522p, MifareREG_COMM_IRQ);
        i--;
    } while ((i!=0) && !(n&0x01) && !(n&waitIRq));

    MFRC522ClearBitMask(mfrc522p, MifareREG_BIT_FRAMING, 0x80);            //StartSend=0

    if (i != 0)  {
        if (!(MFRC522ReadRegister(mfrc522p, MifareREG_ERROR) & 0x1B)) {
            status = MIFARE_OK;
            if (n & irqEn & 0x01) {
                status = MIFARE_NOTAGERR;
            }

            if (command == PCD_TRANSCEIVE) {
                n = MFRC522ReadRegister(mfrc522p, MifareREG_FIFO_LEVEL);
                lastBits = MFRC522ReadRegister(mfrc522p, MifareREG_CONTROL) & 0x07;
                if (lastBits) {
                    *backLen = (n - 1) * 8 + lastBits;
                } else {
                    *backLen = n * 8;
                }

                if (n == 0) {
                    n = 1;
                }
                if (n > MifareMAX_LEN) {
                    n = MifareMAX_LEN;
                }

                if (n <= backDataLen)
                {
                    //Reading the received data in FIFO
                    for (i = 0; i < n; i++) {
                        backData[i] = MFRC522ReadRegister(mfrc522p, MifareREG_FIFO_DATA);
                    }
                }
                else
                {
                    status = MIFARE_ERR;
                }

            }
        } else {
            status = MIFARE_ERR;
        }
    }

    return status;
}

uint8_t MFRC522AntiCollisionLoop(MFRC522Driver* mfrc522p, uint8_t selcommand, uint8_t* serNum)
{
    MIFARE_Status_t status;
    uint8_t serNumCheck = 0;
    uint16_t unLen;
    uint8_t cascadeLevel[5];
    uint8_t command[2];
    uint8_t SAK = 0xff;


    command[0] = selcommand; // SEL
    command[1] = 0x20; // NVB
    status = MifareToPICC(mfrc522p, PCD_TRANSCEIVE, command, 2, cascadeLevel, sizeof(cascadeLevel), &unLen);

    if (status == MIFARE_OK) {
        //calc bcc
        int8_t i = 0;
        for (i = 0; i < 4; i++) {
            serNumCheck ^= cascadeLevel[i];
        }

        if (serNumCheck != cascadeLevel[i]) {
            status = MIFARE_ERR;
            return SAK;
        }

        SAK = MifareSelectTag(mfrc522p, selcommand, cascadeLevel);
        if ((SAK & 0x04) == 0)
        {
            for (i = 0; i < 4; i++)
            {
                serNum[i] = cascadeLevel[i];
            }
        }
        else
        {
            serNum[0] = cascadeLevel[1];
            serNum[1] = cascadeLevel[2];
            serNum[2] = cascadeLevel[3];
        }
    }

    return SAK;
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   MFRC522 Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void MFRC522Init(void) {

}

/**
 * @brief   Initializes the standard part of a @p MFRC522Driver structure.
 *
 * @param[out] mfrc522p     pointer to the @p MFRC522Driver object
 *
 * @init
 */
void MFRC522ObjectInit(MFRC522Driver* mfrc522p) {

	mfrc522p->state = MFRC522_STOP;
	mfrc522p->config = NULL;
}

/**
 * @brief   Configures and activates the MFRC522 peripheral.
 *
 * @param[in] MFRC522p      pointer to the @p MFRC522Driver object
 * @param[in] config    pointer to the @p MFRC522Config object
 *
 * @api
 */

void MFRC522Start(MFRC522Driver* mfrc522p, const MFRC522Config* config) {

	osalDbgCheck((mfrc522p != NULL) && (config != NULL));

	osalSysLock();
	osalDbgAssert((mfrc522p->state == MFRC522_STOP) || (mfrc522p->state == MFRC522_READY),
			"invalid state");
	mfrc522p->config = config;

	osalSysUnlock();

	MFRC522Reset(mfrc522p);

    MFRC522WriteRegister(mfrc522p, MifareREG_T_MODE, 0x8D);
    MFRC522WriteRegister(mfrc522p, MifareREG_T_PRESCALER, 0x3E);
    MFRC522WriteRegister(mfrc522p, MifareREG_T_RELOAD_L, 30);
    MFRC522WriteRegister(mfrc522p, MifareREG_T_RELOAD_H, 0);

    /* 48dB gain */
    MFRC522WriteRegister(mfrc522p, MifareREG_RF_CFG, 0x70);

    MFRC522WriteRegister(mfrc522p, MifareREG_TX_AUTO, 0x40);
    MFRC522WriteRegister(mfrc522p, MifareREG_MODE, 0x3D);

	MFRC522AntennaOn(mfrc522p);

	osalSysLock();
	mfrc522p->state = MFRC522_ACTIVE;
	osalSysUnlock();
}

/**
 * @brief   Deactivates the MFRC522 peripheral.
 *
 * @param[in] MFRC522p      pointer to the @p MFRC522Driver object
 *
 * @api
 */
void MFRC522Stop(MFRC522Driver* mfrc522p) {

	osalDbgCheck(mfrc522p != NULL);

	osalSysLock();
	osalDbgAssert((mfrc522p->state == MFRC522_STOP) || (mfrc522p->state == MFRC522_READY),
			"invalid state");
	osalSysUnlock();

	MFRC522AntennaOff(mfrc522p);
	MFRC522Reset(mfrc522p);

	osalSysLock();
	mfrc522p->state = MFRC522_STOP;
	osalSysUnlock();
}

/**
 * @brief
 *
 * @param[in] MFRC522p      pointer to the @p MFRC522Driver object
 *
 * @api
 */
MIFARE_Status_t MifareRequest(MFRC522Driver* mfrc522p, uint8_t reqMode, uint8_t* tagType, uint8_t tagTypeLen) {
    MIFARE_Status_t status;
    uint16_t backBits;          //The received data bits
    uint8_t request[1];

    MFRC522WriteRegister(mfrc522p, MifareREG_BIT_FRAMING, 0x07);        //TxLastBists = BitFramingReg[2..0] ???

    request[0] = reqMode;
    status = MifareToPICC( mfrc522p, PCD_TRANSCEIVE, request, 1, tagType, tagTypeLen, &backBits);

    if ((status != MIFARE_OK) || (backBits != 0x10)) {
        status = MIFARE_ERR;
    }

    return status;
}


/**
 * @brief
 *
 * @param[in] MFRC522p
 *
 * @api
 */


MIFARE_Status_t MifareAnticoll(MFRC522Driver* mfrc522p, struct MifareUID* id) {

    uint8_t SAK = 0;

    MFRC522WriteRegister(mfrc522p, MifareREG_BIT_FRAMING, 0x00);        //TxLastBists = BitFramingReg[2..0]

    // cascade level 1
    SAK = MFRC522AntiCollisionLoop(mfrc522p, PICC_ANTICOLL_CL1, id->bytes);
    if ((SAK & 0x04) == 0)
    {
        id->size = 4;
        return MIFARE_OK;
    }

    SAK = MFRC522AntiCollisionLoop(mfrc522p, PICC_ANTICOLL_CL2, id->bytes + 3);
    if ((SAK & 0x04) == 0)
    {
        id->size = 7;
        return MIFARE_OK;
    }

    SAK = MFRC522AntiCollisionLoop(mfrc522p, PICC_ANTICOLL_CL3, id->bytes + 6);
    if ((SAK & 0x04) != 0)
    {
        return MIFARE_ERR;
    }

    id->size = 10;
    return MIFARE_OK;
}

/**
 * @brief
 *
 * @param[in] MFRC522p
 *
 * @api
 */
void MifareCalculateCRC(MFRC522Driver* mfrc522p, uint8_t*  pIndata, uint8_t len, uint8_t* pOutData)
{
    MFRC522ClearBitMask(mfrc522p, MifareREG_DIV_IRQ, 0x04);         //CRCIrq = 0
    MFRC522SetBitMask(mfrc522p, MifareREG_FIFO_LEVEL, 0x80);            //Clear the FIFO pointer

    //Writing data to the FIFO
    uint8_t i;
    for (i = 0; i < len; i++) {
        MFRC522WriteRegister(mfrc522p, MifareREG_FIFO_DATA, *(pIndata+i));
    }
    MFRC522WriteRegister(mfrc522p, MifareREG_COMMAND, PCD_CALCCRC);

    //Wait CRC calculation is complete
    uint8_t n;
    i = 0xFF;
    do {
        n = MFRC522ReadRegister(mfrc522p, MifareREG_DIV_IRQ);
        i--;
    } while ((i!=0) && !(n&0x04));          //CRCIrq = 1

    //Read CRC calculation result
    pOutData[0] = MFRC522ReadRegister(mfrc522p, MifareREG_CRC_RESULT_L);
    pOutData[1] = MFRC522ReadRegister(mfrc522p, MifareREG_CRC_RESULT_M);
}

/**
 * @brief
 *
 * @param[in] MFRC522p
 *
 * @api
 */
uint8_t MifareSelectTag(MFRC522Driver* mfrc522p, uint8_t command, uint8_t* serNum) {
    uint8_t i;
    MIFARE_Status_t status;
    uint8_t SAK;
    uint16_t recvBits;
    uint8_t buffer[9];

    buffer[0] = command;
    buffer[1] = 0x70;
    for (i = 0; i < 5; i++) {
        buffer[i+2] = *(serNum+i);
    }
    MifareCalculateCRC(mfrc522p, buffer, 7, &buffer[7]);     //??
    status = MifareToPICC(mfrc522p, PCD_TRANSCEIVE, buffer, 9, buffer, 9, &recvBits);

    if ((status == MIFARE_OK) && (recvBits == 0x18)) {
        SAK = buffer[0];
    } else {
        SAK = 0xff;
    }

    return SAK;
}

/**
 * @brief
 *
 * @param[in] MFRC522p
 *
 * @api
 */
MIFARE_Status_t MifareAuth(MFRC522Driver* mfrc522p, uint8_t authMode, uint8_t BlockAddr, uint8_t* Sectorkey, uint8_t* serNum) {
    MIFARE_Status_t status;
    uint16_t recvBits;
    uint8_t i;
    uint8_t buff[12];

    //Verify the command block address + sector + password + card serial number
    buff[0] = authMode;
    buff[1] = BlockAddr;
    for (i = 0; i < 6; i++) {
        buff[i+2] = *(Sectorkey+i);
    }
    for (i=0; i<4; i++) {
        buff[i+8] = *(serNum+i);
    }
    status = MifareToPICC(mfrc522p, PCD_AUTHENT, buff, 12, buff, 12,&recvBits);

    if ((status != MIFARE_OK) || (!(MFRC522ReadRegister(mfrc522p, MifareREG_STATUS2) & 0x08))) {
        status = MIFARE_ERR;
    }

    return status;
}

/**
 * @brief
 *
 * @param[in] MFRC522p
 *
 * @api
 */
MIFARE_Status_t MifareRead(MFRC522Driver* mfrc522p, uint8_t blockAddr, uint8_t* recvData) {
    MIFARE_Status_t status;
    uint16_t unLen;

    recvData[0] = PICC_READ;
    recvData[1] = blockAddr;
    MifareCalculateCRC(mfrc522p, recvData,2, &recvData[2]);
    status = MifareToPICC(mfrc522p, PCD_TRANSCEIVE, recvData, 4, recvData, 4, &unLen);

    if ((status != MIFARE_OK) || (unLen != 0x90)) {
        status = MIFARE_ERR;
    }

    return status;
}

/**
 * @brief
 *
 * @param[in] MFRC522p
 *
 * @api
 */
MIFARE_Status_t MifareWrite(MFRC522Driver* mfrc522p, uint8_t blockAddr, uint8_t* writeData) {
    MIFARE_Status_t status;
    uint16_t recvBits;
    uint8_t i;
    uint8_t buff[18];

    buff[0] = PICC_WRITE;
    buff[1] = blockAddr;
    MifareCalculateCRC(mfrc522p, buff, 2, &buff[2]);
    status = MifareToPICC(mfrc522p, PCD_TRANSCEIVE, buff, 4, buff, 18, &recvBits);

    if ((status != MIFARE_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A)) {
        status = MIFARE_ERR;
    }

    if (status == MIFARE_OK) {
        //Data to the FIFO write 16Byte
        for (i = 0; i < 16; i++) {
            buff[i] = *(writeData+i);
        }
        MifareCalculateCRC(mfrc522p, buff, 16, &buff[16]);
        status = MifareToPICC(mfrc522p, PCD_TRANSCEIVE, buff, 18, buff, 18, &recvBits);

        if ((status != MIFARE_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A)) {
            status = MIFARE_ERR;
        }
    }

    return status;
}

/**
 * @brief
 *
 * @param[in] MFRC522p
 *
 * @api
 */
MIFARE_Status_t MifareCheck(MFRC522Driver* mfrc522p, struct MifareUID* id)
{
    MIFARE_Status_t status;
    uint8_t tagType[2];

    status = MifareRequest(mfrc522p, PICC_REQALL, tagType, 2);
    if (status == MIFARE_OK) {
        //Card detected
        //Anti-collision, return card serial number 4 bytes
        status = MifareAnticoll(mfrc522p, id);
    }
    MifareHalt(mfrc522p);          //Command card into hibernation

    return status;
}

MIFARE_Status_t MifareHalt(MFRC522Driver* mfrc522p) {
    uint16_t unLen;
    uint8_t buff[4];

    buff[0] = PICC_HALT;
    buff[1] = 0;
    MifareCalculateCRC(mfrc522p, buff, 2, &buff[2]);

    return MifareToPICC(mfrc522p, PCD_TRANSCEIVE, buff, 4, buff, 4, &unLen);
}

#endif /* HAL_USE_MFRC522 */

/** @} */
