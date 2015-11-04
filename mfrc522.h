
/**
 * @file    mfrc522.h
 * @brief   MFRC 522 Driver macros and structures.
 *
 * @addtogroup mfrc522
 * @{
 */

#ifndef _MFRC522_H_
#define _MFRC522_H_

#if HAL_USE_MFRC522 || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/


/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/
/**
 * @brief   MFRC522 interrupt priority level setting.
 */
#if !defined(MFRC522_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define MFRC522_IRQ_PRIORITY         10
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/
/**
 * @brief   Mifare states.
 */
typedef enum {
    MIFARE_OK = 0,
    MIFARE_NOTAGERR,
    MIFARE_ERR
} MIFARE_Status_t;

struct MifareUID
{
    uint8_t size;
    uint8_t bytes[10];
} ;

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  MFRC522_UNINIT = 0,                   /**< Not initialized.                   */
  MFRC522_STOP = 1,                     /**< Stopped.                           */
  MFRC522_READY = 2,                    /**< Ready.                             */
  MFRC522_ACTIVE = 3,                   /**< Active.                            */
} MFRC522state_t;
/**
 * @brief   Type of a structure representing an MFRC522Driver driver.
 */
typedef struct MFRC522Driver MFRC522Driver;
/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
} MFRC522Config;


/**
 * @brief   Structure representing an MFRC522 driver.
 */
struct MFRC522Driver {
  /**
   * @brief   Driver state.
   */
	MFRC522state_t                state;
  /**
   * @brief   Current configuration data.
   */
  const MFRC522Config           *config;
  /* End of the mandatory fields.*/
};
/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void MFRC522Init(void);
  void MFRC522ObjectInit(MFRC522Driver* mfrc522p);
  void MFRC522Start(MFRC522Driver* mfrc522p, const MFRC522Config *config);
  void MFRC522Stop(MFRC522Driver* mfrc522p);

  MIFARE_Status_t MifareRequest(MFRC522Driver* mfrc522p, uint8_t reqMode, uint8_t* TagType);
  MIFARE_Status_t MifareAnticoll(MFRC522Driver* mfrc522p, struct MifareUID* id);
  void MifareCalculateCRC(MFRC522Driver* mfrc522p, uint8_t*  pIndata, uint8_t len, uint8_t* pOutData);
  uint8_t MifareSelectTag(MFRC522Driver* mfrc522p, uint8_t command, uint8_t* serNum);
  MIFARE_Status_t MifareAuth(MFRC522Driver* mfrc522p, uint8_t authMode, uint8_t BlockAddr, uint8_t* Sectorkey, uint8_t* serNum);
  MIFARE_Status_t MifareRead(MFRC522Driver* mfrc522p, uint8_t blockAddr, uint8_t* recvData);
  MIFARE_Status_t MifareWrite(MFRC522Driver* mfrc522p, uint8_t blockAddr, uint8_t* writeData);
  MIFARE_Status_t MifareCheck(MFRC522Driver* mfrc522p, struct MifareUID* id);
  MIFARE_Status_t MifareHalt(MFRC522Driver* mfrc522p);


  extern void MFRC522WriteRegister(MFRC522Driver* mfrc522p, uint8_t addr, uint8_t val);
  extern uint8_t MFRC522ReadRegister(MFRC522Driver* mfrc522p, uint8_t addr);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_MFRC522 */

#endif /* _MFRC522_H_ */

/** @} */
