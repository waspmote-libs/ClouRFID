/*! \file ClouRFID.h
    \brief Clou RFID reader minimal driver for waspmote (RS232/RS485).
    \version 0.1
    \autors  Bykov Ivan <bykov.i.a.74@gmail.com> 
    \date    June 19, 2018 
    for ANO RRC Airalab Rus
 */


#ifndef ClouRFID_h
#define ClouRFID_h

/*******************************************************************************
 * Definitions 
 ******************************************************************************/

/* Lib control defines
 * RAM optimization:
 *  1. Set ClouRFID_EPC_max_len to 0 or ClouRFID_TID_max_len to 0 if you not need read EPC or TID
 *  2. Set minimal size of ClouRFID_EPC_max_len to and/or ClouRFID_TID_max_len if you not need read EPC and/or TID
 *  3. Set minimal size of ClouRFID_TAG_FIFO_len
 *  4. Adjast ClouRFID_MaxDataLen, it must be greater than the maximum size of EPC (EPC is always read fully) + ClouRFID_TID_max_len + 15
 *  5. Disable debug messages (RFID_DEBUG_ON to 0)
 * Default values optmazed for 96 bit EPC and 96 bit TID
 */

/*! 
 * \def RFID_DEBUG_ON 
 * \brief Possible values:
 *  0: No debug mode enabled
 *  1: debug mode enabled for high level output messages
 *  2: debug mode enabled for high and low level output messages
 */
#define RFID_DEBUG_ON         0                          
/*! 
 * \def ClouRFID_EPC_max_len 
 * \brief Len of EPC code - 96bits / 12 bytes
 */
#define ClouRFID_EPC_max_len  12 

/*! 
 * \def ClouRFID_TID_max_len 
 * \brief Len of TID code - 96bits / 12 bytes
 */                         
#define ClouRFID_TID_max_len  12  

/*! 
 * \def ClouRFID_MaxDataLen 
 * \brief Max len of data in message
 * It must be greater than the maximum size of EPC (EPC is always read fully) + ClouRFID_TID_max_len + 15 
 */  
#define ClouRFID_MaxDataLen   12+ClouRFID_TID_max_len+15  

/*! 
 * \def ClouRFID_TAG_FIFO_len 
 * \brief Qty of EPC tags
 */ 
#define ClouRFID_TAG_FIFO_len 20                          

//! Error message if used wrong define values
#if (ClouRFID_EPC_max_len==0)&&(ClouRFID_TID_max_len==0)
  #error "ClouRFID: Wrong read settings set EPC or/and TID length"
#endif

/******************************************************************************
 * Includes
 ******************************************************************************/

#include <inttypes.h>

/******************************************************************************
 * Type definitions
 ******************************************************************************/
/*! reader frame type */
typedef struct{
  uint8_t Control;    /*< Protocol control word MSB */
  uint8_t MessageID;  /*< Protocol control word LSB */
  uint16_t Len;       /*< Data content length  */
  uint8_t  Data[ClouRFID_MaxDataLen];  /*< Message data   */
} ClouRFID_Mes_t;

/*! reader RFID ability type */
typedef struct{
  uint8_t TxPowerMin;     /*< Min. transmission power */
  uint8_t TxPowerMax;     /*< Max. transmission power */
  uint8_t AntenaQty;      /*< Antenna qty */
} ClouRFID_Params_t;

/*! interface type enum. */
typedef enum {
  RS232=0,    /*!< RS232 (duplex) serial port */ 
  RS485=1     /*!< RS485 (half duplex) serial port */ 
}ClouRFID_Interface_t;

/*! function result enum. */
typedef enum {
  ClouRFID_OK=0,        /*!< Success */ 
  ClouRFID_ERROR=0xFF   /*!< Fail */   
}ClouRFID_RETURN_t;

typedef struct{
  #if ClouRFID_EPC_max_len>0
    uint8_t EPC[ClouRFID_EPC_max_len];  /*!< EPC code data */ 
    uint16_t EPC_Len;                   /*!< EPC total length */ 
  #endif  
  
  #if ClouRFID_TID_max_len>0
    uint8_t TID[ClouRFID_TID_max_len];  /*!< TID code data */
    uint16_t TID_Len;                   /*!< TID total length */
  #endif 

  /* +++ User data array here */
  
  uint8_t Ant;                        /*!< Antenna number */
  uint8_t RSSIdBm;                    /*!< RSSI level */
} ClouRFID_Tag_t;

/******************************************************************************
 * Class
 ******************************************************************************/

class ClouRFID
{

//**********************************************************************
// Public functions. 
//**********************************************************************
  public:
   /*!
    *  \def Start work with RS232/RS485 and USB (for debug)
    *  \param[in] Baudrate - speed of port (bits / sec)
    *  \param[in] Intrface - RS485 /  RS232 (\ref <ClouRFID_Interface_t>)
    *  \param[in] RS485addres - addres on RS485 bus 
    *  \return ClouRFID_OK / ClouRFID_ERROR (\ref <ClouRFID_RETURN_t>)
    */
    ClouRFID_RETURN_t Start(uint32_t Baudrate,ClouRFID_Interface_t Intrface, uint8_t RS485addres);

   /*! 
    *  \def Scan tags and add to FIFO
    *  \param[in]  Antenna - Antena ID 
    */
    void ScanTags(uint8_t Ant);

   //! Stop work with RS232/RS485 and USB (for debug)
    void Stop();

   /*! 
    *  \def Get tag from FIFO
    *  \param[out] Out - pointer to reading ClouRFID_Tag_t
    *  \return ClouRFID_OK / ClouRFID_ERROR (\ref <ClouRFID_RETURN_t>)
    */
    ClouRFID_RETURN_t GetTag(ClouRFID_Tag_t* Out);
    
   //! Get quantity of tags in FIFO 
    uint16_t GetTagQty();

   //! Get quantity of antennas 
    uint8_t GetAntQty();

//**********************************************************************
// Private functions and variables
//**********************************************************************

  private:

    uint8_t RS485addr; /*!< RS485 reader addres */
    uint8_t RS485on;   /*!< RS485 interface enable */
    
    //!RFID data FIFO
    ClouRFID_Tag_t tagFIFO[ClouRFID_TAG_FIFO_len+1];
    //RFID data FIFO control values
    uint16_t tagFIFO_in;   /*!< empty cell (ready for write) index */
    uint16_t tagFIFO_out;  /*!< oldest full cell (ready for read) index */
    uint16_t tagFIFO_count; /*!< number of full cells in FIFO */
    
    ClouRFID_Mes_t cMess;      /*!< temporary frame (RX/TX) */
    ClouRFID_Params_t cParams; /*!< RFID reader params */
    
    //!Parse frame state
    uint8_t PackState; /*!< frame part  */
    uint16_t CRC;      /*!< frame CRC  */
    uint16_t Temp;     /*!< temporary var (CRC/Len) */

    /* Low lewel protocol and interface functions */
    
    //Parse / make frame
    //! Calculation CRC16 CCITT for one byte 
    void CalcCRC16(uint16_t* crcValue,uint8_t newByte);
    //! Send one byte 
    void SendByte(uint8_t Data); 
    //! Send frame to reader 
    void SendPacket(ClouRFID_Mes_t* Mess);
    //! Receive frame from reader
    uint8_t GetPacket(ClouRFID_Mes_t* Mess);
    //! RS232/RS485 port enable
    uint8_t PortIni(uint32_t Speed);
    //! RS232/RS485 port disable
    void PortDeIni();
    
    /* High lewel protocol functions */

    //! Illegal command response detection
    uint8_t ErrorFilter(ClouRFID_Mes_t* Mess); 
    //! Stop all RFID opperations 
    void StopRFID();
    //! Receive response from reader
    uint8_t GetResp(ClouRFID_Mes_t* Mess);
    //! Parse EPC read response and update tag FIFO
    void AddTag(ClouRFID_Mes_t* Mess);
};

#endif //ClouRFID_h
