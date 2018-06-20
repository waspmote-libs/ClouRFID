/* Autor: Bykov Ivan <bykov.i.a.74@gmail.com> Date: June 19, 2018 for ANO RRC Airalab Rus */

#ifndef ClouRFID_h
#define ClouRFID_h

#include <inttypes.h>



/* Lib control defines
 * RAM optimization:
 *  1. Set ClouRFID_EPC_max_len to 0 or ClouRFID_TID_max_len to 0 if you not need read EPC or TID
 *  2. Set minimal size of ClouRFID_EPC_max_len to and/or ClouRFID_TID_max_len if you not need read EPC and/or TID
 *  3. Set minimal size of ClouRFID_TAG_FIFO_len
 *  4. Adjast ClouRFID_MaxDataLen, it must be greater than the maximum size of EPC (EPC is always read fully) + ClouRFID_TID_max_len + 15
 *  5. Disable debug messages (RFID_DEBUG_ON to 0)
 */
// Optmazed for 96 bit EPC and 96 bit TID
#define RFID_DEBUG_ON         0                          //Allow USB debug messages 0- Off/ 1- High level / 2- High + Low level
#define ClouRFID_EPC_max_len  12                          //Len of EPC code - 96bits / 12 bytes
#define ClouRFID_TID_max_len  12                          //Len of TID code - 96bits / 12 bytes
#define ClouRFID_MaxDataLen   12+ClouRFID_TID_max_len+15  //Max len of data in message. It must be greater than the maximum size of EPC (EPC is always read fully) + ClouRFID_TID_max_len + 15 
#define ClouRFID_TAG_FIFO_len 20                          //Qty of EPC tags


#if (ClouRFID_EPC_max_len==0)&&(ClouRFID_TID_max_len==0)
  #error "ClouRFID: Wrong read settings set EPC or/and TID length"
#endif

typedef struct{
  uint8_t Control;    //Protocol control word MSB
  uint8_t MessageID;  //Protocol control word LSB
  uint16_t Len;       //Data content length 
  uint8_t  Data[ClouRFID_MaxDataLen];      //Message data 
} ClouRFID_Mes_t;

typedef struct{
  uint8_t TxPowerMin;     //Min. transmission power
  uint8_t TxPowerMax;     //Max. transmission power
  uint8_t AntenaQty;      //Antenna qty
} ClouRFID_Params_t;

typedef enum {
  RS232=0,
  RS485=1
}ClouRFID_Interface_t;

typedef enum {
  ClouRFID_OK=0,
  ClouRFID_ERROR=0xFF
}ClouRFID_RETURN_t;

typedef struct{
  #if ClouRFID_EPC_max_len>0
    uint8_t EPC[ClouRFID_EPC_max_len];  //EPC code 
    uint16_t EPC_Len;                   //EPC len in bits
  #endif  
  
  #if ClouRFID_TID_max_len>0
    uint8_t TID[ClouRFID_TID_max_len];  //TID code
    uint16_t TID_Len;                   //TID len in bits
  #endif 

  /* +++ User data array here */
  
  uint8_t Ant;                        //Antenna number
  uint8_t RSSIdBm;                    //RSSI level
} ClouRFID_Tag_t;

class ClouRFID
{
  public:
   /* Start work with RS232/RS485 and USB (for debug)
    *  Prams:
    *    Baudrate - speed of port (bits / sec)
    *    Intrface - RS485 /  RS232
    *    RS485addres - addres on RS485 bus 
    *  Return ClouRFID_OK / ClouRFID_ERROR
    */
    ClouRFID_RETURN_t Start(uint32_t Baudrate,ClouRFID_Interface_t Intrface, uint8_t RS485addres);

   /* Scan tags and add to FIFO
    *  Prams:
    *    Antenna - Antena ID 
    */
    void ScanTags(uint8_t Ant);

   /* Stop work with RS232/RS485 and USB (for debug)
    */
    void Stop();

   /* Get tag from FIFO
    *  Prams:
    *    Out - pointer to reading ClouRFID_Tag_t
    */
    ClouRFID_RETURN_t GetTag(ClouRFID_Tag_t* Out);
    
    /* Get quantity of tags in FIFO */
    uint16_t GetTagQty();

    /* Get quantity of antennas */
    uint8_t GetAntQty();
    
  private:
    /* Private vars */
  
    //RS485 settings
    uint8_t RS485addr, RS485on;
    //RFID data FIFO
    ClouRFID_Tag_t tagFIFO[ClouRFID_TAG_FIFO_len+1];
    uint16_t tagFIFO_in,tagFIFO_out,tagFIFO_count;
    //Protocol data structure
    ClouRFID_Mes_t cMess;       //temporary frame (RX/TX) 
    ClouRFID_Params_t cParams;  //RFID reader params
    //Parse frame state
    uint8_t PackState; //frame part
    uint16_t CRC;      //frame CRC
    uint16_t Temp;     //temporary var (CRC/Len)

    /* Low lewel protocol and interface functions */
    
    //Parse / make frame
    void CalcCRC16(uint16_t* crcValue,uint8_t newByte);
    void SendByte(uint8_t Data);
    void SendPacket(ClouRFID_Mes_t* Mess);
    uint8_t GetPacket(ClouRFID_Mes_t* Mess);
    //RS232/RS485 port control
    uint8_t PortIni(uint32_t Speed);
    void PortDeIni();
    
    /* High lewel protocol functions */
    
    uint8_t ErrorFilter(ClouRFID_Mes_t* Mess); //illegal command response detection
    void StopRFID();//stop all RFID opperations 
    uint8_t GetResp(ClouRFID_Mes_t* Mess);//get response with timeout 
    void AddTag(ClouRFID_Mes_t* Mess);//Parse EPC read response and update tag FIFO
};

#endif //ClouRFID_h
