/*! \file testRFID.pde
    \brief test sketch (waspmote) for Clou RFID reader minimal driver (RS232/RS485).
    \version 0.1
    \autors  Bykov Ivan <bykov.i.a.74@gmail.com> 
    \date    June 19, 2018 
    for ANO RRC Airalab Rus
 */

#include <Wasp485.h>
#include <inttypes.h>
#include "WaspClasses.h"
#include "ClouRFID.h"

ClouRFID RFID;

void setup()
{
  
}

void loop()
{
  if(RFID.Start(115200,RS485,42)==ClouRFID_OK){                                         //Connection to reader success (start using RS485 and USB) 
      uint8_t Ant_Qty=RFID.GetAntQty();                                                 //Get antenna qty
      for(uint8_t ant=1;ant<=Ant_Qty;ant++) RFID.ScanTags(ant);                         //Scan tags on all antennas
      RFID.Stop();                                                                      //Stop connection (stop using RS485 and USB) 
      if(RFID.GetTagQty()>0){                                                           //Process tags FIFO
        ClouRFID_Tag_t Display_Tag; 
        USB.ON();
        USB.printf("\n\n\f");                                                           //Clear page
        #if ClouRFID_EPC_max_len>0 
            #if ClouRFID_TID_max_len>0 
                USB.printf("\nEPC and TID read mode"); 
            #else
                USB.printf("\nEPC only read mode"); 
            #endif
        #else
            #if ClouRFID_TID_max_len>0 
                USB.printf("\nTID only read mode"); 
            #else
                USB.printf("\nERROR: no EPC or TID readed"); 
            #endif
        #endif
        uint8_t Tag_No=1;
        while(RFID.GetTag(&Display_Tag)==ClouRFID_OK){                                  //FIFO get tag data success

          USB.printf("\n\nEPC tag: %d",Tag_No++);                                       //Print Tag No
          USB.printf("\nANT: %d RSSI: %d dBm",Display_Tag.Ant, Display_Tag.RSSIdBm);    //Print ANT / RSSI
          
          uint8_t line,cnt;

          //Print EPC
          #if ClouRFID_EPC_max_len>0
            USB.printf("\nEPC (length: total %d / saved %d  ):\n", Display_Tag.EPC_Len, ClouRFID_EPC_max_len );
            if(Display_Tag.EPC_Len>0){
              cnt=Display_Tag.EPC_Len < ClouRFID_EPC_max_len ? Display_Tag.EPC_Len : ClouRFID_EPC_max_len ;
              line=0;
              for(uint16_t i=0;i<cnt;i++){
                USB.printf(" %02x",Display_Tag.EPC[i]);
                line++;
                if(line%4==0){
                   USB.printf(" ");
                   if(line%16==0) USB.printf("\n");
                }
              } 
            }else{
              USB.printf(" NULL");
            }         
          #endif //ClouRFID_EPC_max_len>0

          //Print TID
          #if ClouRFID_TID_max_len>0
            USB.printf("\nTID (length: total %d / saved %d  ):\n", Display_Tag.TID_Len, ClouRFID_TID_max_len );
            if(Display_Tag.TID_Len>0){
              cnt=Display_Tag.TID_Len < ClouRFID_TID_max_len ? Display_Tag.TID_Len : ClouRFID_TID_max_len ;
              line=0;
              for(uint16_t i=0;i<cnt;i++){
                USB.printf(" %02x",Display_Tag.TID[i]);
                line++;
                if(line%4==0){
                   USB.printf(" ");
                   if(line%16==0) USB.printf("\n");
                }
              }
            }else{
              USB.printf(" NULL");
            }
            
          #endif //#if ClouRFID_TID_max_len>0

          /* +++ User data print here */
          
        }
      }else{
         USB.printf("\n -- NO TAGS -- ");
      }
      USB.OFF();
  }
  delay(10000);
}
