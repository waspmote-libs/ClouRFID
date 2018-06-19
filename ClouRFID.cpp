/* Autor: Bykov Ivan <bykov.i.a.74@gmail.com> Date: June 19, 2018 for ANO RRC Airalab Rus */
		
#include "ClouRFID.h"
#include <Wasp485.h>
#include <inttypes.h>
#include "WaspClasses.h"

#define CR_HEAD     0xAA //Frame head
#define CR_IT_RS485 (1<<(13-8)) //RS485 bit in Protocol control word 
#define CR_IT_RINI  (1<<(12-8)) //Reader initiate message mark bit
#define CR_MT_RERR  0 //Reader error or warning message
#define CR_MT_RCFG  1 //Reader configuration and management message
#define CR_MT_RFID  2 //RFID Configuration and operation message
#define CR_MT_RLOG  3 //Reader log message
#define CR_MT_RUPD  4 //Reader app processor software and baseband software upgrade message.
#define CR_MT_RTST  5 //Testing command
#define CR_MT_MASK  0x07 

#define CR_ERR                          0x00  //Illegal command response

#define CR_RFID_QueryReaderRFIDability  0x00  //Query reader RFID ability
#define CR_RFID_ReadEPCtag              0x10  //Read EPC tag
#define CR_RFID_StopCommand             0xFF  //Stop command

/* Start work with RS232/RS485 and USB (for debug)
 *  Prams:
 *    Baudrate - speed of port (bits / sec)
 *    Intrface - RS485 /  RS232
 *    RS485addres - addres on RS485 bus 
 *  Return ClouRFID_OK / ClouRFID_ERROR
 */
ClouRFID_RETURN_t ClouRFID::Start(uint32_t Baudrate,ClouRFID_Interface_t Intrface, uint8_t RS485addres){
  #if RFID_DEBUG_ON>0
   USB.ON();
   USB.printf("\n\n\f");
  #endif
  //Open port
  uint8_t Retry=5;
  RS485on=(Intrface==RS485) ? 1: 0;
  RS485addr=RS485addres;
  while(PortIni(Baudrate)!=0){
    Retry--;
    if(Retry==0){
      #if RFID_DEBUG_ON>0
        USB.printf("\nRFID ERROR RS485 port not opened");
      #endif
      PortDeIni();
      return ClouRFID_ERROR;
    }
  }
  #if RFID_DEBUG_ON>0
    USB.printf("\nRFID RS485 port opened");
  #endif
  //Send stop
  StopRFID();
  cParams.AntenaQty=0;
  //Send query reader RFID ability
  cMess.Control=CR_MT_RFID;
  cMess.MessageID=CR_RFID_QueryReaderRFIDability;
  cMess.Len=0;
  SendPacket(&cMess);
  //Wait for response
   if((GetResp(&cMess)==0)&&(cMess.MessageID==CR_RFID_QueryReaderRFIDability)){
      #if RFID_DEBUG_ON>0
        USB.printf("\nRFID Connect OK, power: %d-%d dBm, antQty: %d ",cMess.Data[0],cMess.Data[1],cMess.Data[2]);
      #endif
      cParams.TxPowerMin=cMess.Data[0];
      cParams.TxPowerMax=cMess.Data[1];
      cParams.AntenaQty=cMess.Data[2];
      if((cParams.AntenaQty<=4)&&(cMess.Data[0]<cMess.Data[1])&&(cMess.Data[1]<=36)){
        return ClouRFID_OK;
      }

  }
  //No response
  #if RFID_DEBUG_ON>0
    USB.printf("\nRFID ERROR connection");
  #endif
  PortDeIni();
  return ClouRFID_ERROR;
}

void ClouRFID::ScanTags(uint8_t Ant){
  #if (ClouRFID_TID_max_len!=0)||(ClouRFID_EPC_max_len!=0)  /* +++ User data len test here */
    if(cParams.AntenaQty==0) return;
    StopRFID();
    if(Ant>=cParams.AntenaQty){
      Ant=cParams.AntenaQty;
    }
    Ant--;
     #if ClouRFID_TID_max_len==0 //EPC only mode
       cMess.Control=CR_MT_RFID;
       cMess.MessageID=CR_RFID_ReadEPCtag;
       cMess.Len=2;
       cMess.Data[0]=(1<<Ant);   //Antenna port No.
       cMess.Data[1]=0;     //0 - Single read mode: reader make one round tag reading on each enabled antenna, and then enter idle mode.
       #if RFID_DEBUG_ON>0
            USB.printf("\nRFID Start scan EPC only  ");
       #endif
     #else //EPC+TID or TID mode
       cMess.Control=CR_MT_RFID;
       cMess.MessageID=CR_RFID_ReadEPCtag;
       cMess.Len=5;
       cMess.Data[0]=(1<<Ant);   //Antenna port No.
       cMess.Data[1]=0;     //0 - Single read mode: reader make one round tag reading on each enabled antenna, and then enter idle mode
       //PID 2: TID read parameter
       cMess.Data[2]=2;                         //PID Number
       cMess.Data[3]=0;                         //Byte 0: TID read mode configuration，0，TID read length self-adapter, but max. length not exceed byte 1 defined length.
       cMess.Data[4]=(ClouRFID_TID_max_len/2);      //Byte 1：TID data word length to be read (word，16bits，below same). (6+1)*2=14 bytes
       #if RFID_DEBUG_ON>0
            USB.printf("\nRFID Start scan EPC & TID");
       #endif
     #endif //ClouRFID_TID_max_len==0 
     
     /* +++ User data read command here */
  
    if(RS485on>0){     //RS485 is on
      cMess.Data[1]=0; //MUST BE ZERO - continius read blocked intrface!
    }
     
    SendPacket(&cMess);
  
     //Wait for response
     if((GetResp(&cMess)==0)&&(cMess.MessageID==CR_RFID_ReadEPCtag)){
       #if RFID_DEBUG_ON>0
           USB.printf("\nRFID Tag read Start! ");
       #endif
       //Wait for tag read (MessageID == 0)
       while(GetResp(&cMess)==0){ //EPC tag data upload 
         if(cMess.MessageID==0){
           #if RFID_DEBUG_ON>0
               USB.printf("\nRFID Tag read, Ant: %d ",Ant);
           #endif
           AddTag(&cMess);
         }else if(cMess.MessageID==1){ //EPC tag reading finish 
           #if RFID_DEBUG_ON>0
               USB.printf("\nRFID Tag read End");
           #endif
           return;
         }
       }
    }
    #if RFID_DEBUG_ON>0
        USB.printf("\nRFID Scan End");
    #endif

  #endif (ClouRFID_TID_max_len!=0)&&(ClouRFID_EPC_max_len!=0)
}

void ClouRFID::Stop(){
  StopRFID();
  PortDeIni();
  #if RFID_DEBUG_ON>0
    USB.printf("\nRFID Stoped");
  #endif
}

/* Get tag from FIFO */
ClouRFID_RETURN_t ClouRFID::GetTag(ClouRFID_Tag_t* Out){
  if(tagFIFO_count){
     //Get TAG from FIFO
     memcpy((uint8_t*)(Out),(uint8_t*)(&tagFIFO[tagFIFO_out]),sizeof(ClouRFID_Tag_t));
     //Update in index
     tagFIFO_out++;
     if(tagFIFO_out>=ClouRFID_TAG_FIFO_len+1){tagFIFO_out=0;}
     //Update data count
     tagFIFO_count--;
     return ClouRFID_OK;
  }
  return ClouRFID_ERROR;
}
/* Get count of tags in FIFO */
uint16_t ClouRFID::GetTagQty(){
  return tagFIFO_count;
}

/* Get count of tags in FIFO */
uint8_t ClouRFID::GetAntQty(){
  return cParams.AntenaQty;
}




/* ini RS485 interface */
uint8_t ClouRFID::PortIni(uint32_t baudRate){

  if(W485.ON()!=0) return 0xFF;
  W485.baudRateConfig(baudRate);   // Configure the baud rate of the module
  W485.parityBit(DISABLE);      // Configure the parity bit as disabled 
  W485.stopBitConfig(1);        // Use one stop bit configuration  
  W485.transmission(DISABLE);   // Disables the transmission - sniffing the bus
  W485.flush();                 // Clear both the receive and transmit FIFOs of all data contents.
  return 0;
}

/* deini RS485 interface */
void ClouRFID::PortDeIni(){
  W485.OFF();
  #if RFID_DEBUG_ON>0
   USB.OFF();
  #endif
}


//Private functions
/* Calculate CRC16 CCITT for 1 byte */ 
//adopt CCITT-16, calibration polynomials is X16 + X15 + X2 + 1, initiation value is set as 0.
void ClouRFID::CalcCRC16(uint16_t* crcValue,uint8_t newByte) {
  for (uint8_t i = 0; i < 8; i++) {
    if (((*crcValue & 0x8000) >> 8) ^ (newByte & 0x80)){
                                              //   X^16 +  X^15 +  X^2 +  1
      *crcValue = (*crcValue << 1)  ^ ((uint16_t)((1<<16)|(1<<15)|(1<<2)|(1<<0))); //CRC16_CCITT  
    }else{
      *crcValue = (*crcValue << 1);
    }
    newByte <<= 1;
  }
}

/*  Send byte to RS485  */ 
void ClouRFID::SendByte(uint8_t Data){
  W485.send(Data); 
  #if RFID_DEBUG_ON>1
    USB.printf(" %02x",Data);
  #endif
}

/*  Build packet and send to RS485 */ 
void ClouRFID::SendPacket(ClouRFID_Mes_t* Mess){
  //On TX
  W485.reception(DISABLE);
  W485.transmission(ENABLE);
  delay(2);
  #if RFID_DEBUG_ON>1
    USB.printf("\nRFID send:   ");
  #endif
  
  //Frame head
  SendByte((uint8_t)(CR_HEAD)); 

  CRC=0;
  
  //Protocol control word 
  if(RS485on>0){
    //Serial device address for RS485
    Mess->Control|=CR_IT_RS485;
  }
  
  SendByte(Mess->Control);
  CalcCRC16(&CRC,Mess->Control);
  SendByte(Mess->MessageID);
  CalcCRC16(&CRC,Mess->MessageID);
  
  if(RS485on>0){
    //Serial device address for RS485
    SendByte(RS485addr);
    CalcCRC16(&CRC,RS485addr);
  }

  //Data content length 
  if(Mess->Len>ClouRFID_MaxDataLen) Mess->Len=ClouRFID_MaxDataLen;
  SendByte((uint8_t)(Mess->Len>>8));
  CalcCRC16(&CRC,(uint8_t)(Mess->Len>>8));
  SendByte((uint8_t)(Mess->Len&0xFF));
  CalcCRC16(&CRC,(uint8_t)(Mess->Len&0xFF));

  //Send data
  for(uint16_t i=0;i<Mess->Len;i++){
    SendByte((uint8_t)Mess->Data[i]); 
    CalcCRC16(&CRC,Mess->Data[i]); 
  }


  //Send CRC
  SendByte((uint8_t)(CRC>>8));
  SendByte((uint8_t)(CRC&0xFF));

  #if RFID_DEBUG_ON>1
    USB.printf("\n");
  #endif
}



uint8_t ClouRFID::GetPacket(ClouRFID_Mes_t* Mess){
  #if RFID_DEBUG_ON>1
    USB.printf("\nRFID resive: ");
  #endif
  //Process FIFO
  #if RFID_DEBUG_ON>0
  uint8_t Line=0;
  #endif
  uint8_t Data=0;
  while (W485.available()) {
      Data = W485.read();
      #if RFID_DEBUG_ON>1
        USB.printf(" %02x",Data);
        Line++;
        if(Line>16){
          Line=0;
          USB.printf("\n             ");
        }
      #endif
      switch(PackState){
        //case 0 in default section
        case 1: //Protocol control word MSB
          CalcCRC16(&CRC,Data);
          Mess->Control=Data;
          PackState++;
          break;
        case 2: //Protocol control word LSB
          CalcCRC16(&CRC,Data);
          Mess->MessageID=Data;
          PackState++;
          if((Mess->Control&CR_IT_RS485)==0)PackState++; //Skip addres if not RS485
          break;
        case 3: //RS485 addres
          CalcCRC16(&CRC,Data);
          PackState++;
          break;
        case 4: //Data content length MSB
          CalcCRC16(&CRC,Data);
          Temp=Data;
          Temp<<=8;
          PackState++;
          break;
        case 5: //Data content length LSB
          CalcCRC16(&CRC,Data);
          Temp+=Data;
          Mess->Len=Temp;
          Temp=0;
          PackState++;
          if(Mess->Len==0) PackState++; //Skip data  
          if(Mess->Len>ClouRFID_MaxDataLen){
            PackState=0;
            #if RFID_DEBUG_ON>0
              USB.printf("\nRFID ERROR packet too long %d bytes",Mess->Len);
            #endif
          }         
          break;
        case 6: //Message data 
          CalcCRC16(&CRC,Data);
          Mess->Data[Temp]=Data;
          Temp++;
          if(Temp>=Mess->Len) PackState++;
          break;
        case 7: //СRC MSB 
          Temp=Data;
          Temp<<=8;
          PackState++;
          break;
        case 8: //СRC LSB 
          Temp+=Data;
          if(Temp==CRC){
            #if RFID_DEBUG_ON>1
              USB.printf(" CRC OK");
            #endif
            return 0;
          }
          PackState=0;
          #if RFID_DEBUG_ON>0
            USB.printf("\nRFID ERROR CRC %04x != %04x",CRC,Temp);
          #endif
          break;
         default: //Frame head and wrong state
          if(Data==CR_HEAD){
            CRC=0;
            PackState=1;
            #if RFID_DEBUG_ON>0
                USB.printf("\n             ");
            #endif
          }
          break;
      }
  }
  return 0xFF;
  #if RFID_DEBUG_ON>1
    USB.printf("\n");
  #endif
}

/*Illegal command response detection */ 
uint8_t ClouRFID::ErrorFilter(ClouRFID_Mes_t* Mess){
  if(((Mess->Control&CR_IT_RINI)!=0)&& //Means this message is initiated by reader.
      (Mess->MessageID==CR_ERR)&&     //Illegal command response
      (Mess->Len==6)){     //6 bit in error message 
    #if RFID_DEBUG_ON>0
      USB.printf("\nRFID ERROR Illegal command %x %x %x%x %x%x",
      Mess->Data[0],Mess->Data[1],Mess->Data[2],
      Mess->Data[3],Mess->Data[4],Mess->Data[5]);
    #endif
    return 0xFF;
  }
  return 0;
}

/*Get response with timeout */
uint8_t ClouRFID::GetResp(ClouRFID_Mes_t* Mess){
  //On RX
  W485.transmission(DISABLE);
  W485.reception(ENABLE);
  uint8_t RetI=5;
  PackState=0; CRC=0; Temp=0; //Reset parse state
  while(RetI!=0){ //response received
      //response is walid
      if((GetPacket(Mess)==0)&&(!ErrorFilter(Mess))) return 0;
      RetI--;
      delay(10);
  }
  return 1;
}


/*Stop RFID operations */ 
void ClouRFID::StopRFID(){
  uint8_t Ret=5;
  while(Ret!=0){
    //stopping all RFID operations, & reader enter idle status.
    cMess.Control=CR_MT_RFID;
    cMess.MessageID=CR_RFID_StopCommand;
    cMess.Len=0;
    //Send message
    SendPacket(&cMess);
    delay(100);
    //Wait for response
    if((GetResp(&cMess)==0)&&(cMess.MessageID==CR_RFID_StopCommand)){
      if(cMess.Data[0]==0){
        #if RFID_DEBUG_ON>0
          USB.printf("\nRFID Stop OK");
        #endif
        return;
      }
      #if RFID_DEBUG_ON>0
        USB.printf("\nRFID Stop FAIL");
      #endif
    }
  }
  delay(200);
}


/* Parse EPC read response and update tag FIFO */
void ClouRFID::AddTag(ClouRFID_Mes_t* Mess){

    uint16_t temp=0; uint16_t index=2; uint16_t End;
    ClouRFID_Tag_t Tag_Tmp;
    
    //EPC LEN
    uint16_t Tmp_Len=Mess->Data[0]; Tmp_Len<<=8;  Tmp_Len+=Mess->Data[1];

    //Parse EPC
    #if ClouRFID_EPC_max_len>0
      //Set length
      Tag_Tmp.EPC_Len=Tmp_Len;
      //Read EPC
      temp=0;
      index=2;
      End= Tag_Tmp.EPC_Len>ClouRFID_EPC_max_len ? ClouRFID_EPC_max_len : Tag_Tmp.EPC_Len;
      while((End!=0)&&(index<Mess->Len)){
        Tag_Tmp.EPC[temp++]=Mess->Data[index++];
        End--;
      }
    #endif //ClouRFID_EPC_max_len>0

    //Skip EPC end & PC
    index=Tmp_Len+2+2;

    //Read ANT
    Tag_Tmp.Ant=Mess->Data[index++];
    
    //Read PIDs
    
    if(Mess->Data[index++]==1){//RSSI PID
       Tag_Tmp.RSSIdBm=Mess->Data[index++];
    }
    if(Mess->Data[index++]==2){//tag data read result PID
      if(Mess->Data[index++]!=0){
        #if RFID_DEBUG_ON>0
           USB.printf("\nRFID ERROR tag read %02x ",cMess.Data[index]);
        #endif
        return;
      }
    }

    if(Mess->Data[index++]==3){//Tag TID data  PID
      Tmp_Len=Mess->Data[index++]; Tmp_Len<<=8;  Tmp_Len+=Mess->Data[index++];
      #if ClouRFID_TID_max_len>0
        Tag_Tmp.TID_Len=Tmp_Len;
      #endif //ClouRFID_TID_max_len>0
      Tmp_Len+=index;
      #if ClouRFID_TID_max_len>0
        End=Tag_Tmp.TID_Len>ClouRFID_TID_max_len ? ClouRFID_TID_max_len :  Tag_Tmp.TID_Len;
        //Read TID
        temp=0;
        while((End!=0)&&(index<Mess->Len)){
          Tag_Tmp.TID[temp++]=Mess->Data[index++];
          End--;
        }
      #endif //ClouRFID_TID_max_len>0
      index=Tmp_Len;
    }
    
    /* +++ User data load here */

    //Find same tags in FIFO
    if(tagFIFO_count){

      temp=tagFIFO_out;
      uint8_t match=0;
      while(temp!=tagFIFO_in){
        
        /* EPC match test */
        #if ClouRFID_EPC_max_len>0
          if(tagFIFO[temp].EPC_Len==Tag_Tmp.EPC_Len){                                                     //if EPC == 0 it not add to FIFO
            End= Tag_Tmp.EPC_Len>ClouRFID_EPC_max_len ? ClouRFID_EPC_max_len : Tag_Tmp.EPC_Len;           //number of bytes for compare
            index=0;                                                                                      //compare index
            while(End>0){ 
              if(Tag_Tmp.EPC[index] != tagFIFO[temp].EPC[index]) break;                                   //compare, break if no match
              End--;  index++; 
            }                     
            if(End>0){                                                                                      //EPC does not match
              temp=(temp>=ClouRFID_TAG_FIFO_len-1) ? 0 : (temp+1);                                          //to next tag in FIFO
              continue;                                                                                     //go to next tag in FIFO (next loop of while(temp!=tagFIFO_in) )
            }
          }
        #endif //ClouRFID_EPC_max_len>0

        /* TID match test */
        #if ClouRFID_TID_max_len>0
          if(tagFIFO[temp].TID_Len==Tag_Tmp.TID_Len){
            End= Tag_Tmp.TID_Len>ClouRFID_TID_max_len ? ClouRFID_TID_max_len : Tag_Tmp.TID_Len; //number of bytes for compare
            index=0;                                                                                        //compare index
            while(End>0){ 
              if(Tag_Tmp.TID[index] != tagFIFO[temp].TID[index]) break;                                     //compare, break if no match
              End--;  index++; 
            }                     
            if(End>0){                                                                                      //TID does not match
              temp=(temp>=ClouRFID_TAG_FIFO_len-1) ? 0 : (temp+1);                                          //to next tag in FIFO
              continue;                                                                                     //go to next tag in FIFO (next loop of while(temp!=tagFIFO_in) )
            }                                                                            
          }
        #endif //ClouRFID_EPC_max_len>0

        /* +++ User data match test here */
        
        if(Tag_Tmp.RSSIdBm>tagFIFO[temp].RSSIdBm){ //better signal 
          tagFIFO[temp].RSSIdBm=Tag_Tmp.RSSIdBm;
          tagFIFO[temp].Ant=Tag_Tmp.Ant;
        }
        #if RFID_DEBUG_ON>0
           USB.printf("\nRFID EPC and/or TID match");
        #endif
        return; //EPC and/or TID match - no need add tag in fifo
       }
     }
    
    //Add tag to FIFO
    memcpy((uint8_t*)(&tagFIFO[tagFIFO_in]),(uint8_t*)(&Tag_Tmp),sizeof(ClouRFID_Tag_t));
    //Update in index
    tagFIFO_in++;
    if(tagFIFO_in>=ClouRFID_TAG_FIFO_len+1){tagFIFO_in=0;}
    //Update data count
    tagFIFO_count++;
    if(tagFIFO_count>ClouRFID_TAG_FIFO_len){tagFIFO_count=ClouRFID_TAG_FIFO_len;}
}

