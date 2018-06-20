# ClouRFID
Clou RFID reader minimal driver for waspmote (RS232/RS485) 

# Installation
Copy ClouRFID.cpp, ClouRFID.h, keywords.txt in dirrectory ~/Documents/Waspmote/libraries 
(for Waspmote project ([waspmote IDE](https://dillinger.io/)))

# Usage

Before start using driver select reading mode: EPC or/and TID maximum length in ClouRFID.h

Read EPC and TID (ClouRFID.h): 
```
#define ClouRFID_EPC_max_len  12                          //Len of EPC code - 96bits / 12 bytes
#define ClouRFID_TID_max_len  12                          //Len of TID code - 96bits / 12 bytes
```
Read only EPC (ClouRFID.h):
```
#define ClouRFID_EPC_max_len  12                          //Len of EPC code - 96bits / 12 bytes
#define ClouRFID_TID_max_len  0                           //Len of TID code - 0bits / 0 bytes
```
Read only TID (ClouRFID.h):
```
#define ClouRFID_EPC_max_len  0                          //Len of EPC code - 0bits / 0 bytes
#define ClouRFID_TID_max_len  12                         //Len of TID code - 12bits / 12 bytes
```
If you use EPC longer then 12 bytes chenge ClouRFID_MaxDataLen  in ClouRFID.h
```
#define ClouRFID_MaxDataLen   24/*Max EPC length*/+ClouRFID_TID_max_len+15  //EPC 192bits / 24 bytes
```
In main project (.pde) file create RFID object (dynamic memory allocation (maloc / new) NOT recomendated)

```
ClouRFID RFID;
```
In loop section add code (minimal code): 

```
void loop()
{
...
  /* Begin using RS485/RS232 */
  if(RFID.Start(115200,RS485,42)==ClouRFID_OK){                   //Connection to reader success 
        /* Using RS485/RS232 */
        uint8_t Ant_Qty=RFID.GetAntQty();                         //Get antenna qty
        for(uint8_t ant=1;ant<=Ant_Qty;ant++) RFID.ScanTags(ant); //Scan tags on all antennas
        RFID.Stop();                                              //Stop connection (stop using RS485 and USB) 
        /* End RS485/RS232 */
        if(RFID.GetTagQty()>0){                                   //Process tags FIFO
          ClouRFID_Tag_t Test_Tag; 
          while(RFID.GetTag(&Test_Tag)==ClouRFID_OK){             //FIFO get tag data success

             /* Process tag data here */

          }
        }
    }
    /* End RS485/RS232 */
...
}
```
