#include <Arduino.h>
#include <can.h>

uint8_t counter = 0;
uint8_t frameLength = 0;
extern void get_ecu_data();
extern void handle_can();
void setup() {
//bool ret = CANInit(CAN_500KBPS, 0);  // CAN_RX mapped to PA11, CAN_TX mapped to PA12
  bool ret = CANInit(CAN_500KBPS, 2);  // CAN_RX mapped to PB8, CAN_TX mapped to PB9
  if (!ret) while(true);

  Serial1.begin(115200);

  
}

void loop() {
  get_ecu_data();

  if(CANMsgAvail()){
    handle_can();
  }
}

#define NEW_CAN_PACKET_SIZE   123
#define CAN_PACKET_SIZE   75
byte fullStatus[NEW_CAN_PACKET_SIZE];
byte cmd;
byte packet_size;
uint32_t tm0;
#define timems 1000

void get_ecu_data(){
  if(millis() - tm0 > timems){
    tm0 = millis();
    Serial1.print("n");
    Serial.println("send command");
    while(!Serial1.available()){
      if(millis() - tm0 > timems){
        Serial.println("timeout");
        continue;
      }
    }
    byte tmp = Serial1.read();
    Serial.println(tmp);
    if(tmp == 'n'){
      cmd = Serial1.read();
      packet_size = Serial1.read();
      Serial.println(cmd);
      Serial.println(packet_size);
      for(int i; i<packet_size;i++){
        if(Serial1.available()){
          fullStatus[i] = Serial1.read();
        }
      }
      Serial.println("got data");
    }
    while(Serial1.available()){Serial1.read();}
  }
}

#define PAD 0x00
#define REPLY_ID 0x7E9
#define LISTEN_ID 0x7E1
#define FUNCTIONAL_ID 0x7DF  
CAN_msg_t CAN_TX_msg;
CAN_msg_t CAN_RX_msg;
void handle_can(){
  CANReceive(&CAN_RX_msg);
  if(CAN_RX_msg.id == FUNCTIONAL_ID){
    byte numofBytes = CAN_RX_msg.data[0];
    byte mode = CAN_RX_msg.data[1] & 0x0F;
    byte pid = CAN_RX_msg.data[2];
    bool tx = false;
    CAN_TX_msg.ch = 0;
    CAN_TX_msg.len = 8;
    CAN_TX_msg.format = STANDARD_FORMAT;
    CAN_TX_msg.type = DATA_FRAME;
    CAN_TX_msg.data[0] = 0x0;// length after 0
    CAN_TX_msg.data[1] = (0x40 | mode); //mode
    CAN_TX_msg.data[2] = pid; //pid
    CAN_TX_msg.data[3] = PAD; //data A
    CAN_TX_msg.data[4] = PAD; //data B
    CAN_TX_msg.data[5] = PAD; //data C
    CAN_TX_msg.data[6] = PAD; //data D
    CAN_TX_msg.data[7] = PAD; //PAD

    if(mode == 0x01){
      if(pid == 0){// pid 0-20 supported
        //04 05 0A 0B 0V 0D 0E 0F 11 12 1C 1D 1F - CURENTLY
        CAN_TX_msg.data[0] = 0x06;
        CAN_TX_msg.data[3] = B00001000;
        CAN_TX_msg.data[4] = B00111110;
        CAN_TX_msg.data[5] = B10100000;
        CAN_TX_msg.data[6] = B00010001;
        tx = true;
      }else if( pid == 1){
        bool MIL = false;
        byte DTC = 0;
        CAN_TX_msg.data[0] = 0x06;
        CAN_TX_msg.data[3] = (MIL << 7) | (DTC & 0x7F);
        CAN_TX_msg.data[4] = 0;
        CAN_TX_msg.data[5] = 0;
        CAN_TX_msg.data[6] = 0;
        tx = true;
      }else if (pid == 5){
        CAN_TX_msg.data[0] = 0x03;
        CAN_TX_msg.data[3] = fullStatus[7];
        tx = true;
      }else if (pid == 11){
        uint16_t temp_engineMap;
        uint16_t _map = fullStatus[4] | (fullStatus[5] << 8);
        temp_engineMap = (highByte(_map - fullStatus[40])<<8) | lowByte(_map - fullStatus[40]);
        CAN_TX_msg.data[0] = 0x03;
        CAN_TX_msg.data[3] = temp_engineMap;
        tx = true;
      }else if (pid == 12){
        uint16_t temp_revs; 
        temp_revs = (fullStatus[14] | (fullStatus[15] << 8)) << 2 ;
        CAN_TX_msg.data[0] = 0x04;
        CAN_TX_msg.data[3] = highByte(temp_revs);
        CAN_TX_msg.data[4] = lowByte(temp_revs);;
        tx = true;
      }else if(pid == 13){
        uint8_t temp_vehiclespeed = min(255, fullStatus[100] | (fullStatus[101] << 8));
        CAN_TX_msg.data[0] = 0x03;
        CAN_TX_msg.data[3] = temp_vehiclespeed;
        tx = true;
      }else if(pid == 14){
        int temp_timingadvance = ((fullStatus[23] +64) << 1);
        CAN_TX_msg.data[0] = 0x03;
        CAN_TX_msg.data[3] = temp_timingadvance;
        tx = true;
      }
      else if(pid == 15){
        CAN_TX_msg.data[0] = 0x03;
        CAN_TX_msg.data[3] = fullStatus[6];
        tx = true;
      }else if(pid == 17){
        CAN_TX_msg.data[0] = 0x03;
        CAN_TX_msg.data[3] = fullStatus[24];
        tx = true;
      }else if(pid == 19){
        CAN_TX_msg.data[0] = 0x03;
        CAN_TX_msg.data[3] = B10001000;
        tx = true;
      }
      
      else{
        CAN_TX_msg.data[3] = 0x12;
        tx = true;
      }

    }else{
      CAN_TX_msg.data[3] = 0x12;
      tx = true;
    }

    if(tx){
      CANSend(&CAN_TX_msg);
    }
  }
}