#include <SPI.h>
#include <map>
#include <Arduino.h>

#define TX_BUFFER 0x09
#define TX_FCTRL 0x08
#define  WRITE  0x80; // regular write
#define JUNK 0x00
static uint8 tx_msg[] = {0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6};
const uint8_t PIN_RST = D1; // wake up
const uint8_t PIN_IRQ = D2; // irq pin
const uint8_t PIN_SS = D8; // spi select pin
const SPISettings SPI_SETTINGS = SPISettings(20000000L, MSBFIRST, SPI_MODE0);
volatile boolean messageSent = false;
                                                  
void writeBytes(byte addr, byte data[], int data_size)
{
  // presume nosub address for the moment
  byte header[3];
  uint8_t  headerLen = 1;
  uint16_t  i = 0;
  
  header[0] = 0x80 | addr;
  
  SPI.beginTransaction(SPI_SETTINGS);
  digitalWrite(PIN_SS, LOW);
  for(i = 0; i < headerLen; i++) {
    SPI.transfer(header[i]); // send header
  }
  for(i = 0; i < data_size; i++) {
    SPI.transfer(data[i]); // write values
  }
  delayMicroseconds(5);
  digitalWrite(PIN_SS, HIGH);
  SPI.endTransaction();
}

void readBytes(byte a[],byte cmd, uint16_t n) {
  byte header[3];
  uint8_t headerLen = 1;
  uint16_t i = 0;

  // build SPI header
  header[0] = 0x00 | cmd;
  digitalWrite(PIN_SS, LOW);
  SPI.beginTransaction(SPI_SETTINGS);
  
  for(i = 0; i < headerLen; i++) {
   SPI.transfer(header[i]); // send header
  }
  for(i = 0; i < n; i++) {
    a[i] = SPI.transfer(JUNK); // read values
  }
  delayMicroseconds(5);
  digitalWrite(PIN_SS, HIGH);
  SPI.endTransaction();
}

std::map<std::string, byte> readControl()
{
   // create a hash map to store values regarding the frame controll register
   std::map<std::string, byte> hashMap;
   
   // allocate memory in heap for the data read from the register
   byte * data = new byte[5];

   // read the data
   readBytes(data, TX_FCTRL, 5); 
   short TXBOFFS = data[3] + (data[2] & 0xC0) >> 6;
   byte PE = (data[2] & 0x30) >> 4;
   byte TXPSR = (data[2] & 0x0C) >> 2;
   byte TXPRF = data[2] & 0x03;
   byte TR = (data[1] & 0x80) >> 7;
   byte TXBR = (data[1] & 0x60) >> 5;
   byte TFLEN = (data[0] & 0x7F); // Transmit Frame Length
   byte TFLE = ((data[1] & 0x03) << 1) & ((data[0] & 0x80) >> 7);  // Transmit Frame Length Extension
   hashMap["TXBR"] = TXBR;
   hashMap["TFLEN"] = TFLEN;
   hashMap["TFLE"] = TFLE;
   hashMap["TR"] = TR;
   hashMap["TXPRF"] = TXPRF;
   hashMap["TXPSR"] = TXPSR;
   hashMap["PE"] = PE;
   hashMap["TXBOFFS"] = TXBOFFS;
   delete[] data;
   data = NULL;
   return hashMap;
}

void writeDataToBuffer(byte data[])
{
  writeBytes(0x09, data, 12);
}

void sendMessage()
{
  byte data[4];
  byte startByte = 3;
  data[0] = startByte;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  writeBytes(0x0D, data, 4);
  messageSent = true;
}

std::map<std::string, byte> readSystemEvent()
{
  std::map<std::string, byte> eventRegister;
  byte * data = new byte[5];
  readBytes(data, 0x0F, 5);
  byte TXFRS = (data[0] & 0x80) >> 7;
  byte RXDFR = (data[1] & 0x20) >> 5;
  byte IRQ = (data[0] & 0x01);

  eventRegister["TXFRS"] = TXFRS;
  eventRegister["RXDFR"] = RXDFR;
  eventRegister["IRQ"] = IRQ;

  delete[] data;
  data = NULL;

  return eventRegister;
}

void setup() {
  Serial.begin(9600);
  // initialize the driver
  pinMode(PIN_IRQ, INPUT);
  pinMode(PIN_SS, OUTPUT);
  digitalWrite(PIN_SS, HIGH);
  SPI.begin();
  delay(1000);
}

volatile boolean test;

void checkForMessage()
{
  std::map<std::string, byte> event;
  event = readSystemEvent();
  if(event["IRQ"] == 1)
    Serial.println("message received");
}


void loop() {
//    if(!messageSent){
      Serial.println("sending message");
      writeDataToBuffer(tx_msg);   
      sendMessage();
//      delay(400);
//    }
  
}
