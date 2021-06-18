#include <SPI.h>
#include <map>
#include <Arduino.h>

#define TX_BUFFER 0x09
#define TX_FCTRL 0x08
#define  WRITE  0x80; // regular write
#define JUNK 0x00
#define SYS_STATUS 0x0F
#define SYS_CTRL 0x0D
#define TXFRB_BIT 4
#define TXPRS_BIT 5
#define TXPHS_BIT 6
#define TXFRS_BIT 7
#define TRXOFF_BIT 6

static byte tx_msg1[] = {1, 1, 54, 34, 2, 4, 5, 4, 7, 1, 2, 1};
static byte tx_msg2[] = {2, 2, 54, 34, 2, 4, 5, 4, 7, 1, 2, 2};
static byte tx_msg3[] = {3, 3, 54, 34, 2, 4, 5, 4, 7, 1, 2, 3};
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

void setBit(byte data[], uint16_t n, uint16_t bit, boolean val) {
	uint16_t idx;
	uint8_t shift;
	
	idx = bit/8;
	if(idx >= n) {
		return; // TODO proper error handling: out of bounds
	}
	byte* targetByte = &data[idx];
	shift = bit%8;
	if(val) {
		bitSet(*targetByte, shift);
	} else {
		bitClear(*targetByte, shift);
	}
}

void goToIdleState()
{
  byte sysCtrl[4] = {0};
  setBit(sysCtrl, 4, TRXOFF_BIT, true);
  writeBytes(SYS_CTRL, sysCtrl, 4);
}

void clearTransmitConfig()
{
  byte sysStatus[5];
  setBit(sysStatus, 5, TXFRB_BIT, true);
  setBit(sysStatus, 5, TXPRS_BIT, true);
  setBit(sysStatus, 5, TXPHS_BIT, true);
  setBit(sysStatus, 5, TXFRS_BIT, true);
  writeBytes(SYS_STATUS, sysStatus, 5);
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
  goToIdleState();
  clearTransmitConfig();
  byte data[4];
  byte startByte = 3;
  data[0] = startByte; // SFCST AND TXSTRT are set to 1
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  writeBytes(SYS_CTRL, data, 4);
  messageSent = true;
}

std::map<std::string, byte> readSystemEvent()
{
  std::map<std::string, byte> eventRegister;
  byte * data = new byte[5];
  readBytes(data, SYS_STATUS, 5);
  byte TXFRS = (data[0] & 0x80) >> 7;
  byte RXDFR = (data[1] & 0x20) >> 5; // received data frame
  byte IRQ = (data[0] & 0x01);
  byte RXPRD = data[1] & 0x01; // received preamble
  byte RXSFDD = (data[1] & 0x02) >> 1; 
  byte LDEDONE = (data[1] & 0x04) >> 2; // is timestamp computation done
  byte RXPHD = (data[1] & 0x08) >> 3; // receiver PHD headere detected
  byte RXPHE = (data[1] & 0x10) >> 4; // received PHY header error

  eventRegister["TXFRS"] = TXFRS;
  eventRegister["RXDFR"] = RXDFR;
  eventRegister["IRQ"] = IRQ;
  eventRegister["RXPRD"] = RXPRD;
  eventRegister["RXSFDD"] = RXSFDD;
  eventRegister["LDEDONE"] = LDEDONE;
  eventRegister["RXPHD"] = RXPHD;
  eventRegister["RXPHE"] = RXPHE;  

 Serial.println("------------------");
 Serial.println(eventRegister["TXFRS"]);
 Serial.println(eventRegister["RXDFR"]);
 Serial.println(eventRegister["IRQ"]);
 Serial.println(eventRegister["RXPRD"]);
 Serial.println(eventRegister["RXSFDD"]);
 Serial.println(eventRegister["LDEDONE"]);
 Serial.println(eventRegister["RXPHD"]);
 Serial.println(eventRegister["RXPHE"]);
 Serial.println("------------------");

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
    if(!messageSent){
      Serial.println("sending message"); 
      writeDataToBuffer(tx_msg3);  
      sendMessage();
      Serial.println("sent first message");
      // writeDataToBuffer(tx_msg2);  
      // sendMessage();
      readSystemEvent();
      // delay(7000);
      // writeDataToBuffer(tx_msg2); 
      // sendMessage();
      // Serial.println("sent second message");
      // delay(7000);
      // writeDataToBuffer(tx_msg3); 
      // sendMessage();
      // Serial.println("sent third message");
   }
  
}
