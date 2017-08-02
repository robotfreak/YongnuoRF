// Yongnuo RF-603 sniffer
// using n A7105 chip and a Arduino /pro/micro with 3.3V
// based on Flysky Tx Code by midelic on RCgroups.com
//
// created by RobotFreak & Seegel Systeme
// www.photofreak.de

#include "a7105.h"

const uint8_t CHANNEL = 1; /* 1 - 16 */

const uint8_t RX_MODE_TRX = 1; //receive FOCUS & SHOOT commands from 603 in TRX mode
const uint8_t RX_MODE_TX = 0; //receive FLASH commands from 603 in TX mode or TRX mode & mounted on cam

const uint8_t RX_MODE = RX_MODE_TX;


const uint8_t id[4] = { 0x35, 0x99, 0x9A, 0x5A };
const uint8_t PAYLOAD_SIZ = 2;
const uint8_t RX_SIZE = 4;

const uint8_t channel_table[16] = {
  0x71, 0x6B, 0x65, 0x59, 0x53, 0x4D, 0x41, 0x3B, 0x35, 0x29, 0x23, 0x1D, 0x17, 0x11, 0x0B, 0x05
};

const uint8_t TRX_CH_OFFSET = 2; //in TRX mode, different channels are used!
const uint8_t RX_CH_OFFSET = 1; //Receiving one IF lower

static const uint8_t A7105_regs[] = {
  0xFF, // MODE_REG           0x00
  0x42, // MODECTRL_REG       0x01
  0xFF, // CALIBRATION_REG    0x02
  RX_SIZE - 1, // FIFO1_REG   0x03
  0x00, // FIFO2_REG          0x04
  0x00, // FIFO_REG           0x05
  0x00, // IDCODE_REG         0x06
  0x00, // RCOSC1_REG         0x07
  0x00, // RCOSC2_REG         0x08
  0x00, // RCOSC3_REG         0x09
  0x00, // CKO_REG            0x0A
  0x01, // GPIO1_REG          0x0B
  0x21, // GPIO2_REG          0x0C
  0x05, // CLOCK_REG          0x0D
  0x00, // DATARATE_REG       0x0E
  0x50, // PLL1_REG           0x0F
  0x9E, // PLL2_REG           0x10
  0x4B, // PLL3_REG           0x11
  0x00, // PLL4_REG           0x12
  0x02, // PLL5_REG           0x13
  0x16, // TX1_REG            0x14
  0x2B, // TX2_REG            0x15
  0x12, // DELAY1_REG         0x16
  0x00, // DELAY2_REG         0x17
  0x62, // RX_REG             0x18
  0x80, // RXGAIN1_REG        0x19
  0x80, // RXGAIN2_REG        0x1A
  0x00, // RXGAIN3_REG        0x1B
  0x0A, // RXGAIN4_REG        0x1C
  0x32, // RSSI_REG           0x1D
  0xC3, // ADC_REG            0x1E
  0x07, // CODE1_REG          0x1F
  0x16, // CODE2_REG          0x20
  0x00, // CODE3_REG          0x21
  0x00, // IFCAL1_REG         0x22
  0x00, // IFCAL2_REG         0x23
  0x00, // VCOCCAL_REG        0x24
  0x00, // VCOCAL1_REG        0x25
  0x3B, // VCOCAL2_REG        0x26
  0x00, // BATTERY_REG        0x27
  0x17, // TXTEST_REG         0x28
  0x47, // RXDEM1_REG         0x29
  0x80, // RXDEM2_REG         0x2A
  0x03, // CPC_REG            0x2B
  0x01, // CRYSTALTEST_REG    0x2C
  0x45, // PLLTEST_REG        0x2D
  0x18, // VCOTEST1_REG       0x2E
  0x00, // VCOTEST2_REG       0x2F
  0x01, // IFAT_REG           0x30
  0x0F, // RSCALE_REG         0x31
  0x00, // FILTERTEST_REG     0x32
};

//Spi Comm.pins with A7105/PPM
#define SDI_pin 11 //SDIO-D11 
#define SCLK_pin 13 //SCK-D13
#define CS_pin 10//CS-D10
#define GIO_pin 2
//---------------------------------
#define  CS_on PORTB |= 0x04 //D10
#define  CS_off PORTB &= 0xFB //D10
//
#define  SCK_on PORTB |= 0x20//D13
#define  SCK_off PORTB &= 0xDF//D13
#define  SDI_on PORTB |= 0x08 //D11
#define  SDI_off PORTB &= 0xF7 //D11
//
#define  SDI_1 (PINB & 0x08) == 0x08 //D11
#define  SDI_0 (PINB & 0x08) == 0x00 //D11
//
#define RED_LED_pin A3
#define Red_LED_ON  PORTC |= _BV(3);
#define Red_LED_OFF  PORTC &= ~_BV(3);
#define NOP() __asm__ __volatile__("nop")

//########## Variables #################
static uint8_t aid[4];//for debug only
uint8_t in[RX_SIZE];
uint8_t out[PAYLOAD_SIZ];


void sendFlash() {
  _spi_write_adress(0x0F, channel_table[CHANNEL - 1]);
  uint8_t data[PAYLOAD_SIZ] = {0x88, 0x77};
  for (uint8_t c = 0; c < 10; c++) {
    A7105_WriteData(PAYLOAD_SIZ, data);
    _spi_strobe(A7105_STANDBY);
    _spi_strobe(A7105_TX);
    delay(12);
  }
  Serial.println("Sent flash");
}

void sendShoot() {
  _spi_write_adress(0x0F, channel_table[CHANNEL - 1] + TRX_CH_OFFSET);
  uint8_t data[PAYLOAD_SIZ] = {0x22, 0xdd};
  for (uint8_t c = 0; c < 30; c++) {
    A7105_WriteData(PAYLOAD_SIZ, data);
    _spi_strobe(A7105_STANDBY);
    _spi_strobe(A7105_TX);
    delay(12);
  }
  Serial.println("Sent shoot");
}

void sendFocus() {
  _spi_write_adress(0x0F, channel_table[CHANNEL - 1] + TRX_CH_OFFSET);
  uint8_t data[PAYLOAD_SIZ] = {0x11, 0xee};
  for (uint8_t c = 0; c < 50; c++) {
    A7105_WriteData(PAYLOAD_SIZ, data);
    _spi_strobe(A7105_STANDBY);
    _spi_strobe(A7105_TX);
    delay(12);
  }
  Serial.println("Sent focus");
}

void startRx() {
  if (RX_MODE == RX_MODE_TRX)
    _spi_write_adress(0x0F, channel_table[CHANNEL - 1] - RX_CH_OFFSET + TRX_CH_OFFSET);
  if (RX_MODE == RX_MODE_TX)
    _spi_write_adress(0x0F, channel_table[CHANNEL - 1] - RX_CH_OFFSET);
    
  _spi_strobe(A7105_STANDBY);
  _spi_strobe(A7105_RST_RDPTR);
  _spi_strobe(A7105_RX);  
}

void setup() {
  Serial.begin(57600);
  pinMode(RED_LED_pin, OUTPUT);
  //RF module pins
  pinMode(SDI_pin, OUTPUT);//SDI   SDIO
  pinMode(SCLK_pin, OUTPUT);//SCLK SCL
  pinMode(CS_pin, OUTPUT);//CS output
  CS_on;//start CS high
  SDI_on;//start SDIO high
  SCK_off;//start sck low

  //
  //for debug
  delay(10);//wait 10ms for A7105 wakeup
  A7105_reset();//reset A7105
  delay(100);
  Serial.print("ID: ");
  A7105_ReadID();//for debug only
  A7105_DumpRegs();   // Dump registers
  A7105_WriteID((uint8_t *) id);
  A7105_Calibrate();  // calibrate A7105

  startRx();
}

//############ MAIN LOOP ##############
void loop() {
  int i, c;
  char cmd;
  
  if (digitalRead(GIO_pin) == LOW)   {
    A7105_ReadData(RX_SIZE, in);
    startRx();
  }
  else
  {
    if (Serial.available())
    {
      cmd = Serial.read();
      switch (cmd)
      {
        case 'f':
          sendFocus();
          startRx();
          break;
          
        case 's':
          sendShoot();
          startRx();
          break;

        case 'l':
          sendFlash();
          startRx();
          break;
          
        default:
          break;
      }
    }
  }
}

//-------------------------------
//-------------------------------
//A7105 SPI routines
//-------------------------------
//-------------------------------

//--------------------------------------
void A7105_reset(void) {
  _spi_write_adress(0x00, 0x00);
}

//--------------------------------------
void A7105_WriteID(uint8_t *ida) {
  int i;
  CS_off;
  _spi_write(0x06);
  for (i = 0; i < 4; i++)
  {
    _spi_write(*ida);
    Serial.print(*ida, HEX);
    ida++;
  }
  Serial.println("");
  CS_on;
}

//--------------------------------------
void A7105_ReadID() {
  int i;
  CS_off;
  _spi_write(0x46);
  for (i = 0; i < 4; i++) {
    aid[i] = _spi_read();
  }
  CS_on;
}

//--------------------------------------
void A7105_DumpRegs() {
  int i;
  uint8_t r;
  for (i = 0; i < 0x33; i++) {

    if (A7105_regs[i] != 0xff)
      _spi_write_adress(i, A7105_regs[i]);
    r = _spi_read_adress(i);
    Serial.print(r, HEX);
    Serial.print(" ");
  }
  Serial.println("");
}

//--------------------------------------
void A7105_Calibrate() {
  uint8_t if_calibration1;
  uint8_t vco_calibration0;
  uint8_t vco_calibration1;

  _spi_strobe(0xA0);//stand-by
  _spi_write_adress(0x02, 0x01);
  while (_spi_read_adress(0x02)) {
    if_calibration1 = _spi_read_adress(0x22);
    if (if_calibration1 & 0x10) { //do nothing
    }
  }
  _spi_write_adress(0x24, 0x13);
  _spi_write_adress(0x26, 0x3b);
  _spi_write_adress(0x0F, 0x00); //channel 0
  _spi_write_adress(0x02, 0x02);
  while (_spi_read_adress(0x02)) {
    vco_calibration0 = _spi_read_adress(0x25);
    if (vco_calibration0 & 0x08) { //do nothing
    }
  }
  _spi_write_adress(0x0F, 0xA0);
  _spi_write_adress(0x02, 0x02);
  while (_spi_read_adress(0x02)) {
    vco_calibration1 = _spi_read_adress(0x25);
    if (vco_calibration1 & 0x08) { //do nothing
    }
  }
  _spi_write_adress(0x25, 0x08);
  _spi_write_adress(0x28, 0x1F); //set power to 1db maximum
  _spi_strobe(0xA0);//stand-by strobe command
  delay(100);
  _spi_strobe(A7105_RST_WRPTR);
  _spi_write_adress(0x0F, 0x70);
  _spi_strobe(A7105_STANDBY);
  _spi_strobe(A7105_RST_RDPTR);
  _spi_strobe(A7105_RX);
}

//--------------------------------------
void A7105_ReadData(int len, uint8_t * data) {
  uint8_t i;
  CS_off;
  _spi_write(0x45);
  Serial.print("<");
  for (i = 0; i < len; i++) {
    *data = _spi_read();
    Serial.print(*data, HEX);
    Serial.print(" ");
    data++;
  }
  Serial.println("");
  CS_on;
}

//--------------------------------------
void A7105_WriteData(int len, uint8_t * data) {
  int i;
  CS_off;
  _spi_write(0x05);
  for (i = 0; i < len; i++)   {
    _spi_write(*data);
    data++;
  }
  CS_on;
}

//--------------------------------------
void _spi_write(uint8_t command) {
  uint8_t n = 8;
  SCK_off;
  SDI_off;
  while (n--) {
    if (command & 0x80)
      SDI_on;
    else
      SDI_off;
    SCK_on;
    NOP();
    SCK_off;
    command = command << 1;
  }
  SDI_on;
}
//--------------------------------------
void _spi_write_adress(uint8_t address, uint8_t data) {
  CS_off;
  _spi_write(address);
  NOP();
  _spi_write(data);
  CS_on;
}
//-----------------------------------------
uint8_t _spi_read(void) {
  uint8_t result;
  uint8_t i;
  result = 0;
  pinMode(SDI_pin, INPUT); //make SDIO pin input
  //SDI_on;
  for (i = 0; i < 8; i++) {
    if (SDI_1) ///if SDIO =1
      result = (result << 1) | 0x01;
    else
      result = result << 1;
    SCK_on;
    NOP();
    SCK_off;
    NOP();
  }
  pinMode(SDI_pin, OUTPUT); //make SDIO pin output again
  return result;
}
//--------------------------------------------
uint8_t _spi_read_adress(uint8_t address) {
  uint8_t result;
  CS_off;
  address |= 0x40;
  _spi_write(address);
  result = _spi_read();
  CS_on;
  return (result);
}
//------------------------
void _spi_strobe(uint8_t address) {
  CS_off;
  _spi_write(address);
  CS_on;
}

















