/*
 WhereOTechno

 runs on Lilygo T-Echo

add https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
to board manager list

install 'Adafruit nRF52 by Adafruit' (not 'Arduino nRF52 boards')
 select Nordic nRF52840 DK something

 

 getting-started: https://github.com/Xinyuan-LilyGO/T-Echo

 memory layout facts: https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide/hathach-memory-map#sram-layout-39-8


 */

#include "utilities.h"
#include <SPI.h>
#include <Wire.h>

#include <GxEPD.h> // GxEPD by Jean-Marc Zingg
//#include <GxGDEP015OC1/GxGDEP015OC1.h>    // 1.54" b/w
//#include <GxGDEH0154D67/GxGDEH0154D67.h>  // 1.54" b/w
#include <GxDEPG0150BN/GxDEPG0150BN.h>  // 1.54" b/w 

#include GxEPD_BitmapExamples
#include <Fonts/FreeMonoBold12pt7b.h>
#include <GxIO/GxIO_SPI/GxIO_SPI.h>
#include <GxIO/GxIO.h>

#include <TinyGPS++.h> // TinyGPSPlus by Mikal Hart

#include <RadioLib.h> // RadioLib by Jan Gromes 6.5.0

#include "fix.h"
#include "fixes.h"


void setupDisplay();
void enableBacklight();
void configVDD(void);
void boardInit();

bool setupGPS();
void loopGPS();

SPIClass        *dispPort  = nullptr;
SPIClass        *rfPort    = nullptr;
GxIO_Class      *io        = nullptr;
GxEPD_Class     *display   = nullptr;
SX1262          radio      = nullptr; 

uint32_t        lastFixSendMillis = 0;

uint32_t        lastGpsFixMillis = 0;

uint8_t rgb = 0;

TinyGPSPlus     *gps;



Fix myFix;
Fix theirFix;

Fixes* myFixes;
Fixes* theirFixes;

double maxRange = 0;

char* techno = (char*) "Techno!";


// flag to indicate that a packet was sent
volatile bool transmittedFlag = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;
// flag to indicate that a packet was received
volatile bool receivedFlag = false;


// save transmission state between loops
int transmissionState =  RADIOLIB_ERR_NONE;

uint8_t sendBuf[200];
int16_t sendBufSize = 0;

uint8_t receiveBuf[200];
uint16_t receiveBufSize = 0;

int displayState = 0;

uint32_t fixSendInterval = 30000;

uint32_t blinkStartMillis = 0;
uint32_t bootMillis = 0;

#define CRASH_TRACE 0

bool doSend = true;
bool doReceive = true;


void setup()
{
  bootMillis = millis();

  myFixes = new Fixes();
  theirFixes = new Fixes();

  if( getMacAddress() == 0x67CDF1D2 )
  {
    doSend = true;
    doReceive = false;
    Serial.println("I'm a sender");
  }

  if( getMacAddress() == 0x52A2DF5C )
  {
    doSend = false;
    doReceive = true;

    Serial.println("I'm a receiver");
  }

  Serial.begin(115200);
  delay(4000);
  boardInit();
  delay(200);
   
}



// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setReceivedFlag(void)
{
    // check if the interrupt is enabled
    if (!enableInterrupt) {
        return;
    }

    // we got a packet, set the flag
    receivedFlag = true;
}

bool loopTouchPin()
{
  static bool lastB = false;

  bool refreshDisplay = false;

  bool b = digitalRead(UserButton_Pin);

  if( b && ! lastB )
  {
    nextDisplayState();
    refreshDisplay = true;
  }


  lastB = b;  

  return refreshDisplay;
}

#define DISPLAY_STATE_MY_MAP 0
#define DISPLAY_STATE_THEIR_MAP 1
#define DISPLAY_STATE_FIXES 2
#define DISPLAY_STATE_MY_FIX 3
#define DISPLAY_STATE_THEIR_FIX 4

void nextDisplayState()
{

  if(doReceive && ! doSend)
  {
    switch( displayState )
    {
      case DISPLAY_STATE_MY_MAP:
        displayState = DISPLAY_STATE_THEIR_MAP;
        break;

      case DISPLAY_STATE_THEIR_MAP:
        displayState = DISPLAY_STATE_FIXES;
        break;

      case DISPLAY_STATE_FIXES:
        displayState = DISPLAY_STATE_MY_FIX;
        break;
      
      case DISPLAY_STATE_MY_FIX:
        displayState = DISPLAY_STATE_THEIR_FIX;
        break;

      case DISPLAY_STATE_THEIR_FIX:
      default:
        displayState = DISPLAY_STATE_MY_MAP;
        break; 
    }
  }
  else if(!doReceive && doSend)
  {
    switch( displayState )
    {
      case DISPLAY_STATE_MY_MAP:
      case DISPLAY_STATE_THEIR_MAP:
      case DISPLAY_STATE_FIXES:
       displayState = DISPLAY_STATE_MY_FIX;
        break;

      case DISPLAY_STATE_MY_FIX:
      case DISPLAY_STATE_THEIR_FIX:
      default:
        displayState = DISPLAY_STATE_MY_MAP;
        break; 
    }
  }
  else // both
  {
    switch( displayState )
    {
      case DISPLAY_STATE_MY_MAP:
        displayState = DISPLAY_STATE_THEIR_MAP;
        break;

      case DISPLAY_STATE_THEIR_MAP:
        displayState = DISPLAY_STATE_FIXES;
        break;

      case DISPLAY_STATE_FIXES:
       displayState = DISPLAY_STATE_MY_FIX;
        break;

      case DISPLAY_STATE_MY_FIX:
        displayState = DISPLAY_STATE_THEIR_FIX;
        break;

      case DISPLAY_STATE_THEIR_FIX:
      default:
        displayState = DISPLAY_STATE_MY_MAP;
        break; 
    }
  }
}

uint32_t lastDisplayMillis = 0;
uint32_t displayInterval = 10000;

void loop()
{

  if( CRASH_TRACE ) Serial.print("loop1\n");


  if( doReceive )
    loopReceive();

  if( CRASH_TRACE ) Serial.print("loop2\n");

  loopGPS();
  
  if( CRASH_TRACE ) Serial.print("loop3\n");

  bool refreshDisplay = loopTouchPin();

  if( CRASH_TRACE ) Serial.print("loop4\n");

  if( refreshDisplay || millis() - lastDisplayMillis > displayInterval)
  {
    loopDisplay();
    blinkStartMillis = millis();
    lastDisplayMillis = millis();
    digitalWrite(GreenLed_Pin, LOW);
    digitalWrite(RedLed_Pin, HIGH);
    digitalWrite(BlueLed_Pin, HIGH);
  }
  

  if( millis() - blinkStartMillis > 300 ) // short blink every refresh
  {
     digitalWrite(GreenLed_Pin, HIGH);
     digitalWrite(RedLed_Pin, HIGH);
     digitalWrite(BlueLed_Pin, HIGH);
  }

  if( doSend )
  {
  if (millis() - lastFixSendMillis > fixSendInterval) 
  {

    if( CRASH_TRACE ) Serial.print("loop8\n");

    loopSender();

    if( CRASH_TRACE ) Serial.print("loop9\n");

    lastFixSendMillis = millis() + random(2000); // randomise sending so we don't risk overriding the other end   
  }
}
 
  if( CRASH_TRACE ) Serial.print("loop end\n");

}

void loopSender()
{
    sendBufSize = myFix.encode(sendBuf);

    //strcpy((char*)sendBuf, "qwerqwerqwerqwerqwerqwerqwerqwrqwerqwerqwer");

    if( sendBufSize < 1 )
    {
      SerialMon.println(F("no packet to send"));
      return;
    }

    SerialMon.print(F("[SX1262] Transmitting packet size "));
    SerialMon.print(sendBufSize);
    SerialMon.print(" ");
    
    int state = radio.transmit(sendBuf, sendBufSize);
    

    if (state == RADIOLIB_ERR_NONE) {
        // the packet was successfully transmitted
        SerialMon.println(F("success!"));

        // print measured data rate
        SerialMon.print(F("[SX1262] Datarate:\t"));
        SerialMon.print(radio.getDataRate());
        SerialMon.println(F(" bps"));

    } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
        // the supplied packet was longer than 256 bytes
        SerialMon.println(F("too long!"));

    } else if (state ==  RADIOLIB_ERR_TX_TIMEOUT) {
        // timeout occured while transmitting packet
        SerialMon.println(F("timeout!"));

    } else {
        // some other error occurred
        SerialMon.print(F("failed, code "));
        SerialMon.println(state);

    }


}




void loopDisplay(void)
{
  if( displayState == DISPLAY_STATE_MY_MAP)
    displayMap(myFixes, theirFixes, false, "me");
  else if( displayState == DISPLAY_STATE_THEIR_MAP)
    displayMap(theirFixes, myFixes, true, "them");
  else if( displayState == DISPLAY_STATE_FIXES)
    displayFixes();
  else if( displayState == DISPLAY_STATE_MY_FIX)
    displayFix(myFix, "My Fix"); 
  else if( displayState == DISPLAY_STATE_THEIR_FIX)
    displayFix(theirFix, "Their Fix");    
  
  display->update();
}


void displayMap(Fixes *fixes, Fixes *other, bool drawOther, const char*label)
{
  if( CRASH_TRACE )  Serial.println("map1");

  display->fillScreen(GxEPD_WHITE);
  //display->drawExampleBitmap(BitmapExample1, 0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, GxEPD_BLACK);
  display->setCursor(0,20);
  //display->print("age ");
  //display->println(myFixAge/1000);

  if( CRASH_TRACE ) Serial.println("map2");
 

  display->print(label);
  display->print(" (");

  if( CRASH_TRACE ) Serial.println("map3");
 
  display->print(fixes->numFixes);
  display->print(") ");
  
  if( CRASH_TRACE ) Serial.println("map4");
 
  //display->drawCircle(30,30, 10, GxEPD_BLACK); 
  //display->drawCircle(50,50,20,GxEPD_BLACK); 

  fixes->calcScale(myFix, display->width(), display->height());

  display->print(fixes->screenWidthInM,0);
  display->print("m");


  if( CRASH_TRACE ) Serial.println("map5");
 

  double lastX = 0;
  double lastY = 0;

  
  if( CRASH_TRACE ) Serial.println("map7");
 
  // draw a 10m square in the screen center
  double cw = fixes->pixForM(10);
  double r1 = display->width()/2.0 - cw/2.0;
  
  display->drawRect(r1, r1, cw, cw, 0);

  cw = fixes->pixForM(100);
  r1 = display->width()/2.0 - cw/2.0;
  display->drawRect(r1, r1, cw, cw, 0);

  cw = fixes->pixForM(1000);
  r1 = display->width()/2.0 - cw/2.0;
  display->drawRect(r1, r1, cw, cw, 0);

  if( CRASH_TRACE ) Serial.println("map8");

  bool first = true; 

  int i = fixes->numFixes - 1;
  while( i >= 0 )
  {
 
    if( CRASH_TRACE ) Serial.println("map9");
 
   
   
    if( fixes->fixes[i].goodFix())
    {
      double x = fixes->x(i);
      double y = fixes->y(i);

      if( ! first )
      {
        if( i < 10 )
        { // highlight the last few segments
          display->drawLine(x+1, y, lastX+1, lastY, GxEPD_WHITE);
          display->drawLine(x-1, y, lastX-1, lastY, GxEPD_WHITE);
          display->drawLine(x, y+1, lastX, lastY+1, GxEPD_WHITE);
          display->drawLine(x, y-1, lastX, lastY-1, GxEPD_WHITE);
        }
        display->drawLine(x, y, lastX, lastY, GxEPD_BLACK);

      }  

      if( i == 0 )
      {
        display->fillCircle(x,y, 8, GxEPD_WHITE);
        display->drawCircle(x,y, 6, GxEPD_BLACK);
        
      }
      else
      {
        display->drawCircle(x,y, 2, GxEPD_BLACK);
      }

      lastX = x;
      lastY = y;
      first = false;
    

      
    }

    if( CRASH_TRACE ) Serial.println("map9");
 
    i--;
    
  }

  first = true;
  if( drawOther )
  {
  // draw us on the map

    
    i = min( 20, other->numFixes - 1);
    while( i >= 0 ) // draw 0 last
    {
      if( CRASH_TRACE ) Serial.println("map6");
 
     
      if( other->fixes[i].goodFix())
      {
        double mx = fixes->x(other->fixes[i].lng);
        double my = fixes->y(other->fixes[i].lat);
    
        if( ! first )
        {
          if( i < 10 )
          { // highlight the last few segments
            display->drawLine(mx+1, my, lastX+1, lastY, GxEPD_WHITE);
            display->drawLine(mx-1, my, lastX-1, lastY, GxEPD_WHITE);
            display->drawLine(mx, my+1, lastX, lastY+1, GxEPD_WHITE);
            display->drawLine(mx, my-1, lastX, lastY-1, GxEPD_WHITE);
          }
          display->drawLine(mx, my, lastX, lastY, GxEPD_BLACK);
        }

        if( i == 0 )
        {
          display->fillCircle(mx, my, 8, GxEPD_WHITE);
          display->fillCircle(mx, my, 6, GxEPD_BLACK);
        }
        else
        {
          display->fillCircle(mx, my, 2, GxEPD_BLACK);
        }

        lastX = mx;
        lastY = my;
        first = false;
      }

      i --;
      
    }

    
  }

  if( CRASH_TRACE ) Serial.println("map end");
 
  //Serial.println("mapped");
}

void displayFix(Fix fix, const char*label)
{
  display->fillScreen(GxEPD_WHITE);
  
  display->setCursor(0,20);
  display->println(label);
  display->printf("%X\n",fix.id);
  display->print(fix.lat, 6);
  display->println(",");
  display->println(fix.lng, 6);

  
  display->print(fix.age()/1000);
  display->print("s ");
  display->print(fix.satellites);
  display->println("sats");
  
  display->print("hdp");
  display->print(fix.hdop);
  display->print(" snr");
  display->println(fix.snr, 0);

  display->print("rssi");
  display->print(fix.rssi, 0);
  display->print(" ");
  display->print((int) (100.0 *fix.batteryFraction()));
  display->println("%");
  
  display->print("up ");
  display->print(fix.uptime/1000);
  display->println("s");
 
   
}

void displayFixes()
{
  display->fillScreen(GxEPD_WHITE);
  //display->drawExampleBitmap(BitmapExample1, 0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, GxEPD_BLACK);
  display->setCursor(0,20);
  //display->print("age ");
  //display->println(myFixAge/1000);
  display->print("me ");
  display->println(myFix.lat, 6);
  display->print(", ");
  display->println(myFix.lng, 6);

  display->print("t ");
  display->println(theirFix.lat, 6);
  display->print(", ");
  display->println(theirFix.lng, 6);



  display->print(myFix.distanceTo(theirFix), 1);
  display->println(" m");
  display->print(myFix.headingTo(theirFix), 1);
  display->println(" deg");

  display->print("max ");
  display->print(maxRange, 0);
  display->println(" m");
    
}

void enableBacklight(bool en)
{
    digitalWrite(ePaper_Backlight, en);
}

void setupDisplay()
{
    dispPort = new SPIClass(
        /*SPIPORT*/NRF_SPIM2,
        /*MISO*/ ePaper_Miso,
        /*SCLK*/ePaper_Sclk,
        /*MOSI*/ePaper_Mosi);

    io = new GxIO_Class(
        *dispPort,
        /*CS*/ ePaper_Cs,
        /*DC*/ ePaper_Dc,
        /*RST*/ePaper_Rst);

    display = new GxEPD_Class(
        *io,
        /*RST*/ ePaper_Rst,
        /*BUSY*/ ePaper_Busy);

    dispPort->begin();
    display->init(/*115200*/);
    display->setRotation(3);
    display->fillScreen(GxEPD_WHITE);
    display->setTextColor(GxEPD_BLACK);
    display->setFont(&FreeMonoBold12pt7b);
}


void configVDD(void)
{
    // Configure UICR_REGOUT0 register only if it is set to default value.
    if ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) ==
            (UICR_REGOUT0_VOUT_DEFAULT << UICR_REGOUT0_VOUT_Pos)) {
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}

        NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
                            (UICR_REGOUT0_VOUT_3V3 << UICR_REGOUT0_VOUT_Pos);

        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}

        // System reset is needed to update UICR registers.
        NVIC_SystemReset();
    }
}

void boardInit()
{

    uint8_t rlst = 0;

#ifdef HIGH_VOLTAGE
    configVDD();
#endif
    Serial.print("1\n");

    SerialMon.begin(MONITOR_SPEED);
    // delay(5000);
    // while (!SerialMon);
    SerialMon.println("Start\n");

    uint32_t reset_reason;
    sd_power_reset_reason_get(&reset_reason);
    SerialMon.print("sd_power_reset_reason_get:");
    SerialMon.println(reset_reason, HEX);

    pinMode(Power_Enable_Pin, OUTPUT);
    digitalWrite(Power_Enable_Pin, HIGH);

    pinMode(ePaper_Backlight, OUTPUT);
    //enableBacklight(true); //ON backlight
    enableBacklight(false); //OFF  backlight

    pinMode(GreenLed_Pin, OUTPUT);
    pinMode(RedLed_Pin, OUTPUT);
    pinMode(BlueLed_Pin, OUTPUT);

    pinMode(UserButton_Pin, INPUT_PULLUP);
    pinMode(Touch_Pin, INPUT_PULLUP);

    //pinMode( Adc_Pin, ANALOG_IN );

    int i = 10;
    while (i--) {
        digitalWrite(GreenLed_Pin, !digitalRead(GreenLed_Pin));
        digitalWrite(RedLed_Pin, !digitalRead(RedLed_Pin));
        digitalWrite(BlueLed_Pin, !digitalRead(BlueLed_Pin));
        delay(300);
    }
    digitalWrite(GreenLed_Pin, HIGH);
    digitalWrite(RedLed_Pin, HIGH);
    digitalWrite(BlueLed_Pin, HIGH);

    Serial.print("2\n");
    setupDisplay();

    Serial.print("3\n");
    setupGPS();

    Serial.print("4\n");
    setupLoRa();

    Serial.print("5\n");
    display->update();
    delay(500);
    Serial.print("6\n");


}

void setTransmittedFlag(void)  
{
    // check if the interrupt is enabled 
    if (!enableInterrupt) {
        return;
    }

    // we got a packet, set the flag
    transmittedFlag = true;
}

bool setupLoRa()
{
    rfPort = new SPIClass(
        /*SPIPORT*/NRF_SPIM3,
        /*MISO*/ LoRa_Miso,
        /*SCLK*/LoRa_Sclk,
        /*MOSI*/LoRa_Mosi);
    rfPort->begin();

    SPISettings spiSettings;

    radio = new Module(LoRa_Cs, LoRa_Dio1, LoRa_Rst, LoRa_Busy, *rfPort, spiSettings);

    SerialMon.print("[SX1262] Initializing ...  ");
    // carrier frequency:           868.0 MHz
    // bandwidth:                   125.0 kHz
    // spreading factor:            9
    // coding rate:                 7
    // sync word:                   0x12 (private network)
    //    or 0x34 for TTN - see https://www.thethingsnetwork.org/forum/t/should-private-lorawan-networks-use-a-different-sync-word/34496
    // output power:                14 dBm
    // current limit:               60 mA
    // preamble length:             8 symbols
    // TCXO voltage:                1.6 V (set to 0 to not use TCXO)
    // regulator:                   DC-DC (set to true to use LDO)
    // CRC:                         enabled
    int state = radio.begin(868.0);
    if (state !=  RADIOLIB_ERR_NONE) {
        SerialMon.print(("failed, code "));
        SerialMon.println(state);
        return false;
    }


    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
        radio.setOutputPower(17);
        radio.setBandwidth(125);
        radio.setCurrentLimit(120);
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true);
    }

    if( doReceive )
    {
      // set the function that will be called
      // when new packet is received
      radio.setPacketReceivedAction(setReceivedFlag);

      // start listening for LoRa packets
      Serial.print(F("[SX1276] Starting to listen ... "));
      state = radio.startReceive();
    }

    SerialMon.println(" success");
    return true;
}


void loopReceive()
{
    // check if the flag is set
    if (receivedFlag) {
        // disable the interrupt service routine while
        // processing the data
        enableInterrupt = false;

        // reset flag
        receivedFlag = false;

        receiveBufSize = radio.getPacketLength();
       
        strcpy((char*)receiveBuf, "EMPTYEMPTYEMPTYEMPTYEMPTYEMPTYEMPTYEMPTYEMPTYEMPTYEMPTYEMPTYEMPTYEMPTYEMPTYEMPTY");

        int state = radio.readData(receiveBuf,receiveBufSize);
        
        if (state == RADIOLIB_ERR_NONE) {
            // packet was successfully received
            Serial.println(F("[SX1276] Received packet!"));

                      
            Serial.print(receiveBufSize);
            Serial.println(F(" bytes"));

            Serial.println((char*)receiveBuf);

            if( receiveBufSize == sizeof(Fix))
            {
              Fix temp;

              if( temp.decode(receiveBuf, receiveBufSize))
              {
                theirFix = temp;

                theirFix.rssi = radio.getRSSI();
                theirFix.snr = radio.getSNR();
                
                Serial.print("temp ");
                Serial.println(temp.lat);

                Serial.print("their ");
                Serial.println(theirFix.lat);

                if( myFix.goodFix() && theirFix.goodFix())
                {
                  double range = myFix.distanceTo(theirFix);
                  maxRange = max(maxRange, range);
                }

                theirFixes->add(theirFix);
              }
            }
            else
            {
              Serial.print(F("wrong len ( not "));
               Serial.print(sizeof(Fix));
               Serial.println(F(" ), not for us"));
            }
            // print data of the packet
            //Serial.print(F("[SX1276] Data:\t\t"));
            //Serial.println(str);

            // print RSSI (Received Signal Strength Indicator)
            //Serial.print(F("[SX1276] RSSI:\t\t"));
            //Serial.print(radio.getRSSI());
            //Serial.println(F(" dBm"));

            // print SNR (Signal-to-Noise Ratio)
            //Serial.print(F("[SX1276] SNR:\t\t"));
            //Serial.print(radio.getSNR());
            //Serial.println(F(" dB"));

            // print frequency error
            //Serial.print(F("[SX1276] Frequency error:\t"));
            //Serial.print(radio.getFrequencyError());
            //Serial.println(F(" Hz"));

        } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            // packet was received, but is malformed
            Serial.println(F("[SX1276] CRC error!"));

        } else {
            // some other error occurred
            Serial.print(F("[SX1276] Failed, code "));
            Serial.println(state);
        }

        // put module back to listen mode
        radio.startReceive();

        // we're ready to receive more packets,
        // enable interrupt service routine
        enableInterrupt = true;
    }
}






bool setupGPS()
{
    SerialMon.println("[GPS] Initializing ... ");
    SerialMon.flush();
#ifndef PCA10056
    SerialGPS.setPins(Gps_Rx_Pin, Gps_Tx_Pin);
#endif
    SerialGPS.begin(9600);
    SerialGPS.flush();

    pinMode(Gps_pps_Pin, INPUT);

    pinMode(Gps_Wakeup_Pin, OUTPUT);
    digitalWrite(Gps_Wakeup_Pin, HIGH);

    delay(10);
    pinMode(Gps_Reset_Pin, OUTPUT);
    digitalWrite(Gps_Reset_Pin, HIGH); delay(10);
    digitalWrite(Gps_Reset_Pin, LOW); delay(10);
    digitalWrite(Gps_Reset_Pin, HIGH);
    gps = new TinyGPSPlus();

    return true;
}

uint32_t getMacAddress()
{
  // read the mac address

  uint32_t id = NRF_FICR->DEVICEADDR[1] & 0xffff;
  id = id | (NRF_FICR->DEVICEADDR[0] << 4);

  return id;
}


void loopGPS()
{
  static bool firstFix = true;


    while (SerialGPS.available() > 0)
        gps->encode(SerialGPS.read());

    if (gps->location.isUpdated())
    {
      if (millis() - lastGpsFixMillis > fixSendInterval) {


        if( ! firstFix ) // always ignore first fix
        {
          // but send every 10s even when not updated

          myFix.id =  getMacAddress();
          //Serial.printf("sending id %X", getMacAddress());
          myFix.lat = gps->location.lat();
          myFix.lng = gps->location.lng();
          myFix.ageWhenReceived = gps->location.age();
          myFix.timeWhenReceived = millis();
          myFix.hdop = gps->hdop.value();
          myFix.satellites = gps->satellites.value();
          myFix.batteryVoltage = analogRead(Adc_Pin);
          myFix.uptime = millis() - bootMillis;
        
          myFixes->add(myFix);

          lastGpsFixMillis = millis();
        
        }

      
        firstFix = false;
      
    }        

            

          if (gps->location.isUpdated()) {
/*
            SerialMon.print(F("LOCATION   Fix Age="));
            SerialMon.print(gps->location.age());
            SerialMon.print(F("ms Raw Lat="));
            SerialMon.print(gps->location.rawLat().negative ? "-" : "+");
            SerialMon.print(gps->location.rawLat().deg);
            SerialMon.print("[+");
            SerialMon.print(gps->location.rawLat().billionths);
            SerialMon.print(F(" billionths],  Raw Long="));
            SerialMon.print(gps->location.rawLng().negative ? "-" : "+");
            SerialMon.print(gps->location.rawLng().deg);
            SerialMon.print("[+");
            SerialMon.print(gps->location.rawLng().billionths);
            SerialMon.print(F(" billionths],  Lat="));
            SerialMon.print(gps->location.lat(), 6);
            SerialMon.print(F(" Long="));
            SerialMon.println(gps->location.lng(), 6);
            */
        }

        else if (gps->date.isUpdated()) {
          /*
            SerialMon.print(F("DATE       Fix Age="));
            SerialMon.print(gps->date.age());
            SerialMon.print(F("ms Raw="));
            SerialMon.print(gps->date.value());
            SerialMon.print(F(" Year="));
            SerialMon.print(gps->date.year());
            SerialMon.print(F(" Month="));
            SerialMon.print(gps->date.month());
            SerialMon.print(F(" Day="));
            SerialMon.println(gps->date.day());
            */
        }

        else if (gps->time.isUpdated()) {
          /*
            SerialMon.print(F("TIME       Fix Age="));
            SerialMon.print(gps->time.age());
            SerialMon.print(F("ms Raw="));
            SerialMon.print(gps->time.value());
            SerialMon.print(F(" Hour="));
            SerialMon.print(gps->time.hour());
            SerialMon.print(F(" Minute="));
            SerialMon.print(gps->time.minute());
            SerialMon.print(F(" Second="));
            SerialMon.print(gps->time.second());
            SerialMon.print(F(" Hundredths="));
            SerialMon.println(gps->time.centisecond());
            */
        }

        else if (gps->speed.isUpdated()) {
          /*
            SerialMon.print(F("SPEED      Fix Age="));
            SerialMon.print(gps->speed.age());
            SerialMon.print(F("ms Raw="));
            SerialMon.print(gps->speed.value());
            SerialMon.print(F(" Knots="));
            SerialMon.print(gps->speed.knots());
            SerialMon.print(F(" MPH="));
            SerialMon.print(gps->speed.mph());
            SerialMon.print(F(" m/s="));
            SerialMon.print(gps->speed.mps());
            SerialMon.print(F(" km/h="));
            SerialMon.println(gps->speed.kmph());
            */
        }

        else if (gps->course.isUpdated()) {
          /*
            SerialMon.print(F("COURSE     Fix Age="));
            SerialMon.print(gps->course.age());
            SerialMon.print(F("ms Raw="));
            SerialMon.print(gps->course.value());
            SerialMon.print(F(" Deg="));
            SerialMon.println(gps->course.deg());
            */
        }

        else if (gps->altitude.isUpdated()) {
          /*
            SerialMon.print(F("ALTITUDE   Fix Age="));
            SerialMon.print(gps->altitude.age());
            SerialMon.print(F("ms Raw="));
            SerialMon.print(gps->altitude.value());
            SerialMon.print(F(" Meters="));
            SerialMon.print(gps->altitude.meters());
            SerialMon.print(F(" Miles="));
            SerialMon.print(gps->altitude.miles());
            SerialMon.print(F(" KM="));
            SerialMon.print(gps->altitude.kilometers());
            SerialMon.print(F(" Feet="));
            SerialMon.println(gps->altitude.feet());
            */
        }

        else if (gps->satellites.isUpdated()) {
          /*
            SerialMon.print(F("SATELLITES Fix Age="));
            SerialMon.print(gps->satellites.age());
            SerialMon.print(F("ms Value="));
            SerialMon.println(gps->satellites.value());
            */
        }

        else if (gps->hdop.isUpdated()) {
          /*
            SerialMon.print(F("HDOP       Fix Age="));
            SerialMon.print(gps->hdop.age());
            SerialMon.print(F("ms Value="));
            SerialMon.println(gps->hdop.value());
            */
        }

        if (gps->charsProcessed() < 10)
            Serial.println(F("WARNING: No GPS data.  Check wiring."));
        
    }
}


float fmapConstrained(float x, float in_min, float in_max, float out_min, float out_max)
{
  float f = fmap( x,  in_min, in_max, out_min, out_max);

  if( f < out_min )
    f = out_min;

  if( f > out_max )
    f = out_max;

  return f;
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


float fconstrain(float f, float out_min, float out_max)
{
  if( f < out_min )
    f = out_min;

  if( f > out_max )
    f = out_max;

  return f;
}

