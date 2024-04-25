/*
 WhereOTechno

 runs on Lilygo T-Echo

 select Nordic nRF52840 DKit something
 */

#include "utilities.h"
#include <SPI.h>
#include <Wire.h>

#include <GxEPD.h>
//#include <GxGDEP015OC1/GxGDEP015OC1.h>    // 1.54" b/w
//#include <GxGDEH0154D67/GxGDEH0154D67.h>  // 1.54" b/w
#include <GxDEPG0150BN/GxDEPG0150BN.h>  // 1.54" b/w 

#include GxEPD_BitmapExamples
#include <Fonts/FreeMonoBold12pt7b.h>
#include <GxIO/GxIO_SPI/GxIO_SPI.h>
#include <GxIO/GxIO.h>

#include <TinyGPS++.h>

#include <RadioLib.h>


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

uint32_t        blinkMillis = 0;

uint32_t        last = 0;

uint8_t rgb = 0;

TinyGPSPlus     *gps;

double myLat = 0;
double myLng = 0;
uint32_t myFixAge = 0;
uint32_t myFixMillis = 0;

char* techno = (char*) "Techno!";
uint32_t theirFixMillis = 0;
double theirLat = 0;
double theirLng = 0;


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


#define DO_SEND 1
#define DO_RECEIVE 1


void setup()
{
    Serial.begin(115200);
    delay(200);
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

void loop()
{

  loopReceive();

  loopGPS();
  
  

    if (millis() - blinkMillis > 10000) {

      loopDisplay();
      if( DO_SEND )
      {
        loopSender();
      }

        blinkMillis = millis();
        switch (rgb) {
        case 0:
            digitalWrite(GreenLed_Pin, LOW);
            digitalWrite(RedLed_Pin, HIGH);
            digitalWrite(BlueLed_Pin, HIGH);
            break;
        case 1:
            digitalWrite(GreenLed_Pin, HIGH);
            digitalWrite(RedLed_Pin, LOW);
            digitalWrite(BlueLed_Pin, HIGH);
            break;
        case 2:
            digitalWrite(GreenLed_Pin, HIGH);
            digitalWrite(RedLed_Pin, HIGH);
            digitalWrite(BlueLed_Pin, LOW);
            break;
        default :
            break;
        }
        rgb++;
        rgb %= 3;
    }
  //display->print("hello!")   ;
  //display->update();
}

void loopSender()
{
    encode();

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

    display->fillScreen(GxEPD_WHITE);
    //display->drawExampleBitmap(BitmapExample1, 0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, GxEPD_BLACK);
    display->setCursor(0,20);
    //display->print("age ");
   //display->println(myFixAge/1000);
   display->print("me ");
   display->println(myLat, 6);
  display->print(", ");
   display->println(myLng, 6);

   display->print("t ");
   display->println(theirLat, 6);
  display->print(", ");
   display->println(theirLng, 6);

  double distance = TinyGPSPlus::distanceBetween(myLat, myLng, theirLat, theirLng);
  double heading = TinyGPSPlus::courseTo(myLat, myLng, theirLat, theirLng);

 display->print(distance, 1);
   display->println(" m");
    display->print(heading, 1);
   display->println(" deg");
    display->update();
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

    setupDisplay();

    setupGPS();

    setupLoRa();

    display->update();
    delay(500);

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
    // set the function that will be called
    // when new packet is received
    radio.setPacketReceivedAction(setReceivedFlag);

    // start listening for LoRa packets
    Serial.print(F("[SX1276] Starting to listen ... "));
    state = radio.startReceive();


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

            if( receiveBufSize == msgSize())
            {
              decode();
            }
            else
            {
              Serial.print(F("wrong len ( not "));
               Serial.print(msgSize());
               Serial.println(F(" ), not for us"));
            }
            // print data of the packet
            //Serial.print(F("[SX1276] Data:\t\t"));
            //Serial.println(str);

            // print RSSI (Received Signal Strength Indicator)
            Serial.print(F("[SX1276] RSSI:\t\t"));
            Serial.print(radio.getRSSI());
            Serial.println(F(" dBm"));

            // print SNR (Signal-to-Noise Ratio)
            Serial.print(F("[SX1276] SNR:\t\t"));
            Serial.print(radio.getSNR());
            Serial.println(F(" dB"));

            // print frequency error
            Serial.print(F("[SX1276] Frequency error:\t"));
            Serial.print(radio.getFrequencyError());
            Serial.println(F(" Hz"));

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


uint16_t msgSize()
{
  return( strlen(techno) +sizeof(myFixMillis)+sizeof(myLat)+sizeof(myLng));
  // 7 + 8 + 8 + 4 = 27
}

void decode()
{
  int pos = 0;

  char t[1000];

  strncpy(t, (char*)receiveBuf, msgSize());
  t[msgSize()]='\0';
  Serial.println(t);
  if( 0 != strncmp( (char*)receiveBuf, techno, strlen(techno)))
  {
    Serial.println("not techno");
    return;
  }

  pos += strlen(techno);
  
  memcpy(&theirFixMillis, receiveBuf+pos,sizeof(theirFixMillis));
  pos+=sizeof(theirFixMillis);
  
  memcpy(&theirLat, receiveBuf+pos,sizeof(theirLat));
  pos+=sizeof(theirLat);
  
  memcpy(&theirLng, receiveBuf+pos,sizeof(theirLng));
  pos+=sizeof(theirLng);
  

}

void encode()
{
  int pos = 0;

  
  memcpy(sendBuf+pos, techno, strlen(techno));
  pos += strlen(techno);
  memcpy(sendBuf+pos, &myFixMillis, sizeof(myFixMillis));
  pos += sizeof(myFixMillis);
  memcpy(sendBuf+pos, &myLat, sizeof(myLat));
  pos += sizeof(myLat);
  memcpy(sendBuf+pos, &myLng, sizeof(myLng));
  pos += sizeof(myLng);
  
  sendBufSize = pos;

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


void loopGPS()
{
    while (SerialGPS.available() > 0)
        gps->encode(SerialGPS.read());

    if (millis() - last > 5000) {
        if (gps->location.isUpdated()) {
            
            myLat = gps->location.lat();
            myLng = gps->location.lng();
            myFixAge = gps->location.age();
            myFixMillis = millis() - myFixAge;
            
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
        }

        else if (gps->date.isUpdated()) {
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
        }

        else if (gps->time.isUpdated()) {
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
        }

        else if (gps->speed.isUpdated()) {
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
        }

        else if (gps->course.isUpdated()) {
            SerialMon.print(F("COURSE     Fix Age="));
            SerialMon.print(gps->course.age());
            SerialMon.print(F("ms Raw="));
            SerialMon.print(gps->course.value());
            SerialMon.print(F(" Deg="));
            SerialMon.println(gps->course.deg());
        }

        else if (gps->altitude.isUpdated()) {
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
        }

        else if (gps->satellites.isUpdated()) {
            SerialMon.print(F("SATELLITES Fix Age="));
            SerialMon.print(gps->satellites.age());
            SerialMon.print(F("ms Value="));
            SerialMon.println(gps->satellites.value());
        }

        else if (gps->hdop.isUpdated()) {
            SerialMon.print(F("HDOP       Fix Age="));
            SerialMon.print(gps->hdop.age());
            SerialMon.print(F("ms Value="));
            SerialMon.println(gps->hdop.value());
        }

        if (gps->charsProcessed() < 10)
            Serial.println(F("WARNING: No GPS data.  Check wiring."));
        last = millis();
        Serial.println();
    }
}



