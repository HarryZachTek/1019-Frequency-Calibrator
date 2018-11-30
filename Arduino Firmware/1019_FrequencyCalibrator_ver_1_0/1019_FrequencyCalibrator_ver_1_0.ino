//Software for Zachtek 1019 Frequency Calibrator with a 8MHz Mini Pro Arduno
//This will program an NEO-6 Gps module at startup to output an RF signal when locked.
//The NEO-6 has an internal 48MHz PLL that can be diveded down to any freq below 10MHz.
//Divide to a frequency that is an integer fraction of 48MHz to get the lowest jitter.
//Examples off low jitter freqencies is 1, 2, 4 and 8 MHz
//To compile you need to install the Library "NeoGps by SlashDevin", you can download it using The Library manager in the Arduino IDE
//ZachTec 2017-2018
//For Arduino Pro Mini.

#include <EEPROM.h>
#include <NMEAGPS.h> //NeoGps by SlashDevin"
#include <SoftwareSerial.h>

/*  Enable these defines in the file NMEAGPS_cfg.h usually found in ..\Documents\Arduino\libraries\NeoGPS\src
#define NMEAGPS_PARSE_GSV
#define NMEAGPS_PARSE_SATELLITE_INFO
#define NMEAGPS_PARSE_SATELLITES
*/

NMEAGPS  gps; // This parses the GPS characters
gps_fix  fix; // This holds on to the latest values
SoftwareSerial GPSSerial(2, 3); // RX, TX

#define StatusLED 4  //Yellow LED next to Green Power LED
#define RFOutLED 5   //RF On LED next to RF Out SMA conenctor

boolean GPSOK;

int fixstate; //GPS Fix state machine 0=Init, 1=wating for fix,2=fix accuired
int64_t freq;//Holds the Output frequency. Stored and API transmitted as deciHertz. E.g Hertz with two decimaldigits as an integer Example - 23Hertz=2300


const uint8_t  SoftwareVersion =  1; //0 to 255. 0=Beta
const uint8_t  SoftwareRevision = 0; //0 to 255
const uint8_t  HardwareVersion =  1; //0 to 255. 0=Beta
const uint8_t  HardwareRevision = 1; //0 to 255
const uint8_t  SerCMDLength =    50; //Max number of char on a command in the SerialAPI

//                     WSPR-TX_LP1 transmitter=1011
//                 WSPR-TX Desktop transmitter=1012
//                        Frequency Calibrator=1019
const uint16_t Product_Model                 = 1019;


#define UMesLocator 2
#define UMesTime 3
#define UMesGPSLock 4
#define UMesNoGPSLock 5
#define UMesFreq 6
#define UMesTXOn 7
#define UMesTXOff 8



//--------------------------  SETUP -----------------------

void setup()
{
  Serial.begin (9600);  //USB Serial port
  Serial.setTimeout(2000);
  GPSSerial.begin(9600); //Internal GPS Serial port

  if (Product_Model == 1019)
  {
    Serial.println(F("{MIN} ZachTek Frequency Calibrator"));
  }

  Serial.print(F("{MIN} Firmware version "));
  if (SoftwareVersion == 0) {
    Serial.print(F("Beta "));
  }
  Serial.print(SoftwareVersion);
  Serial.print(F("."));
  Serial.println(SoftwareRevision);

  // Use the Red LED as a Transmitt indicator and the Yellow LED as Status indicator
  pinMode(StatusLED, OUTPUT);
  pinMode(RFOutLED, OUTPUT);

  digitalWrite(RFOutLED, LOW);

  //Blink StatusLED to indicate Reboot
  for (int i = 0; i <= 15; i++) {
    digitalWrite(StatusLED, HIGH);
    delay (50);
    digitalWrite(StatusLED, LOW);
    delay (50);
  }

  //Read startup frequency from EEPROM
  if (LoadFromEEPROM() == false)
  {
    Serial.println ("{MIN} No startup frequency data was found, using 1MHz");
    freq = 100000000ULL;
  }

  //freq = 00ULL;

  GPSSerial.begin(9600);
  GPSOK = true;
  //Program GPS to output RF
  if (setGPS_OutputFreq()) {
    Serial.println (F("{MIN} GPS Initialized"));
    GPSOK = true;
  }
  else
  {
    Serial.println (F("{MIN} Error! Could not program GPS!"));
    GPSOK = false;
  }
  SendAPIUpdate(UMesNoGPSLock);
  SendAPIUpdate(UMesTXOff);
  fixstate = 0; //Initial State for the fixstate that helps only output the GPS postion once after getting a position fix
}

//--------------------------  Main loop -----------------------
void loop()
{
  if (Serial.available()) {  //Handle  Serial API request from the PC
    DoSerialHandling();
  }
 
  while (gps.available( GPSSerial )) { //Handle Serial data from teh GPS as they arrive
    fix = gps.read();
    if (fix.valid.location && fix.valid.date && fix.valid.time ) //GPS got a valid fix
    {
      digitalWrite(StatusLED, HIGH);   // Turn on the Status LED
      digitalWrite(RFOutLED, HIGH);    // Turn on RF Out LED
      SendAPIUpdate(UMesTXOn);
      SendAPIUpdate(UMesGPSLock);
      
      if (fixstate != 2) {
        Serial.print( F("{MIN} Fix acuired, RF turned ON. (Location is: ") );
        Serial.print( fix.latitude(), 6 );
        Serial.print( ',' );
        Serial.print( fix.longitude(), 6 );
        Serial.println(")");
        fixstate=2;// Fix accuired
      }      
    }
    else
    {
      if (GPSOK) { //No valid fix
        digitalWrite(RFOutLED, LOW);          // Turns off RF Out LED
        LEDBlink();
        SendAPIUpdate(UMesTXOff);
        SendAPIUpdate(UMesNoGPSLock);
        if (fixstate != 1) {
          Serial.println(F("{MIN}  Waiting for GPS location fix"));
          fixstate = 1; //Waiting for fix
        }
      }
    }
    SendSatData(); //Send Satelite information to the PC Gui
  }
}



//Get Serial data from the PC and pass it on to the Serial API decoder
void DoSerialHandling()
//Parts from NickGammon Serial Input example
//http://www.gammon.com.au/serial
{
  static char SerialLine[SerCMDLength]; //A single line of incoming serial command and data
  static uint8_t input_pos = 0;
  char InChar;

  while (Serial.available () > 0)
  {
    InChar = Serial.read ();
    switch (InChar)
    {
      case '\n':   // end of text
        SerialLine [input_pos] = 0;  // terminating null byte
        // terminator reached, process Command
        DecodeSerialCMD (SerialLine);
        // reset buffer for next time
        input_pos = 0;
        break;

      case '\r':   // discard carriage return
        break;

      default:
        // keep adding if not full ... allow for terminating null byte
        if (input_pos < (SerCMDLength - 1))
          SerialLine [input_pos++] = InChar;
        break;

    }  // end of switch
  } // end of processIncomingByte
}


//Serial API commands and data decoding
void DecodeSerialCMD(const char * InputCMD) {
  char CharInt[13];
  bool EnabDisab;

  if ((InputCMD[0] == '[') && (InputCMD[4] == ']')) { //A Command,Option or Data input

    if (InputCMD[1] == 'C') {  //Commmand

      //Store Current configuration data to EEPROM
      if ((InputCMD[2] == 'S') && (InputCMD[3] == 'E')) {
        if (InputCMD[6] == 'S') { //Set option
          SaveToEEPROM();
          Serial.println(F("{MIN} Output frequency saved to EEPROM"));
        }
      }
      exit;
    }

    if (InputCMD[1] == 'O') {//Option
      exit;
    }//All Options

    
    if (InputCMD[1] == 'D') { //Data

      //Generator Frequency
      if ((InputCMD[2] == 'G') && (InputCMD[3] == 'F')) {
        if (InputCMD[6] == 'S') { //Set option
          for (int i = 0; i <= 11; i++) {
            CharInt[i] = InputCMD[i + 8];
          }
          CharInt[12] = 0;
          freq = StrTouint64_t(CharInt);
          if (setGPS_OutputFreq()) {
            GPSOK = true;
          }
          else
          {
            Serial.println (F("{MIN} Error! Could not reprogram GPS!"));
            GPSOK = false;
          }
        }
        else //Get
        {
          Serial.print (F("{DGF} "));
          Serial.println (uint64ToStr(freq, true));
          SendAPIUpdate(UMesFreq);
        }
      }//Generator Frequency

      exit;
    }//Data

   
    if (InputCMD[1] == 'F') { //Factory settings

      //Product model Number
      if ((InputCMD[2] == 'P') && (InputCMD[3] == 'N')) {
        if (InputCMD[6] == 'G')
        { //Get option
          Serial.print (F("{FPN} "));
          if (Product_Model < 10000) Serial.print ("0");
          Serial.println (Product_Model);
          SendAPIUpdate(UMesFreq);
        }
      }//Product model Number

      //Hardware Version
      if ((InputCMD[2] == 'H') && (InputCMD[3] == 'V')) {
        if (InputCMD[6] == 'S') { //Set option

        }//Set
        else //Get Option
        {
          Serial.print (F("{FHV} "));
          if (HardwareVersion < 100) Serial.print ("0");
          if (HardwareVersion < 10) Serial.print ("0");
          Serial.println (HardwareVersion);
        }
      }//Hardware Version

      //Hardware Revision
      if ((InputCMD[2] == 'H') && (InputCMD[3] == 'R')) {
        if (InputCMD[6] == 'S') { //Set option

        }//Set
        else //Get Option
        {
          Serial.print (F("{FHR} "));
          if (HardwareRevision < 100) Serial.print ("0");
          if (HardwareRevision < 10) Serial.print ("0");
          Serial.println (HardwareRevision);
        }
      }//Hardware Revision

      //Software Version
      if ((InputCMD[2] == 'S') && (InputCMD[3] == 'V')) {
        if (InputCMD[6] == 'G') { //Get option
          Serial.print (F("{FSV} "));
          Serial.println (SoftwareVersion);
        }
      }//Software Version

      //Software Revision
      if ((InputCMD[2] == 'S') && (InputCMD[3] == 'R')) {
        if (InputCMD[6] == 'G') { //Get option
          Serial.print (F("{FSR} "));
          Serial.println (SoftwareRevision);
        }
      }//Software Revision

     

      exit;
    }//Factory settings
  }
}


//Delay loop that checks if the GPS serial port is sending data and in that case passes it of the GPS object
static void smartdelay(unsigned long ms)
{
// This custom version of delay() ensures that the gps object
// is being "fed". Original code from the TinyGPS example but here used for the NeoGPS 
  long TimeLeft;
  unsigned long EndTime = ms + millis();

  do
  {
    while (gps.available( GPSSerial )) fix = gps.read(); //If GPS data available - process it
    TimeLeft = EndTime - millis();
    if ((TimeLeft > 1000) && ((TimeLeft % 1000) < 20)) {
      //Send API update every  second
      Serial.print (F("{MPS} "));
      Serial.println (TimeLeft / 1000);
    }
  } while ((TimeLeft > 0) && (!Serial.available())) ; //Until time is up or there is serial data received
  //Serial.println (F("{MPS} 0"));
}


//Brief flash on the Status LED
void LEDBlink()
{
  digitalWrite(StatusLED, HIGH);
  smartdelay (100);
  digitalWrite(StatusLED, LOW);
  smartdelay (100);
}

//Send different types of Serial API updates
void SendAPIUpdate (uint8_t UpdateType)
{
  switch (UpdateType) {
    case UMesLocator:
      Serial.print (F("{GL4} "));
      //Serial.println (GadgetData.WSPRData.MaidenHead4);;
      break;

    case UMesTime:
      Serial.print (F("{GTM} "));
      /*
        if (GPSH < 10) Serial.print (F("0"));
        Serial.print (GPSH);
        Serial.print (F(":"));
        if (GPSM < 10) Serial.print (F("0"));
        Serial.print (GPSM);
        Serial.print (F(":"));
        if (GPSS < 10) Serial.print (F("0"));
        Serial.println (GPSS);
      */
      break;

    case UMesGPSLock:
      Serial.println (F("{GLC} T"));
      break;

    case UMesNoGPSLock:
      Serial.println (F("{GLC} F"));
      break;

    case UMesFreq:
      Serial.print (F("{TFQ} "));
      Serial.println (uint64ToStr(freq, false));
      break;

    case UMesTXOn:
      Serial.println (F("{TON} T"));
      break;

    case UMesTXOff:
      Serial.println (F("{TON} F"));
      break;

  }
}

//Sends the Sattelite data like Elevation, Azimuth SNR and ID using the Serial API {GSI} format
void SendSatData()
{
uint8_t SNR;  
  for (uint8_t i=0; i < gps.sat_count; i++) {
    Serial.print ("{GSI} ");
    if (gps.satellites[i].id < 10) Serial.print("0");
    Serial.print( gps.satellites[i].id);
    Serial.print(" ");
    if (gps.satellites[i].azimuth < 100) Serial.print("0");
    if (gps.satellites[i].azimuth < 10) Serial.print("0");
    Serial.print( gps.satellites[i].azimuth );
    Serial.print(" ");
    if (gps.satellites[i].elevation < 10) Serial.print("0");
    Serial.print( gps.satellites[i].elevation );
    Serial.print(" ");
    SNR=0;
    if (gps.satellites[i].tracked) 
      {
      SNR=gps.satellites[i].snr ;
      }
    else
     { 
      SNR=0; 
    }
    if (SNR<10) Serial.print("0");
    Serial.println(SNR);
  }

  Serial.println();

} // displaySatellitesInView





// For details of the UBX protocol, see
// https://www.u-blox.com/sites/default/files/products/documents/u-blox7-V14_ReceiverDescriptionProtocolSpec_%28GPS.G7-SW-12001%29_Public.pdf
// Documentation of this packet is under the heading CFG-TP5 (35.19.2) in the current documentation.
uint8_t ubx_SetFreqPacket[] = {
  0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x01, 0x00,
  0x00, 0x00, 0xA0, 0x86, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
  0x00, 0x00, 0xEF, 0x00, 0x00, 0x00, 0x20, 0x1B
};

#define OFFSET_FREQUENCY_LOCKED (18)
#define OFFSET_CKSUM (38)

// Note: Normally payload_start is 2 bytes past the start of buf, because the first two bytes
// are the message type and are not checksummed.
void ubx_compute_checksum(uint8_t *payload_start, uint8_t *payload_end, uint8_t *cksum) {
  uint8_t ck_a = 0, ck_b = 0;
  for (const uint8_t *p = payload_start; p != payload_end; p++)
  {
    ck_a += *p;
    ck_b += ck_a;
  }
  cksum[0] = ck_a;
  cksum[1] = ck_b;
}

//Reprogram the Ublox Neo-6 GPS to output a specific frequency when it has a fix.
// Orginal code by Harry Zachrisson and Jeff Epler  https://github.com/jepler
bool setGPS_OutputFreq() {
  uint32_t Freq32;
  Freq32=freq/100;
  for (int i = 0; i < 4; i++) {
    ubx_SetFreqPacket[OFFSET_FREQUENCY_LOCKED + i] = Freq32 & 0xff;
    Freq32 >>= 8;
  }
  ubx_compute_checksum(ubx_SetFreqPacket + 2, ubx_SetFreqPacket + 38, ubx_SetFreqPacket + 38);
  sendUBX(ubx_SetFreqPacket, sizeof(ubx_SetFreqPacket) / sizeof(uint8_t));
  bool gps_set_sucess = getUBX_ACK(ubx_SetFreqPacket);
  SendAPIUpdate(UMesFreq);
  return gps_set_sucess;
}


void sendUBX(uint8_t *MSG, uint8_t len) {
  GPSSerial.flush();
  GPSSerial.write(0xFF);
  _delay_ms(500);
  for (int i = 0; i < len; i++) {
    GPSSerial.write(MSG[i]);
  }
}


//Calculate and receive an Ublox Ack. Orginal code from Dave Hibberd
//https://raw.githubusercontent.com/Hibby/Strathclyde-HAB-Project/master/main/main.pde
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();

  // Construct the expected ACK packet
  ackPacket[0] = 0xB5; // header
  ackPacket[1] = 0x62; // header
  ackPacket[2] = 0x05; // class
  ackPacket[3] = 0x01; // id
  ackPacket[4] = 0x02; // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2]; // ACK class
  ackPacket[7] = MSG[3]; // ACK id
  ackPacket[8] = 0; // CK_A
  ackPacket[9] = 0; // CK_B

  // Calculate the checksums
  ubx_compute_checksum(ackPacket + 2, ackPacket + 8, ackPacket + 8);

  while (1) {  // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) {
      return false;
    }

    // Make sure data is available to read
    if (GPSSerial.available()) {
      b = GPSSerial.read();
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) {
        ackByteID++;
      }
      else {
        ackByteID = 0; // Reset and look again, invalid order
      }//else
    }//If
  }//While
}//getUBX_ACK



//Calculate 32bit CRC
//Original code from Arduino example https://www.arduino.cc/en/Tutorial/EEPROMCrc
unsigned long GetEEPROM_CRC(void) {

  const unsigned long crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  unsigned long crc = ~0L;

  for (int index = 0 ; index < sizeof(freq) ; ++index) {
    crc = crc_table[(crc ^ EEPROM[index]) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (EEPROM[index] >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }
  return crc;
}


void SaveToEEPROM ()
{
  unsigned long CRCFromEEPROM;
  EEPROM.put(0, freq);                     //Save the Freqency to EEPROM at address0
  CRCFromEEPROM = GetEEPROM_CRC ();        //Calculate CRC on the saved data
  EEPROM.put(sizeof(freq), CRCFromEEPROM); //Save the CRC after the data
}


bool LoadFromEEPROM (void)
{
  unsigned long CRCFromEEPROM, CalculatedCRC;

  EEPROM.get(0, freq);                           //Load the Frequency from EEPROM address 0
  EEPROM.get(sizeof(freq), CRCFromEEPROM);       //Load the CRC value that is stored in EEPROM
  CalculatedCRC = GetEEPROM_CRC();               //Calculate the CRC of the Frequency
  return (CRCFromEEPROM == CalculatedCRC);       //If  Stored and Calculated CRC are the same then return true
}


//Convert a string to 64 bits unsigned integer 
uint64_t  StrTouint64_t (String InString)
{
  uint64_t y = 0;

  for (int i = 0; i < InString.length(); i++) {
    char c = InString.charAt(i);
    if (c < '0' || c > '9') break;
    y *= 10;
    y += (c - '0');
  }
  return y;
}

//Convert a 64 bits unsigned integer to a string, optionally add leading zeros so the return string always is 12 characters long
String  uint64ToStr (uint64_t p_InNumber, boolean p_LeadingZeros)
{
  char l_HighBuffer[7]; //6 digits + null terminator char
  char l_LowBuffer[7]; //6 digits + null terminator char
  char l_ResultBuffer [13]; //12 digits + null terminator char
  String l_ResultString = "";
  uint8_t l_Digit;

  sprintf(l_HighBuffer, "%06lu", p_InNumber / 1000000L); //Convert high part of 64bit unsigned integer to char array
  sprintf(l_LowBuffer, "%06lu", p_InNumber % 1000000L); //Convert low part of 64bit unsigned integer to char array
  l_ResultString = l_HighBuffer;
  l_ResultString = l_ResultString + l_LowBuffer; //Copy the 2 part result to a string

  if (!p_LeadingZeros) //If leading zeros should be removed
  {
    l_ResultString.toCharArray(l_ResultBuffer, 13);
    for (l_Digit = 0; l_Digit < 12; l_Digit++ )
    {
      if (l_ResultBuffer[l_Digit] == '0')
      {
        l_ResultBuffer[l_Digit] = ' '; // replace zero with a space character
      }
      else
      {
        break; //We have found all the leading Zeros, exit loop
      }
    }
    l_ResultString = l_ResultBuffer;
    l_ResultString.trim();//Remove all leading spaces
  }
  return l_ResultString;
}
