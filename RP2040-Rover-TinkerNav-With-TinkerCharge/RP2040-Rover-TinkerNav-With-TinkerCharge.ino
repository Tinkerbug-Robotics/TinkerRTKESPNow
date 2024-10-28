/**
 * Firmware for RP204 running on the rover or base station to support sending/receiving of 
 * correction data via ESP32 radio. This firmware reads data from TinkerCharge and makes it 
 * available to the ESP32 to display on a webpage. The firmware also outputs GNSS serial data
 * to the RP2040 USB serial port.
 * !!! Note this must be compiled with the Earle Philhower RP2040 board set !!!
 * This is becuase it uses software serial to communicate to the ESP32. Hardware serial is
 * used for the more time critical GNSS communications.
 * Copyright Tinkerbug Robotics 2023
 * Provided under GNU GPL 3.0 License
 */

// !!! Note this must be compiled with the Earle Philhower RP2040 board set !!!

#include "Arduino.h"
#include <Wire.h>
#include "SerialTransfer.h"
#include <SoftwareSerial.h>
#include "pico/stdlib.h"
#include <MAX17055_TR.h>
#include "programSkyTraq.h"
#include <TinyGPSPlus.h>

// Configuration for pins is in User_Setup.h in the TFT_eSPI library folder
#include <TFT_eSPI.h>
#include <FS.h>
#include <LittleFS.h>

#include "TBR_Logo.h"

// Define a small font to use
#define AA_FONT_SMALL "NotoSansKannadaBold10"
#define AA_FONT_MED "NotoSansKannadaBold20"
#define AA_FONT_LARGE "NotoSansKannadaBold40"

// Font files and touch screen calibration are stored in Flash FS
#define FlashFS LittleFS

// TFT library instance
TFT_eSPI tft;

// TFT sprite
TFT_eSprite spr = TFT_eSprite(&tft); 
TFT_eSprite gps_time_spr = TFT_eSprite(&tft); 
TFT_eSprite pos_spr = TFT_eSprite(&tft); 
TFT_eSprite nmea_spr = TFT_eSprite(&tft); 

uint8_t screen_rotation = 1;

// The TinyGPSPlus object
TinyGPSPlus gps;
TinyGPSCustom gnss_quality(gps, "GPGGA", 6);

programSkyTraq program_skytraq;

// Serial connection to ESP32 radio (RX, TX)
SoftwareSerial swSerial(20, 3);

// Library and structure for transfering data to TinkerSend radio
SerialTransfer radioTransfer;
struct STRUCT 
{
    float voltage;
    float avg_voltage;
    float current;
    float avg_current;
    float battery_capacity;
    float battery_age;
    float cycle_counter;
    float SOC;
    float temperature;
} dataForTinkerSend;

// MAX17055 Battery Fuel Cell Gauge

// I2C pins
#define SDA 26
#define SCL 27

MAX17055 max17055;

// Timer to write SOC data on a specified periodic (ms)
unsigned long last_soc_time = 0;
int soc_periodic = 2000;

// RTK NMEA parameters
double rtk_e = 0.0;
double rtk_n = 0.0;
double rtk_u = 0.0;
double rtk_ratio = 0.0;
double rtk_age = 0.0;

void setup() 
{

    // Pico USB Serial
    Serial.begin(115200);
    // Pauses till serial starts. Do not use when running without a computer attached
    // or it will pause indefinetly
    //while (!Serial){};
    delay(1000);

    // Initialze library to program SkyTraq
    program_skytraq.init(Serial1);

    // Initialize TFT display
    tft.begin();

    if (!LittleFS.begin()) 
    {
        Serial.println("Flash FS initialization failed!");
        while (1) yield();
    }
    Serial.println("Flash FS available!");

    bool file_missing = false;
    if (LittleFS.exists("/NotoSansKannadaBold20.vlw") == false) file_missing = true;
    if (LittleFS.exists("/NotoSansKannadaBold40.vlw") == false) file_missing = true;
  
    if (file_missing)
    {
        Serial.println("\nFont file missing in file system, upload fonts using the LittleFS upload tool (Tools->Pico LitleFS Data Upload)?");
        while(1)
            yield();
    }
    else
    {
        Serial.println("\nFonts found OK.");
    }
    
    // ESP32 serial connection
    swSerial.begin(9600);
    radioTransfer.begin(swSerial);

    // Set I2C pins for communicating with MAX17055
    Wire1.setSDA(SDA);
    Wire1.setSCL(SCL);
    Wire1.begin();

    // Configure MAX17055
    max17055.setResistSensor(0.01); 
    max17055.setCapacity(4400);
    max17055.setChargeTermination(44);
    max17055.setEmptyVoltage(3.3);

    last_soc_time = millis();

    Serial.println("Setup Complete");

    tft.init();
    screen_rotation = 1;
    tft.setRotation(screen_rotation);
    
    tft.fillScreen(TFT_WHITE);
    
    tft.setSwapBytes(true);

    tft.pushImage (205, 5, 105, 115, TBR_Logo);
    tft.fillRoundRect(290, 115, 20, 5, 5, TFT_WHITE);

    // Set the font colour and the background colour
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    
    // Set datum to top center
    tft.setTextDatum(TL_DATUM);

    // Load font
    tft.loadFont(AA_FONT_MED, LittleFS);
    
    tft.drawString("Lat:", 3, 24);
    tft.drawString("Lon:", 3, 44);
    tft.drawString("Alt:", 3, 64);
    tft.drawString("Mode:", 3, 84);
    tft.drawString("RTK Ratio:", 3, 104);
    tft.drawString("RTK Age:", 3, 124);
    tft.drawString("East:", 3, 147);
    tft.drawString("North:", 110, 147);
    tft.drawString("Up:", 220, 147);

    tft.setTextColor(TFT_WHITE, TFT_PURPLE);

    // Scatter plot button
    tft.fillRoundRect(3, 183, 120, 25, 3, TFT_PURPLE);
    tft.drawString("Scatter Plot", 8, 187);

    // Satellite viewer button
    tft.fillRoundRect(125, 183, 110, 25, 3, TFT_PURPLE);
    tft.drawString("Sat Viewer", 131, 187);

    // Signals button
    tft.fillRoundRect(237, 183, 80, 25, 3, TFT_PURPLE);
    tft.drawString("Signals", 242, 187);

    // NMEA messages button
    tft.fillRoundRect(3, 210, 120, 25, 3, TFT_PURPLE);
    tft.drawString("NMEA Msgs", 7, 215);

    // Receiver button
    tft.fillRoundRect(125, 210, 110, 25, 3, TFT_PURPLE);
    tft.drawString("Settings", 140, 215);
    
    // Receiver settings
    tft.fillRoundRect(237, 210, 80, 25, 3, TFT_PURPLE);
    tft.drawString("Rcvr", 255, 215);

    // GPS time sprite setup
    gps_time_spr.unloadFont();
    gps_time_spr.setColorDepth(16);
    gps_time_spr.loadFont(AA_FONT_SMALL, LittleFS);
    gps_time_spr.setTextColor(TFT_BLACK, TFT_WHITE);

    // GNSS input/output Serial is Serial1 using default 0,1 (TX, RX) pins
    // Loop through valid baud rates and determine the current setting
    // Set Serial1 to the detected baud rate, stop if a baud rate is not found
    // From NavSpark binary protocol. Search for "SkyTrq Application Note AN0037"
    // Currently available at: https://www.navsparkforum.com.tw/download/file.php?id=1162&sid=dc2418f065ec011e1b27cfa77bf22b19
    if(!autoSetBaudRate())
    {
        Serial.println("No valid baud rate found to talk to receiver, stopping");
        while(1);
    }
    
    delay(250);

}

void loop() 
{

    // Read GPS serial data
    if (Serial1.available() > 0)
    {
        unsigned long last_serial1_read = millis();
        bool in_sentence = false;
        String sentence = "";

        while (Serial1.available() > 0 || millis()-last_serial1_read<50)
        {
            if (Serial1.available() > 0)
            {
                char data = Serial1.read();
                gps.encode(data);
                Serial.write(data);

                if (data == '$') 
                {
                    // Start of a new NMEA sentence
                    sentence = "";
                    in_sentence = true;
                }
                //else if (data == '\n' || data == '\r') 
                else if (in_sentence && (data == '\n' ||data == '\r')) 
                {
                    in_sentence = false; // End of the sentence
                    if (sentence.startsWith("PSTI"))
                    {
                        //Serial.println();
                        //Serial.println("PSTI sentence: " + sentence);
                        // Parse the GPGSV sentence here
                        parsePSTI(sentence);
                    }
                }
                else if (in_sentence)
                {
                    sentence += data;
                }

                last_serial1_read = millis();
            }
        }
    }

    unsigned long current_time = millis();

    // Periodically write SOC data to ESP32
    if (current_time > last_soc_time + soc_periodic || last_soc_time > current_time)
    { 
        readAndSendSOC();
        last_soc_time = current_time;
    }

    uint8_t gps_second = 0;

    if (gps.time.isUpdated() && gps.time.second() != gps_second)
    {
        // ### Time ###
        
        // Cursor position for printing time
        tft.setCursor(3, 2);

        uint8_t gps_minute = gps.time.minute();
        String minute_string;
        if (gps_minute < 10)
            minute_string = "0" + (String)gps_minute;
        else
            minute_string = (String)gps_minute;

        gps_second = gps.time.second();
        String second_string;
        if (gps_second < 10)
            second_string = "0" + (String)gps_second;
        else
            second_string = (String)gps_second;
        
        gps_time_spr.printToSprite(" " + (String)gps.date.month() + "/" + (String)gps.date.day() + "/" + (String)gps.date.year() + " "
                                    + (String)gps.time.hour() + ":" + minute_string + ":" + second_string + " UTC");

    }
            // Print to screen updated GNSS information
    if (gps.location.isUpdated())
    {

        // GPS position sprite setup
        pos_spr.unloadFont();
        pos_spr.setColorDepth(16);
        pos_spr.loadFont(AA_FONT_MED, LittleFS);
        pos_spr.setTextColor(TFT_BLUE, TFT_WHITE);
        
        // ### Lattitude ###
        
        // Cursor position for printing lattitude
        tft.setCursor(43, 24);

        String lat_direction_symbol = gps.location.rawLat().negative ? "-" : "+";
        String lat_str = String(gps.location.rawLat().billionths);

        pos_spr.printToSprite(" " + lat_direction_symbol + (String)gps.location.rawLat().deg + "."
                              + lat_str.substring(0, 8) + " ");

        // ### Longitude ###
        
        // Cursor position for printing longitude
        tft.setCursor(48, 44);

        String lon_direction_symbol = gps.location.rawLng().negative ? "-" : "+";
        String lon_str = String(gps.location.rawLng().billionths);

        pos_spr.printToSprite(" " + lon_direction_symbol + (String)gps.location.rawLng().deg + "."
                              + lon_str.substring(0, 8) + " ");

        // ### Altitude ###
        
        // Cursor position for printing longitude
        tft.setCursor(48, 64);

        String alt_str = String(gps.altitude.meters());

        pos_spr.printToSprite(" " + alt_str + " ");

        // ### Fix mode ###
        
        // Cursor position for printing RTK mode
        tft.setCursor(55, 84);
        
        String rtk_mode_str;
        uint8_t mode = atoi(gnss_quality.value());
        if (mode == 0)
            rtk_mode_str = "No Fix    ";
        else if (mode == 1)
            rtk_mode_str = "GPS Fix   ";
        else if (mode == 2)
            rtk_mode_str = "DGPS Fix  ";
        else if (mode == 4)
            rtk_mode_str = "RTK Fix   ";
        else if (mode == 5)
            rtk_mode_str = "RTK Float ";
        else
            rtk_mode_str = "Unk Fix";

        pos_spr.printToSprite(" " + rtk_mode_str + " ");

        // ### Fix mode ###
        
        // Cursor position for printing RTK ratio
        tft.setCursor(105, 104);

        pos_spr.printToSprite(" " + (String)rtk_ratio + " ");

        // ### RTK age ###
        
        // Cursor position for printing RTK age
        tft.setCursor(105, 124);

        pos_spr.printToSprite(" " + (String)rtk_age + " ");

        // ### ENU Coordinates ###
        pos_spr.unloadFont();
        pos_spr.setColorDepth(16);
        pos_spr.loadFont(AA_FONT_SMALL, LittleFS);
        pos_spr.setTextColor(TFT_BLUE, TFT_WHITE);
        
        // Cursor position for printing RTK age
        tft.setCursor(10, 164);
        pos_spr.printToSprite(" " + (String)rtk_e + " ");

        tft.setCursor(110, 164);
        pos_spr.printToSprite(" " + (String)rtk_n + " ");

        tft.setCursor(220, 164);
        pos_spr.printToSprite(" " + (String)rtk_u + " ");

    }

}

void parsePSTI(String sentence)
{
    // Remove checksum part if present
    int checksumIndex = sentence.indexOf('*');
    if (checksumIndex != -1)
    {
        sentence = sentence.substring(0, checksumIndex);
    }

    // Split the sentence by commas
    int index = 0;
    String fields[50]; // Adjust size based on expected fields
    while (sentence.length() > 0)
    {
        int commaIndex = sentence.indexOf(',');
        if (commaIndex == -1) 
        {
            fields[index++] = sentence;
            break;
        }
        fields[index++] = sentence.substring(0, commaIndex);
        sentence = sentence.substring(commaIndex + 1);
    }

    // Example: Extract specific PSTI fields
    if (index >= 4)
    {
        int psti_msg_num = fields[1].toInt();
        //Serial.println();Serial.print("psti_msg_num ");Serial.println(psti_msg_num);

        // RTK age and ratio values
        if (psti_msg_num == 30)
        {
            rtk_e = fields[9].toFloat();
            rtk_n = fields[10].toFloat();
            rtk_u = fields[11].toFloat();
            // Serial.print("rtk_e ");Serial.println(rtk_e);
            // Serial.print("rtk_n ");Serial.println(rtk_n);
            // Serial.print("rtk_u ");Serial.println(rtk_u);

            rtk_age = fields[14].toFloat();
            rtk_ratio = fields[15].toFloat();
            // Serial.print("rtk_age ");Serial.println(rtk_age);
            // Serial.print("rtk_ratio ");Serial.println(rtk_ratio);
        }

    }
}

// Read and send state of charge (SOC) data to ESP32
void readAndSendSOC()
{
    // Read data and pack into structure
    dataForTinkerSend.voltage = max17055.getInstantaneousVoltage();
    dataForTinkerSend.avg_voltage = max17055.getAvgVoltage();
    dataForTinkerSend.current = max17055.getInstantaneousCurrent();
    dataForTinkerSend.avg_current = max17055.getAvgCurrent();
    dataForTinkerSend.battery_capacity = max17055.getCalculatedCapacity();
    dataForTinkerSend.battery_age = max17055.getBatteryAge();
    dataForTinkerSend.cycle_counter = max17055.getChargeCycle();
    dataForTinkerSend.SOC = max17055.getSOC();
    dataForTinkerSend.temperature = max17055.getTemp();

    // Serial.println("");
    // Serial.print("Voltage: ");Serial.println(dataForTinkerSend.voltage);
    // Serial.print("Avg Voltage: ");Serial.println(dataForTinkerSend.avg_voltage);
    // Serial.print("Current: ");Serial.println(dataForTinkerSend.current);
    // Serial.print("Avg Current: ");Serial.println(dataForTinkerSend.avg_current);
    // Serial.print("Battery Capactity: ");Serial.println(dataForTinkerSend.battery_capacity);
    // Serial.print("Battery Age: ");Serial.println(dataForTinkerSend.battery_age);
    // Serial.print("Number of Cycles: ");Serial.println(dataForTinkerSend.cycle_counter);
    // Serial.print("SOC: ");Serial.println(dataForTinkerSend.SOC);
    // Serial.print("Temperature: ");Serial.println(dataForTinkerSend.temperature);
    // Serial.println("------------------------------------------------------------");
    
    uint16_t sendSize = 0;

    // Send data to TinkerSend radio using serial connection
    sendSize = radioTransfer.txObj(dataForTinkerSend,sendSize);
    radioTransfer.sendData(sendSize);

}

// Loop through valid baud rates for the GNSS receiver and determine the current setting
bool autoSetBaudRate()
{
    // Start serial connections to send correction data to GNSS receiver
    // This loop will detect the current baud rate of the GNSS receiver
    // by sending a message and determining which buad rate returns a valid
    // ACK message
    int valid_baud_rates[9] = {4800, 9600, 19200, 38400, 57600, 115200, 
                               230400, 460800, 921600};

    // Message to reset receiver to defaults
    uint8_t res_payload_length[]={0x00, 0x02};
    int res_payload_length_length = 2;
    uint8_t res_msg_id[]={0x04};
    int res_msg_id_length = 1;
    uint8_t res_msg_body[]={0x01};
    int res_msg_body_length = 1;

    // Loop through possible baud rates
    for (int i=0;i<9;i++)
    {
        // Open the serial connection to the receiver
        Serial1.begin(valid_baud_rates[i]);

        // Send a message to reset receiver to defaults
        if (program_skytraq.sendGenericMsg(res_msg_id,
                                           res_msg_id_length,
                                           res_payload_length,
                                           res_payload_length_length,
                                           res_msg_body,
                                           res_msg_body_length) == 1)
        {
            Serial.print("Found correct baud rate of ");
            Serial.print(valid_baud_rates[i]);
            Serial.println(" for GNSS receiver");
            return true;            
        }               
        else
        {
            Serial1.end();
        }
    }

    return false;
}
