#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include <TinyGPS++.h>

#define MAX_PACKET_LENGTH 240
char rtcm_data[MAX_PACKET_LENGTH];

typedef struct struct_message 
{
    char rtcm_data[MAX_PACKET_LENGTH];
    uint8_t bytes_sent;
} struct_message;

struct_message rtcm_message;

#define NEO_PIN 4
Adafruit_NeoPixel pixels(1, NEO_PIN, NEO_GRB + NEO_KHZ800);

// GNSS receiver data is parsed by the TinyGPSPlus library
TinyGPSPlus gnss;

TinyGPSCustom gnss_quality(gnss, "GPGGA", 6);
TinyGPSCustom psti_1(gnss, "PSTI", 1);
TinyGPSCustom psti_5(gnss, "PSTI", 5);
TinyGPSCustom psti_6(gnss, "PSTI", 6);
TinyGPSCustom psti_7(gnss, "PSTI", 7);
TinyGPSCustom psti_8(gnss, "PSTI", 8);
TinyGPSCustom psti_13(gnss, "PSTI", 13);
TinyGPSCustom psti_14(gnss, "PSTI", 14);
TinyGPSCustom psti_15(gnss, "PSTI", 15);
TinyGPSCustom psti_18(gnss, "PSTI", 18);

// GNSS parsing
String gps_quality_text = "";
float rtk_age = 0;
int num_cycle_slip_gps = 0;
int num_cycle_slip_bds = 0;
int num_cycle_slip_gal = 0;
float rtk_east = 0.0;
float rtk_north = 0.0;
float rtk_up = 0.0;
float rtk_ratio = 0;
float lattitude = 0.0;
float longitude = 0.0;

void onReceive(const uint8_t * mac, const uint8_t *data, int len) 
{
    // Cast the incoming data to the correct type
    memcpy(&rtcm_message, data, sizeof(rtcm_message));

    Serial.print("Bytes received: ");Serial.println(len);
    Serial.print("Valid bytes received: ");Serial.println(rtcm_message.bytes_sent);
    Serial.print("Message: ");
    for (int i=0;i<rtcm_message.bytes_sent;i++)
    {
        Serial.print(rtcm_message.rtcm_data[i],HEX);Serial.print(" ");
    }
    Serial.println();
        
    // Send RTCM data to GNSS receiver correction input
    Serial1.write(rtcm_message.rtcm_data, rtcm_message.bytes_sent);
}
 
void setup() 
{
    // Initialize USB  serial
    Serial.begin(115200);
    delay(1000);

    // GNSS hardware serial connection
    Serial1.begin(115200, SERIAL_8N1, 21, 20);
  
    //Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    //Init ESP-NOW
    if (esp_now_init() != ESP_OK) 
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Initialize NeoPixel
    pixels.begin();
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(50, 0, 0));
    pixels.show();
    
    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info
    esp_now_register_recv_cb(esp_now_recv_cb_t(onReceive));
}
 
void loop() 
{
    readAndParseGNSS();
}

// Read and parse data from GNSS receiver
void readAndParseGNSS()
{

    // Read a burst of GNSS data, the PX11XX receivers send RTCM
    // data in bursts of messages with hundreds of milliseconds
    // between each burst
    unsigned long last_read_time = millis();
    while (Serial1.available() || (millis() - last_read_time) < 50)
    {
        if (Serial1.available())
        {
            // Read data from GNSS via serial
            char ch = Serial1.read();
            gnss.encode(ch);
            //Serial.write(ch);
            last_read_time = millis();
        }
    }

    // If message is updated, then populate fields with GNSS data
    if (gnss_quality.isUpdated())
    {
        
        // Clear NeoPixel
        pixels.clear();

        // Convert GNSS Fix type to string and set NeoPixel color
        if(atoi(gnss_quality.value())==0)
        {
           gps_quality_text = "Invalid";
           pixels.setPixelColor(0, pixels.Color(50, 0, 0));
        }
        else if(atoi(gnss_quality.value())==1)
        {
           gps_quality_text = "GPS";
           pixels.setPixelColor(0, pixels.Color(50, 50, 0));
        }
        else if(atoi(gnss_quality.value())==2)
        {
           gps_quality_text = "DGPS";
           pixels.setPixelColor(0, pixels.Color(50, 50, 0));
        }
        else if(atoi(gnss_quality.value())==4)
        {
           gps_quality_text = "RTK Fix";
           pixels.setPixelColor(0, pixels.Color(0, 0, 50));
        }
        else if(atoi(gnss_quality.value())==5)
        {
           gps_quality_text = "RTK Float";
           pixels.setPixelColor(0, pixels.Color(0, 50, 0));
        }
        else
        {
           gps_quality_text = "NA";
           pixels.setPixelColor(0, pixels.Color(50, 0, 0));
        }
           
        pixels.show();
    
        // Lattitude and longitude values
        lattitude = gnss.location.lat();
        longitude = gnss.location.lng();

    }
    if (psti_1.isUpdated())
    {
        if(strcmp(psti_1.value(),"030") == 0)
        {
            // Age of correction data if present
            rtk_age = atof(psti_14.value());
            rtk_ratio = atof(psti_15.value());
        }
        else if (strcmp(psti_1.value(),"032") == 0)
        {
            rtk_east = atof(psti_6.value());
            rtk_north = atof(psti_7.value());
            rtk_up = atof(psti_8.value());
        }
        else if (strcmp(psti_1.value(),"033") == 0)
        {
            num_cycle_slip_gps = atoi(psti_6.value());
            num_cycle_slip_bds = atoi(psti_7.value());
            num_cycle_slip_gal = atoi(psti_8.value());
        }
    }

}