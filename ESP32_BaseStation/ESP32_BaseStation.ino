#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>

// MAC Address of the Rover ESP32
uint8_t broadcastAddress[] = {0xD4, 0xF9, 0x8D, 0x34, 0x3A, 0xAC};

// LED indicator
#define NEO_PIN 4
Adafruit_NeoPixel pixels(1, NEO_PIN, NEO_GRB + NEO_KHZ800);

#define MAX_PACKET_LENGTH 240

#define MAX_RTCM_DATA 1024
char rtcm_data[MAX_RTCM_DATA];

typedef struct struct_message 
{
    char rtcm_data_to_send[MAX_PACKET_LENGTH];
    uint8_t bytes_sent;
} struct_message;

struct_message rtcm_message;

esp_now_peer_info_t peer_info;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    char macStr[18];
    Serial.print("Packet to: ");
    // Copies the sender mac address to a string
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.print(macStr);
    Serial.print(" send status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() 
{
    // USB serial connection
    Serial.begin(115200);

    // Wait for serial connection
    delay(1000);
 
    // GNSS hardware serial connection (rx/tx)
    // Receives RTCM correction data from the PX1125R
    Serial1.begin(115200, SERIAL_8N1, 21, 20);

    // Initialize ESP_Now
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) 
    {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
  
    esp_now_register_send_cb(OnDataSent);
    
    // Register peer
    peer_info.channel = 0;  
    peer_info.encrypt = false;

    // Register first peer  
    memcpy(peer_info.peer_addr, broadcastAddress, 6);
    while (esp_now_add_peer(&peer_info) != ESP_OK)
    {
        Serial.println("Rover ESP32 not found, delay and try again ...");
        delay(500);
    }
    Serial.println("Paired with Rover ESP32");
  
}
 
void loop() 
{

    int data_counter = 0;
    bool data_available = false;

    // Get RTCM correction data from PX1125R
    if (Serial1.available())
    {
        // Read a burst of GNSS data, the PX11XX receivers send RTCM
        // data in bursts of messages with hundreds of milliseconds
        // between each burst
        unsigned long last_read_time = millis();
        while (Serial1.available() || (millis() - last_read_time) < 50)
        {
            if (Serial1.available() && data_counter<MAX_RTCM_DATA)
            {
                // Read data from GNSS via serial
                rtcm_data[data_counter] = Serial1.read();
                data_counter++;
                data_available = true;
                last_read_time = millis();
            }
        }
    }
    if (data_available)
    {
        int data_length = data_counter;

        char last_rtcm_char = 0;

        unsigned int message_index = 0;

        // Loop over all data break into separate RTCM messages
        for (int i=0;i<data_length;i++)
        {
            // Read data into message to send
            rtcm_message.rtcm_data_to_send[message_index] = rtcm_data[i];
            Serial.print(rtcm_message.rtcm_data_to_send[message_index],HEX);Serial.print(" ");

            // End of data set
            if(i == data_length-1)
            {
                // TODO: Send message rtcm_data_to_send of length message_index+1 (0 base storage)
                rtcm_message.bytes_sent = message_index+1;
                sendTransmission();
                Serial.println();Serial.print("Sending message of length ");Serial.println(message_index+1);
            }
            // New message start
            else if (i>2 && (last_rtcm_char == 0XD3 && rtcm_message.rtcm_data_to_send[message_index] == 0X00))
            {
                // Message ended two data points ago (0 base storage)
                int msg_length = message_index - 1;
                // TODO: Send message rtcm_data_to_send of msg_length
                rtcm_message.bytes_sent = msg_length;
                sendTransmission();
                Serial.println();Serial.print("Sending message of length ");Serial.println(msg_length);

                // Start the next message
                rtcm_message.rtcm_data_to_send[0] = 0XD3;
                Serial.print(rtcm_message.rtcm_data_to_send[0],HEX);Serial.print(" ");

                rtcm_message.rtcm_data_to_send[1] = 0X00;
                Serial.print(rtcm_message.rtcm_data_to_send[1],HEX);Serial.print(" ");

                message_index = 1;
            }
            // Save previous value and increment message index
            last_rtcm_char = rtcm_message.rtcm_data_to_send[message_index];
            message_index++;

        }
        
        // All data sent, no data available
        data_available = false;
    }

}

void sendTransmission()
{
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &rtcm_message, sizeof(rtcm_message));
   
    if (result == ESP_OK) 
    {
        Serial.println("Sent with success");
    }
    else 
    {
        Serial.println("Error sending the data");
    }
}