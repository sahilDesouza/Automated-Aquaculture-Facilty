#include <WiFi.h>
#include <esp_now.h>
#include <stdint.h>
#include "Wire.h"
#include "DHT.h"
#include "DFRobot_PH.h"

#define DHTTYPE DHT20   // DHT 20 device address
#define TEMPERATURE_SENSOR_T2 3
#define LIGHT_SENSOR    4
/*Notice: The DHT10 and DHT20 is different from other DHT* sensor ,it uses i2c interface rather than one wire*/
/*So it doesn't require a pin.*/
DHT dht(DHTTYPE);         //   DHT10 DHT20 don't need to define Pin
#define WATER_SENSOR 1// water sensor pin
#define PH_PIN      A1

#if defined(ARDUINO_ARCH_AVR)
    #define debug  Serial

#elif defined(ARDUINO_ARCH_SAMD) ||  defined(ARDUINO_ARCH_SAM)
    #define debug  SerialUSB
#else
    #define debug  Serial
#endif


DFRobot_PH ph;

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message 
{
    int id;
    float temperature;
    float water;
    char dummyString[36];
    float phValue;
    // uint8_t crc; //CRC for error checking
    // bool d;
} struct_message;

typedef struct SlaveMessage
{
    char boardIDChar;
}SslaveMessage;

SslaveMessage slaveMSG;

int flagSend = 0;
/*------------------ CRC-8 Calculation-----------------------*/
// CRC-8 lookup table for polynomial 0x8C (reverse of 0x31)
const uint8_t crc8_table[256] = {
    0x00, 0x8C, 0x31, 0xBD, 0x45, 0xC9, 0x74, 0xF8, 0x8A, 0x06, 0xBB, 0x37, 0xCF, 0x43, 0xFE, 0x72,
    0x29, 0xA5, 0x18, 0x94, 0x6C, 0xE0, 0x5D, 0xD1, 0xA3, 0x2F, 0x92, 0x1E, 0xE6, 0x6A, 0xD7, 0x5B,
    0x52, 0xDE, 0x63, 0xEF, 0x17, 0x9B, 0x26, 0xAA, 0xD8, 0x54, 0xE9, 0x65, 0x9D, 0x11, 0xAC, 0x20,
    0x7B, 0xF7, 0x4A, 0xC6, 0x3E, 0xB2, 0x0F, 0x83, 0xF1, 0x7D, 0xC0, 0x4C, 0xB4, 0x38, 0x85, 0x09,
    0xA4, 0x28, 0x95, 0x19, 0xE1, 0x6D, 0xD0, 0x5C, 0x2E, 0xA2, 0x1F, 0x93, 0x6B, 0xE7, 0x5A, 0xD6,
    0x8D, 0x01, 0xBC, 0x30, 0xC8, 0x44, 0xF9, 0x75, 0x07, 0x8B, 0x36, 0xBA, 0x42, 0xCE, 0x73, 0xFF,
    0xF6, 0x7A, 0xC7, 0x4B, 0xB3, 0x3F, 0x82, 0x0E, 0x7C, 0xF0, 0x4D, 0xC1, 0x39, 0xB5, 0x08, 0x84,
    0xDF, 0x53, 0xEE, 0x62, 0x9A, 0x16, 0xAB, 0x27, 0x55, 0xD9, 0x64, 0xE8, 0x10, 0x9C, 0x21, 0xAD,
    0x91, 0x1D, 0xA0, 0x2C, 0xD4, 0x58, 0xE5, 0x69, 0x1B, 0x97, 0x2A, 0xA6, 0x5E, 0xD2, 0x6F, 0xE3,
    0xB8, 0x34, 0x89, 0x05, 0xFD, 0x71, 0xCC, 0x40, 0x32, 0xBE, 0x03, 0x8F, 0x77, 0xFB, 0x46, 0xCA,
    0xC3, 0x4F, 0xF2, 0x7E, 0x86, 0x0A, 0xB7, 0x3B, 0x49, 0xC5, 0x78, 0xF4, 0x0C, 0x80, 0x3D, 0xB1,
    0xEA, 0x66, 0xDB, 0x57, 0xAF, 0x23, 0x9E, 0x12, 0x60, 0xEC, 0x51, 0xDD, 0x25, 0xA9, 0x14, 0x98,
    0x21, 0xAD, 0x10, 0x9C, 0x64, 0xE8, 0x55, 0xD9, 0xAB, 0x27, 0x9A, 0x16, 0xEE, 0x62, 0xDF, 0x53,
    0x08, 0x84, 0x39, 0xB5, 0x4D, 0xC1, 0x7C, 0xF0, 0x82, 0x0E, 0xB3, 0x3F, 0xC7, 0x4B, 0xF6, 0x7A,
    0x73, 0xFF, 0x42, 0xCE, 0x36, 0xBA, 0x07, 0x8B, 0xF9, 0x75, 0xC8, 0x44, 0xBC, 0x30, 0x8D, 0x01,
    0x5A, 0xD6, 0x6B, 0xE7, 0x1F, 0x93, 0x2E, 0xA2, 0xD0, 0x5C, 0xE1, 0x6D, 0x95, 0x19, 0xA4, 0x28
};

// Replace with the MAC address of the receiver ESP32S3 board
uint8_t broadcastAddress[] = {0x34, 0x85, 0x18, 0x91, 0xB1, 0x84}; //34:85:18:8E:29:18 60B: 34:85:18:AC:BD:6C; 59A: 0x34, 0x85, 0x18, 0x91, 0x30, 0xA4

// Create a struct_message called myData
struct_message myData;
esp_now_peer_info_t peerInfo;

// Function to calculate CRC-8
uint8_t calculateCRC8(const void* data, size_t length) 
{
    uint8_t crc = 0;
    uint8_t* buffer = (uint8_t*) data;

    for (size_t i = 0; i < length; i++) 
    {
        crc = crc8_table[crc ^ buffer[i]];
    }

    return crc;
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) 
{
    char macStr[18];
    Serial.print("Packet received from: ");
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.println(macStr);
    memcpy(&slaveMSG, incomingData, sizeof(slaveMSG));
    if(slaveMSG.boardIDChar == '1')
    //{
        flagSend = 1;
        //Serial.printf("Master messaged received %c\n", slaveMSG.boardIDChar);
    //}

    uint8_t ReceivedCRC = incomingData [len - 1];
    uint8_t calculatedCRC; 
    int CRCCheckSum;
    byte* byteArray;
}
// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() 
{
    //initialise I2C
    debug.begin(115200);
    Wire.begin();
    dht.begin();

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) 
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);
    
    // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add peer");
        return;
    }

    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info
    esp_now_register_recv_cb(OnDataRecv);

    pinMode(WATER_SENSOR, INPUT);
    pinMode(PH_PIN, INPUT);
    pinMode(TEMPERATURE_SENSOR_T2, INPUT);
    pinMode(LIGHT_SENSOR, INPUT);
}

void loop() 
{
    //initialise required variables
    float phValue;
    float voltage_PH;
    float ph;
    float waterval;
    float tempVal[2] = {0};
    int sensorValue_op_lux;
    int sensorValue_tm;
    float voltage_op_lux;
    float voltage_tm;
    float temperatureT1;
    float light;

    //read PH values
    phValue = analogRead(PH_PIN);
    //read water level values
    waterval = analogRead(WATER_SENSOR);

    //read sensor data from designed board
    sensorValue_op_lux = analogRead(LIGHT_SENSOR);
    sensorValue_tm = analogRead(TEMPERATURE_SENSOR_T2);

    //calculate voltage
    voltage_op_lux = sensorValue_op_lux * (3.3 / 4096.0);
    voltage_tm = sensorValue_tm * (3.3 / 4096.0);

    //temperature result from linear equation for temperature and light
    temperatureT1 = -29.42 * voltage_tm + 85.808 ;
    light = -987  * voltage_op_lux + 2059.6;


    //next read data from DHT20 Sensor
    // Set values to send
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    //store temperature in array
    if (!dht.readTempAndHumidity(tempVal)) 
    {
        Serial.printf("Real time Temperature sensor value T1: %.2f\n", temperatureT1);
        debug.print("Real time Temperature sensor value T2: ");
        debug.print(tempVal[1]);
        debug.println(" *C");
        myData.id = 1;
        myData.temperature = tempVal[1];
        
        Serial.printf("Real time LUX(Light Intensity) sensor value : %.2f\n", light);

        debug.print("Real time water level value: ");
        if(waterval >= 0 && waterval <= 800 )
        {
            Serial.println("Water Level Normal: ");
        }
        else if(waterval > 800 && waterval <= 1000)
        {
            Serial.println("Water Level Dropped");
        }
        else if (waterval > 1000)
        {
            Serial.println("Insufficiant water levels");
        }
        myData.water = waterval;


        
        voltage_PH = phValue * (5.0 / 4096.0); // Assuming a 5V ADC reference
        Serial.printf("VolatgePH value : %.2f\n", voltage_PH);
        ph = -3.7324 * voltage_PH + 15.339;
        //ph = 3.5 * voltage_PH;
        Serial.printf("PH value : %.2f\n", ph);
        myData.phValue = ph;


        // Calculate CRC  
        uint8_t CRC = calculateCRC8(&myData, sizeof(myData));

        // Send data including CRC using ESP-NOW
        //create array of payload plus CRC
        uint8_t dataToSend[sizeof(myData) + 1];

        //copy myData to this new array created
        memcpy(dataToSend, &myData, sizeof(myData));

        //add CRC to last index
        dataToSend[sizeof(myData)] = CRC;

        // Send message via ESP-NOW
        //delay(17000);
        if(flagSend == 1)
        {
            esp_err_t result = esp_now_send(broadcastAddress, dataToSend, sizeof(dataToSend));
            flagSend = 0;
            
            if (result == ESP_OK) 
            {
                Serial.print(F(" Temperature Data Transmitted: "));
                Serial.print(myData.temperature);
                debug.println(" *C");

                Serial.print(F(" waterlevel Data Transmitted: "));
                Serial.print(myData.water);

                Serial.print(F(" PH Data Transmitted: "));
                Serial.print(myData.phValue);

                Serial.println(" Sender 1: Sent with success");
                Serial.print("  Calculated CRC: ");
                Serial.println(dataToSend[sizeof(myData)]);
            }
            else 
            {
                Serial.println("Error sending the data");
            }
        }
    } 
    else 
    {
      debug.println("Failed to get temprature and humidity value.");
    }
    

    delay(2000);
}