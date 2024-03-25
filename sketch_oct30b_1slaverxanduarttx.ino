
//receiver code
#include <esp_now.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <BlynkSimpleEsp32.h>
HardwareSerial SerialPort(2); //use UART2

#define UART_TX_PIN 43
#define UART_RX_PIN 44
#define BAUD_RATE 115200

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message 
{
    int id;
    // char a[32];
    float b;
    float c;
    char dataRx[36];
    float d;
    // uint8_t crc; //CRC for error checking
    // bool d;
} struct_message;

typedef struct UART_message 
{
    int id;
    float b;
    float c;
    char string[36];
    float d;
    uint8_t CRC_checksum; //CRC for error checking
    // bool d;
} UART_message;  
UART_message dataUARTSend;
// Create a struct_message called myData
struct_message myData;
struct_message sender1;
struct_message sender2;
struct_message sender3;
struct_message sender4;
struct_message sender5;

typedef struct MasterMessage
{
    char boardIDChar;
}SMasterMessage;

SMasterMessage masterMSG;
BlynkTimer timer;
int flagCheck = 0, receiverFlag = 0;
esp_now_peer_info_t peerInfo;
esp_now_peer_info_t peerInfo2;
// Create an array with all the structures
struct_message boardsStruct[5] = {sender1, sender2, sender3, sender4, sender5};
uint8_t broadcastAddressSlave1Node[] = {0x34, 0x85, 0x18, 0x91, 0xC7, 0x90}; //tempslavemacadd
uint8_t broadcastAddressSlave2Node[] = {0x35, 0x85, 0x18, 0x91, 0xC0, 0x6C};

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

// Function to calculate CRC-8
uint8_t calculateCRC8(const void* data, size_t length) 
{
    uint8_t crc = 0;
    uint8_t* buffer = (uint8_t*)data;

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
    memcpy(&myData, incomingData, sizeof(myData));
    uint8_t ReceivedCRC = incomingData [len - 1];
    uint8_t calculatedCRC; 
    int CRCCheckSum;
    byte* byteArray;
    receiverFlag = 1;
    int id = myData.id;
    /**********----Process different types of incoming data----**********/    
    switch (id)
    {
        
        case 2:
            // Calculate CRC-8 for the received data
            calculatedCRC = calculateCRC8(&myData, sizeof(myData));  // Mind how many data types are sent, //Board ID 2
            boardsStruct[myData.id-1].b = myData.b;

           // boardsStruct[myData.id-1].c = myData.c;
            Serial.printf("    Board ID %u: %u bytes\n", myData.id, len);
            Serial.printf("    Light: %.2f %%\n", boardsStruct[myData.id-1].b);
            Serial.printf("    Calculated CRC: %d \n", calculatedCRC);
            Serial.printf("    Received CRC: %d \n", ReceivedCRC);
            if (calculatedCRC == ReceivedCRC){
                Serial.printf("    Data Tramission Right!");
                CRCCheckSum = 1;
            }
            else
            {
                Serial.printf("    Data Tramission Wrong!");
                CRCCheckSum = 0;
            }  
            Serial.println();
            
            // UART Data Transmission
            dataUARTSend.id = myData.id;
            dataUARTSend.b = boardsStruct[myData.id-1].b;
            //dataUARTSend.b = myDataRx;
            //dataUARTSend.c=boardsStruct[myData.id-1].c; 
            dataUARTSend.CRC_checksum = CRCCheckSum;           
            byteArray = reinterpret_cast<byte*>(&dataUARTSend);
            Serial.printf(" Board Id written %d\n", dataUARTSend.id);
            SerialPort.write(byteArray, sizeof(dataUARTSend));  
            receiverFlag = 0;           
            break;
            
        
        case 1:
            calculatedCRC = calculateCRC8(&myData, sizeof(myData));  // Mind how many data types are sent,//Board ID 1
            boardsStruct[myData.id-1].b = myData.b;
            boardsStruct[myData.id-1].c = myData.c;
            boardsStruct[myData.id-1].d = myData.d;
            
            //myDataRx2= myData.b;
            //boardsStruct[myData.id-1].c = myData.c;
            Serial.printf("    Board ID %u: %u bytes from temperature sensor\n", myData.id, len);
            Serial.printf("    Temperature Value: %.2f C\n", boardsStruct[myData.id-1].b);
            Serial.printf("    Water level: %.2f\n", boardsStruct[myData.id-1].c);
            Serial.printf("    Ph Value: %.2f\n", boardsStruct[myData.id-1].d);
            Serial.printf("    Calcultaed CRC: %d \n", calculatedCRC);
            Serial.printf("    Received CRC: %d \n", ReceivedCRC);
            if (calculatedCRC == ReceivedCRC)
            {
                Serial.printf("    Data Tramission Right!");
                CRCCheckSum = 1;
            }
            else
            {
                Serial.printf("    Data Tramission Wrong!");
                CRCCheckSum = 0;
            }  
            Serial.println();

            dataUARTSend.id = myData.id;
            dataUARTSend.b = boardsStruct[myData.id-1].b;
            dataUARTSend.c = boardsStruct[myData.id-1].c;
            dataUARTSend.d = boardsStruct[myData.id-1].d;
            dataUARTSend.CRC_checksum = CRCCheckSum;           
            byteArray = reinterpret_cast<byte*>(&dataUARTSend);
            Serial.printf(" Board Id written %d\n", dataUARTSend.id);
            SerialPort.write(byteArray, sizeof(dataUARTSend));
            receiverFlag = 0;  
            break;   

        case 4:
            calculatedCRC = calculateCRC8(&myData, sizeof(myData));  // Mind how many data types are sent,//Board ID 4
            boardsStruct[myData.id-1].b = myData.b;
            strcpy(boardsStruct[myData.id-1].dataRx, myData.dataRx);
            //myDataRx2= myData.b;
            //boardsStruct[myData.id-1].c = myData.c;
            Serial.printf("    Board ID %u: %u bytes from Dust and Air Quality Sensor\n", myData.id, len);
            Serial.printf("    Dust Value: %.2f\n", boardsStruct[myData.id-1].b);
            Serial.printf("    Air Quality data RX: %s \n", boardsStruct[myData.id-1].dataRx);

            Serial.printf("    Calcultaed CRC: %d \n", calculatedCRC);
            Serial.printf("    Received CRC: %d \n", ReceivedCRC);
            if (calculatedCRC == ReceivedCRC)
            {
                Serial.printf("    Data Tramission Right!");
                CRCCheckSum = 1;
            }
            else
            {
                Serial.printf("    Data Tramission Wrong!");
                CRCCheckSum = 0;
            }  
            Serial.println();

            dataUARTSend.id = myData.id;
            dataUARTSend.b = boardsStruct[myData.id-1].b;
            strcpy(dataUARTSend.string, boardsStruct[myData.id-1].dataRx);
            dataUARTSend.CRC_checksum = CRCCheckSum;           
            byteArray = reinterpret_cast<byte*>(&dataUARTSend);
            Serial.printf(" Board Id written %d\n", dataUARTSend.id);
            SerialPort.write(byteArray,sizeof(dataUARTSend));  
            receiverFlag = 0; 
            break;       

        case 3:
            calculatedCRC = calculateCRC8(&myData, sizeof(myData));  // Mind how many data types are sent,//Board ID 3
            strcpy(boardsStruct[myData.id-1].dataRx, myData.dataRx);
            //myDataRx2= myData.b;
            //boardsStruct[myData.id-1].c = myData.c;
            Serial.printf("    Board ID %u: %u bytes from Air Quality sensor\n", myData.id, len);
            Serial.printf("    Air Quality data RX: %s C\n", boardsStruct[myData.id-1].dataRx);
            Serial.printf("    Calculated CRC: %d \n", calculatedCRC);
            Serial.printf("    Received CRC: %d \n", ReceivedCRC);
            if (calculatedCRC == ReceivedCRC)
            {
                Serial.printf("    Data Tramission Right!");
                CRCCheckSum = 1;
            }
            else
            {
                Serial.printf("    Data Tramission Wrong!");
                CRCCheckSum = 0;
            }  
            Serial.println();

            dataUARTSend.id = myData.id;
            strcpy(dataUARTSend.string, boardsStruct[myData.id-1].dataRx);
            dataUARTSend.CRC_checksum = CRCCheckSum;           
            byteArray = reinterpret_cast<byte*>(&dataUARTSend);
            Serial.printf(" Board Id written %d\n", dataUARTSend.id);
            SerialPort.write(byteArray,sizeof(dataUARTSend));
            receiverFlag = 0;   
            break;     

        case 5:
            calculatedCRC = calculateCRC8(&myData, sizeof(myData));  // Mind how many data types are sent,//Board ID 5
            boardsStruct[myData.id-1].b = myData.b;
            //myDataRx2= myData.b;
            //boardsStruct[myData.id-1].c = myData.c;
            Serial.printf("    Board ID %u: %u bytes from Water Level sensor\n", myData.id, len);
            Serial.printf("    Water Value: %.2f C\n", boardsStruct[myData.id-1].b);
            Serial.printf("    Calcultaed CRC: %d \n", calculatedCRC);
            Serial.printf("    Received CRC: %d \n", ReceivedCRC);
            if (calculatedCRC == ReceivedCRC)
            {
                Serial.printf("    Data Tramission Right!");
                CRCCheckSum = 1;
            }
            else
            {
                Serial.printf("    Data Tramission Wrong!");
                CRCCheckSum = 0;
            }  
            Serial.println();

            dataUARTSend.id = myData.id;
            dataUARTSend.b = boardsStruct[myData.id-1].b;
            dataUARTSend.CRC_checksum = CRCCheckSum;           
            byteArray = reinterpret_cast<byte*>(&dataUARTSend);
            Serial.printf(" Board Id written %d\n", dataUARTSend.id);
            SerialPort.write(byteArray,sizeof(dataUARTSend));
            receiverFlag = 0;   
            break;         

        
        default:
            Serial.printf("Invalid Board ID received......");
            receiverFlag = 0; 
            break; 
               
    }
    
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void updateBoardDetails()
{
    if(masterMSG.boardIDChar == '1')
    {
        masterMSG.boardIDChar = '4';
        flagCheck = 0;
    }
    else
    {
        masterMSG.boardIDChar = '1';
        flagCheck = 0;
    }
}
 
void setup() 
{
    // Initialize Serial Monitor
    Serial.begin(115200);
    Serial.begin(BAUD_RATE);
    SerialPort.begin(BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);  
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) 
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    // Setup a function to be called every 5 second
    timer.setInterval(2000L, updateBoardDetails);
    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);

    // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddressSlave1Node, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add peer");
        return;
    }

        // Register peer
    memcpy(peerInfo2.peer_addr, broadcastAddressSlave2Node, 6);
    peerInfo2.channel = 1;  
    peerInfo2.encrypt = false;
    
    // Add peer        
    if (esp_now_add_peer(&peerInfo2) != ESP_OK)
    {
        Serial.println("Failed to add peer");
        return;
    }
    masterMSG.boardIDChar = '1';

    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info
    esp_now_register_recv_cb(OnDataRecv);
    

}
 
void loop() 
{
    //constantly interate and receive data from sensors
    //Serial.printf(" %d %d\n", flagCheck, receiverFlag);
    if((flagCheck == 0) && (receiverFlag == 0))
    {
        // Calculate CRC  
        uint8_t CRC = calculateCRC8(&masterMSG, sizeof(masterMSG));

        // Send data including CRC using ESP-NOW
        //create array of payload plus CRC
        uint8_t dataToSend[sizeof(masterMSG) + 1];

        //copy myData to this new array created
        memcpy(dataToSend, &masterMSG, sizeof(masterMSG));

        //add CRC to last index
        dataToSend[sizeof(masterMSG)] = CRC;

        if(masterMSG.boardIDChar == '1')
        {
            esp_err_t result = esp_now_send(broadcastAddressSlave1Node, dataToSend, sizeof(dataToSend));
            
            if (result == ESP_OK) 
            {
                Serial.printf(" Master sent request to Node %c: ", masterMSG.boardIDChar);
            }
            else 
            {
                Serial.println("Error sending the data");
            }
        }
        else if(masterMSG.boardIDChar == '4')
        {
            esp_err_t result = esp_now_send(broadcastAddressSlave2Node, dataToSend, sizeof(dataToSend));
            
            if (result == ESP_OK) 
            {
                Serial.printf(" Master sent request to Node %c: ", masterMSG.boardIDChar);
           }
            else 
           {
                Serial.println("Error sending the data");
            }
        }
        //delay(1000);
        flagCheck = 1;
        //receiverFlag = 1;
    }
    timer.run();

}
