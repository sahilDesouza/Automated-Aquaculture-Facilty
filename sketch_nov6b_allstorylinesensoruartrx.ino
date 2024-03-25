
#include <HardwareSerial.h>
#include <WiFi.h>
#define BLYNK_TEMPLATE_ID "TMPL5vyC8Wat6"
#define BLYNK_TEMPLATE_NAME "Smart Aquaculture Facility"
#define BLYNK_AUTH_TOKEN "AhiXhMfwqW_AVrVtkU9sx_IbjmQNA_Il"
/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial


#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include "Air_Quality_Sensor.h"

HardwareSerial SerialPort(2); //use UART2
#define UART_TX_PIN 43
#define UART_RX_PIN 44
#define BAUD_RATE 115200

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Redmi";
char pass[] = "";

float waterLevelRX;
float temperatureValueRX;
float dustValueRX;
char airQualityRX[36];
float phValueRX;
int boardID;
float concentrationPercentage;
BlynkTimer timer;
 
unsigned long previousMillis = 0;
unsigned long interval = 30000;

//define structure
typedef struct UART_message 
{
    int id;
    float b;
    float c;
    char string[36];
    float d;
    uint8_t CRC_checksum; //CRC for error checking
} UART_message;



// This function is called every time the Virtual Pin 0 state changes
BLYNK_WRITE(V0)
{
    // Set incoming value from pin V0 to a variable
    int value = param.asInt();

    // Update state
    Blynk.virtualWrite(V1, value);
}

// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED()
{
    // Change Web Link Button message to "Congratulations!"
    Blynk.setProperty(V3, "offImageUrl", "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations.png");
    Blynk.setProperty(V3, "onImageUrl",  "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations_pressed.png");
    Blynk.setProperty(V3, "url", "https://docs.blynk.io/en/getting-started/what-do-i-need-to-blynk/how-quickstart-device-was-made");
}

// This function sends Arduino's uptime every second to Virtual Pin 2.
void myTimerEvent()
{
    if(boardID == 1)
    {
        Serial.printf("Water Level read %f\n", waterLevelRX);
        Blynk.virtualWrite(V2, waterLevelRX);
        Serial.printf("Temperature value written to Blynk %f\n", temperatureValueRX);
        Blynk.virtualWrite(V4, temperatureValueRX);
        Serial.printf("PH value written to Blynk %f\n", phValueRX);
        Blynk.virtualWrite(V7, phValueRX);
    }
    else if(boardID == 4)
    {
       
        concentrationPercentage = ( (float(dustValueRX) / 28000) * 100);

        if((concentrationPercentage > 0) && (concentrationPercentage < 15))
        {
            Serial.println("Low Dust Levels");
            Serial.printf("Dust % value written to Blynk %f\n", concentrationPercentage);
            Blynk.virtualWrite(V5, concentrationPercentage);
        }
        else if((concentrationPercentage > 15) && (concentrationPercentage < 45))
        {
            Serial.println("High Dust levels");
             Serial.printf("Dust % value written to Blynk %f\n", concentrationPercentage);
            Blynk.virtualWrite(V5, concentrationPercentage);
        }
        else if((concentrationPercentage > 45))
        {
            Serial.println("Highest Dust levels");
             Serial.printf("Dust % value written to Blynk %f\n", concentrationPercentage);
            Blynk.virtualWrite(V5, concentrationPercentage);
        }
        
        //Serial.printf("Temperature value written to Blynk %f\n", waterLevelrx);

        if (strcmp(airQualityRX, "High pollution! Force signal active") == 0) 
        {
            Serial.printf("%s\n", airQualityRX);
            Serial.printf("Air Quality value written to Blynk %d\n", 0);
            Blynk.virtualWrite(V6, 0);
        } 
        else if (strcmp(airQualityRX, "High pollution!") == 0)
        {
            Serial.printf("%s\n", airQualityRX);
            Serial.printf("Air Quality value written to Blynk %d\n", 1);
            Blynk.virtualWrite(V6, 1);
        } 
        else if (strcmp(airQualityRX, "Low pollution!") == 0)
        {
            Serial.printf("%s\n", airQualityRX);
            Serial.printf("Air Quality value written to Blynk %d\n", 2);
            Blynk.virtualWrite(V6, 2);
        } 
        else if (strcmp(airQualityRX, "Fresh air") == 0)
        {
            Serial.printf("%s\n", airQualityRX);
            Serial.printf("Air Quality value written to Blynk %d\n", 3);
            Blynk.virtualWrite(V6, 3);
        }
    }
}

void setup() 
{
    Serial.begin(BAUD_RATE);
    SerialPort.begin(BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    // Debug console
    Serial.begin(115200);

    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

    // You can also specify server:
    //Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass, "blynk.cloud", 80);
    //Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass, IPAddress(192,168,1,100), 8080);

    // Setup a function to be called every second
    timer.setInterval(1000L, myTimerEvent);

}

void loop() 
{
    
    if (SerialPort.available()) 
    {
        delay(100);
        Blynk.run();
        // String message = SerialPort.readString();
        // Serial.println("Received message: " + message);
        size_t byteSize = sizeof(UART_message);
        byte* byteArray = new byte[byteSize];
        SerialPort.read(byteArray, byteSize);

        //Deserialize the byte array back into the structure
        UART_message UARTReceivedData;
        memcpy(&UARTReceivedData, byteArray, byteSize);
        int id = UARTReceivedData.id;
        Serial.printf("ID : %d\n", id);
        /*--------Process different types of data----------*/
        switch(id)
        {
            case 2:      
                // Print the received structure data and check the data valid or not
                //if (UARTReceivedData.b > 0 && (UARTReceivedData.c > -50 && UARTReceivedData.c < 50))
                //{
                Serial.print("Received from Board ID: ");
                Serial.println(UARTReceivedData.id);    
                Serial.print("    Light: ");
                // b is linked to Light
                Serial.print(UARTReceivedData.b);   
                Serial.print("    CRC = ");  // CRC=1 means data transmission right
                Serial.println(UARTReceivedData.CRC_checksum);
                //delay(1000);
                //}
                break;

            //temperature and water
            case 1:         
                //if (UARTReceivedData.b > 0) 
                //{
                    Serial.print("Received from Board ID: ");
                    Serial.println(UARTReceivedData.id);
                    boardID = UARTReceivedData.id;

                    // b is linked to temperature
                    Serial.print("    Temperature C: ");
                    Serial.print(UARTReceivedData.b);   
                    temperatureValueRX = UARTReceivedData.b;

                    // b is linked to water
                    Serial.print("    Water: ");
                    Serial.print(UARTReceivedData.c);   
                    waterLevelRX = UARTReceivedData.c;

                    // d is linked to PH
                    Serial.print("    PH: ");
                    Serial.print(UARTReceivedData.d);   
                    phValueRX = UARTReceivedData.d;

                    Serial.print("    CRC = ");  // CRC=1 means data transmission right
                    Serial.println(UARTReceivedData.CRC_checksum);
                //}      
                break;

            //air quality
            case 3:      
                // Print the received structure data and check the data valid or not
                //if (UARTReceivedData.b > 0 && (UARTReceivedData.c > -50 && UARTReceivedData.c < 50))
                //{
                Serial.print("Received from Board ID: ");
                Serial.println(UARTReceivedData.id);    
                Serial.print("    Air Quality: ");
                // string is linked to pollution levels
                Serial.print(UARTReceivedData.string);   

                // CRC=1 means data transmission right
                Serial.print("    CRC = ");  
                Serial.println(UARTReceivedData.CRC_checksum);
                //}
                break;

            //Dust and Air Quality
            case 4:        
                // Print the received structure data and check the data valid or not
                //if (UARTReceivedData.b > 0 && (UARTReceivedData.c > -50 && UARTReceivedData.c < 50))
                //{
                Serial.print("Received from Board ID: ");
                Serial.println(UARTReceivedData.id);
                boardID = UARTReceivedData.id;

                // b is linked to dust values    
                Serial.print("    Dust Values: ");
                Serial.print(UARTReceivedData.b); 
                dustValueRX = UARTReceivedData.b;


                // string is linked to pollution levels
                Serial.print("    Air Quality: ");
                Serial.printf("Air Quality  value %s\n", UARTReceivedData.string);
                strcpy(airQualityRX, UARTReceivedData.string);

                // CRC=1 means data transmission right
                Serial.print("    CRC = ");  
                Serial.println(UARTReceivedData.CRC_checksum);
                //delay(1000);
                //}
                break;

            //water sensor levels
            case 5:      
                // Print the received structure data and check the data valid or not
                //if (UARTReceivedData.b > 0 && (UARTReceivedData.c > -50 && UARTReceivedData.c < 50))
                //{
                Serial.print("Received from Board ID: ");
                Serial.println(UARTReceivedData.id);  
                // b is linked to water level  
                Serial.print("    Water Level: ");                Serial.print(UARTReceivedData.b);   

                // CRC=1 means data transmission right
                Serial.print("    CRC = ");  
                Serial.println(UARTReceivedData.CRC_checksum);
                //delay(1000);
                //}
                break;
            
            default:
                Serial.print("INVALID ID!");              
                break;
        }

        timer.run();
        delay(1000);
    }

}