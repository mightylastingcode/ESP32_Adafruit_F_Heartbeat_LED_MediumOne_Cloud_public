/***************************************************************************
  This is an example of program for connected the Adafruit Huzzah ESP32 Feather
  and Sparkfun ESP32 Thing and Expressif ESP32 DevKitC v4
  to the Medium One Prototyping Sandbox.  Visit www.medium.one for more information.
  Author: Medium One
  Last Revision Date: May 9, 2018

  The program includes a library and portions of sample code from Adafruit
  with their description below:
  
  This is a library for LED control.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

//Uncomment this line below if Expressif ESP32 DevKitC v4 is used because 
//this board does not have a user LED.  An external LED is used and connected
//to GPIO pin 16.
  
//#define LED_BUILTIN 16

 /*
  * Current Revsion Log for Adafruit Huzzad ESP32 Feather (Michael Li)
  *  IDE: Arduino IDE 1.89
  * 
  */

/*-------------------------------------------------------------------------*
 *          Includes: 
 *-------------------------------------------------------------------------*/
 
#include <PubSubClient.h>
#include <WiFi.h>        // new ESP32 WiFi library header's name
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <SPI.h>
#include "freertos/event_groups.h"  // Use the event group feature
/*-------------------------------------------------------------------------*
 * Constants:
 *-------------------------------------------------------------------------*/
/* define RTOS event bits (32 bits max)*/
#define LED_REUEST_EVENT_BIT        ( 1 << 0 ) //1  // Bit 0
/*-------------------------------------------------------------------------*
 * Types:
 *-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*
 * Globals: (Timer related)
 *-------------------------------------------------------------------------*/

// Create the event group
EventGroupHandle_t eventgroupflags;

/*-------------------------------------------------------------------------*
 * Globals: (Timer related)
 *-------------------------------------------------------------------------*/
// ongoing timer counter for heartbeat
static int heartbeat_timer = 0;

// set heartbeat period in milliseconds
static int heartbeat_period = 60000;

// track time when last connection error occurs
long lastReconnectAttempt = 0;

// Global Variable 
bool      g_LED_request = false; // LED request flag set by the WiFi callback

uint16_t  g_count = 0;      // counter variable
uint16_t  g_LED_state = 0;  // LED state variable
/*-------------------------------------------------------------------------*
 * Globals: (I/O pin related)
 *-------------------------------------------------------------------------*/
/*  (LED's location)
  *   1. Adafruit Huzzah ESP32  (Red LED near the USB header)
  *   2. SparkFun ESP32 Thing (Blue LED at the middle of the board) 
  *   3. GPIO16 (the pin with the "16" marker)
  */
// set pin for LED
const int LED_PIN = LED_BUILTIN;

/*-------------------------------------------------------------------------*
 * Globals: (WiFi Secruity Credential related)
 *-------------------------------------------------------------------------*/
// wifi client with security
WiFiClientSecure wifiClient; 

// MQTT Connection info
char server[] = "mqtt.mediumone.com";
int port = 61620;



char pub_topic[]="0/<Project MQTT ID>/<User MQTT>/esp32/";
char sub_topic[]="1/<Project MQTT ID>/<User MQTT>/esp32/event";
char mqtt_username[]="<Project MQTT ID>/<User MQTT>";
char mqtt_password[]="<API Key>/<User Password>";

// Wifi credentials
char WIFI_SSID[] = "<WIFI_SSID>";
char WIFI_PASSWORD[] = "<WIFI_SSID>";



/*-------------------------------------------------------------------------*
 * Prototypes:
 *-------------------------------------------------------------------------*/
// Subroutine Prototypes
// Function Prototypes

// Store a new recieved message from the MQTT broker
void    callback(char* topic, byte* payload, unsigned int length);

// Check if the MQTT connection is still good.
boolean connectMQTT();

// Send a message with the sense data to the MQTT broker
void    sensor_loop();

// Send a regular message to keep MQTT connection if no sense data is to be sent.
void    heartbeat_loop(); 

/*-------------------------------------------------------------------------*
 * Class Varaible Globals: 
 *-------------------------------------------------------------------------*/

// Class Variable Declaratioin (Require callback and other variables to be already defined.)
PubSubClient client(server, port, callback, wifiClient);

/*-------------------------------------------------------------------------*
 * Main Program Start: (Setup)
 *-------------------------------------------------------------------------*/
void setup (){
  
  // init uart
  Serial.begin(112500);
  while(!Serial){}   // Wait for connection.

  // wifi setup
  WiFi.mode(WIFI_STA);  // STA mode
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  //not sure this is needed
  delay(5000);
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("Failed to connect, resetting"));
    //ESP.reset();  // not supported by esp32 WiFi library
  }

  // optinally 
  //while (WiFi.status() != WL_CONNECTED) {}
  
  // if you get here you have connected to the WiFi
  Serial.println(F("Connected to Wifi!"));

  Serial.println(F("Init hardware LED"));
  pinMode(LED_PIN, OUTPUT);
 
  // connect to MQTT broker to send board reset msg
  connectMQTT();

  Serial.println("Complete Setup");

  /* Create new event group */
  eventgroupflags = xEventGroupCreate();

  /* Create new tasks here */
  xTaskCreate(
      WiFi_M1_Task,            /* Task function. */
      "WiFi M1 Task",           /* name of task. */
      10000,                    /* Stack size of task */
      NULL,                     /* parameter of the task */
      1,                        /* priority of the task */
      NULL);                    /* Task handle to keep track of created task */

  xTaskCreate(
      LED_Task,                 /* Task function. */
      "LED Task",              /* name of task. */
      1000,                    /* Stack size of task */
      NULL,                     /* parameter of the task */
      1,                        /* priority of the task */
      NULL);                    /* Task handle to keep track of created task */      


}

/*-------------------------------------------------------------------------*
 * Main Program Start: (loop [task])
 *-------------------------------------------------------------------------*/

void loop() {
  
  // Do nothing right now
  delay(200);  // To keep the watchdog timer from triggered.
               // Prevent the loop task from monopolizing the cpu0 time.
}

/*-------------------------------------------------------------------------*
 * Main Program Start: (RTOS Tasks)
 *-------------------------------------------------------------------------*/

/* WiFi M1 Task: Check for the MQTT connection. Send messages to the MQTT broker. */
void WiFi_M1_Task( void * parameter )
{
  /* loop forever */
  for(;;){
    if (!client.connected()) {
      long now = millis();
      if (now - lastReconnectAttempt > 1000) {
        lastReconnectAttempt = now;
        // Attempt to reconnect
        if (connectMQTT()) {
          lastReconnectAttempt = 0;
        }
      }
    } else {
      // Client connected
      client.loop();
    }
    heartbeat_loop();
    process_request_loop();
  }
  /* delete a task when finish, 
  this will never happen because this is infinity loop */
  vTaskDelete( NULL );
}

void LED_Task( void * parameter ) {
  /* loop forever */
  for(;;){
    // Wait for LED request bit indefinitely.  
    EventBits_t xbit = xEventGroupWaitBits(eventgroupflags, LED_REUEST_EVENT_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    // Process message to turn LED on and off
    Serial.println(F("This is LED Task."));
    if (g_LED_state == 0) { 
     // Turn off LED
      digitalWrite(LED_PIN, LOW);   // for Adafruit HUZZAH ESP32
      Serial.println(F("LED request: Turn LED off "));
    } else { 
      // Turn on LED
      digitalWrite(LED_PIN, HIGH);  // for Adafruit HUZZAHESP32
      Serial.println(F("LED request: Turn LED on "));
    }      
    Serial.println("");
    delay(100);
  }
  /* delete a task when finish, 
  this will never happen because this is infinity loop */
  vTaskDelete( NULL );  
}

/*-------------------------------------------------------------------------*
 *  Subroutines' Body: 
 *-------------------------------------------------------------------------*/


/*-------------------------------------------------------------------------*
 *  A callback function used by Wifi Client to pass the recived message from
 *  the MQTT broker to the MQTT client.
 *
 *  For this example, only 1 value (0 or 1) is expected and is used to turn
 *  on or off the LED light.
 *
 *  input:  topic (MQTT topic string)
 *          payload (MQTT message payload string)
 *          length  (MQTT message payload string's length)
 *  Return : None
 *  
 *-------------------------------------------------------------------------*/ 
void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
  int i = 0;
  uint32_t temp = 0;
  char message_buff[length + 1];
  for(i=0; i < length; i++) {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';

  Serial.println(F("This is WiFi Task."));
  Serial.print(F("Received some data from the MQTT broker: "));
  Serial.println(String(message_buff));
  Serial.print(F("Received data len: "));
  Serial.println(strlen(message_buff));
  // Process message to turn LED on and off
  
  // L2:0 led on 
  // L2:1 led off
  if (message_buff[0] == 'L')
  {
    Serial.println(F("Request: Change RED Led state."));
    if (strlen(message_buff) == 4) 
    {
      if (String(message_buff[3]) == "0") { 
        // Turn off LED
        g_LED_request = true;
        g_LED_state = 0;
      } else if (String(message_buff[3]) == "1") { 
        // Turn on LED
        g_LED_request = true;
        g_LED_state = 1;
      }    
    }
  }
  else if (message_buff[0] == 'S') 
  {
    Serial.println(F("Request: Change the sampling rate."));
    sscanf(&message_buff[1],"%d", &temp);
    heartbeat_period = temp * 10;  // multiply by 10 for ms clock.
    Serial.print(F("New sampling interval time is "));
    Serial.print(heartbeat_period);
    Serial.println(F(" ms"));
  } 
  else 
  {
    Serial.println(F("Unknown request from the cloud app."));
  }  
  
}

/*-------------------------------------------------------------------------*
 *  Connect to the MQTT broker.   
 *
 *  input:  mqtt_user (a unique id name is required) -- global variable access
 *          mqtt_password (the username's password)-- global variable access
 *  Return : Connection status
 *  
 *-------------------------------------------------------------------------*/ 
boolean connectMQTT()
{    
  // Important Note: MQTT requires a unique id (UUID), we are using the mqtt_username as the unique ID
  // Besure to create a new device ID if you are deploying multiple devices.
  // Learn more about Medium One's self regisration option on docs.mediumone.com
  if (client.connect((char*) mqtt_username,(char*) mqtt_username, (char*) mqtt_password)) {
    Serial.println(F("Connected to MQTT broker"));

    // send a connect message
    //if (client.publish((char*) pub_topic, "{\"event_data\":{\"mqtt_connected\":true}}")) {
    // send a connect message with add_client_ip enable.  So, M1 cloud will pull the board IP address to find
    // the board's GPS location.
    if (client.publish((char*) pub_topic, "{\"event_data\":{\"connected\":true}, \"add_client_ip\":true}")) {
      Serial.println("Publish connected message ok");
    } else {
      Serial.print(F("Publish connected message failed: "));
      Serial.println(String(client.state()));
    }

    // subscrive to MQTT topic
    if (client.subscribe((char *)sub_topic,1)){
      Serial.println(F("Successfully subscribed"));
    } else {
      Serial.print(F("Subscribed failed: "));
      Serial.println(String(client.state()));
    }
  } else {
    Serial.println(F("MQTT connect failed"));
    Serial.println(F("Will reset and try again..."));
    abort();
  }
  return client.connected();
}


/*-------------------------------------------------------------------------*
 *  Send a dummy  message to the MQTT broker to keep the MQTT connection.   
 *
 *  input:  send a message at a rate determined by heartbeat_period.
 *  Return : None
 *  
 *-------------------------------------------------------------------------*/ 
void heartbeat_loop() {
  if ((millis()- heartbeat_timer) > heartbeat_period) {
    heartbeat_timer = millis();
    String payload = "{\"event_data\":{\"millis\":";
    payload += millis();
    payload += ",\"heartbeat\":true}}";
    
    if (client.loop()){
      Serial.println(F("This is WiFi Task."));
      Serial.print(F("Sending heartbeat: "));
      Serial.println(payload);
  
      if (client.publish((char *) pub_topic, (char*) payload.c_str()) ) {
        Serial.println(F("Publish ok"));
      } else {
        Serial.print(F("Failed to publish heartbeat: "));
        Serial.println(String(client.state()));
      }
    }
    delay(1000);  // To keep the watchdog timer from triggered.
                 // Prevent the loop task from monopolizing the cpu1 time.
  }
}

/*-------------------------------------------------------------------------*
 *  Set an event request bit for the task that has been waiting for it.   
 *
 *  input:  None
 *  Return : None
 *  
 *-------------------------------------------------------------------------*/ 
void process_request_loop() {
  if (g_LED_request) {
    g_LED_request = false;
    xEventGroupSetBits(eventgroupflags,LED_REUEST_EVENT_BIT);
  }
}