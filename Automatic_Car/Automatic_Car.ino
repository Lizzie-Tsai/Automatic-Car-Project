/**********************************************************************
  Filename    : Automatic_Car
  Auther      : Lizzie
  Modification: 2024/11/12
**********************************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include "esp_camera.h"
#include "WiFi.h"
#include "Emotion.h"
#include "WS2812.h"
#include "ESP32.h"
#include "BluetoothSerial.h"
//#include "event_groups.h"
#define SPEED_LV4       (4000)
#define SPEED_LV3       (3000)
#define SPEED_LV2       (2500)
#define SPEED_LV1       (1500)
#define OBSTACLE_DISTANCE      35
#define OBSTACLE_DISTANCE_LOW  10

#define E1_BIT   (1UL << 0UL) // 0 shift for bit 1        //AUTO
#define E2_BIT   (1UL << 1UL) // 1 shift for bit 1        //TRACK
#define E3_BIT   (1UL << 2UL) // 2 shift for bit 1        //Obstacle Stop
#define E4_BIT   (1UL << 3UL) // 3 shift for bit 1        //No Track Stop
#define E5_BIT   (1UL << 4UL) // 4 shift for bit 1        //Bluetooth Stop
#define E6_BIT   (1UL << 5UL) // 5 shift for bit 1        //Override Stop
#define TB_BIT   (1UL << 6UL) // 6 shift for bit 1        //Begin tracking
#define STOP_BIT  (E3_BIT|E4_BIT|E5_BIT|E6_BIT) 
#define ALL_BIT   (E1_BIT|E2_BIT|E3_BIT|E4_BIT|E5_BIT|E6_BIT|TB_BIT) 

#define STATE_O           "STATE_O"         // Obstacle avoidance + Tracking
#define STATE_T           "STATE_T"         // Tracking
#define STATE_OS          "STATE_OS"        // Obstacle stop
#define STATE_TS          "STATE_TS"        // No track stop
#define STATE_BS          "STATE_BS"        // Bluetooth stop
#define STATE_RS          "STATE_RS"        // Override stop
#define STATE_NONE        "STATE_NONE"      // State none

//  declare a event grounp handler variable 
EventGroupHandle_t  xEventGroup;
static SemaphoreHandle_t mutex;
static SemaphoreHandle_t binary_sem;

TaskHandle_t obst_Avoid_Handle = NULL;
TaskHandle_t line_Track_Handle = NULL;

String CmdArray[8];
int paramters[8];
bool videoFlag = 0;

void WiFi_Init() {
  ssid_Router     =   "Galaxy A33 5G5974";    //Modify according to your router name
  password_Router =   "llhj1918";             //Modify according to your router password
  ssid_AP         =   "Sunshine";             //ESP32 turns on an AP and calls it Sunshine
  password_AP     =   "Sunshine";             //Set your AP password for ESP32 to Sunshine
  frame_size      =    FRAMESIZE_CIF;         //400*296
}
WiFiServer server_Cmd(4000);
WiFiServer server_Camera(7000);

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;
const char turnON ='a';
const char turnOFF ='b';
const char restart ='c';
const char stop = 'd';
char message;

void Bluetooth_Setup(){
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("Bluetooth Link Established!");
}

void setup() {
  Buzzer_Setup();               //Buzzer initialization
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  WiFi_Init();                 //WiFi paramters initialization
  WiFi_Setup(0);               //Start AP Mode. If you want to connect to a router, change 1 to 0.
  server_Cmd.begin(4000);      //Start the command server
  // server_Camera.begin(7000);//Turn on the camera server
  Bluetooth_Setup();

  // cameraSetup();         //Camera initialization
  Emotion_Setup();          //Emotion initialization
  WS2812_Setup();           //WS2812 initialization
  PCA9685_Setup();          //PCA9685 initialization
  Light_Setup();            //Light initialization
  Track_Setup();            //Track initialization
  Ultrasonic_Setup();       //Initialize the ultrasonic module

  //create event group and assign it a earlier created referene handler
  xEventGroup  =  xEventGroupCreate();
  mutex = xSemaphoreCreateMutex();
  binary_sem = xSemaphoreCreateBinary();


  //xTaskCreateUniversal(loopTask_Camera, "loopTask_Camera", 8192, NULL, 0, NULL, 0);
  xTaskCreateUniversal(loopTask_WTD, "loopTask_WTD", 8192, NULL, 0, NULL, 0);
  xTaskCreate(                    // Use xTaskCreate() in  FreeRTOS
              start_Obst_Avoid,   // Function to be called
              "start_Obst_Avoid", // Name of task
              2000,               // Stack size (bytes in ESP32, words in FreeRTOS)
              NULL,               // Parameter to pass to function
              3,                  // Task priority (0 to configMAX_PRIORITIES - 1)
              &obst_Avoid_Handle   // Task handle
  );     
  xTaskCreate(                    // Use xTaskCreate() in  FreeRTOS
              start_Line_Track,   // Function to be called
              "start_Line_Track", // Name of task
              2000,               // Stack size (bytes in ESP32, words in FreeRTOS)
              NULL,               // Parameter to pass to function
              2,                  // Task priority (0 to configMAX_PRIORITIES - 1)
              &line_Track_Handle                // Task handle        
  );     


}

void loop() {
  WiFiClient client = server_Cmd.accept();                    //listen for incoming clients
  if (client) {                                               //if you get a client
    Serial.println("Cmd_Server connected to a client.");
    while (client.connected()) {                              //loop while the client's connected    
      // WiFi Cmd                       
      if (client.available()) {                               //if there's bytes to read from the client
        String inputStringTemp = client.readStringUntil('\n');//Read the command by WiFi
        Serial.println(inputStringTemp);                      //Print out the command received by WiFi
        Get_Command(inputStringTemp);

        if (CmdArray[0] == CMD_LED_MOD)//Set the display mode of car colored lights
          WS2812_SetMode(paramters[1]);
        if (CmdArray[0] == CMD_LED) //Set the color and brightness of the car lights
          WS2812_Set_Color_1(paramters[1], paramters[2], paramters[3], paramters[4]);
        if (CmdArray[0] == CMD_MATRIX_MOD)//Set the display mode of the LED matrix
          Emotion_SetMode(paramters[1]);
        if (CmdArray[0] == CMD_VIDEO)//Video transmission command
          videoFlag = paramters[1];
        if (CmdArray[0] == CMD_BUZZER) //Buzzer control command
          Buzzer_Variable(paramters[1], paramters[2]);
        if (CmdArray[0] == CMD_POWER) {//Power and Light query command
          float battery_voltage = Get_Battery_Voltage();
          float light_value = Get_Photosensitive();
          EventBits_t xEventGroupValue = xEventGroupGetBits(xEventGroup);
          client.print(CMD_POWER);
          client.print(INTERVAL_CHAR);
          client.print(battery_voltage);
          client.print(INTERVAL_CHAR);
          client.print(light_value);
          // State update
          if((xEventGroupValue & (E1_BIT)) != 0){
            client.print(INTERVAL_CHAR);
            client.print(STATE_O);
          }else if((xEventGroupValue & (E2_BIT)) != 0){
            client.print(INTERVAL_CHAR);
            client.print(STATE_T);
          }else if((xEventGroupValue & (E3_BIT)) != 0){
            client.print(INTERVAL_CHAR);
            client.print(STATE_OS);
          }else if((xEventGroupValue & (E4_BIT)) != 0){
            client.print(INTERVAL_CHAR);
            client.print(STATE_TS);
          }else if((xEventGroupValue & (E5_BIT)) != 0){
            client.print(INTERVAL_CHAR);
            client.print(STATE_BS);
          }else if((xEventGroupValue & (E6_BIT)) != 0){
            client.print(INTERVAL_CHAR);
            client.print(STATE_RS);
          }else{
            client.print(INTERVAL_CHAR);
            client.print(STATE_NONE);
          }
          client.print(ENTER);
        }
        if (CmdArray[0] == CMD_MOTOR) {//Network control car movement command
          Car_SetMode(0);
          if (paramters[1] == 0 && paramters[3] == 0)
            Motor_Move(0, 0, 0, 0);//Stop the car
          else //If the parameters are not equal to 0
            Motor_Move(paramters[1], paramters[1], paramters[3], paramters[3]);
        }
        if (CmdArray[0] == CMD_SERVO) {//Network control servo motor movement command
          if (paramters[1] == 0)
            Servo_1_Angle(paramters[2]);
          else if (paramters[1] == 1)
            Servo_2_Angle(paramters[2]);
        }
        if (CmdArray[0] == CMD_CAMERA) {//Network control servo motor movement command
          Servo_1_Angle(paramters[1]);
          Servo_2_Angle(paramters[2]);
        }
        if (CmdArray[0] == CMD_LIGHT) { //Automatic car command
          if (paramters[1] == 1){
            Car_SetMode(0);  //1
            xEventGroupClearBits(xEventGroup, ALL_BIT);    // Clear all bit
            xEventGroupSetBits(xEventGroup, E1_BIT);    // Set E1 bit
          }else if (paramters[1] == 0){
            Car_SetMode(0);
            xEventGroupClearBits(xEventGroup, ALL_BIT);  // Clear all bit
            xEventGroupSetBits(xEventGroup, E6_BIT);   // Set E6 bit
          }  
        }
        else if (CmdArray[0] == CMD_TRACK) { //Tracking car command
          if (paramters[1] == 1){
            Car_SetMode(0); //2
            xEventGroupClearBits(xEventGroup, ALL_BIT);    // Clear all bit
            xEventGroupSetBits(xEventGroup, (E2_BIT|TB_BIT));    // Set E2/TB bit
          }else if (paramters[1] == 0){
            Car_SetMode(0);
            xEventGroupClearBits(xEventGroup, ALL_BIT);  // Clear all bit
            xEventGroupSetBits(xEventGroup, E6_BIT);   // Set E6 bit
          }  
        }
        if (CmdArray[0] == CMD_CAR_MODE) { //Car command Mode
          Car_SetMode(paramters[1]);
        }
        if (CmdArray[0] == CMD_OVERRIDE) { //Override Mode
          xEventGroupClearBits(xEventGroup, ALL_BIT);    // Clear all bit
          xEventGroupSetBits(xEventGroup, E6_BIT);   // Set E6 bit
        }
        //Clears the command array and parameter array
        memset(CmdArray, 0, sizeof(CmdArray));
        memset(paramters, 0, sizeof(paramters));
      }
      Emotion_Show(emotion_task_mode);//Led matrix display function
      WS2812_Show(ws2812_task_mode);//Car color lights display function
      Car_Select(carFlag);//ESP32 Car mode selection function

      // Bluetooth Cmd
      if (SerialBT.available()){
        message=SerialBT.read();
        Serial.write(message);
        
        if(message==turnON){
          WS2812_Show(4);     //Car color lights display function
          Serial.println(F(" :LED ON"));
          SerialBT.println(F("LED ON"));   
        }
        else if(message==turnOFF){
          WS2812_Show(0);        //Turn LED Off
          Serial.println(F(" :LED OFF"));
          SerialBT.println(F("LED OFF"));   
        }
        else if(message==restart){
          xEventGroupClearBits(xEventGroup, ALL_BIT);    // Clear all bit
          xEventGroupSetBits(xEventGroup, E1_BIT);    // Set E1 bit
          Serial.println(F(" :RESTART"));
          SerialBT.println(F("RESTART"));   
        }
        else if(message==stop){
          xEventGroupClearBits(xEventGroup, ALL_BIT);    // Clear all bit
          xEventGroupSetBits(xEventGroup, E5_BIT);    // Set E5 bit
          Serial.println(F(" :STOP"));
          SerialBT.println(F("STOP"));   
        }
        else{
          WS2812_Show(5);
          Serial.println(F(" :Invalid Input")); 
          SerialBT.println(F("Invalid Input"));
        } 
      }
    }
    client.stop();//close the connection:
    Serial.println("Command Client Disconnected.");
    ESP.restart();
  }
}

void start_Line_Track(void *pvParameters){
  Serial.println("start_Line_Track Created");
  // define a variable which holds the state of events 
  const EventBits_t xBitsToWaitFor  = (E1_BIT|E2_BIT);
  EventBits_t xEventGroupValue;
  while(1){
    if (xSemaphoreTake(mutex, 1/portTICK_PERIOD_MS) == pdTRUE){
      xEventGroupValue  = xEventGroupWaitBits(xEventGroup,
                                              xBitsToWaitFor,
                                              pdFALSE,
                                              pdFALSE,
                                              portMAX_DELAY
                                              );
      if((xEventGroupValue & (TB_BIT)) != 0){
        Serial.println("Enter start_Line_Track");
        Track_Read();
        switch (sensorValue[3])
        {
          case 2:   //010
          case 5:   //101
            Emotion_SetMode(3);
            Motor_Move(SPEED_LV1, SPEED_LV1, SPEED_LV1, SPEED_LV1);    //Move Forward
            break;
          case 0:   //000
          case 7:   //111
            Emotion_SetMode(6);
            vTaskDelay(100);
            Motor_Move(0, 0, 0, 0);                                    //Stop
            xEventGroupClearBits(xEventGroup, ALL_BIT);  // Clear all bit
            xEventGroupSetBits(xEventGroup, E4_BIT);  // Set E4 bit
            Buzzer_Alert(1, 1);
            Emotion_SetMode(0);
            Emotion_Show(emotion_task_mode);
            break;
          case 1:   //001
          case 3:   //011
            Emotion_SetMode(4);
            Motor_Move(-SPEED_LV3, -SPEED_LV3, SPEED_LV4, SPEED_LV4);  //Turn Left
            break;
          case 4:   //100
          case 6:   //110
            Emotion_SetMode(5);
            Motor_Move(SPEED_LV4, SPEED_LV4 , - SPEED_LV3, -SPEED_LV3);//Turn Right
            break;

          default:
            break;
        }
        Emotion_Show(emotion_task_mode);//Led matrix display function
      }
      xSemaphoreGive(mutex);
    }else{
        continue;
    }
  }
}



void start_Obst_Avoid(void *pvParameters){
  Serial.println("start_Obst_Avoid Created");
  // define a variable which holds the state of events 
  const EventBits_t xBitsToWaitFor  = (E1_BIT);
  EventBits_t xEventGroupValue;
  while(1){
    xEventGroupValue  = xEventGroupWaitBits(xEventGroup,
                                            xBitsToWaitFor,
                                            pdFALSE,
                                            pdTRUE,
                                            portMAX_DELAY
                                            );
    if((xEventGroupValue & E1_BIT) != 0){
      Serial.println("Enter start_Obst_Avoid");
      xEventGroupSetBits(xEventGroup, TB_BIT);  // Set TB bit  //2
      float distance = get_distance();
      if (distance <= OBSTACLE_DISTANCE){
        xSemaphoreTake(mutex, portMAX_DELAY);
        Motor_Move(0, 0, 0, 0);    //Stop the car to judge the situation
        Buzzer_Alert(2, 1);
        Emotion_SetMode(0);
        Emotion_Show(emotion_task_mode);
        Serial.println("STOP");
        xEventGroupClearBits(xEventGroup, ALL_BIT);  // Clear all bit
        xEventGroupSetBits(xEventGroup, E3_BIT);  // Set E3 bit
        xSemaphoreGive(mutex);
      }
      vTaskDelay(450);
    }
  }
}


void loopTask_Camera(void *pvParameters) {
  Serial.println("loopTask_Camera Created");
  while (1) {
    WiFiClient client = server_Camera.accept();//listen for incoming clients
    if (client) {//if you get a client
      Serial.println("Camera_Server connected to a client.");
      if (client.connected()) {
        camera_fb_t * fb = NULL;
        while (client.connected()) {//loop while the client's connected
          if (videoFlag == 1) {
            fb = esp_camera_fb_get();
            if (fb != NULL) {
              uint8_t slen[4];
              slen[0] = fb->len >> 0;
              slen[1] = fb->len >> 8;
              slen[2] = fb->len >> 16;
              slen[3] = fb->len >> 24;
              client.write(slen, 4);
              client.write(fb->buf, fb->len);
              Serial.println("Camera send");
              esp_camera_fb_return(fb);
            }
          }
        }
        //close the connection:
        client.stop();
        Serial.println("Camera Client Disconnected.");
        ESP.restart();
      }
    }
  }
}

void Get_Command(String inputStringTemp)
{
  int string_length = inputStringTemp.length();
  for (int i = 0; i < 8; i++) {//Parse the command received by WiFi
    int index = inputStringTemp.indexOf(INTERVAL_CHAR);
    if (index < 0) {
      if (string_length > 0) {
        CmdArray[i] = inputStringTemp;         //Get command
        paramters[i] = inputStringTemp.toInt();//Get parameters
      }
      break;
    }
    else {
      string_length -= index;                                //Count the remaining words
      CmdArray[i] = inputStringTemp.substring(0, index);     //Get command
      paramters[i] = CmdArray[i].toInt();                    //Get parameters
      inputStringTemp = inputStringTemp.substring(index + 1);//Update string
    }
  }
}












//
