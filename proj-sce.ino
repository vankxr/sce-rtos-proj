#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include "BluetoothSerial.h"
#include "MPU9250.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <ESP32Servo.h>

#define TFT_CS    2
#define TFT_RST   33
#define TFT_DC    4
#define TFT_MOSI  23 
#define TFT_SCLK  18
#define TFT_LITE  32

#define BAT_ADC_PIN 36

#define SERVO_AZ_PIN  19
#define SERVO_EL_PIN  5

#define IMU_IRQ_PIN   16

#define BTN_LEFT_PIN   13
#define BTN_RIGHT_PIN  14
#define BTN_CENTER_PIN 15


const uint16_t  Display_Color_Black   = 0x0000;
const uint16_t  Display_Color_Blue    = 0x001F;
const uint16_t  Display_Color_Red     = 0xF800;
const uint16_t  Display_Color_Green   = 0x07E0;
const uint16_t  Display_Color_Cyan    = 0x07FF;
const uint16_t  Display_Color_Magenta = 0xF81F;
const uint16_t  Display_Color_Yellow  = 0xFFE0;
const uint16_t  Display_Color_White   = 0xFFFF;

uint16_t        Display_Text_Color      = Display_Color_Black;
uint16_t        Display_Backround_Color = Display_Color_Blue;

SemaphoreHandle_t xI2CMutex;
SemaphoreHandle_t xIMUIRQCounter;

bool BTConnected = false;

void vDisplayTask(void *);
void vServoTask(void *);
void vIMUTask(void *);
void vBTSerialTask(void *);
void IRAM_ATTR vIMUISR();
void IRAM_ATTR vLeftIRQ();
void IRAM_ATTR vRightIRQ();
void IRAM_ATTR vCenterIRQ();

void displayUpTime(Adafruit_ST7735 &tft)
{
    unsigned long upSeconds = millis() / 1000;
    unsigned long days = upSeconds / 86400;
    upSeconds = upSeconds % 86400;
    unsigned long hours = upSeconds / 3600;
    upSeconds = upSeconds % 3600;
    unsigned long minutes = upSeconds / 60;
    upSeconds = upSeconds % 60;

    static char oldTimeString[16];
    char newTimeString[16];
    
    snprintf(
        newTimeString,
        16,
        "%lu %02lu:%02lu:%02lu",
        days, hours, minutes, upSeconds
    );

    if(strcmp(newTimeString, oldTimeString) != 0)
    {
        tft.setCursor(0, 0);
        tft.setTextColor(Display_Backround_Color);
        tft.print(oldTimeString);
        
        tft.setCursor(0, 0);
        tft.setTextColor(Display_Text_Color);
        tft.print(newTimeString);

        tft.fillRect(0, 15, 128, 15, Display_Color_Yellow);
        tft.setCursor(0, 15);
        tft.setTextColor(Display_Backround_Color);
        tft.print(oldTimeString);
        
        tft.setCursor(0, 15);
        tft.setTextColor(Display_Text_Color);
        tft.print(newTimeString);
        
        strcpy(oldTimeString, newTimeString);
    }
}

void printRPY(MPU9250 &mpu)
{
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 2);
    /*
    Serial.print("Acc: ");
    Serial.print(mpu.getAcc(0), 2);
    Serial.print(", ");
    Serial.print(mpu.getAcc(1), 2);
    Serial.print(", ");
    Serial.println(mpu.getAcc(2), 2);
    Serial.print("Gyro: ");
    Serial.print(mpu.getGyro(0), 2);
    Serial.print(", ");
    Serial.print(mpu.getGyro(1), 2);
    Serial.print(", ");
    Serial.println(mpu.getGyro(2), 2);
    Serial.print("Mag: ");
    Serial.print(mpu.getMag(0), 2);
    Serial.print(", ");
    Serial.print(mpu.getMag(1), 2);
    Serial.print(", ");
    Serial.println(mpu.getMag(2), 2);
    */
}

void printIMUCalib(MPU9250 &mpu)
{
    Serial.println("< calibration parameters >");
    Serial.print("mpu.setAccBias(");
    Serial.print(mpu.getAccBiasX());
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY());
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ());
    Serial.println(");");
    Serial.print("mpu.setGyroBias(");
    Serial.print(mpu.getGyroBiasX());
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY());
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ());
    Serial.println(");");
    Serial.print("mpu.setMagBias(");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println(");");
    Serial.print("mpu.setMagScale(");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println(");");
}

void BTSerialCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
  if(event == ESP_SPP_SRV_OPEN_EVT)
    BTConnected = true;

  if(event == ESP_SPP_CLOSE_EVT)
    BTConnected = false;
}

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  
  xI2CMutex = xSemaphoreCreateMutex();
  xIMUIRQCounter = xSemaphoreCreateCounting(10, 0);
  
  xTaskCreatePinnedToCore(vDisplayTask, "Display Task", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(vServoTask, "Servo Task", 1024, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(vIMUTask, "IMU Task", 2048, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(vBTSerialTask, "Bluetooth Task", 2048, NULL, 1, NULL, 0);
}

void vIMUISR()
{
  static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  
  xSemaphoreGiveFromISR(xIMUIRQCounter, (BaseType_t*)&xHigherPriorityTaskWoken);

  if(xHigherPriorityTaskWoken == pdTRUE)
     portYIELD_FROM_ISR();
}

void vLeftIRQ()
{
  static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  
  //xSemaphoreGiveFromISR(xIMUIRQCounter, (BaseType_t*)&xHigherPriorityTaskWoken);

  static int lastPress = millis();
  int now = millis();

  if(now - lastPress > 500)
  {
    Serial.println("LEFT");

    lastPress = now;
  }

  if(xHigherPriorityTaskWoken == pdTRUE)
     portYIELD_FROM_ISR();
}

void vRightIRQ()
{
  static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  
  //xSemaphoreGiveFromISR(xIMUIRQCounter, (BaseType_t*)&xHigherPriorityTaskWoken);

  static int lastPress = millis();
  int now = millis();

  if(now - lastPress > 500)
  {
    Serial.println("RIGHT");

    lastPress = now;
  }

  if(xHigherPriorityTaskWoken == pdTRUE)
     portYIELD_FROM_ISR();
}

void vCenterIRQ()
{
  static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  
  //xSemaphoreGiveFromISR(xIMUIRQCounter, (BaseType_t*)&xHigherPriorityTaskWoken);

  static int lastPress = millis();
  int now = millis();

  if(now - lastPress > 500)
  {
    Serial.println("CENTER");

    lastPress = now;
  }

  if(xHigherPriorityTaskWoken == pdTRUE)
     portYIELD_FROM_ISR();
}

void vDisplayTask(void *pvParam)
{
  Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

  pinMode(TFT_LITE, OUTPUT);

  tft.initR(INITR_BLACKTAB);
  tft.setFont();
  tft.fillScreen(Display_Backround_Color);
  tft.setTextColor(Display_Text_Color);
  tft.setTextSize(2);
  
  digitalWrite(TFT_LITE, HIGH);

  pinMode(BTN_LEFT_PIN, INPUT_PULLUP);
  pinMode(BTN_RIGHT_PIN, INPUT_PULLUP);
  pinMode(BTN_CENTER_PIN, INPUT_PULLUP);
  attachInterrupt(BTN_LEFT_PIN, vLeftIRQ, FALLING);
  attachInterrupt(BTN_RIGHT_PIN, vRightIRQ, FALLING);
  attachInterrupt(BTN_CENTER_PIN, vCenterIRQ, FALLING);

  while(1)
  {
    displayUpTime(tft);

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void vServoTask(void *pvParam)
{
  Servo azimuth_servo;
  Servo elevation_servo;
  
  azimuth_servo.setPeriodHertz(50);
  elevation_servo.setPeriodHertz(50);
  azimuth_servo.attach(SERVO_AZ_PIN, 500, 2400);
  elevation_servo.attach(SERVO_EL_PIN, 500, 2400);

  while(1)
  {
    for (int pos = 0; pos <= 180; pos += 1)
    {
      azimuth_servo.write(pos);
      elevation_servo.write(pos);
      vTaskDelay(15 / portTICK_PERIOD_MS);
    }
    
    for (int pos = 180; pos >= 0; pos -= 1)
    {
      azimuth_servo.write(pos);
      elevation_servo.write(pos);
      vTaskDelay(15 / portTICK_PERIOD_MS);
    }
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void vIMUTask(void *pvParam)
{
  MPU9250 mpu;

  while(!mpu.setup(0x68))
  {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
  
  //Serial.println("Accel Gyro calibration will start in 5sec.");
  //Serial.println("Please leave the device still on the flat plane.");
  //vTaskDelay(5000 / portTICK_PERIOD_MS);
  //mpu.calibrateAccelGyro();

  //Serial.println("Mag calibration will start in 5sec.");
  //Serial.println("Please Wave device in a figure eight until done.");
  //vTaskDelay(5000 / portTICK_PERIOD_MS);
  //mpu.calibrateMag();

  //printIMUCalib(mpu);
  mpu.setAccBias(-320.92, -4.72, 806.15);
  mpu.setGyroBias(-202.33, 64.51, 219.67);
  mpu.setMagBias(-1996.70, 2202.25, -1419.39);
  mpu.setMagScale(0.83, 1.00, 1.27);

  attachInterrupt(IMU_IRQ_PIN, vIMUISR, RISING);

  xSemaphoreTake(xI2CMutex, portMAX_DELAY);
  mpu.available(); // Clear the interrupt
  xSemaphoreGive(xI2CMutex);

  while(1)
  {
    xSemaphoreTake(xIMUIRQCounter, portMAX_DELAY);

    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    bool updated = mpu.update();
    xSemaphoreGive(xI2CMutex);
    
    if(updated)
    {
      //printRPY(mpu);
    }
  }
}

void vBTSerialTask(void *pvParam)
{
  BluetoothSerial BTSerial;
  
  BTSerial.begin("ESP32AntennaServo");
  BTSerial.register_callback(BTSerialCallback);

  vTaskDelay(20 / portTICK_PERIOD_MS);
  
  while(1)
  {
    if(BTConnected)
    {
      while(BTSerial.available())
        Serial.write(BTSerial.read());
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void loop()
{
  vTaskDelete(NULL);
}
