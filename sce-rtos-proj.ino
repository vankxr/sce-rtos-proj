#include "Arduino.h"
#include <stdio.h>
#include <stdarg.h>
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

// IOs
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

// Constants
#define UI_CMD_LEFT     0
#define UI_CMD_RIGHT    1
#define UI_CMD_CENTER   2

// UI Colors
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
uint16_t        Display_Selected_Color  = Display_Color_Yellow;

// Structs
typedef struct
{
    float fAzimuth;
    float fElevation;
} point_direction_t;

volatile unsigned long ulIdleCycleCount = 0UL;
SemaphoreHandle_t xI2CMutex;
SemaphoreHandle_t xIMUIRQCounter;
QueueHandle_t xUICommandQueue;
QueueHandle_t xSerialPortQueue;
QueueHandle_t xTargetQueue;
QueueHandle_t xIMUDataQueue;
QueueHandle_t xServoQueue;

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

        tft.fillRect(0, 15, 128, 15, Display_Selected_Color);
        tft.setCursor(0, 15);
        tft.setTextColor(Display_Backround_Color);
        tft.print(oldTimeString);
        
        tft.setCursor(0, 15);
        tft.setTextColor(Display_Text_Color);
        tft.print(newTimeString);
        
        strcpy(oldTimeString, newTimeString);
    }
}

void vSafePrintf(TickType_t xTicksToWait, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    int sz = vsnprintf(NULL, 0, fmt, args);
    char *c = (char *)malloc(sz + 1);

    if(!c)
    {
        va_end(args);

        return;
    }

    vsnprintf(c, sz + 1, fmt, args);

    if(xQueueSendToBack(xSerialPortQueue, &c, xTicksToWait) != pdPASS)
        free(c);

    va_end(args);
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
    xI2CMutex = xSemaphoreCreateMutex();
    xIMUIRQCounter = xSemaphoreCreateCounting(10, 0);
    
    xUICommandQueue = xQueueCreate(32, sizeof(uint8_t));
    xSerialPortQueue = xQueueCreate(32, sizeof(char *));
    xTargetQueue = xQueueCreate(1, sizeof(point_direction_t));
    xIMUDataQueue = xQueueCreate(1, sizeof(point_direction_t));
    xServoQueue = xQueueCreate(1, sizeof(point_direction_t));

    xTaskCreatePinnedToCore(vDisplayTask, "Display Task", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(vServoTask, "Servo Task", 2048, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(vIMUTask, "IMU Task", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(vBTSerialTask, "Bluetooth Task", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(vSerialTask, "Serial Task", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(vADCTask, "ADC Task", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(vProcessingTask, "Processing Task", 2048, NULL, 3, NULL, 1);
}

void vIMUISR()
{
    static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(xIMUIRQCounter, (BaseType_t *)&xHigherPriorityTaskWoken);

    if(xHigherPriorityTaskWoken == pdTRUE)
        portYIELD_FROM_ISR();
}

void vLeftIRQ()
{
    static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    static int lastPress = 0;
    int now = millis();

    if(now - lastPress > 500)
    {
        uint8_t cmd = UI_CMD_LEFT;
        xQueueSendToBackFromISR(xUICommandQueue, (void *)&cmd, (BaseType_t *)&xHigherPriorityTaskWoken);

        lastPress = now;
    }

    if(xHigherPriorityTaskWoken == pdTRUE)
        portYIELD_FROM_ISR();
}

void vRightIRQ()
{
    static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    static int lastPress = 0;
    int now = millis();

    if(now - lastPress > 500)
    {
        uint8_t cmd = UI_CMD_RIGHT;
        xQueueSendToBackFromISR(xUICommandQueue, (void *)&cmd, (BaseType_t *)&xHigherPriorityTaskWoken);

        lastPress = now;
    }

    if(xHigherPriorityTaskWoken == pdTRUE)
        portYIELD_FROM_ISR();
}

void vCenterIRQ()
{
    static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    static int lastPress = 0;
    int now = millis();

    if(now - lastPress > 500)
    {
        uint8_t cmd = UI_CMD_CENTER;
        xQueueSendToBackFromISR(xUICommandQueue, (void *)&cmd, (BaseType_t *)&xHigherPriorityTaskWoken);

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

        uint8_t cmd;

        if(xQueueReceive(xUICommandQueue, &cmd, 0) == pdPASS)
        {
            switch(cmd)
            {
                case UI_CMD_LEFT:
                    vSafePrintf(portMAX_DELAY, "Display CMD LEFT\r\n");
                break;
                case UI_CMD_RIGHT:
                    vSafePrintf(portMAX_DELAY, "Display CMD RIGHT\r\n");
                break;
                case UI_CMD_CENTER:
                    vSafePrintf(portMAX_DELAY, "Display CMD CENTER\r\n");
                break;
                default:
                    vSafePrintf(portMAX_DELAY, "Display CMD UNKNOWN\r\n");
                break;
            }
        }

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
        point_direction_t xServoPosition;

        if(xQueueReceive(xServoQueue, &xServoPosition, portMAX_DELAY) != pdPASS)
            continue;

        static int lastUpdate = 0;
        int now = millis();

        if(now - lastUpdate > 2000)
        {
            vSafePrintf(portMAX_DELAY, "Servos: %.2f, %.2f\r\n", xServoPosition.fAzimuth, xServoPosition.fElevation);

            lastUpdate = now;
        }

        azimuth_servo.write(xServoPosition.fAzimuth);
        elevation_servo.write(xServoPosition.fElevation);

        /*
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
        */
    }
}

void vIMUTask(void *pvParam)
{
    Wire.begin();

    MPU9250 mpu;

    while(!mpu.setup(0x68))
    {
        vSafePrintf(portMAX_DELAY, "MPU connection failed\r\n");
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
            static int lastUpdate = 0;
            int now = millis();

            if(now - lastUpdate > 2000)
            {
                vSafePrintf(portMAX_DELAY, "Yaw, Pitch, Roll: %.2f, %.2f, %.2f\r\n", mpu.getYaw(), mpu.getPitch(), mpu.getRoll());

                lastUpdate = now;
            }

            point_direction_t xDirection;

            xDirection.fAzimuth = mpu.getYaw();
            xDirection.fElevation = mpu.getPitch();

            xQueueOverwrite(xIMUDataQueue, &xDirection);
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
            int bufp = 0;
            char buf[64];
            point_direction_t xTargetPosition;

            while(BTSerial.available())
            {
                buf[bufp++] = BTSerial.read();

                if(buf[bufp - 1] == '\n')
                {
                    if(sscanf(buf, "%f, %f", &xTargetPosition.fAzimuth, &xTargetPosition.fElevation) == 2)
                    {
                        vSafePrintf(portMAX_DELAY, "Target position: %f, %f\r\n", xTargetPosition.fAzimuth, xTargetPosition.fElevation);

                        xQueueOverwrite(xTargetQueue, &xTargetPosition);
                    }

                    bufp = 0;
                }

                if(bufp >= 64)
                    bufp = 0;
            }
        }

        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void vSerialTask(void *pvParam)
{
    Serial.begin(115200);

    while(!Serial)
        vTaskDelay(1 / portTICK_PERIOD_MS);

    while(1)
    {
        char *pcString;

        xQueueReceive(xSerialPortQueue, &pcString, portMAX_DELAY);
        Serial.print(pcString);
        Serial.flush();

        free(pcString);
    }
}

void vADCTask(void *pvParam)
{
    while(1)
    {
        int val = analogRead(BAT_ADC_PIN);

        vSafePrintf(portMAX_DELAY, "ADC val: %d\r\n", val);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void vProcessingTask(void *pvParam)
{
    while(1)
    {
        point_direction_t xTargetPosition;
        point_direction_t xCurrentPosition;
        point_direction_t xServoPosition;

        if(xQueuePeek(xTargetQueue, &xTargetPosition, portMAX_DELAY) != pdPASS)
            continue;

        if(xQueuePeek(xIMUDataQueue, &xCurrentPosition, portMAX_DELAY) != pdPASS)
            continue;

        xServoPosition.fAzimuth = xTargetPosition.fAzimuth - xCurrentPosition.fAzimuth;
        xServoPosition.fElevation = xTargetPosition.fElevation - xCurrentPosition.fElevation;
        
        xQueueOverwrite(xServoQueue, &xServoPosition);
    }
}

void vStatsPrintTask(void *pvParam)
{
    char buf[512] = {0};

    //vTaskList(buf);
    vSafePrintf(portMAX_DELAY, "**********************************\r\nIdle counter: %lu\r\nTask  State   Prio    Stack    Num\r\n**********************************\r\n%s**********************************\r\n", ulIdleCycleCount, buf);

    vTaskDelete(NULL);
}

bool my_vApplicationIdleHook(void)
{
    ulIdleCycleCount++;

    return true;
}

void loop()
{
    static int lastStatsPrint = 0;
    int now = millis();

    if(now - lastStatsPrint > 60000)
    {
        xTaskCreatePinnedToCore(vStatsPrintTask, "Stats Task", 4096, NULL, 1, NULL, 1);

        lastStatsPrint = now;
    }
}
