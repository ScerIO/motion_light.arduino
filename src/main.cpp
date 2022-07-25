#include "main.h"
#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>

#define DEBUG true

QueueHandle_t queueLightDetected;
QueueHandle_t queueMotionDetected;

void setup() {
  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }

  queueLightDetected = xQueueCreate(1, sizeof(bool));
  queueMotionDetected = xQueueCreate(1, sizeof(bool));

  if (queueLightDetected == NULL || queueMotionDetected == NULL) {
    Serial.println("Queue can not be created");
  }

  xTaskCreate(lightDetectionTask, "lightDetectionTask", 256, NULL, 1, NULL);
  xTaskCreate(motionDetectionTask, "motionDetectionTask", 128, NULL, 2, NULL);
  xTaskCreate(lightControlTask, "lightControlTask", 128, NULL, 3, NULL);

  vTaskStartScheduler();
}

void loop() {}

// Detect light value by photoresistor
void lightDetectionTask(void *pvParameters) {
  const int PIN_PHOTO_SENSOR = A1;

  bool lightIsDetected;

  for (;;) {
    int photoSensorValue = analogRead(PIN_PHOTO_SENSOR);
    if (DEBUG) {
      Serial.println("LightDetectionValue: " + String(photoSensorValue));
    }
    lightIsDetected = photoSensorValue < 700;
    xQueueOverwrite(queueLightDetected, &lightIsDetected);
    int delay = lightIsDetected ? 1000 : 500;
    vTaskDelay(delay / portTICK_PERIOD_MS);
  }
}

// Detect motion by HC-SR501 module
void motionDetectionTask(void *pvParameters) {
  const int MOTION_SENSOR_PIN = 7;
  pinMode(MOTION_SENSOR_PIN, INPUT);

  int motionStatePrevious, motionStateCurrent = LOW;
  bool motionIsDetected;

  for (;;) {
    motionStatePrevious = motionStateCurrent;
    motionStateCurrent = digitalRead(MOTION_SENSOR_PIN);
    if (DEBUG) {
      String powerStatus = (motionStateCurrent == HIGH) ? "DETECTED" : "Clear";
      Serial.println("MotionDetectionPower: " + powerStatus);
    }

    // pin state change: LOW -> HIGH
    if (motionStatePrevious == LOW && motionStateCurrent == HIGH) {
      motionIsDetected = true;
      xQueueOverwrite(queueMotionDetected, &motionIsDetected);
      if (DEBUG) {
        Serial.println("Motion detected!");
      }
    }
    // pin state change: HIGH -> LOW
    else if (motionStatePrevious == HIGH && motionStateCurrent == LOW) {
      motionIsDetected = false;
      xQueueOverwrite(queueMotionDetected, &motionIsDetected);
      if (DEBUG) {
        Serial.println("Motion stopped!");
      }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// Light control
void lightControlTask(void *pvParameters) {
  const int LED_PIN = 3;
  pinMode(LED_PIN, OUTPUT);

  bool lightIsDetected = true, motionIsDetected = false;

  for (;;) {
    xQueueReceive(queueLightDetected, &lightIsDetected, portMAX_DELAY);
    xQueueReceive(queueMotionDetected, &motionIsDetected, portMAX_DELAY);
    if (!lightIsDetected && motionIsDetected) {
      digitalWrite(LED_PIN, HIGH);
      vTaskDelay(2000 / portTICK_PERIOD_MS);
    } else {
      digitalWrite(LED_PIN, LOW);
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
  }
}
