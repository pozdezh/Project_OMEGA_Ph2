#ifndef OMEGA_TASKS_H
#define OMEGA_TASKS_H

#include <Arduino.h>

extern TaskHandle_t netTaskHandle;

void tasksInit();
void tasksMarkBootComplete();
void NetworkTask(void* pvParameters);
void SentryTask(void* pvParameters);

#endif  // OMEGA_TASKS_H
