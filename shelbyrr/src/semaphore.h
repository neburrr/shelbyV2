
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/semphr.h"

SemaphoreHandle_t endSemaphore;
void createSemaphore(){
    endSemaphore = xSemaphoreCreateMutex();
    xSemaphoreGive( (endSemaphore) );
}

void lockVariable(){
    xSemaphoreTake(endSemaphore, portMAX_DELAY);
}

void unlockVariable(){
    xSemaphoreGive(endSemaphore);
}
