/*
 * TaskWrapper.cpp
 *
 *  Created on: 13 рту. 2019 у.
 *      Author: Taras.Melnik
 */

#include "TaskWrapper.h"

/* FreeRtos core includes */
#include <stdio.h>
#include "FreeRTOSConfig.h"
#include "task.h"

TaskWrapper::TaskWrapper() {
    this->task_handle = 0;

}

TaskWrapper::~TaskWrapper() {
    // TODO Auto-generated destructor stub
}

bool TaskWrapper::task_create(const StackType_t stackDepth, UBaseType_t uxPriority,
        const portCHAR * const pcName)
{
    BaseType_t result;
    result = xTaskCreate((TaskFunction_t) TaskWrapper::task_function,
            pcName, stackDepth, this, uxPriority, &task_handle);
    if (result == pdPASS)
        return true;
    else
        return false;
}

void TaskWrapper::task_function(const void *parameters)
{
    ((TaskWrapper*) parameters)->run();
}
