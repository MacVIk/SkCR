/*
 * TaskWrapper.h
 *
 *  Created on: 13 рту. 2019 у.
 *      Author: Taras.Melnik
 */

#ifndef LIB_TASKWRAPPER_H_
#define LIB_TASKWRAPPER_H_

#include "FreeRTOS.h"
#include "portmacro.h"
#include "task.h"

class TaskWrapper {
public:
        TaskWrapper();
        virtual ~TaskWrapper();

        TaskHandle_t task_handle;
        bool task_create(const StackType_t ulStackDepth, UBaseType_t priority,
                        const portCHAR * const name);
protected:
        virtual void run() = 0;
private:
        static void task_function(const void *parameters);
};

#endif /* LIB_TASKWRAPPER_H_ */
