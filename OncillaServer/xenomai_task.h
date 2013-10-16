/**
 * @file xenomai_task.h
 * @author Stanislaw Maciag maciag@student.agh.edu.pl
 * @brief Xenomai utilities for the OncillaRobot
 */

#ifndef XENOMAI_TASK_H
#define	XENOMAI_TASK_H

#include <libsbcp/bus/ScheduledWorkflow.h>
#include <libsbcp/device/RegisterAccessor.h>
#include <libsbcp/devices/amarsi/Devices.h>
#include <libsbcp/utils/HexaByte.h>
#include <libsbcp/utils/Config.h>
#include <string>

#include <biorob-rbio/servo/Servo.h>
#include <biorob-rbio/servo/ServoConfig.h>

#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <native/task.h>
#include <native/timer.h>
#include <rtdk.h>

#include "OncillaRobot.h"
#include "OncillaCmd.h"

void executor(void *oncillaPtr);
//void supervisor(void *oncillaPtr);

#endif
