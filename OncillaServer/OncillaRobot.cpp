#include "OncillaRobot.h"

OncillaRobot::OncillaRobot(const char* filename, bool RT) throw (std::runtime_error) : queueSize(4096), RT(RT), executor_task_period_ns(2 * 1e6)/*, shutdown(false) */ {

    sbcp::Config config;
    config.LoadAllFiles();

    bus = config.BusDefinitions().Bus("default").OpenBus();

    for (unsigned int i = 0; i < 4; ++i)
        legs[i] = bus->OpenDevice<sbcp::amarsi::MotorDriver> (i + 1);

    if (bus == NULL) {
        std::runtime_error ex("Cannot create the default bus. Error");
        throw ex;
    }

    init();

    parseConfigFile(filename);

    mlockall(MCL_CURRENT | MCL_FUTURE);



    /*if (rt_queue_create(&cmdQueue, "CommandQueue", queueSize * sizeof(OncillaCmd), Q_UNLIMITED, Q_FIFO) != 0) {
        std::runtime_error ex("Cannot create the message queue. Error");
        throw ex;
    }*/

    if (rt_queue_create(&posQueue, "PositionQueue", queueSize * 12 * sizeof (double), Q_UNLIMITED, Q_FIFO) != 0) {
        std::runtime_error ex("Cannot create the position queue. Error");
        throw ex;
    }

    if (RT) {

        if (rt_mutex_create(&posArrMutexRT, "PosArrayMutex") != 0) {
            std::runtime_error ex("Xenomai position array mutex initialization failed. Error");
            throw ex;
        }

    } else {
        if (pthread_mutex_init(&posArrMutex, NULL) != 0) {
            std::runtime_error ex("Position array mutex initialization failed. Error");
            throw ex;
        }
    }

    /*if (pthread_mutex_init(&end_mutex, NULL) != 0) {
        std::runtime_error ex("Condition mutex initialization failed. Error");
        throw ex;
    }*/

    if (rt_mutex_create(&end_executor, "EndConditionMutex") != 0) {
        std::runtime_error ex("Xenomai condition mutex initialization failed. Error");
        throw ex;
    }

    /*if (pthread_cond_init(&cond, NULL) != 0) {
        std::runtime_error ex("Condition variable initialization failed. Error");
        throw ex;
    }*/

    // rt_task_spawn(&supervisor_task, "supervisor", STACK_SIZE, SUPERVISOR_PRIO, T_FPU, &supervisor, this);
    rt_task_spawn(&executor_task, "executor", STACK_SIZE, EXECUTOR_PRIO, T_FPU, &executor, this);


}

OncillaRobot::~OncillaRobot() {

    clean_exit();

}

void OncillaRobot::stopMotors() {

    for (unsigned int i = 0; i < 4; ++i) {

        legs[i]->Motor1().MotorControlMode().Set(sbcp::amarsi::MotorDriver::Motor::COAST);
        legs[i]->Motor2().MotorControlMode().Set(sbcp::amarsi::MotorDriver::Motor::COAST);

    }

}

void OncillaRobot::initMotors() {
    for (int i = 0; i < 4; i++) {

        legs[i]->Motor1().MotorControlMode().Set(sbcp::amarsi::MotorDriver::Motor::SMOOTH_POSITION);
        legs[i]->Motor1().SmoothPositionUpdate().Set(2);

        legs[i]->Motor1().MaxTorque().Set(500);
        legs[i]->Motor1().MaxSpeed().Set(31000);
        legs[i]->Motor1().MaxAcceleration().Set(31000);

        legs[i]->Motor1().Preload().Set(1332);
        legs[i]->Motor1().Stiffness().Set(1000);
        legs[i]->Motor1().Damping().Set(1000);

        legs[i]->Motor1().PGain().Set(500);
        legs[i]->Motor1().IGain().Set(50);
        legs[i]->Motor1().DGain().Set(250);

        legs[i]->Motor2().MotorControlMode().Set(sbcp::amarsi::MotorDriver::Motor::SMOOTH_POSITION);
        legs[i]->Motor2().SmoothPositionUpdate().Set(2);

        legs[i]->Motor2().MaxTorque().Set(500);
        legs[i]->Motor2().MaxSpeed().Set(31000);
        legs[i]->Motor2().MaxAcceleration().Set(31000);

        legs[i]->Motor2().Preload().Set(1332);
        legs[i]->Motor2().Stiffness().Set(1000);
        legs[i]->Motor2().Damping().Set(1000);

        legs[i]->Motor2().PGain().Set(500);
        legs[i]->Motor2().IGain().Set(50);
        legs[i]->Motor2().DGain().Set(250);


    }

    maxMotorPos[0] = 1200;
    maxMotorPos[1] = 1350;
    maxMotorPos[2] = 1200;
    maxMotorPos[3] = 1350;
    maxMotorPos[4] = 1200;
    maxMotorPos[5] = 2200;
    maxMotorPos[6] = 1200;
    maxMotorPos[7] = 1350;

}

void OncillaRobot::initServos() {


    for (int i = 0; i < 4; i++) {
        unsigned int channel;
        switch (i) {
            case 0: channel = 15;
                break; // left-fore
            case 1: channel = 13;
                break; // right-fore
            case 2: channel = 9;
                break; // left-hind
            case 3: channel = 11;

                break; // right-hind
            default: channel = 0;
        }
        servos[i] = Servo::Ptr(new Servo(channel));
        servos[i]->SetMinPosition(-1.0);
        servos[i]->SetMaxPosition(1.0);
        servos[i]->SetMinPulseLength(55000);
        servos[i]->SetMaxPulseLength(65000);
        servos[i]->SetReversed(true);
        servos[i]->Enable(true);
    }

}

void OncillaRobot::GetPosAndStatus(
        sbcp::amarsi::MotorDriver::MagneticEncoder & me, int & pos,
        int & status) {

    uint16_t pAs = me.PositionAndStatus().Get();
    pos = pAs & 0x3fff;
    status = (pAs & 0xc000) >> 14;
}

int16_t OncillaRobot::getMEValue(sbcp::amarsi::MotorDriver::MagneticEncoder & me) {

    uint16_t pAs = me.PositionAndStatus().Get();

    return pAs & 0x3fff;

}

bool OncillaRobot::GetCalibrationStatus(sbcp::amarsi::MotorDriver::Motor & m,
        int number) {
    uint16_t val(m.MotorControlMode().Get());

    return val & (1 << 9);
}

void OncillaRobot::calibrateDevice(BusPtr bus, sbcp::amarsi::MotorDriver::Ptr dev) {

    //dev->UncalibrateMotors();

    // Calibration
    bool m1Calibrated(GetCalibrationStatus(dev->Motor1(), 1));
    bool m2Calibrated(GetCalibrationStatus(dev->Motor2(), 2));
    if (!(m1Calibrated || m2Calibrated)) {
        dev->CalibrateMotors();
    } /*else {
        std::cout << "Motor already calibrated" << std::endl;
    }*/

    while (!(m1Calibrated && m2Calibrated)) {

        usleep(1e6);
        m1Calibrated = GetCalibrationStatus(dev->Motor1(), 1);
        m2Calibrated = GetCalibrationStatus(dev->Motor2(), 2);
    }
    //std::cout << "Motor calibrated" << std::endl << std::endl;
}

void OncillaRobot::calibrate() {

    for (int i = 0; i < 4; i++)
        calibrateDevice(bus, legs[i]);


}

/**
 * @todo Should be private, actually
 */
void OncillaRobot::init() {

    bus->Lazy();
    calibrate();
    initMotors();
    initServos();
    zeroPosRaw[0] = maxMotorPos[0] / 2.0;
    zeroPosRaw[1] = maxMotorPos[1];
    zeroPosRaw[2] = maxMotorPos[2] / 2.0;
    zeroPosRaw[3] = 0;
    zeroPosRaw[4] = maxMotorPos[4] / 2.0;
    zeroPosRaw[5] = maxMotorPos[5];
    zeroPosRaw[6] = maxMotorPos[6] / 2.0;
    zeroPosRaw[7] = 0;

}

void OncillaRobot::clean_exit() {

    /*rt_task_delete(&executor_task);
    rt_task_delete(&supervisor_task);

    rt_queue_delete(&cmdQueue);
    rt_queue_delete(&posQueue);
    pthread_mutex_destroy(&posArrMutex);
    rt_mutex_delete(&end_mutex);*/


    //OncillaCmd newCmd(OncillaCmd::SHUTDOWN);
    //pushCmd(newCmd);

    //pthread_mutex_lock(&end_mutex);
    //pthread_cond_wait(&cond, &end_mutex);
    //pthread_mutex_unlock(&end_mutex);


    rt_task_delete(&executor_task);
    //rt_task_delete(&supervisor_task);

    //rt_queue_delete(&cmdQueue);
    rt_queue_delete(&posQueue);
    if (RT)
        rt_mutex_delete(&posArrMutexRT);
    else
        pthread_mutex_destroy(&posArrMutex);

    //pthread_cond_destroy(&cond);
    //pthread_mutex_destroy(&end_mutex);
    rt_mutex_delete(&end_executor);

}

void OncillaRobot::test() {

    std::cout << "TEST" << std::endl;

}

/*void OncillaRobot::pushCmd(OncillaCmd &cmd) {

    if (rt_queue_write(&cmdQueue, &cmd, sizeof (cmd), Q_NORMAL) < 0) {

        std::runtime_error ex("Cannot write command to message queue. Error");
        throw ex;
    }
    //rt_task_resume(&supervisor_task);	


}*/

void OncillaRobot::getPos(double *res) {

    if (RT) {
        rt_mutex_acquire(&posArrMutexRT, TM_INFINITE);
        memcpy(res, currentPos, 12 * sizeof (double));
        rt_mutex_release(&posArrMutexRT);
    } else {

        pthread_mutex_lock(&posArrMutex);
        memcpy(res, currentPos, 12 * sizeof (double));
        pthread_mutex_unlock(&posArrMutex);
    }


}

void OncillaRobot::getNextTrajectPoint(double *res) {

    if (trajectQueue.size() == 0) {
        double nullRes[13] = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        memcpy(res, nullRes, 13 * sizeof (double));
    } else {
        if (RT) {

            rt_mutex_acquire(&posArrMutexRT, TM_INFINITE);
            memcpy(res, trajectQueue.front().coords, 13 * sizeof (double));
            trajectQueue.pop_front();
            rt_mutex_release(&posArrMutexRT);

        } else {

            pthread_mutex_lock(&posArrMutex);
            memcpy(res, trajectQueue.front().coords, 13 * sizeof (double));
            trajectQueue.pop_front();
            pthread_mutex_unlock(&posArrMutex);


        }
    }
}

void OncillaRobot::setPos(double *newPos) {

    OncillaCmd newCmd(OncillaCmd::SET_POS, newPos);

    //if ((rt_queue_write(&posQueue, newPos, 12 * sizeof (double), Q_NORMAL)) < 0) {
    if ((rt_queue_write(&posQueue, &newCmd, sizeof (newCmd), Q_NORMAL)) < 0) {
        std::runtime_error ex("Cannot write command to message queue. Error");
        throw ex;
    }

    //std::cout << ret << std::endl;

}

void OncillaRobot::resetTrajectRec() {
    
    OncillaCmd newCmd(OncillaCmd::RESET_TIMER);
    if ((rt_queue_write(&posQueue, &newCmd, sizeof (newCmd), Q_NORMAL)) < 0) {
        std::runtime_error ex("Cannot write command to message queue. Error");
        throw ex;
    }
}


// Really unfriendly for the programmer
void OncillaRobot::parseConfigFile(const char* filename) {
    libconfig::Config *initConfig = new libconfig::Config();

    try {
        initConfig->readFile(filename);


        int value;
        /*std::string lookupMotorsTmp[] = {"left-fore", "right-fore", "left-hind", "right-hind"};
        std::string lookupMotorsPos[] = {"M1", "M2"};
        std::string lookupMotorsParam[] = {"maxPos"};
         */

        const char *lookupMotorsTmp[] = {"left-fore", "right-fore", "left-hind", "right-hind"};
        const char *lookupMotorsPos[] = {"M1", "M2"};
        const char *lookupMotorsParam[] = {"maxPos", "maxTorque", "maxSpeed", "maxAccel", "preload",
            "stiffness", "damping", "pGain", "iGain", "dGain", "posUpdate"};

        for (unsigned int k = 0; k < sizeof (lookupMotorsParam) / sizeof (char*); k++) {

            char path[256];
            strcpy(path, "motors.");
            strcat(path, lookupMotorsParam[k]);

            if (initConfig->lookupValue(path, value)) {

                if (strcmp(lookupMotorsParam[k], "maxPos") == 0) {

                    for (int i = 0; i < 8; i++)
                        maxMotorPos[i] = value;

                } else if (strcmp(lookupMotorsParam[k], "maxTorque") == 0) {

                    for (int i = 0; i < 4; i++) {
                        legs[i]->Motor1().MaxTorque().Set(value);
                        legs[i]->Motor2().MaxTorque().Set(value);
                    }

                } else if (strcmp(lookupMotorsParam[k], "maxSpeed") == 0) {

                    for (int i = 0; i < 4; i++) {
                        legs[i]->Motor1().MaxSpeed().Set(value);
                        legs[i]->Motor2().MaxSpeed().Set(value);
                    }

                } else if (strcmp(lookupMotorsParam[k], "maxAccel") == 0) {
                    for (int i = 0; i < 4; i++) {
                        legs[i]->Motor1().MaxAcceleration().Set(value);
                        legs[i]->Motor2().MaxAcceleration().Set(value);
                    }

                } else if (strcmp(lookupMotorsParam[k], "preload") == 0) {
                    for (int i = 0; i < 4; i++) {
                        legs[i]->Motor1().Preload().Set(value);
                        legs[i]->Motor2().Preload().Set(value);
                    }

                } else if (strcmp(lookupMotorsParam[k], "stiffness") == 0) {
                    for (int i = 0; i < 4; i++) {
                        legs[i]->Motor1().Preload().Set(value);
                        legs[i]->Motor2().Preload().Set(value);
                    }

                } else if (strcmp(lookupMotorsParam[k], "damping") == 0) {
                    for (int i = 0; i < 4; i++) {
                        legs[i]->Motor1().Damping().Set(value);
                        legs[i]->Motor2().Damping().Set(value);
                    }

                } else if (strcmp(lookupMotorsParam[k], "pGain") == 0) {
                    for (int i = 0; i < 4; i++) {
                        legs[i]->Motor1().PGain().Set(value);
                        legs[i]->Motor2().PGain().Set(value);
                    }

                } else if (strcmp(lookupMotorsParam[k], "iGain") == 0) {
                    for (int i = 0; i < 4; i++) {
                        legs[i]->Motor1().IGain().Set(value);
                        legs[i]->Motor2().IGain().Set(value);
                    }

                } else if (strcmp(lookupMotorsParam[k], "dGain") == 0) {
                    for (int i = 0; i < 4; i++) {
                        legs[i]->Motor1().DGain().Set(value);
                        legs[i]->Motor2().DGain().Set(value);
                    }

                } else if (strcmp(lookupMotorsParam[k], "posUpdate") == 0) {
                    for (int i = 0; i < 4; i++) {
                        legs[i]->Motor1().SmoothPositionUpdate().Set(value);
                        legs[i]->Motor2().SmoothPositionUpdate().Set(value);
                    }

                }

            }

        }

        for (int i = 0; i < 4; i++) {

            for (unsigned int k = 0; k < sizeof (lookupMotorsParam) / sizeof (char*); k++) {

                char path[256];
                strcpy(path, "motors.");
                strcat(path, lookupMotorsTmp[i]);
                strcat(path, ".");
                strcat(path, lookupMotorsParam[k]);

                if (initConfig->lookupValue(path, value)) {

                    if (strcmp(lookupMotorsParam[k], "maxPos") == 0) {

                        maxMotorPos[2 * i] = value;
                        maxMotorPos[2 * i + 1] = value;

                    } else if (strcmp(lookupMotorsParam[k], "maxTorque") == 0) {

                        legs[i]->Motor1().MaxTorque().Set(value);
                        legs[i]->Motor2().MaxTorque().Set(value);

                    } else if (strcmp(lookupMotorsParam[k], "maxSpeed") == 0) {

                        legs[i]->Motor1().MaxSpeed().Set(value);
                        legs[i]->Motor2().MaxSpeed().Set(value);

                    } else if (strcmp(lookupMotorsParam[k], "maxAccel") == 0) {

                        legs[i]->Motor1().MaxAcceleration().Set(value);
                        legs[i]->Motor2().MaxAcceleration().Set(value);

                    } else if (strcmp(lookupMotorsParam[k], "preload") == 0) {

                        legs[i]->Motor1().Preload().Set(value);
                        legs[i]->Motor2().Preload().Set(value);

                    } else if (strcmp(lookupMotorsParam[k], "stiffness") == 0) {

                        legs[i]->Motor1().Preload().Set(value);
                        legs[i]->Motor2().Preload().Set(value);

                    } else if (strcmp(lookupMotorsParam[k], "damping") == 0) {

                        legs[i]->Motor1().Damping().Set(value);
                        legs[i]->Motor2().Damping().Set(value);

                    } else if (strcmp(lookupMotorsParam[k], "pGain") == 0) {

                        legs[i]->Motor1().PGain().Set(value);
                        legs[i]->Motor2().PGain().Set(value);

                    } else if (strcmp(lookupMotorsParam[k], "iGain") == 0) {

                        legs[i]->Motor1().IGain().Set(value);
                        legs[i]->Motor2().IGain().Set(value);

                    } else if (strcmp(lookupMotorsParam[k], "dGain") == 0) {

                        legs[i]->Motor1().DGain().Set(value);
                        legs[i]->Motor2().DGain().Set(value);

                    } else if (strcmp(lookupMotorsParam[k], "posUpdate") == 0) {

                        legs[i]->Motor1().SmoothPositionUpdate().Set(value);
                        legs[i]->Motor2().SmoothPositionUpdate().Set(value);

                    }

                }

            }



            for (int j = 0; j < 2; j++) {
                for (unsigned int k = 0; k < sizeof (lookupMotorsParam) / sizeof (char*); k++) {

                    char path[256];
                    strcpy(path, "motors.");
                    strcat(path, lookupMotorsTmp[i]);
                    strcat(path, ".");
                    strcat(path, lookupMotorsPos[j]);
                    strcat(path, ".");
                    strcat(path, lookupMotorsParam[k]);

                    if (initConfig->lookupValue(path, value)) {

                        if (strcmp(lookupMotorsParam[k], "maxPos") == 0) {

                            if (j == 0)
                                maxMotorPos[2 * i] = value;
                            else
                                maxMotorPos[2 * i + 1] = value;

                        } else if (strcmp(lookupMotorsParam[k], "maxTorque") == 0) {

                            if (j == 0)
                                legs[i]->Motor1().MaxTorque().Set(value);
                            else
                                legs[i]->Motor2().MaxTorque().Set(value);

                        } else if (strcmp(lookupMotorsParam[k], "maxSpeed") == 0) {

                            if (j == 0)
                                legs[i]->Motor1().MaxSpeed().Set(value);
                            else
                                legs[i]->Motor2().MaxSpeed().Set(value);

                        } else if (strcmp(lookupMotorsParam[k], "maxAccel") == 0) {

                            if (j == 0)
                                legs[i]->Motor1().MaxAcceleration().Set(value);
                            else
                                legs[i]->Motor2().MaxAcceleration().Set(value);

                        } else if (strcmp(lookupMotorsParam[k], "preload") == 0) {

                            if (j == 0)
                                legs[i]->Motor1().Preload().Set(value);
                            else
                                legs[i]->Motor2().Preload().Set(value);

                        } else if (strcmp(lookupMotorsParam[k], "stiffness") == 0) {

                            if (j == 0)
                                legs[i]->Motor1().Preload().Set(value);
                            else
                                legs[i]->Motor2().Preload().Set(value);

                        } else if (strcmp(lookupMotorsParam[k], "damping") == 0) {

                            if (j == 0)
                                legs[i]->Motor1().Damping().Set(value);
                            else
                                legs[i]->Motor2().Damping().Set(value);

                        } else if (strcmp(lookupMotorsParam[k], "pGain") == 0) {

                            if (j == 0)
                                legs[i]->Motor1().PGain().Set(value);
                            else
                                legs[i]->Motor2().PGain().Set(value);

                        } else if (strcmp(lookupMotorsParam[k], "iGain") == 0) {

                            if (j == 0)
                                legs[i]->Motor1().IGain().Set(value);
                            else
                                legs[i]->Motor2().IGain().Set(value);

                        } else if (strcmp(lookupMotorsParam[k], "dGain") == 0) {

                            if (j == 0)
                                legs[i]->Motor1().DGain().Set(value);
                            else
                                legs[i]->Motor2().DGain().Set(value);

                        } else if (strcmp(lookupMotorsParam[k], "posUpdate") == 0) {
                            if (j == 0)
                                legs[i]->Motor1().SmoothPositionUpdate().Set(value);
                            else
                                legs[i]->Motor2().SmoothPositionUpdate().Set(value);

                        }

                    }

                }
            }
        }


        const char *lookupServosParam[] = {"maxPos", "minPos", "minPulseLength", "maxPulseLength"};

        for (int i = 0; i < 4; i++) {
            char path[256];
            float valueFloat;
            int valueInt;

            strcpy(path, "servos.");
            strcat(path, lookupServosParam[i]);


            if (i < 2) {
                if (initConfig->lookupValue(path, valueFloat)) {

                    if (i == 0)
                        servos[i]->SetMaxPosition(valueFloat);
                    else
                        servos[i]->SetMinPosition(valueFloat);
                }

            } else {

                if (initConfig->lookupValue(path, valueInt)) {

                    if (i == 2)
                        servos[i]->SetMinPulseLength(valueInt);
                    else
                        servos[i]->SetMaxPulseLength(valueInt);
                }

            }

        }

        if (initConfig->lookupValue("xenomai.executor_period", value))
            executor_task_period_ns = value * 1e6;

        if (initConfig->lookupValue("misc.queue_size", value)) {
            if ((unsigned)value >= trajectQueue.max_size()) {
                std::cout << "*Warning* Trajectory queue size = " << queueSize << " too big, setting default value" << std::endl;
            } else {
                queueSize = value;
            }
        }


    } catch (libconfig::FileIOException ex) {

        std::cout << "*WARNING* Config file " << filename << " not found. Using default values" << std::endl;

    } catch (libconfig::ParseException) {

        std::cout << "*WARNING* Corrupted config file " << filename << ". Using default values" << std::endl;

    }

    delete initConfig;
}
