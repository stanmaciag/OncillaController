#include "xenomai_task.h"

/**
 *
 * @param oncillaPtr
 */
void executor(void *oncillaPtr) {

    int ret;
    unsigned long overrun;
    OncillaCmd msgBuf;

    OncillaRobot *oncilla = static_cast<OncillaRobot*> (oncillaPtr);



    sbcp::ScheduledWorkflow & w = oncilla->bus->Scheduled();
    for (int i = 0; i < 4; i++) {
        w.AppendScheduledDevice(std::tr1::static_pointer_cast<sbcp::Device, sbcp::amarsi::MotorDriver>(oncilla->legs[i]));
        oncilla->legs[i]->Motor1().GoalPosition().Set(oncilla->zeroPosRaw[2 * i]);
        oncilla->legs[i]->Motor2().GoalPosition().Set(oncilla->zeroPosRaw[2 * i + 1]);
        oncilla->servos[i]->SetCommand(oncilla->zeroPosRaw[i + 8]);

    }



    ret = rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks(oncilla->executor_task_period_ns));
    if (ret) {
        std::runtime_error ex("Xenomai task set periodic failed. Error");
        throw ex;
    }

    ret = rt_task_set_mode(0, T_PRIMARY, NULL);
    if (ret) {
        std::runtime_error ex("Xenomai task set mode failed. Error");
        throw ex;
    }

    RTIME start_time = rt_timer_read(), current_time;

    while (true) {

        rt_mutex_acquire(&(oncilla->end_executor), TM_INFINITE);
        //rt_mutex_release(&(oncilla->end_executor));

        ret = rt_task_wait_period(&overrun);
        if (ret) {
            std::runtime_error ex("Xenomai task wait failed. Error");
            //throw ex;
        }

        ret = rt_queue_read(&(oncilla->posQueue), &msgBuf, sizeof (msgBuf), TM_NONBLOCK);

        if (ret >= 0) {

            if (msgBuf.id == OncillaCmd::SET_POS) {

                oncilla->legs[0]->Motor1().GoalPosition().Set((int16_t) (msgBuf.args.doubleArgs[0] / (2.0 * M_PI) * 4096.0 + oncilla->maxMotorPos[0] / 2.0));
                oncilla->legs[0]->Motor2().GoalPosition().Set((int16_t) ((1.0 - msgBuf.args.doubleArgs[1]) * oncilla->maxMotorPos[1]));
                oncilla->legs[1]->Motor1().GoalPosition().Set((int16_t) (-msgBuf.args.doubleArgs[2] / (2.0 * M_PI) * 4096.0 + oncilla->maxMotorPos[2] / 2.0));
                oncilla->legs[1]->Motor2().GoalPosition().Set((int16_t) (msgBuf.args.doubleArgs[3] * oncilla->maxMotorPos[3]));
                oncilla->legs[2]->Motor1().GoalPosition().Set((int16_t) (msgBuf.args.doubleArgs[4] / (2.0 * M_PI) * 4096.0 + oncilla->maxMotorPos[4] / 2.0));
                oncilla->legs[2]->Motor2().GoalPosition().Set((int16_t) ((1.0 - msgBuf.args.doubleArgs[5]) * oncilla->maxMotorPos[5]));
                oncilla->legs[3]->Motor1().GoalPosition().Set((int16_t) (-msgBuf.args.doubleArgs[6] / (2.0 * M_PI) * 4096.0 + oncilla->maxMotorPos[6] / 2.0));
                oncilla->legs[3]->Motor2().GoalPosition().Set((int16_t) (msgBuf.args.doubleArgs[7] * oncilla->maxMotorPos[7]));

                for (int i = 0; i < 4; i++)
                    oncilla->servos[i]->SetCommand(msgBuf.args.doubleArgs[i + 8]);

                rt_queue_free(&(oncilla->posQueue), &msgBuf);

            } else if (msgBuf.id == OncillaCmd::RESET_TIMER) {

                if (oncilla->RT) {
                    rt_mutex_acquire(&(oncilla->posArrMutexRT), TM_INFINITE);
                    oncilla->trajectQueue.clear();
                    rt_mutex_release(&(oncilla->posArrMutexRT));

                } else {
                    pthread_mutex_lock(&(oncilla->posArrMutex));
                    oncilla->trajectQueue.clear();
                    pthread_mutex_unlock(&(oncilla->posArrMutex));
                    rt_task_set_mode(0, T_PRIMARY, NULL);
                }
                
                start_time = rt_timer_read();

            }

        } else if (ret == -EINVAL || ret == -EIDRM) {
            break;
        } else if (ret != -EWOULDBLOCK) {
            std::runtime_error ex("Cannot read command from message queue. Error");
            throw ex;
        }


        try {
            w.StartTransfers();
            w.WaitForTransfersCompletion();
        } catch (sbcp::MultipleTransferError & e) {
            std::runtime_error ex("SBCP communication failed. Error");
            throw ex;
        }


        if (oncilla->RT) {

            rt_mutex_acquire(&(oncilla->posArrMutexRT), TM_INFINITE);

        } else {

            // If causing timeout for real-time task try using pthread_mutex_trylock() instead
            pthread_mutex_lock(&(oncilla->posArrMutex));

        }


        for (int i = 0; i < 4; i++) {

            oncilla->currentPosRaw[2 * i] = oncilla->legs[i]->Motor1().PresentPosition().Get();
            oncilla->currentPosRaw[2 * i + 1] = oncilla->legs[i]->Motor2().PresentPosition().Get();


        }


        oncilla->currentPos[0] = (double) (oncilla->currentPosRaw[0] - oncilla->maxMotorPos[0] / 2.0) / 4096.0 * (2.0 * M_PI);
        oncilla->currentPos[1] = (1.0 - (double) oncilla->currentPosRaw[1] / (double) oncilla->maxMotorPos[1]);
        oncilla->currentPos[2] = (double) -(oncilla->currentPosRaw[2] - oncilla->maxMotorPos[2] / 2.0) / 4096.0 * (2.0 * M_PI);
        oncilla->currentPos[3] = ((double) oncilla->currentPosRaw[3] / (double) oncilla->maxMotorPos[3]);
        oncilla->currentPos[4] = (double) (oncilla->currentPosRaw[4] - oncilla->maxMotorPos[4] / 2.0) / 4096.0 * (2.0 * M_PI);
        oncilla->currentPos[5] = (1.0 - (double) oncilla->currentPosRaw[5] / (double) oncilla->maxMotorPos[5]);
        oncilla->currentPos[6] = (double) -(oncilla->currentPosRaw[6] - oncilla->maxMotorPos[6] / 2.0) / 4096.0 * (2.0 * M_PI);
        oncilla->currentPos[7] = ((double) oncilla->currentPosRaw[7] / (double) oncilla->maxMotorPos[7]);

        for (int i = 0; i < 4; i++)
            oncilla->currentPos[i + 8] = oncilla->servos[i]->Command();

        current_time = rt_timer_read();
        OncillaRobot::trajectPoint newTrajectPoint;
        newTrajectPoint.coords[0] = (double) (current_time - start_time) / 1000000;
        memcpy(newTrajectPoint.coords + 1, oncilla->currentPos, 12 * sizeof (double));

        if (oncilla->trajectQueue.size() >= oncilla->queueSize)
            oncilla->trajectQueue.pop_front();

        oncilla->trajectQueue.push_back(newTrajectPoint);

        if (oncilla->RT) {
            rt_mutex_release(&(oncilla->posArrMutexRT));

        } else {
            pthread_mutex_unlock(&(oncilla->posArrMutex));
            rt_task_set_mode(0, T_PRIMARY, NULL);
        }

        rt_mutex_release(&(oncilla->end_executor));
        //rt_mutex_acquire(&(oncilla->end_executor), TM_INFINITE);
    }

    /*rt_mutex_release(&(oncilla->end_executor));
    
    pthread_mutex_lock(&(oncilla->end_mutex));
    pthread_cond_signal(&(oncilla->cond));
    pthread_mutex_unlock(&(oncilla->end_mutex));
    
     */

}

/*
void supervisor(void *oncillaPtr) {

    OncillaRobot *oncilla = static_cast<OncillaRobot*> (oncillaPtr);
    OncillaCmd cmd;
    int ret;
    bool exit = false;
    
    while (!exit) {

        ret = rt_queue_read(&(oncilla->cmdQueue), &cmd, sizeof (cmd), TM_INFINITE);

        if (ret >= 0) {

            switch (cmd.id) {

                case OncillaCmd::SET_POS:
                {
                    if (rt_queue_write(&(oncilla->posQueue), &(cmd.args.doubleArgs), 12 * sizeof (double), Q_NORMAL) < 0) {
                        std::runtime_error ex("Cannot write positions to message queue. Error");
                        throw ex;
                    }

                    break;
                }

                case OncillaCmd::SHUTDOWN:
                {
                    rt_mutex_acquire(&(oncilla->end_executor), TM_INFINITE);
                    oncilla->shutdown = true;
                    rt_mutex_release(&(oncilla->end_executor));
                    exit = true;
                    break;
                }

                default:

                    break;

            }

        } else if (ret == -EINVAL || ret == -EIDRM) {
            break;
        } else if (ret != -EWOULDBLOCK) {
            std::runtime_error ex("Cannot read command from message queue. Error");
            throw ex;
        }

        //rt_task_suspend(NULL); 	

    }

    std::cout << "SUP" << std::endl;
    
}
 */
