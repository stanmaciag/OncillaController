/**
 * @file OncillaRobot.h
 * @author Stanislaw Maciag maciag@student.agh.edu.pl
 * @brief Software representation of the Oncilla robot
 */

#ifndef ONCILLAROBOT_H
#define	ONCILLAROBOT_H

#include <stdexcept>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <pthread.h>
#include <libconfig.h++>
#include <math.h>

#include <libsbcp/bus/ScheduledWorkflow.h>
#include <libsbcp/device/RegisterAccessor.h>
#include <libsbcp/devices/amarsi/Devices.h>
#include <libsbcp/utils/HexaByte.h>
#include <libsbcp/utils/Config.h>

#include <biorob-rbio/servo/Servo.h>
#include <biorob-rbio/servo/ServoConfig.h>

#include <native/task.h>
#include <native/timer.h>
#include <native/queue.h>
#include <native/mutex.h>
#include <rtdk.h>
#include <deque>

#include "xenomai_task.h"
#include "OncillaCmd.h"

typedef std::tr1::shared_ptr<sbcp::Bus> BusPtr;
typedef sbcp::amarsi::MotorDriver::Ptr MotorPtr;

/**
 * Software representation of the Oncilla robot. Implements methods for moving robot's joints and reading sensors values. OncillaRobot is
 * separate module, which can be used outside OncillaServer utility. Can be constructed and used in both real-time and non real-time context.
 * After construction of the object robot is initialized and ready to use.
 *
 * IMPORTANT: using OncillaRobot in real-time context is the safest solution - then the whole robot control is performed in real-time system.
 * Using OncillaRobot in non real-time context is also possible, but causes the OncillaRobot::executor_task to be temporarily switched to so called secondary mode in every cycle.
 * In secondary mode a Xenomai task is controlled by the regular Linux scheduler, which means that in fact it is no longer real-time. It is necessary, because real-time
 * and non real-time synchronization mechanisms cannot be mixed.
 */
class OncillaRobot {

	/**
	 * Stucture wrapper for array storing current joints positions
	 */
	struct trajectPoint {
		/**
		 * Array which contains time and corresponding joints positions (single trajectory point):
		 * - coords[0] - time in ms
		 * - coords[1] - left fore hip joint angle position in radians
		 * - coords[2] - left fore knee joint position described by bend coefficient
		 * - coords[3] - right fore hip joint angle position in radians
		 * - coords[4] - right fore knee joint position described by bend coefficient
		 * - coords[5] - left hind hip joint angle position in radians
		 * - coords[6] - left hind knee joint position described by bend coefficient
		 * - coords[7] - right hind hip joint angle position in radians
		 * - coords[8] - right hind knee joint position described by bend coefficient
		 * - coords[9] - left fore servo position
		 * - coords[10] - right fore servo position
		 * - coords[11] - left hind servo position
		 * - coords[12] - right hind servo position
		 */
		double coords[13];

	};

	/**
	 * Stack size for xenomai task
	 */
	static const unsigned short STACK_SIZE = 8192;
	/**
	 * Executor task priority
	 */
	static const unsigned short EXECUTOR_PRIO = 2;
	/*
	 * Supervisor task priority
	 */
	//static const unsigned short SUPERVISOR_PRIO = 99;
	/**
	 * Array of positions for all joints, which are considered as neutral (zero) positions, in relative encoder units. After powering up robot sets them as target.
	 * Elements description:
	 * - zeroPosRaw[0] - left fore hip joint position
	 * - zeroPosRaw[1] - left fore knee joint position
	 * - zeroPosRaw[2] - right fore hip joint position
	 * - zeroPosRaw[3] - right fore knee joint position
	 * - zeroPosRaw[4] - left hind hip joint position
	 * - zeroPosRaw[5] - left hind knee joint position
	 * - zeroPosRaw[6] - right hind hip joint position
	 * - zeroPosRaw[7] - right hind knee joint position
	 * - zeroPosRaw[8] - left fore servo position
	 * - zeroPosRaw[9] - right fore servo position
	 * - zeroPosRaw[10] - left hind servo position
	 * - zeroPosRaw[11] - right hind servo position
	 */
	int16_t zeroPosRaw[12];
	/**
	 * Array of maximum positions, only for motors.
	 * Elements description:
	 * - maxMotorPos[0] - left fore hip joint position
	 * - maxMotorPos[1] - left fore knee joint position
	 * - maxMotorPos[2] - right fore hip joint position
	 * - maxMotorPos[3] - right fore knee joint position
	 * - maxMotorPos[4] - left hind hip joint position
	 * - maxMotorPos[5] - left hind knee joint position
	 * - maxMotorPos[6] - right hind hip joint position
	 * - maxMotorPos[7] - right hind knee joint position
	 */
	int16_t maxMotorPos[8];
	/**
	 *  maximal size of both command queue of the robot and the queue which stores the previous trajectory for all joints
	 */
	unsigned int queueSize;
	/**
	 * Describes if the object is constructed and used in real-time (true) or non real-time environment (false)
	 */
	bool RT;
	//bool shutdown;
	/**
	 * SBCP bus pointer, defines bus used by the robot instance
	 */
	BusPtr bus;
	/**
	 * Array of pointers to motor drivers used by the robot. Elements description:
	 * - legs[0] - left fore
	 * - legs[1] - right fore
	 * - legs[2] - left hind
	 * - legs[3] - right hind
	 */
	MotorPtr legs[4];
	/**
	 * Array of pointer to servos used by the robot. Elements description:
	 * - servos[0] - left fore
	 * - servos[1] - right fore
	 * - servos[2] - left hind
	 * - servos[3] - right hind
	 */
	Servo::Ptr servos[4];

	/**
	 * Xenomai task descriptor for executor task. Executor task is periodic task, period of execution is defined by executor_task_period_ns variable.
	 * All control over the robot after initialization is handled by this task, and it is defined in executor()
	 */
	RT_TASK executor_task;
	//RT_TASK supervisor_task;

	/**
	 * Message queue for commands. Commands are received parsed and executed by the OncillaRobot::executor_task. Adding commands to queue is done by
	 * calling appropriate methods - currently setPos() and resetTrajectRec()
	 */
	RT_QUEUE posQueue;
	//RT_QUEUE cmdQueue;
	/**
	 * Mutex for synchronization currentPos and trajectQueue between OncillaRobot::executor_task and context thread/task, non real-time version
	 */
	pthread_mutex_t posArrMutex;
	//pthread_mutex_t end_mutex;
	//pthread_cond_t cond;
	/**
	 * Mutex for synchronization currentPos and trajectQueue between OncillaRobot::executor_task and context thread/task, real-time version
	 */
	RT_MUTEX posArrMutexRT;
	/**
	 * Mutex which is precluding termination of OncillaRobot::executor_task without finishing the current cycle of execution (Xenomai task cannot be
	 * deleted, if any mutex is acquired inside, this is Xenomai utility)
	 */
	RT_MUTEX end_executor;
	/**
	 * Period of execution for OncillaRobot::executor_task
	 */
	RTIME executor_task_period_ns;

	/**
	 * Last positions of all joints, elements description:
	 * - currentPos[0] - left fore hip joint angle position in radians
	 * - currentPos[1] - left fore knee joint position described by bend coefficient
	 * - currentPos[2] - right fore hip joint angle position in radians
	 * - currentPos[3] - right fore knee joint position described by bend coefficient
	 * - currentPos[4] - left hind hip joint angle position in radians
	 * - currentPos[5] - left hind knee joint position described by bend coefficient
	 * - currentPos[6] - right hind hip joint angle position in radians
	 * - currentPos[7] - right hind knee joint position described by bend coefficient
	 * - currentPos[8] - left fore servo position
	 * - currentPos[9] - right fore servo position
	 * - currentPos[10] - left hind servo position
	 * - currentPos[11] - right hind servo position
	 */
	double currentPos[12];

	/**
	 * Last positions of all motors, elements description (all units are relative encoder units), elements description:
	 * - currentPosRaw[0] - left fore hip joint position
	 * - currentPosRaw[1] - left fore knee joint position
	 * - currentPosRaw[2] - right fore hip joint position
	 * - currentPosRaw[3] - right fore knee joint position
	 * - currentPosRaw[4] - left hind hip joint position
	 * - currentPosRaw[5] - left hind knee joint position
	 * - currentPosRaw[6] - right hind hip joint position
	 * - currentPosRaw[7] - right hind knee joint position
	 */
	int16_t currentPosRaw[8];

	/**
	 * Queue storing previous positions of all joints and corresponding time (trajectory)
	 */
	std::deque<trajectPoint> trajectQueue;
	/**
	 * Get motor calibration status
	 * @param m Motor reference
	 * @param number
	 * @return True for calibrated, false for uncalibrated
	 */
	bool GetCalibrationStatus(sbcp::amarsi::MotorDriver::Motor & m, int number);
	/**
	 * Get motor position and status
	 * @param me Magnetic encoder reference
	 * @param pos Reference for result position
	 * @param status Reference for result status
	 */
	void GetPosAndStatus(sbcp::amarsi::MotorDriver::MagneticEncoder & me,
			int & pos, int & status);
	/**
	 * Get reading from magnetic encoder
	 * @param me Magnetic encoder reference
	 * @return Value from encoder
	 */
	int16_t getMEValue(sbcp::amarsi::MotorDriver::MagneticEncoder & me);

	/**
	 * Perform calibration for selected device
	 * @param bus Bus pointer
	 * @param dev Motor driver pointer
	 */
	void calibrateDevice(BusPtr bus, sbcp::amarsi::MotorDriver::Ptr dev);

	/**
	 * Set initial, default values for motors parameters:
	 * - motor control mode - hard coded, SMOOTH_POSTION
	 * - smooth position update - by default 2ms
	 * - maximum torque - by default 500
	 * - maximum speed - by default 31000
	 * - maximum acceleration - by default 31000
	 * - preload - by default 1332
	 * - stiffness - by default 1000
	 * - damping - by default 100
	 * - P gain for PID controller - by default 500
	 * - I gain for PID controller - by default 50
	 * - D gain for PID controller - by default 250
	 * - maximum M1 position - by default 1200
	 * - maximum M2 position - by default 1350
	 * Default values are set for all motors. Modification of parameters are possible by using config file.
	 */

	void initMotors();
	/**
	 * Set initial, default values for servos parameters:
	 * - channel - hard coded: left-fore 15, right-fore 13, left-hind 9, right-hind 11
	 * - maximal position - by default 1
	 * - minimal position - by default -1
	 * - maximal pulse length - by default 65000
	 * - minimal pulse length - by default 55000
	 * Default values are set for all servos. Modification of parameters are possible by using config file.
	 */
	void initServos();

	/**
	 * Perform calibration for all motor drivers, calls calibrateDevice().
	 */
	void calibrate();

	/**
	 * Set control mode for all motors to coast (requires lazy workflow mode).
	 */
	void stopMotors();

	/**
	 * Called on destruction of the object, does the cleaning (kills tasks, frees the resources, etc).
	 */
	void clean_exit();

	/**
	 * Parses the selected config file and updates parameters values. The config file is a text file that has following structure:
	 @code
	 #comments

	 motors = {

	 [<motor_parameter> = <value>;]

	 <leg_template>:
	 {
	 [<motor_parameter> = <value>;]

	 M1:
	 {
	 [<motor_parameter> = <value>;]
	 };

	 M2:
	 {
	 [<motor_parameter> = <value>;]
	 };
	 };

	 };

	 servos = {
	 [<servo_parameter> = <value>];
	 };

	 xenomai = {
	 [<real_time_system_parameter> = <value>];
	 };

	 misc = {
	 [<miscellaneous_parameter> = <value>];
	 };
	 @endcode
	 * Currently there are for main groups for parameters:
	 *	 - motors - parameters for motors
	 *	 - servos - parameters for servos
	 *	 - xenomai - parameters for real time system
	 *	 - misc - miscellaneous parameters
	 *
	 * Each group can contain any number of parameters which are proper conifg parameters (see the list below). Additionally, for motors leg
	 * subgroup can be specified, by adding one from leg templates:
	 *	 - left-fore
	 *	 - right-fore
	 *	 - left-hind
	 *	 - right-hind
	 *
	 * Parameters which are set in leg subgroup are applied for all motors from selected leg and shadow the global parameters from motors group.
	 * Each leg subgroup can also contain subgroup for selected motor:
	 *	- M1 - parameters for hip motor
	 *	- M2 - parameters for knee motor
	 *
	 * This is the most specific level for motor parameter, which is applied to the selected motor from the selected leg. All previous parameter settings are
	 * shadowed.
	 * Currently implemented parameters for all groups:
	 *	 - motors
	 *	 	 -# maxTorque <int_value> - maximum torque
	 *	 	 -# maxSpeed <int_value> - maximum speed
	 *	 	 -# maxAccel <int_value> - maximum acceleration
	 *	 	 -# preload <int_value> - preload
	 *	 	 -# stiffness <int_value> - stiffness
	 *	 	 -# damping <int_value> - damping
	 *	 	 -# pGain <int_value> - P gain value for PID controller
	 *	 	 -# iGain <int_value> - I gain value for PID controller
	 *	 	 -# dGain <int_value> - P gain value for PID controller
	 *	 	 -# posUpdate <int_value> - smooth position update in ms
	 *	 - servos
	 *	 	 -# maxPos <double_value> - maximum position for all servos
	 *	 	 -# minPos <double_value> - minimum position for all servos
	 *	 	 -# maxPulseLength <int_value> - maximal pulse length for all servos
	 *		 -# minPulseLength <int_value> - minimal pulse length for all servos
	 *	 - xenomai
	 *	 	 -# executor_period <int_value> - period of execution for executor task
	 *	 - misc
	 *		 -# queue_size <int_value> - maximal size of both command queue of the robot and the queue which stores the previous trajectory for all joints.
	 *		 In other words, this is number of trajectory samples which are recorded.
	 *
	 * @param filename Name of the text file that contains configuration
	 */
	void parseConfigFile(const char* filename);

	/*
	 * When supervisor task is implemeneted, this function is used to add command to message queue used by it
	 */
	//void pushCmd(OncillaCmd &cmd);
	/*
	 * Function executed by supervisor task
	 */
	//friend void supervisor(void *oncillaPtr);
	/**
	 * Function executed by executor task. Operations performed in the task:
	 * - initialization
	 * 		-# initialize SBCP bus
	 * 		-# set task mode to periodic
	 * 		-# set task mode to primary
	 * - main loop - executed until the parent object is destroyed
	 * 		-# wait the period of execution. If something delays the execution of the task for more than selected period, the runtime
	 * 		exception is thrown
	 * 		-# try to read a command from the message queue. If present, parse and execute action (currently implemented - set new joints positions and
	 * 		reset trajectory record).
	 * 		-# do the transfers over SBCP bus
	 * 		-# read present positions of the joints and store the newest one to the trajectory queue (with corresponding time)
	 *
	 * @param oncillaPtr Pointer to OncillaRobot object, which is calling the execution of the task
	 *
	 */
	friend void executor(void *oncillaPtr);

public:

	/**
	 * Class constructor
	 * @param filename Config file name
	 * @param RT Indicator of context: TRUE - real-time, FALSE - non real-time. When object is constructed and used in another Xenomai task,
	 * this parameter should be set to true, in any other case to false. Setting value to TRUE while executing from non real-time context
	 * could cause undefined behavior (synchronization problems)
	 */
	OncillaRobot(const char* filename = "initial.config", bool RT = false)
			throw (std::runtime_error);
	/**
	 * Destructor, calls clean_exit
	 */
	~OncillaRobot();

	/**
	 * Robot initialization, called in constructor. Initialization includes calibration and setting default values to motors and servos parameters.
	 */
	void init();

	/**
	 * Only for debugging, prints "test" on standard output.
	 */
	void test();

	/**
	 * Adds new command containing new joints positions to command queue.
	 * @param newPos Pointer for valid input double array, which length is 12 elements. Elements meanings:
	 * - newPos[0] - left fore hip joint angle position in radians
	 * - newPos[1] - left fore knee joint position described by bend coefficient
	 * - newPos[2] - right fore hip joint angle position in radians
	 * - newPos[3] - right fore knee joint position described by bend coefficient
	 * - newPos[4] - left hind hip joint angle position in radians
	 * - newPos[5] - left hind knee joint position described by bend coefficient
	 * - newPos[6] - right hind hip joint angle position in radians
	 * - newPos[7] - right hind knee joint position described by bend coefficient
	 * - newPos[8] - left fore servo position
	 * - newPos[9] - right fore servo position
	 * - newPos[10] - left hind servo position
	 * - newPos[11] - right hind servo position
	 */
	void setPos(double *newPos);

	/**
	 * Read last sensors readings for joints positions.
	 * @param res Pointer to valid output double array, which length is 12 elements. Elements meanings:
	 * - res[0] - left fore hip joint angle position in radians
	 * - res[1] - left fore knee joint position described by bend coefficient
	 * - res[2] - right fore hip joint angle position in radians
	 * - res[3] - right fore knee joint position described by bend coefficient
	 * - res[4] - left hind hip joint angle position in radians
	 * - res[5] - left hind knee joint position described by bend coefficient
	 * - res[6] - right hind hip joint angle position in radians
	 * - res[7] - right hind knee joint position described by bend coefficient
	 * - res[8] - left fore servo position
	 * - res[9] - right fore servo position
	 * - res[10] - left hind servo position
	 * - res[11] - right hind servo position
	 */

	void getPos(double *res);
	/**
	 * Get next recorded position from trajectory queue.
	 * @param res Pointer to valid output double array, which length is 13 elements. Elements meanings:
	 * - coords[0] - time in ms
	 * - res[1] - left fore hip joint angle position in radians
	 * - res[2] - left fore knee joint position described by bend coefficient
	 * - res[3] - right fore hip joint angle position in radians
	 * - res[4] - right fore knee joint position described by bend coefficient
	 * - res[5] - left hind hip joint angle position in radians
	 * - res[6] - left hind knee joint position described by bend coefficient
	 * - res[7] - right hind hip joint angle position in radians
	 * - res[8] - right hind knee joint position described by bend coefficient
	 * - res[9] - left fore servo position
	 * - res[10] - right fore servo position
	 * - res[11] - left hind servo position
	 * - res[12] - right hind servo position
	 */
	void getNextTrajectPoint(double *res);

	/**
	 * Resets internal robot timer and clears recorded trejectory. Should be called before executing all sequences of movement, for which
	 * the feedback from sensors is needed.
	 */
	void resetTrajectRec();

	//void pushCmd(oncillaCmd &cmd);

};

#endif	/* ONCILLAROBOT_H */

