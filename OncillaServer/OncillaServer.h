/**
 * @file OncillaServer.h
 * @author Stanislaw Maciag maciag@student.agh.edu.pl
 * @brief TCP/IP server for OncillaRobot
 */

#ifndef ONCILLASERVER_H_
#define ONCILLASERVER_H_

#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <pthread.h>
#include <iostream>
#include <stdexcept>
#include <cstdlib>
#include <deque>
#include "OncillaRobot.h"
#include "Utils.h"
#include "xenomai_task.h"

/**
 * OncillaServer allows user to send commands the from remote host (using dedicated OncillaClient package or another program that
 * sends text messages, which have valid structure, described in OncillaServer::serverMsg). OncillaRobot object is used to control the robot.
 * Received commands are parsed and adequate methods are called to perform desired action. In some cases, reply message is sent back to client.
 * The communication is handled by two threads - OncillaServer::listener_thread and OncillaServer::executor_thread.
 * @todo Make it completely real-time, which means to change pthreads to Xenomai tasks. NOT CHECKED - it is possible that using POSIX socket
 * utilities will cause Xenomai task to go to thr secondary mode. It this case using a real-time networking library (like <a href = http://www.rtnet.org>RTNet</a>) will be necessary.
 */
class OncillaServer {

	/**
	 * Message received and sent by the server
	 */
	class serverMsg {
		/**
		 * OncillaServer communicates have text format. The content of a messages is C style string. The message format is:
		 @code
		 <ID_Char>[ <arg_char><arg_char><arg_char>...]<null>
		 @endcode
		 * ID char has to be one of specified by OncillaServer::Cmd, commands with other IDs are rejected. Message content should be null
		 * terminated (like all C strings). The arguments of the commands are basically string representations of a double numbers. The number
		 * of the arguments is defined for all available commands. Note that message length is dependent on the precision of the arguments, not
		 * on their amount. All currently available commands and their templates are listed below:
		 * - CMD_EXIT<null> - shutdown the server
		 * - CMD_INIT<null> - initialize the robot, by calling constructing OncillaRobot object
		 * - CMD_SHUTDOWN<null> - shutdown the robot, by destroying OncillaRobot object
		 * - CMD_SET_POS <double_arg_chars>{12} <null> - set new position for the joints of the robot (see OncillaRobot::setPos())
		 * - CMD_GET_POS<null> - get the current position of all joints (see OncillaRobot::getPos()), it is sent back to the client
		 * - CMD_PING<null> - test connection, only replies with the same command
		 * - CMD_NEXT_POINT<null> - get the next trajectory point from the queue (see OncillaRobot::getNextTrajectPoint()), it is sent back to the client
		 * - CMD_RESET<null> - reset the robot's internal timer and previously recorded trajectory (see OncillaRobot::resetTrajectRec())
		 */
		char* content;
		/**
		 * Length of the message in chars.
		 */
		int size;

	public:

		/**
		 * Constructor
		 * @param newContent Content of the new message
		 * @param size Size of the new message
		 */
		serverMsg(char newContent[], int size) :
				size(size) {

			content = new char[size + 1];
			strncpy(content, newContent, size);
			content[size] = '\0';
		}

		/**
		 * Copy constructor
		 * @param other Another message
		 */
		serverMsg(const serverMsg& other) {

			size = other.size;
			content = new char[size + 1];
			strncpy(content, other.content, size + 1);

		}

		/**
		 * Assign operator
		 * @param other Other message
		 * @return Referenece to the message identical to the given one
		 */
		serverMsg& operator=(const serverMsg& other) {

			size = other.size;
			content = new char[size + 1];
			strncpy(content, other.content, size + 1);
			return *this;

		}

		/**
		 * Destructor
		 */
		~serverMsg() {

			delete[] content;

		}

		/**
		 * Content getter
		 * @return Content of the message
		 */
		char* getContent() {

			return content;

		}

	};

	/**
	 * Definition of available commands ID (refers to OncillaServer::serverMsg).
	 */
	enum Cmd {
		CMD_EXIT = 0x00,
		CMD_INIT = 0x01,
		CMD_SHUTDOWN = 0x02,
		CMD_SET_POS = 0x04,
		CMD_GET_POS = 0x05,
		CMD_PING = 0x06,
		CMD_GET_NEXT_POINT = 0x09,
		CMD_RESET = 0x10
	};

	/**
	 * Actual length of the message (length of the OncillaServer::serverMsg::content). Should be less or equal to maxial data field length in
	 * the Ethernet frame (1500)
	 */
	static const unsigned int contentLength = 1500;

	int listener_ret, executor_ret;
	/**
	 * Mutex for synchronization of the commands queue
	 */
	pthread_mutex_t lockQueue;
	/**
	 * Server socket descriptor
	 */
	int socket_desc;
	/**
	 * Client socket descriptor
	 */
	int client_sock;
	int c;
	/**
	 * Size of the message received by the server
	 */
	int read_size;
	/**
	 * Address of the server
	 */
	struct sockaddr_in server;
	/**
	 * Address of the client
	 */
	struct sockaddr_in client;
	/**
	 * Message received by the server
	 */
	char client_message[contentLength];
	/**
	 * Message sent by the server
	 */
	char server_reply[contentLength];
	/**
	 * Incoming messages are stored in the queue
	 */
	std::deque<serverMsg> commandQueue;
	/**
	 * Listens for the incoming messages and adds them to the OncillaServer::commandQueue. If the connection is lost call connect() method.
	 */
	pthread_t listener_thread;
	/**
	 * Takes commands from the OncillaServer::commandQueue, parses them and executes appropriate action. Handles all the control over the robot,
	 * using OncillaRobot methods.
	 */
	pthread_t executor_thread;
	/**
	 * State of the connection
	 */
	bool isConnected;
	/**
	 * State of the server - if FALSE, the server is shutting down
	 */
	bool goingDown;
	/**
	 * State of the listener thread
	 */
	bool listener_active;
	/**
	 * Mutex used with OncillaServer::cond to pause the listener thread, when there is no connection
	 */
	pthread_mutex_t lockListener;
	/**
	 * Mutex for synchronization of OncillaServer::goingDown
	 */
	pthread_mutex_t goingDownLock;
	/**
	 * Condition variable used with OncillaServer::lockListener
	 */
	pthread_cond_t cond;
	/**
	 * Handler for the listener(), allows to wrap pthread with the class
	 * @param ptr Argument used by the listener()
	 */
	static void* listenerStart(void * ptr);
	/**
	 * Handler for the executor(), allows to wrap pthread with the class
	 * @param ptr Argument used by the executor()
	 */
	static void* executorStart(void * ptr);
	/**
	 * Function executed by the OncillaServer::listener_thread
	 */
	void *listener(void);
	/**
	 * Function executed by the OncillaServer::executor_thread
	 */
	void *executor(void);

	/**
	 * Pointer to the OncillaRobot object used by the server
	 */
	OncillaRobot *oncilla;

public:
	/**
	 * Main constructor, initializes the connection, waits for the incoming connection from the client, starts OncillaServer::listener_thread
	 * and OncillaServer::executor_thread
	 * @param port Port number for the OncillaServer
	 */
	OncillaServer(int port) throw (std::runtime_error);
	/**
	 * Destructor, calls shutdown()
	 */
	~OncillaServer();
	/**
	 * Blocking method, waiting for the incoming connection from the client
	 */
	void connect() throw (std::runtime_error);
	/**
	 * Connection state getter
	 * @return Connection state
	 */
	bool isConected();
	/**
	 * Send message to the client
	 * @param msg Valid pointer to the char array
	 * @param size Size of the message
	 */
	void send(char* msg, int size);

	/**
	 * Cleaning before finishing work - cancels threads, frees the resources.
	 */
	void shutdown();
};

#endif /* OncillaServer_H_ */
