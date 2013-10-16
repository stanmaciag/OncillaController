#include "OncillaServer.h"

OncillaServer *oncillaServerPtr;

OncillaServer::OncillaServer(int port) throw (std::runtime_error) :
isConnected(false), goingDown(false), listener_active(false), oncilla(NULL) {

    oncillaServerPtr = this;

    //mlockall(MCL_CURRENT | MCL_FUTURE);

    //rt_task_shadow(NULL, NULL, 1, T_FPU);

    socket_desc = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_desc == -1) {
        std::runtime_error ex("Could not create socket. Error");
        throw ex;
    }

    // set SO_REUSEADDR on a socket to true (1):
    int optval = 1;
    setsockopt(socket_desc, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof optval);

    // Prepare the sockaddr_in structure
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons(port);

    // Bind
    if (bind(socket_desc, (struct sockaddr *) &server, sizeof (server)) < 0) {
        std::runtime_error ex("Bind failed. Error");
        throw ex;
    }


    // Clearing buffer
    bzero(client_message, contentLength);

    // Initializing mutex lock for threads synchronization
    if (pthread_mutex_init(&lockQueue, NULL) != 0 || pthread_mutex_init(&lockListener, NULL) != 0 || pthread_mutex_init(&goingDownLock, NULL) != 0) {
        std::runtime_error ex("Mutex initialization failed. Error");
        throw ex;
    }

    if (pthread_cond_init(&cond, NULL) != 0) {
        std::runtime_error
        ex("Condition variable initialization failed. Error");
        throw ex;
    }

    connect();

    // Creating and starting separate threads for listening and executing commands

    listener_ret = pthread_create(&listener_thread, 0,
            OncillaServer::listenerStart, this);
    executor_ret = pthread_create(&executor_thread, 0,
            OncillaServer::executorStart, this);

    if (listener_ret > 0 || executor_ret > 0) {
        std::runtime_error ex("Threads creating failed. Error");
        throw ex;
    }

    pthread_join(listener_thread, 0);
    pthread_join(executor_thread, 0);

}

OncillaServer::~OncillaServer() {

    shutdown();

}

void OncillaServer::connect() throw (std::runtime_error) {
    if (!isConnected) {
        // Listen
        listen(socket_desc, 3);

        // Accept and incoming connection
        c = sizeof (struct sockaddr_in);

        // Accept connection from an incoming client
        client_sock = accept(socket_desc, (struct sockaddr *) &client,
                (socklen_t*) & c);
        if (client_sock < 0) {
            std::runtime_error ex("Accept failed. Error");
            throw ex;
        }
        isConnected = true;

        pthread_mutex_lock(&lockListener);
        listener_active = true;
        pthread_cond_signal(&cond);
        pthread_mutex_unlock(&lockListener);
        std::cout << "Client connected" << std::endl;
    }
}

bool OncillaServer::isConected() {
    return isConnected;
}

void * OncillaServer::listener(void) {


    while (!goingDown) {

        pthread_mutex_unlock(&goingDownLock);

        // Pausing thread if no connection
        pthread_mutex_lock(&lockListener);
        while (!listener_active) {
            pthread_cond_wait(&cond, &lockListener);
        }
        pthread_mutex_unlock(&lockListener);
        // Continue execution

        // Receive a command from client
        if ((read_size = recv(client_sock, client_message,
                contentLength, 0)) > 0) {
            // Add message to queue
            pthread_mutex_lock(&lockQueue);
            serverMsg newCommand(client_message, read_size);
            commandQueue.push_back(newCommand);

            pthread_mutex_unlock(&lockQueue);
            bzero(client_message, contentLength);

        }

        if (read_size == 0) {

            isConnected = false;
            close(client_sock);
            pthread_mutex_lock(&lockListener);
            listener_active = false;
            pthread_mutex_unlock(&lockListener);

            if (oncilla != NULL) {

                pthread_mutex_lock(&lockQueue);
                char newContent[1];
                newContent[0] = CMD_SHUTDOWN;
                serverMsg newCommand(newContent, 1);
                commandQueue.push_back(newCommand);
                pthread_mutex_unlock(&lockQueue);

            }

            std::cout << "Client disconnected" << std::endl;

            connect();

        } else if (read_size == -1) {

            isConnected = false;
            close(client_sock);
            pthread_mutex_lock(&lockListener);
            listener_active = false;
            pthread_mutex_unlock(&lockListener);

            if (oncilla != NULL) {

                pthread_mutex_lock(&lockQueue);
                char newContent[1];
                newContent[0] = CMD_SHUTDOWN;
                serverMsg newCommand(newContent, 1);
                commandQueue.push_back(newCommand);
                pthread_mutex_unlock(&lockQueue);

            }

            connect();
        }

        pthread_mutex_lock(&goingDownLock);

    }

    return 0;
}

void OncillaServer::send(char* msg, int size) {
    bzero(server_reply, contentLength);
    strncpy(server_reply, msg, size);
    write(client_sock, server_reply, contentLength);
}

void * OncillaServer::executor(void) {

    while (!goingDown) {

        pthread_mutex_unlock(&goingDownLock);

        if (commandQueue.size() > 0) {

            pthread_mutex_lock(&lockQueue);
            serverMsg newCommand = commandQueue.front();
            commandQueue.pop_front();
            pthread_mutex_unlock(&lockQueue);

            switch (newCommand.getContent()[0]) {

                case CMD_EXIT:

                    pthread_mutex_lock(&goingDownLock);
                    goingDown = true;
                    pthread_cancel(listener_thread);
                    pthread_mutex_unlock(&goingDownLock);

                    break;

                case CMD_INIT:
                    if (oncilla == NULL) {
                        oncilla = new OncillaRobot();
                        char reply[1];
                        reply[0] = CMD_INIT;
                        send(reply, 1);
                    }
                    break;

                case CMD_SHUTDOWN:
                    if (oncilla != NULL) {
                        delete oncilla;
                        oncilla = NULL;
                    }
                    break;

                case CMD_SET_POS:
                    if (oncilla != NULL) {

                        double newPos[12];
                        atoda(newPos, newCommand.getContent() + 2, ' ');
                        oncilla->setPos(newPos);

                    }
                    break;

                case CMD_GET_POS:
                    if (oncilla != NULL) {

                        double currentPos[12];
                        oncilla->getPos(currentPos);
                        char reply[contentLength];
                        int currentIdx = 0;

                        for (int i = 0; i < 12; i++) {

                            currentIdx += dtoa(currentPos[i], reply + currentIdx);
                            reply[currentIdx] = ' ';
                            currentIdx++;

                        }

                        currentIdx--;

                        send(reply, currentIdx);

                    }
                    break;

                case CMD_GET_NEXT_POINT:
                    if (oncilla != NULL) {

                        double currentPos[13];
                        oncilla->getNextTrajectPoint(currentPos);
                        char reply[contentLength];
                        int currentIdx = 0;

                        for (int i = 0; i < 13; i++) {

                            currentIdx += dtoa(currentPos[i], reply + currentIdx);
                            reply[currentIdx] = ' ';
                            currentIdx++;

                        }

                        currentIdx--;

                        send(reply, currentIdx);

                    }
                    break;

                case CMD_PING:
                    char reply[1];
                    reply[0] = CMD_PING;
                    send(reply, 1);
                    break;

                case CMD_RESET:
                    if (oncilla != NULL) {     
                        oncilla->resetTrajectRec();
                    }
                    break;
                    
                default:
                    //send(command);
                    break;

            }

        }
        pthread_mutex_lock(&goingDownLock);

    }

    pthread_mutex_unlock(&goingDownLock);

    if (oncilla != NULL) {
        delete oncilla;
        oncilla = NULL;
    }

    return 0;
}

void* OncillaServer::executorStart(void * arg) {
    return ((OncillaServer*) arg)->executor();
}

void* OncillaServer::listenerStart(void * arg) {
    return ((OncillaServer*) arg)->listener();
}

void OncillaServer::shutdown() {

    pthread_mutex_lock(&goingDownLock);
    goingDown = true;
    pthread_mutex_unlock(&goingDownLock);

    pthread_cancel(executor_thread);
    pthread_cancel(listener_thread);

    pthread_mutex_destroy(&lockQueue);
    pthread_mutex_destroy(&lockListener);
    pthread_cond_destroy(&cond);
    close(client_sock);

    if (oncilla != NULL) {
        delete oncilla;
        oncilla = NULL;
    }


}
