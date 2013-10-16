/**
 * @mainpage OncillaServer
 * OncillaServer is an utility that allows to control the Oncilla robot remotely, from stationary machine, using dedicated
 * OncillaClient program (which is written in Java programming language, and easy to integrate with Matlab enviroment). The project contains
 * two main modules - OncillaServer and OncillaRobot.
 * Usage:
 @code
 sudo ./oncillaserver [-p <port_number>]
 @endcode
 * Default port number for OncillaServer is 6789.
 * @file main.cpp
 * @author Stanislaw Maciag maciag@student.agh.edu.pl
 * @brief Main program for OncillaServer utility
 */

#include "OncillaServer.h"
#include "Utils.h"
#include <cstdlib>
#include <ctype.h>
#include <unistd.h>

using namespace std;

/**
 * Signal handler for the main program, terminates it after receiving signal.
 * @param signum Unused parameter, contains numeric symbol of the received signal
 */
void signalHandler(int signum) {
    
    cout << endl;
    cout << "Terminated by user" << endl;
    
    exit(EXIT_SUCCESS); 
    
}

/**
 * Main function, parses arguments and constructs OncillaServer object.
 * @param argc Input arguments count
 * @param argv Input arguments values
 * @return EXIT_SUCCESS for successful execution and termination, EXIT_FAILURE otherwise
 */
int main(int argc, char *argv[]) {

    int port(6789);
    opterr = 0;
    int opt;

    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    while ((opt = getopt(argc, argv, "p:")) != -1) {
        switch (opt) {
            case 'p':
                if (is_number(optarg)) {
                    port = atoi(optarg);
                    if (port < 1024 || port > 65535) {
                        cerr
                                << "Port number must be within range from 1024 to 65535"
                                << endl;
                        exit(EXIT_FAILURE);
                    }
                } else {
                    cerr << "Port number must be a digit" << endl;
                    exit(EXIT_FAILURE);
                }
                break;
            case '?':
                if (optopt == 'p')
                    cerr << "Option -" << (char) optopt << " requires an argument"
                        << endl;
                else
                    cerr << "Unknown option character -" << (char) optopt << endl;
                exit(EXIT_FAILURE);
            default:
                exit(EXIT_FAILURE);
        }
    }

    if (optind < argc) {
        cerr << "Unrecognized options or arguments: ";
        while (optind < argc)
            cerr << argv[optind++] << " ";
        cerr << endl;
        exit(EXIT_FAILURE);
    }


    try {
        cout << "Server started" << endl;
        OncillaServer server(port);

    } catch (runtime_error ex) {
        cerr << ex.what() << endl;
        exit(EXIT_FAILURE);
    }

    cout << "Server is shutting down now" << endl;
    exit(EXIT_SUCCESS);
}
