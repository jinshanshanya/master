//
// Created by wss on 7/1/17.
//

#ifndef PROJECT_MONITOR_WINDOWS_H
#define PROJECT_MONITOR_WINDOWS_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <cstdlib>            // For atoi()
#include <pthread.h>          // For POSIX threads
#include <boost/thread/thread.hpp>
#include <arpa/inet.h>

#include "UDPSocket.h"
#include "Node.h"


class Application
{

public:
    Application(int argc, char** argv);
    ~Application();

private:
    void start(void);

    void processPendingDatagramsIns( void);

    void vehicleSendUdp();

private:
    UserNode userNode;
    boost::thread *insThread;
    boost::thread *vehicleThread;

    UDPSocket *insUdpSocket;


    string insUrl = "192.168.43.106";

    uint16_t insPort;
//    uint16_t vehiclePort;

    uint8_t ins_rx_msg[INS_INFO_LENGTH];

};
#endif
