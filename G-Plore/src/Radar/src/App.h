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

    void processPendingDatagramsRadarF(void);
    void processPendingDatagramsRadarB(void);

    void processPendingDatagramsRadarFL(void);
    void processPendingDatagramsRadarFR(void);

    void processPendingDatagramsRadarBL(void);
    void processPendingDatagramsRadarBR(void);

    void RadarFSendUdp();


private:
    UserNode userNode;


    boost::thread *RadarF_Thread;
    boost::thread *RadarB_Thread;

    boost::thread *RadarFL_Thread;
    boost::thread *RadarFR_Thread;

    boost::thread *RadarBL_Thread;
    boost::thread *RadarBR_Thread;

    UDPSocket *RadarF_Socket;
    UDPSocket *RadarB_Socket;

    UDPSocket *RadarFL_Socket;
    UDPSocket *RadarFR_Socket;

    UDPSocket *RadarBL_Socket;
    UDPSocket *RadarBR_Socket;

    string RadarUrl = "192.168.43.104";

    uint16_t RadarF_Port;
    uint16_t RadarB_Port;

    uint16_t RadarFL_Port;
    uint16_t RadarFR_Port;

    uint16_t RadarBL_Port;
    uint16_t RadarBR_Port;



    uint8_t RadarF_msg[RADARF_INFO_LENGHT];
    uint8_t RadarB_msg[RADARF_INFO_LENGHT];

    uint8_t RadarFL_msg[RADARF_INFO_LENGHT];
    uint8_t RadarFR_msg[RADARF_INFO_LENGHT];

    uint8_t RadarBL_msg[RADARF_INFO_LENGHT];
    uint8_t RadarBR_msg[RADARF_INFO_LENGHT];

    uint8_t RadarF_tx_cmd_msg[RADARF_CMD_LENGHT];


    uint8_t vehicle_tx_cmd_msg[VEHICLE_CMD_LENGHT];
};
#endif
