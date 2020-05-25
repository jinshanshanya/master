
#include "App.h"

Application::Application(int argc, char **argv)
        : userNode(argc, argv)
{
    start();

    insThread = new boost::thread(boost::bind(&Application::processPendingDatagramsIns, this));

    userNode.init();
}

Application::~Application()
{

    insUdpSocket->disconnect();
    insThread->join();
}

void Application::start(void) {

    insPort = 20002;
    insUdpSocket = new UDPSocket(insPort);
}


void Application::processPendingDatagramsIns( void) {
    int iLen;
    uint8_t ins_buf[1024];

    std::cout << "IFSimu "<< std::endl;

    while (true) {
        iLen = insUdpSocket->recvFrom(ins_buf, INS_INFO_LENGTH, insUrl, insPort);

        if ((iLen == INS_INFO_LENGTH) &&(userNode.getReady())) {
            memcpy(ins_rx_msg, ins_buf, INS_INFO_LENGTH);
			userNode.publishIns((void*) ins_rx_msg);
        }
    }
}


