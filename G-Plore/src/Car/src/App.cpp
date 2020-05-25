
#include "App.h"

Application::Application(int argc, char **argv)
        : userNode(argc, argv)
{
    start();

    vehicleThread = new boost::thread(boost::bind(&Application::processPendingDatagramsVehicle, this));


    userNode.setVehicleCallBack(std::bind(&Application::vehicleSendUdp, this));

    userNode.init();
}

Application::~Application()
{
    vehicleUdpSocket->disconnect();
    vehicleThread->join();

}

void Application::start(void) {

  
    vehiclePort = 20001;

    vehicleUdpSocket= new UDPSocket(vehiclePort);

}

void Application::processPendingDatagramsVehicle( void){
    int iLen;
    uint8_t vehicle_buf[1024];

    std::cout << "hai_luo vehicle "<< std::endl;

    while(true){
        iLen = vehicleUdpSocket->recvFrom(vehicle_buf, VEHICLE_INFO_LENGHT, vehicleUrl, vehiclePort);
        
        if((iLen == VEHICLE_INFO_LENGHT) &&(userNode.getReady()))
        {
            memcpy(vehicle_rx_msg,vehicle_buf,VEHICLE_INFO_LENGHT);
                        userNode.publishVehicle((void*) vehicle_rx_msg);
                        std::cout << ".";
        }
    }
}



void Application::vehicleSendUdp(){
    uint8_t msgLength;
    msgLength = userNode.getVehicleUdpMessage((void*)vehicle_tx_cmd_msg);
    vehicleUdpSocket->sendTo((void*)vehicle_tx_cmd_msg, msgLength, vehicleUrl, vehiclePort);
}


