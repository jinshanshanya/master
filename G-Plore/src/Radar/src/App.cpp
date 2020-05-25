
#include "App.h"

Application::Application(int argc, char **argv)
        : userNode(argc, argv)
{
    start();

    //front Radar
    RadarF_Thread = new boost::thread(boost::bind(&Application::processPendingDatagramsRadarF, this));
    //back Radar
    RadarB_Thread = new boost::thread(boost::bind(&Application::processPendingDatagramsRadarB, this));

    RadarFL_Thread = new boost::thread(boost::bind(&Application::processPendingDatagramsRadarFL, this));
    RadarFR_Thread = new boost::thread(boost::bind(&Application::processPendingDatagramsRadarFR, this));

    RadarBL_Thread = new boost::thread(boost::bind(&Application::processPendingDatagramsRadarBL, this));
    RadarBR_Thread = new boost::thread(boost::bind(&Application::processPendingDatagramsRadarBR, this));


    userNode.setRadarFCallBack(std::bind(&Application::RadarFSendUdp, this));

    userNode.init();
}

Application::~Application()
{
     RadarF_Socket->disconnect();
     RadarB_Socket->disconnect();

     RadarFL_Socket->disconnect();
     RadarFR_Socket->disconnect();

     RadarBL_Socket->disconnect();
     RadarBR_Socket->disconnect();

     RadarF_Thread->join();
     RadarB_Thread->join();

     RadarFL_Thread->join();
     RadarFR_Thread->join();

     RadarBL_Thread->join();
     RadarBR_Thread->join();

}

void Application::start(void) {

    RadarF_Port=20004;
    RadarF_Socket=new UDPSocket(RadarF_Port);
    RadarB_Port=20005;
    RadarB_Socket=new UDPSocket(RadarB_Port);

    RadarFL_Port=20006;
    RadarFL_Socket=new UDPSocket(RadarFL_Port);
    RadarFR_Port=20007;
    RadarFR_Socket=new UDPSocket(RadarFR_Port);

    RadarBL_Port=20008;
    RadarBL_Socket=new UDPSocket(RadarBL_Port);
    RadarBR_Port=20009;
    RadarBR_Socket=new UDPSocket(RadarBR_Port);

}

void Application::processPendingDatagramsRadarF( void){
    uint16_t iLen;
    uint8_t RadarF_buf[1300];

    std::cout << "RadarF Thread"<< std::endl;

    while(true){
        iLen = RadarF_Socket->recvFrom(RadarF_buf, RADARF_INFO_LENGHT, RadarUrl, RadarF_Port);


        //if((0xAA == vehicle_buf[0]) && (0x55 == vehicle_buf[1]) &&(iLen == VEHICLE_INFO_LENGHT) &&(userNode.getReady()))
        if((iLen == RADARF_INFO_LENGHT) &&(userNode.getReady()))
        {
            memcpy(RadarF_msg,RadarF_buf,RADARF_INFO_LENGHT);
                        userNode.publishRadarF((void*) RadarF_msg);
                        std::cout << ".";
        }
    }
}

void Application::processPendingDatagramsRadarB( void){
    uint16_t iLen;
    uint8_t RadarB_buf[1300];

    std::cout << "RadarB Thread"<< std::endl;

    while(true){
        iLen = RadarB_Socket->recvFrom(RadarB_buf, RADARF_INFO_LENGHT, RadarUrl, RadarB_Port);


        //if((0xAA == vehicle_buf[0]) && (0x55 == vehicle_buf[1]) &&(iLen == VEHICLE_INFO_LENGHT) &&(userNode.getReady()))
        if((iLen == RADARF_INFO_LENGHT) &&(userNode.getReady()))
        {
            memcpy(RadarB_msg,RadarB_buf,RADARF_INFO_LENGHT);
                        userNode.publishRadarB((void*) RadarB_msg);
                        std::cout << "*";
        }
    }
}


void Application::processPendingDatagramsRadarFL( void){
    uint16_t iLen;
    uint8_t RadarFL_buf[1300];

    std::cout << "RadarFL Thread"<< std::endl;

    while(true){
        iLen = RadarFL_Socket->recvFrom(RadarFL_buf, RADARF_INFO_LENGHT, RadarUrl, RadarFL_Port);


        //if((0xAA == vehicle_buf[0]) && (0x55 == vehicle_buf[1]) &&(iLen == VEHICLE_INFO_LENGHT) &&(userNode.getReady()))
        if((iLen == RADARF_INFO_LENGHT) &&(userNode.getReady()))
        {
            memcpy(RadarFL_msg,RadarFL_buf,RADARF_INFO_LENGHT);
                        userNode.publishRadarFL((void*) RadarFL_msg);
                        std::cout << "!";
        }
    }
}

void Application::processPendingDatagramsRadarFR( void){
    uint16_t iLen;
    uint8_t RadarFR_buf[1300];

    std::cout << "RadarFR Thread"<< std::endl;

    while(true){
        iLen = RadarFR_Socket->recvFrom(RadarFR_buf, RADARF_INFO_LENGHT, RadarUrl, RadarFR_Port);


        //if((0xAA == vehicle_buf[0]) && (0x55 == vehicle_buf[1]) &&(iLen == VEHICLE_INFO_LENGHT) &&(userNode.getReady()))
        if((iLen == RADARF_INFO_LENGHT) &&(userNode.getReady()))
        {
            memcpy(RadarFR_msg,RadarFR_buf,RADARF_INFO_LENGHT);
                        userNode.publishRadarFR((void*) RadarFR_msg);
                        std::cout << "&";
        }
    }
}

void Application::processPendingDatagramsRadarBL( void){
    uint16_t iLen;
    uint8_t RadarBL_buf[1300];

    std::cout << "RadarBL Thread"<< std::endl;

    while(true){
        iLen = RadarBL_Socket->recvFrom(RadarBL_buf, RADARF_INFO_LENGHT, RadarUrl, RadarBL_Port);


        //if((0xAA == vehicle_buf[0]) && (0x55 == vehicle_buf[1]) &&(iLen == VEHICLE_INFO_LENGHT) &&(userNode.getReady()))
        if((iLen == RADARF_INFO_LENGHT) &&(userNode.getReady()))
        {
            memcpy(RadarBL_msg,RadarBL_buf,RADARF_INFO_LENGHT);
                        userNode.publishRadarBL((void*) RadarBL_msg);
                        std::cout << "$";
        }
    }
}

void Application::processPendingDatagramsRadarBR( void){
    uint16_t iLen;
    uint8_t RadarBR_buf[1300];

    std::cout << "RadarBR Thread"<< std::endl;

    while(true){
        iLen = RadarBR_Socket->recvFrom(RadarBR_buf, RADARF_INFO_LENGHT, RadarUrl, RadarBR_Port);


        //if((0xAA == vehicle_buf[0]) && (0x55 == vehicle_buf[1]) &&(iLen == VEHICLE_INFO_LENGHT) &&(userNode.getReady()))
        if((iLen == RADARF_INFO_LENGHT) &&(userNode.getReady()))
        {
            memcpy(RadarBR_msg,RadarBR_buf,RADARF_INFO_LENGHT);
                        userNode.publishRadarBR((void*) RadarBR_msg);
                        std::cout << "%";
        }
    }
}

void Application::RadarFSendUdp()
{
    uint8_t msgLength;
    msgLength = userNode.getRadarFUdpMessage((void*)RadarF_tx_cmd_msg);
    RadarF_Socket->sendTo((void*)RadarF_tx_cmd_msg, msgLength, RadarUrl, RadarFL_Port);
}

