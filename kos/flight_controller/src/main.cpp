#include "../include/mission.h"
#include "../../shared/include/initialization_interface.h"
#include "../../shared/include/ipc_messages_initialization.h"
#include "../../shared/include/ipc_messages_autopilot_connector.h"
#include "../../shared/include/ipc_messages_credential_manager.h"
#include "../../shared/include/ipc_messages_navigation_system.h"
#include "../../shared/include/ipc_messages_periphery_controller.h"
#include "../../shared/include/ipc_messages_server_connector.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <thread>

#define RETRY_DELAY_SEC 1
#define RETRY_REQUEST_DELAY_SEC 5
#define FLY_ACCEPT_PERIOD_US 500000
#define EARTH_RADIUS 6371000
int32_t CargoPoint = -1;

uint32_t commandNum1 = 0; // тут хранятся количество команд
MissionCommand *commands1 = NULL; // команды
double degreesToRadians(double degrees) // переводит градусы в радианы
    {
        return degrees * M_PI / 180;
    }
double distance(double lat1, double lon1, double lat2, double lon2) // расстояние между двумя точками на земле
    {
        double phi1 = degreesToRadians(lat1);
        double phi2 = degreesToRadians(lat2);
        double deltaPhi = degreesToRadians(lat2 - lat1);
        double deltaLambda = degreesToRadians(lon2 - lon1);

        double a = sin(deltaPhi / 2) * sin(deltaPhi / 2) +cos(phi1) * cos(phi2) *sin(deltaLambda / 2) * sin(deltaLambda / 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));

        return EARTH_RADIUS * c;
    }
struct coridor // коридор от одной точки до другой
{
    CommandWaypoint n1; // первая точка
    CommandWaypoint n2; //вторая



    double distance_to_trajectory(double drone_lat,double drone_lon) // минимальное расстояние от точки до коридора
    {
        double d1 = distance(drone_lat, drone_lon,n1.latitude*1e-7, n1.longitude*1e-7);
        double d2 = distance(drone_lat, drone_lon,n2.latitude*1e-7, n2.longitude*1e-7);
        double d = distance(n1.latitude*1e-7, n1.longitude*1e-7, n2.latitude*1e-7, n2.longitude*1e-7);
        
        double s = (d1 + d2 + d) / 2;
        return (2 * sqrt(s * (s - d) * (s - d1) * (s - d2))) / d; //по Герону
    }

    bool check_coridor(double drone_lat, double drone_lon, int32_t drone_alt, double max_distance, int32_t max_height) // проверка выхода за коридор (ограничение в max_distance метров)
    {
        return distance_to_trajectory(drone_lat,drone_lon)<=max_distance&&(getDeltaH((int32_t)(drone_lat * 1e7), (int32_t)(drone_lon * 1e7), drone_alt)<=max_height);
    }

    // void check_height(int32_t lat, int32_t drone_alt, int32_t max_height)
    // {
    //     if(!(getDeltaH(lat, drone_alt)<=max_height))
    //     changeAltitude(n2.altitude);
    // }

   int32_t getDeltaH(int32_t lat, int32_t lon, int32_t alt){
        if(n1.altitude != n2.altitude){
            double full = distance(n1.latitude*1e-7, n1.longitude*1e-7, n2.latitude*1e-7, n2.longitude*1e-7);
            double passed = distance(lat*1e-7, lon*1e-7, n1.latitude*1e-7, n1.longitude*1e-7);
            double begin = n1.altitude;
            double delta = n2.altitude-begin;
            fprintf(stderr, "full:[%f]  passed:[%f] begin:[%f] delta:[%f]\n", full, passed, begin, delta);
            return alt - (begin + delta*(passed/full));
        }
        fprintf(stderr,"height:[%d] alt:[%d] \n", alt, n2.altitude);
        return alt-n2.altitude;
    }
    
};


coridor *coridors = NULL; // коридоры

int sendSignedMessage(char* method, char* response, char* errorMessage, uint8_t delay) {
    char message[512] = {0};
    char signature[257] = {0};
    char request[1024] = {0};
    snprintf(message, 512, "%s?%s", method, BOARD_ID);

    while (!signMessage(message, signature)) {
        fprintf(stderr, "[%s] Warning: Failed to sign %s message at Credential Manager. Trying again in %ds\n", ENTITY_NAME, errorMessage, delay);
        sleep(delay);
    }
    snprintf(request, 1024, "%s&sig=0x%s", message, signature);

    while (!sendRequest(request, response)) {
        fprintf(stderr, "[%s] Warning: Failed to send %s request through Server Connector. Trying again in %ds\n", ENTITY_NAME, errorMessage, delay);
        sleep(delay);
    }

    uint8_t authenticity = 0;
    while (!checkSignature(response, authenticity) || !authenticity) {
        fprintf(stderr, "[%s] Warning: Failed to check signature of %s response received through Server Connector. Trying again in %ds\n", ENTITY_NAME, errorMessage, delay);
        sleep(delay);
    }

    return 1;
}

void coridor_control(coridor* coridors,int count_coridor)
{
    while(!setCargoLock(0));
    int count=0, numH = 0;
    int32_t x,y,z, znach= -1;
    bool cargoLock = false;
    double passed = 0;
    while(true) 
    {
        if(numH!=0)
        numH++;
        if(numH>=10)
        numH = 0;
        getCoords(x,y,z);
        if(znach== -1 )
        znach = z;
        if(x!=0 and y!=0 and z!=0)// чтобы небыло нулевых координат, когда дрон настравиается 
        {
             //вывод координат
        fprintf(stderr,"»»»»»»>\n");
        fprintf(stderr,"latitude:[%f] longitude:[%f] altitude: [%f]\n",x*1e-7,y*1e-7,z*1e-2);
        //fprintf(stderr,"Altitude: %f\n", coridors[count].getDeltaH(z-commands1[0].content.waypoint.altitude));
        int32_t heigh=z-znach;
        
        // если дрон находится в пределах двух коридоров, текущего или соседнего, то все ок
        if(coridors[count+1].distance_to_trajectory(x*1e-7, y*1e-7)<=coridors[count].distance_to_trajectory(x*1e-7, y*1e-7))
        {
            count++;       
            passed = 0;     
        }

        if(count == CargoPoint && distance(x*1e-7,y*1e-7, coridors[count].n2.latitude*1e-7, coridors[count].n2.longitude*1e-7)<1 && !cargoLock){
            while(!setCargoLock(1));
            cargoLock = true;
        }

        fprintf(stderr,"delta h %d\n", coridors[count].getDeltaH(x, y, heigh));
        fprintf(stderr,"height:[%d] alt:[%d] \n", heigh, coridors[count].n2.altitude);
        bool check_cor = coridors[count].check_coridor(x*1e-7,y*1e-7, heigh, 2.0, 100);

        if(numH==0)
        {
            if( !coridors[count].check_coridor(x*1e-7,y*1e-7, heigh, 1.5, 10) )
            {
                while(!changeAltitude(coridors[count].n2.altitude)) {}
                while(!changeWaypoint(coridors[count].n2.latitude, coridors[count].n2.longitude, coridors[count].n2.altitude)){}
                fprintf(stderr, "changed waypoint[%d][%d][%d]\nsoft error\nchanged height%d\n", coridors[count].n2.latitude, coridors[count].n2.longitude, coridors[count].n2.altitude, coridors[count].n2.altitude);
                numH++;
            }
            
        }
        
        double len = distance(x * 1e-7, y*1e-7, coridors[count].n1.latitude*1e-7, coridors[count].n1.longitude*1e-7);
        passed = (passed > len ? passed : len);
        fprintf(stderr, "%f %f\n", passed, len);
        if(passed - len > 1.5){
            enableBuzzer();
            while(!setKillSwitch(0)){}
            fprintf(stderr, "ne tyda\n");
        }
        

        if((check_cor or coridors[count+1].check_coridor(x*1e-7,y*1e-7, heigh, 2.0, 100) ) and count<count_coridor-1)
        {
            // проверяем, где находится дрон в текущем или соседнем. Сравнение идет по минимальной длине от точки до каждого коридора
            fprintf(stderr,"coridor %d pogreshnost %f Vse good\n",count+1, coridors[count].distance_to_trajectory(x*1e-7,y*1e-7));
            
        }
        else if((check_cor) and count==count_coridor-1)
        {
            fprintf(stderr,"last coridor %d Vse good pogreshnost %f \n",count+1,coridors[count].distance_to_trajectory(x*1e-7,y*1e-7));
        }
        else // если вышел за пределы
        {
            fprintf(stderr," coridor %d Vse ploxo pogreshnost %f  \n",count+1, coridors[count].distance_to_trajectory(x*1e-7,y*1e-7));
            int32_t p=0;
            changeSpeed(0);
            getCoords(x,y,z);
            enableBuzzer();
            int32_t z=setKillSwitch(0);
            break;
            /*
            if(!coridors[count].check_coridor(x*1e-7,y*1e-7,3.0))
              {
                enableBuzzer();
                int32_t z=setKillSwitch(0);
                break;
              }
            while(!changeWaypoint(x,y,700))
            {
              getCoords(x,y,z);
              p++;
              if(p==1 or !coridors[count].check_coridor(x*1e-7,y*1e-7,3.0))
              {
                enableBuzzer();
                int32_t z=setKillSwitch(0);
                break;
              }
            }
            */
        }
        }
    }
}

void speed_control(int vmax)
{
     changeSpeed(vmax);
    int32_t x,y,z,a,b,c;
    getCoords(x,y,z);
        a=x;
        b=y;
        c=z;
        sleep(1);
    while(true)
    {
        getCoords(x,y,z);
        double speed_drone=distance(a*1e-7,b*1e-7,x*1e-7,y*1e-7);
        fprintf(stderr,"speed:%f\n",speed_drone);
        if(speed_drone>vmax)
        {
            changeSpeed(vmax);
        }
        a=x;
        b=y;
        c=z;
        sleep(1);
    }
}
int main(void) {
    //Before do anything, we need to ensure, that other modules are ready to work
    while (!waitForInit("periphery_controller_connection", "PeripheryController")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Periphery Controller. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("autopilot_connector_connection", "AutopilotConnector")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Autopilot Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("navigation_system_connection", "NavigationSystem")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Navigation System. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("server_connector_connection", "ServerConnector")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Server Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("credential_manager_connection", "CredentialManager")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Credential Manager. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    fprintf(stderr, "[%s] Info: Initialization is finished\n", ENTITY_NAME);

    //Enable buzzer to indicate, that all modules has been initialized
    if (!enableBuzzer())
        fprintf(stderr, "[%s] Warning: Failed to enable buzzer at Periphery Controller\n", ENTITY_NAME);
    

    //Copter need to be registered at ORVD
    char authResponse[1024] = {0};
    sendSignedMessage("/api/auth", authResponse, "authentication", RETRY_DELAY_SEC);
    fprintf(stderr, "[%s] Info: Successfully authenticated on the server\n", ENTITY_NAME);

    //Constantly ask server, if mission for the drone is available. Parse it and ensure, that mission is correct
    while (true) {
        char missionResponse[1024] = {0};
        if (sendSignedMessage("/api/fmission_kos", missionResponse, "mission", RETRY_DELAY_SEC) && parseMission(missionResponse)) {
            fprintf(stderr, "[%s] Info: Successfully received mission from the server\n", ENTITY_NAME);
            printMission();
            commands1=com(); // беру команды из mission.cpp
            commandNum1=numcom(); // беру количество команд
            break;
        }
        sleep(RETRY_REQUEST_DELAY_SEC);
    }

    //The drone is ready to arm
    fprintf(stderr, "[%s] Info: Ready to arm\n", ENTITY_NAME);
    while (true) {
        //Wait, until autopilot wants to arm (and fails so, as motors are disabled by default)
        while (!waitForArmRequest()) {
            fprintf(stderr, "[%s] Warning: Failed to receive an arm request from Autopilot Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
            sleep(RETRY_DELAY_SEC);
        }
        fprintf(stderr, "[%s] Info: Received arm request. Notifying the server\n", ENTITY_NAME);

        //When autopilot asked for arm, we need to receive permission from ORVD
        char armRespone[1024] = {0};
        sendSignedMessage("/api/arm", armRespone, "arm", RETRY_DELAY_SEC);

        if (strstr(armRespone, "$Arm: 0#") != NULL) {
            //If arm was permitted, we enable motors
            fprintf(stderr, "[%s] Info: Arm is permitted\n", ENTITY_NAME);
            while (!setKillSwitch(true)) {
                fprintf(stderr, "[%s] Warning: Failed to permit motor usage at Periphery Controller. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
                sleep(RETRY_DELAY_SEC);
            }
            if (!permitArm())
                fprintf(stderr, "[%s] Warning: Failed to permit arm through Autopilot Connector\n", ENTITY_NAME);
            break;
        }
        else if (strstr(armRespone, "$Arm: 1#") != NULL) {
            fprintf(stderr, "[%s] Info: Arm is forbidden\n", ENTITY_NAME);
            if (!forbidArm())
                fprintf(stderr, "[%s] Warning: Failed to forbid arm through Autopilot Connector\n", ENTITY_NAME);
        }
        else
            fprintf(stderr, "[%s] Warning: Failed to parse server response\n", ENTITY_NAME);
        fprintf(stderr, "[%s] Warning: Arm was not allowed. Waiting for another arm request from autopilot\n", ENTITY_NAME);
    };

    //If we get here, the drone is able to arm and start the mission
    //The flight is need to be controlled from now on
    //Also we need to check on ORVD, whether the flight is still allowed or it is need to be paused
    
    // коридор
    int32_t k=0; // количество waypoint 
    for(int i=0;i<commandNum1;i++)
    {
        if((commands1[i].type==WAYPOINT )) // вывод всех путевых точек
        {
            fprintf(stderr," Waypoint  %d latitude:[%d] longitude:[%d] altitude: [%d]\n",k+1,commands1[i].content.waypoint.latitude,commands1[i].content.waypoint.longitude,commands1[i].content.waypoint.altitude);
            k++;
        }
        if((commands1[i].type == HOME)){
            fprintf(stderr," HOME  %d latitude:[%d] longitude:[%d] altitude: [%d]\n",k+1,commands1[i].content.waypoint.latitude,commands1[i].content.waypoint.longitude,commands1[i].content.waypoint.altitude);
            
        }
    }

    int32_t count_coridor=k; //количество коридоров
    coridors = (coridor*)malloc(count_coridor * sizeof(coridor));
    int32_t x0, y0, z0;
    getCoords(x0, y0, z0);
    coridors[0].n1.latitude = x0;
    coridors[0].n1.longitude = y0;
    coridors[0].n1.altitude = z0;
    for(int i = 0; i < commandNum1; i++){
        if(commands1[i].type == WAYPOINT){
            coridors[0].n2.latitude = commands1[i].content.waypoint.latitude;
            coridors[0].n2.longitude = commands1[i].content.waypoint.longitude;
            coridors[0].n2.altitude = commands1[i].content.waypoint.altitude;
            break;
        }
    }
    int check=1;
    for(int i=1,j=0;i<count_coridor,j<commandNum1;j++) {// заполняю массив коридоров
        if((commands1[j].type==WAYPOINT ) and check==1){
           coridors[i].n1.latitude=commands1[j].content.waypoint.latitude;
           coridors[i].n1.longitude=commands1[j].content.waypoint.longitude;
           coridors[i].n1.altitude=commands1[j].content.waypoint.altitude;
           check=0;
        }
        else if((commands1[j].type==WAYPOINT ) and check==0){
           coridors[i].n2.latitude=commands1[j].content.waypoint.latitude;
           coridors[i].n2.longitude=commands1[j].content.waypoint.longitude;
           coridors[i].n2.altitude=commands1[j].content.waypoint.altitude;
           j--;
           i++;
           check=1;
        }
        else if(commands1[j].type == SET_SERVO){
            CargoPoint = i-1;
        }
    }
    for(int i = 0; i < count_coridor; i++){
        CommandWaypoint c1 = coridors[i].n1;
        CommandWaypoint c2 = coridors[i].n2;
        fprintf(stderr, "{[%d] [%d] [%d]}====={[%d] [%d] [%d]}\n", c1.longitude, c1.latitude, c1.altitude, c2.longitude, c2.latitude, c2.altitude);
    }
    int32_t x,y,z; // географические координаты
    int vmax=2;
    /* конец коридора*/
    std::thread cch(coridor_control,coridors,count_coridor);
    std::thread ssc(speed_control,vmax);
    bool isFlighted = true;
    while (true)
    {
        int32_t x,y,z; // географические координаты
        getCoords(x,y,z);
        fprintf(stderr,"latitude:[%f] longitude:[%f] altitude: [%f]\n",x*1e-7,y*1e-7,z*1e-2);

        char armRespone[1024] = {0};
        sendSignedMessage("/api/fly_accept", armRespone, "arm", RETRY_DELAY_SEC);
        fprintf(stderr,">>> [%s] [%d]\n", armRespone, isFlighted);

        if (strstr(armRespone, "$Arm: 1#") != NULL) {
            //fprintf(stderr, "[%s] Info: Arm is forbidden\n", ENTITY_NAME);
            if(isFlighted)
            {
                while (!pauseFlight()) {
                    fprintf(stderr, "[%s] Warning: Failed to permit motor usage at Periphery Controller. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
                    sleep(RETRY_DELAY_SEC);
                }
            isFlighted = false;
            if (!forbidArm())
                fprintf(stderr, "321321[%s] Warning: Failed to permit motor usage at Periphery Controller. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
            }
            
                //fprintf(stderr, "[%s] Warning: Failed to forbid arm through Autopilot Connector\n", ENTITY_NAME);   
        }
        else if (strstr(armRespone, "$Arm: 0#") != NULL) {
            //If arm was permitted, we enable motors
            //fprintf(stderr, "[%s] Info: Arm is permitted\n", ENTITY_NAME);
            if(!isFlighted)
            {
                while (!resumeFlight()) {
                    fprintf(stderr, "[%s] Warning: Failed to permit motor usage at Periphery Controller. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
                    sleep(RETRY_DELAY_SEC);
                }
                isFlighted = true;
                if (!permitArm())
                    fprintf(stderr, "[%s] 123123Warning: Failed to permit motor usage at Periphery Controller. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
            }
               
        }

       sleep(1);
    }
    
    cch.join();
    ssc.detach();
    return EXIT_SUCCESS;
}