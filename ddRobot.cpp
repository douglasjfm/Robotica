/**
 * Robô de direção diferencial
 * Disciplina de Robótica CIn/UFPE
 *
 * @autor Prof. Hansenclever Bassani
 *
 * Este código é proporcionado para facilitar os passos iniciais da programação.
 * Porém, não há garantia de seu correto funcionamento.
 *
 * Testado em: Ubuntu 14.04 + Netbeans
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <unistd.h>

#define V_REP_IP_ADDRESS "127.0.0.1"
#define V_REP_PORT 19997//1999;

#include "remoteApi/extApi.h"
#include "include/v_repConst.h"
#include "remoteApi/extApiPlatform.h"
/*	#include "extApiCustom.h" if you wanna use custom remote API functions! */


simxInt ddRobotHandle;
simxInt leftMotorHandle;
simxInt rightMotorHandle;
simxInt sensorHandle;
simxInt graphOdometryHandle;
simxInt caminhoHandle;
simxInt pathPlanTaskHandle;
simxInt fimHandle;

void getPosition(int clientID, simxFloat pos[])   //[x,y,theta]
{

    simxInt ret = simxGetObjectPosition(clientID, ddRobotHandle, -1, pos, simx_opmode_oneshot_wait);
    if (ret > 0)
    {
        printf("Error reading robot position\n");
        return;
    }

    simxFloat orientation[3];
    ret = simxGetObjectOrientation(clientID, ddRobotHandle, -1, orientation, simx_opmode_oneshot_wait);
    if (ret > 0)
    {
        printf("Error reading robot orientation\n");
        return;
    }

    simxFloat theta = orientation[2];
    pos[2] = theta;
}

simxInt getSimTimeMs(int clientID)   //In Miliseconds
{
    return simxGetLastCmdTime(clientID);
}

float to_positive_angle(float angle)
{

    angle = fmod(angle, 2 * M_PI);
    while (angle < 0)
    {
        angle = angle + 2 * M_PI;
    }
    return angle;
}

float smallestAngleDiff(float target, float source)
{
    float a;
    a = to_positive_angle(target) - to_positive_angle(source);

    if (a > M_PI)
    {
        a = a - 2 * M_PI;
    }
    else if (a < -M_PI)
    {
        a = a + 2 * M_PI;
    }
    return a;
}

void readOdometers(int clientID, simxFloat &dPhiL, simxFloat &dPhiR)
{
    //old joint angle position
    static simxFloat lwprev=0;
    static simxFloat rwprev=0;

    //current joint angle position
    simxFloat lwcur=0;
    simxFloat rwcur=0;

    simxGetJointPosition(clientID, leftMotorHandle, &lwcur, simx_opmode_oneshot);
    simxGetJointPosition(clientID, rightMotorHandle, &rwcur, simx_opmode_oneshot);

    dPhiL = smallestAngleDiff(lwcur, lwprev);
    dPhiR = smallestAngleDiff(rwcur, rwprev);
    lwprev = lwcur;
    rwprev = rwcur;
}

void setTargetSpeed(int clientID, simxFloat phiL, simxFloat phiR)
{
    simxSetJointTargetVelocity(clientID, leftMotorHandle, phiL, simx_opmode_oneshot);
    simxSetJointTargetVelocity(clientID, rightMotorHandle, phiR, simx_opmode_oneshot);
}

inline double to_deg(double radians)
{
    return radians * (180.0 / M_PI);
}

int main(int argc, char* argv[])
{

    char *ipAddr = (char*) V_REP_IP_ADDRESS;
    int portNb = V_REP_PORT;
    float goal[3];

    if (argc > 1)
    {
        ipAddr = argv[1];
    }

    printf("Iniciando conexao com: %s...\n", ipAddr);

    int clientID = simxStart((simxChar*) (simxChar*) ipAddr, portNb, true, true, 2000, 5);
    if (clientID != -1)
    {
        printf("Conexao efetuada\n");

        //Get handles for robot parts, actuators and sensores:
        simxGetObjectHandle(clientID, "RobotFrame#", &ddRobotHandle, simx_opmode_oneshot_wait);
        simxGetObjectHandle(clientID, "LeftMotor#", &leftMotorHandle, simx_opmode_oneshot_wait);
        simxGetObjectHandle(clientID, "RightMotor#", &rightMotorHandle, simx_opmode_oneshot_wait);
        simxGetObjectHandle(clientID, "Graph#", &graphOdometryHandle, simx_opmode_oneshot_wait);
        simxGetObjectHandle(clientID, "Caminho#", &caminhoHandle, simx_opmode_oneshot_wait);
        simxGetObjectHandle(clientID, "Fim#", &fimHandle, simx_opmode_oneshot_wait);

        printf("RobotFrame: %d\n", ddRobotHandle);
        printf("LeftMotor: %d\n", leftMotorHandle);
        printf("RightMotor: %d\n", rightMotorHandle);
        printf("GraphOdometry: %d\n", graphOdometryHandle);
        printf("Caminho: %d\n", caminhoHandle);
        printf("Fim: %d\n", fimHandle);
        //simxReadStringStream()
        //start simulation

//        printf("Digite Goal x: ");
//        scanf("%f",&goal[0]);
//        printf("Digite Goal y: ");
//        scanf("%f",&goal[1]);
//        printf("Digite Goal theta: ");
//        scanf("%f",&goal[2]);

        simxSetObjectPosition(clientID,fimHandle,-1,goal,simx_opmode_oneshot_wait);

        int ret = simxStartSimulation(clientID, simx_opmode_oneshot_wait);

        if (ret==-1)
        {
            printf("Não foi possível iniciar a simulação.\n");
            return -1;
        }

        printf("Simulação iniciada.\n");

        //While is connected:
        while (simxGetConnectionId(clientID) != -1)
        {

            //Read current position:
            simxFloat pos[3]; //[x,y,theta] in [cm cm rad]
            getPosition(clientID, pos);

            //Read simulation time of the last command:
            simxInt time = getSimTimeMs(clientID); //Simulation time in ms or 0 if sim is not running
            //stop the loop if simulation is has been stopped:
            if (time == 0) break;
            printf("Posicao: [%.2f %.2f %.2fº], time: %dms\n", pos[0], pos[1], to_deg(pos[2]), time);

            //Read current wheels angle variation:
            simxFloat dPhiL, dPhiR; //rad
            readOdometers(clientID, dPhiL, dPhiR);
            printf("dPhiL: %.2f dPhiR: %.2f\n", dPhiL, dPhiR);

            //Set new target speeds: robot going in a circle:
            simxFloat phiL = 5; //rad/s
            simxFloat phiR = 20; //rad/s
            setTargetSpeed(clientID, phiL, phiR);

            //Let some time for V-REP do its work:
            extApi_sleepMs(2);
        }

        //Stop the robot and disconnect from V-Rep;
        setTargetSpeed(clientID, 0, 0);
        simxPauseSimulation(clientID, simx_opmode_oneshot_wait);
        simxFinish(clientID);

    }
    else
    {
        printf("Nao foi possivel conectar.\n");
        return -2;
    }

    return 0;
}


