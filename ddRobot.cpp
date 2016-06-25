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
#include <string.h>
#include <unistd.h>
#include <time.h>

#define V_REP_IP_ADDRESS "127.0.0.1"
#define V_REP_PORT 19997//1999;

#include "remoteApi/extApi.h"
#include "include/v_repConst.h"
#include "remoteApi/extApiPlatform.h"
/*	#include "extApiCustom.h" if you wanna use custom remote API functions! */

int clientID;
simxInt ddRobotHandle;
simxInt leftMotorHandle;
simxInt rightMotorHandle;
simxInt sensorHandle;
simxInt graphOdometryHandle;
simxInt caminhoHandle;
simxInt pathPlanTaskHandle;
simxInt fimHandle;

float pose0[] = {1.25,0.25,90.0};
simxFloat vdd = 0.04;

void markov_load();
void markov_free();
float ideal_dist(int s, float x ,float y, int tht);

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

simxUChar* droppoint (simxUChar *p, float *x, float *y, float *z)
{
    int i=0;
    if (p[i] == '\0') return NULL;
    while(p[i] != '|' && p[i] != '\0')
    {
        if (p[i] == ',')
            p[i] = '.';
        i++;
    }
    p[i] = '\0';
    sscanf((const char*)p,"%f;%f",x,y);
    i++;
    p = p+i;
    return p;
}

float trans_get_phi (float x, float y, float t, float x1, float y1);

float calcphi (float xyt[3], float pxy[3])
{
    return trans_get_phi(xyt[0],xyt[1],xyt[2],pxy[0],pxy[1]);
}

int main(int argc, char* argv[])
{
    simxFloat dis,d,v_r,v_l,r_w,v_des,om_des,omega_right,omega_left,phi;
    char *ipAddr = (char*) V_REP_IP_ADDRESS;
    int portNb = V_REP_PORT,c=0;
    float tt;
    float cam_pos[] = {-1.0,1.5,0.0}, pose[3];
    clock_t t0;
    simxFloat lwcur,rwcur;

    clientID = simxStart((simxChar*) (simxChar*) ipAddr, portNb, true, true, 2000, 5);

    //Get handles for robot parts, actuators and sensores:
    simxGetObjectHandle(clientID, "RobotFrame#", &ddRobotHandle, simx_opmode_oneshot_wait);
    simxGetObjectHandle(clientID, "LeftMotor#", &leftMotorHandle, simx_opmode_oneshot_wait);
    simxGetObjectHandle(clientID, "RightMotor#", &rightMotorHandle, simx_opmode_oneshot_wait);
    simxGetObjectHandle(clientID, "GraphOdometry#", &graphOdometryHandle, simx_opmode_oneshot_wait);
    printf("Iniciando conexao com: %s...\n", ipAddr);

    printf("Conexao efetuada\n");

    int ret = simxStartSimulation(clientID, simx_opmode_oneshot_wait);
    t0 = clock();
    c=0;
    pose[0] = pose0[0];
    pose[1] = pose0[1];
    pose[2] = pose0[2];
    for(;;)
    {
        tt = CLOCKS_PER_SEC;
        tt = (clock() - t0)/tt;
        tt = tt * 1000;
        //printf("%f\n",tt);
        if(tt > 1000.0)
        {
            t0 = clock();
            simxGetJointPosition(clientID, leftMotorHandle, &lwcur, simx_opmode_oneshot);
            simxGetJointPosition(clientID, rightMotorHandle, &rwcur, simx_opmode_oneshot);
            printf("%f %f\n%d %d\n",lwcur,rwcur,leftMotorHandle,rightMotorHandle);
            c++;
        }
//    setTargetSpeed(clientID,0.1,0.1);
        if(c == 10)
            break;

        phi = atan2(cam_pos[1] - pose[1],cam_pos[0] - pose[0]);//calcphi(pose0,cam_pos);
        v_des = vdd;
        om_des=0.8*phi;
        d=0.20;
        v_r=(v_des+d*om_des);
        v_l=(v_des-d*om_des);
        r_w=0.02; ///wheel radius;
        omega_right = v_r/r_w;
        omega_left = v_l/r_w;
        setTargetSpeed(clientID, omega_left, omega_right);
    }
    setTargetSpeed(clientID, 0, 0);
    //simxPauseSimulation(clientID, simx_opmode_oneshot_wait);
    simxStopSimulation(clientID,simx_opmode_oneshot_wait);
    simxFinish(clientID);
    return 0;
}
