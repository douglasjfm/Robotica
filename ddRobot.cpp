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
 * Adaptado Pela Equipe 6 2016.1
 * @autor djfm
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>

#define V_REP_IP_ADDRESS "127.0.0.1"
#define V_REP_PORT 19997//1999;

#include "remoteApi/extApi.h"
#include "include/v_repConst.h"
#include "remoteApi/extApiPlatform.h"

#define pi M_PI
/*	#include "extApiCustom.h" if you wanna use custom remote API functions! */

int readingOdo;
int clientID;
pthread_t tid1;

simxInt ddRobotHandle;
simxInt leftMotorHandle;
simxInt rightMotorHandle;
simxInt sensorHandle;
simxInt graphOdometryHandle;
simxInt caminhoHandle;
simxInt pathPlanTaskHandle;
simxInt fimHandle;
simxInt sensorFrontHandle;
simxInt sensorLeftHandle;
simxInt sensorRightHandle;

float vdd = 0.03, rodaRaio = 0.0325, rodasDiff = 0.15;

extern float estado[];

int rotaflag;

float smallestAngleDiff(float target, float source);

void stopOdom();
void startOdom();

void markov_load();
void markov_free();
int markov_move(float dwl, float dwr);
void markov_correct(float distF, float distL, float distR);

float ideal_dist(int s, float x ,float y, int tht);

simxInt getSimTimeMs(int clientID)   //In Miliseconds
{
    return simxGetLastCmdTime(clientID);
}

int readOdometers(int clientID, simxFloat &dwL, simxFloat &dwR)
{
    static bool first = true;
    //old joint angle position
    static simxFloat lwprev=0;
    static simxFloat rwprev=0;

    //current joint angle position
    simxFloat lwcur=0;
    simxFloat rwcur=0;

    if (first)
    {
        simxInt ret = simxGetJointPosition(clientID, leftMotorHandle, &lwprev, simx_opmode_streaming);
        if (ret>0) return -1;

        ret = simxGetJointPosition(clientID, rightMotorHandle, &rwprev, simx_opmode_streaming);
        if (ret>0) return -1;

        dwR = dwL = 0;
        first = false;
    }

    simxInt ret = simxGetJointPosition(clientID, leftMotorHandle, &lwcur, simx_opmode_buffer);
    if (ret>0) return -1;

    ret = simxGetJointPosition(clientID, rightMotorHandle, &rwcur, simx_opmode_buffer);
    if (ret>0) return -1;

    dwL = smallestAngleDiff(lwcur, lwprev);
    dwR = smallestAngleDiff(rwcur, rwprev);

//    dwR = rwcur - rwprev;
//    dwL = lwcur - lwprev;

    if (fabs(dwR)>M_PI || fabs(dwL)>M_PI)
    {
        printf("wL: %f - (%f) = %f\n", lwcur, lwprev, dwL);
        printf("wR: %f - (%f) = %f\n", rwcur, rwprev, dwR);
        lwprev = lwcur;
        rwprev = rwcur;
        return -1;
    }

    lwprev = lwcur;
    rwprev = rwcur;
    return 0;
}

void setTargetSpeed(int clientID, simxFloat phiL, simxFloat phiR)
{
    simxSetJointTargetVelocity(clientID, leftMotorHandle, phiL, simx_opmode_oneshot);
    simxSetJointTargetVelocity(clientID, rightMotorHandle, phiR, simx_opmode_oneshot);
}

char* droppoint (char *p, float *x, float *y)
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
    float phi = trans_get_phi(xyt[0],xyt[1],xyt[2],pxy[0],pxy[1]);
    if (phi>pi)
    {
        phi = phi - 2*pi;
    }
    return phi;
}

int main(int argc, char* argv[])
{
    simxFloat d,v_r,v_l,r_w,v_des,om_des,omega_right,omega_left,phi;
    char strrota[100], *prota;
    char *ipAddr = (char*) V_REP_IP_ADDRESS;
    int portNb = V_REP_PORT;
    float x,y,sx,sy,distp,cam_pos[3];

    clientID = simxStart((simxChar*) (simxChar*) ipAddr, portNb, true, true, 2000, 5);
    //if (clientID < 1) return 0;

    //Get handles for robot parts, actuators and sensores:
    simxGetObjectHandle(clientID, "RobotFrame#", &ddRobotHandle, simx_opmode_oneshot_wait);
    simxGetObjectHandle(clientID, "LeftMotor#", &leftMotorHandle, simx_opmode_oneshot_wait);
    simxGetObjectHandle(clientID, "RightMotor#", &rightMotorHandle, simx_opmode_oneshot_wait);
    simxGetObjectHandle(clientID, "GraphOdometry#", &graphOdometryHandle, simx_opmode_oneshot_wait);
    simxGetObjectHandle(clientID, "ProximitySensorF#", &sensorFrontHandle, simx_opmode_oneshot_wait);
    simxGetObjectHandle(clientID, "ProximitySensorL#", &sensorLeftHandle, simx_opmode_oneshot_wait);
    simxGetObjectHandle(clientID, "ProximitySensorR#", &sensorRightHandle, simx_opmode_oneshot_wait);
    printf("Iniciando conexao com: %s...\n", ipAddr);

    printf("Conexao efetuada\n");

    simxStartSimulation(clientID, simx_opmode_oneshot_wait);
    setTargetSpeed(clientID, 0, 0);
    //strcpy(strrota,"-0,85;-0,275|-0,825;-0,85");
    strcpy(strrota,"-0,85;-0,275|-0,825;-0,85");
    prota = droppoint(strrota,&x,&y);
    printf("goal: %.2f %.2f\n",x,y);
    markov_load();
    startOdom();
    rotaflag = 1;
    for(; rotaflag;)
    {
        //getPosition(clientID,estado);
        //markov_pos(estado);
        sx = estado[0];
        sy = estado[1];
        distp = sqrt((x-sx)*(x-sx) + (y-sy)*(y-sy));
        if (distp < 0.01)
        {
            prota = droppoint(prota,&x,&y);
            if(!prota)
            {
                setTargetSpeed(clientID, 0, 0);
                break;
            }
            printf("goal: %.2f %.2f\n",x,y);
        }
        cam_pos[0] = x;
        cam_pos[1] = y;
        phi = calcphi(estado,cam_pos);
        v_des = vdd;
        om_des=0.5*phi;
        //printf("%.3f %.3f %.3f\n",estado[0],estado[1],estado[2]);
        d=rodasDiff;
        v_r=(v_des-d*om_des);
        v_l=(v_des+d*om_des);
        r_w=rodaRaio; ///wheel radius;
        omega_right = v_r/r_w;
        omega_left = v_l/r_w;
        setTargetSpeed(clientID, omega_left, omega_right);
    }
    stopOdom();
    markov_free();
    //simxPauseSimulation(clientID, simx_opmode_oneshot_wait);
    simxStopSimulation(clientID,simx_opmode_oneshot_wait);
    simxFinish(clientID);
    return 0;
}

simxFloat readSonar(int clientID, simxInt sensorHandle)
{
    simxFloat detectedPoint[3];
    simxUChar detectionState=1;
    simxInt ret = simxReadProximitySensor(clientID, sensorHandle, &detectionState, detectedPoint, NULL, NULL, simx_opmode_buffer);

    //printf("ret: %d ds: %d\n", ret, detectionState);
    if (ret<=0 && detectionState==1)
        return detectedPoint[2];

    return -1;
}

void* odom(void* arg)
{
    int clid = clientID, cells,sfails=0,sok=0;
    simxFloat dFiL, dFiR;
    cells = 0;
    simxFloat detectedPoint[3];
    simxUChar detectionState=1;
    simxReadProximitySensor(clid, sensorFrontHandle, &detectionState, detectedPoint, NULL, NULL, simx_opmode_streaming);
    simxReadProximitySensor(clid, sensorLeftHandle, &detectionState, detectedPoint, NULL, NULL, simx_opmode_streaming);
    simxReadProximitySensor(clid, sensorRightHandle, &detectionState, detectedPoint, NULL, NULL, simx_opmode_streaming);
    while (readingOdo)
    {
        simxFloat distF=-1, distR=-1, distL=-1;
        distF = readSonar(clid, sensorFrontHandle);
        distL = readSonar(clid, sensorLeftHandle);
        distR = readSonar(clid, sensorRightHandle);
        if (cells > 0 && distF > 0.0 && distL > 0.0 && distR > 0.0)
            {markov_correct(distF,distR,distL);sok++;}
        else sfails++;

        readOdometers(clid, dFiL, dFiR);
        cells = markov_move(dFiL,dFiR);

        extApi_sleepMs(2);
    }
    printf("falhas/suc sensores: %d-%d\n",sfails,sok);
    readingOdo = 1;
    return NULL;
}

void startOdom()
{
    pthread_t tid1;
    pthread_attr_t tattr1;
    pthread_attr_init(&tattr1);
    pthread_attr_setdetachstate(&tattr1,PTHREAD_CREATE_DETACHED);
    readingOdo = 1;
    pthread_create(&tid1,&tattr1,odom,NULL);
}

void stopOdom()
{
    readingOdo = 0;
    while(!readingOdo) {}
    readingOdo = 0;
}
