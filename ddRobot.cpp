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

float vdd = 0.002, rodaRaio = 0.0325, rodasDiff = 0.15;

extern float estado[];

int rotaflag;


void stopOdom();
void startOdom();

void markov_load();
void markov_free();
void markov_move(float dl, float dr);

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
    static simxFloat lwprev=0.0;
    static simxFloat rwprev=0.0;

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
    return trans_get_phi(xyt[0],xyt[1],xyt[2],pxy[0],pxy[1]);
}

int main(int argc, char* argv[])
{
    simxFloat dis,d,v_r,v_l,r_w,v_des,om_des,omega_right,omega_left,phi;
    char strrota[100], *prota;
    char *ipAddr = (char*) V_REP_IP_ADDRESS;
    int portNb = V_REP_PORT,c=0;
    float tt,x,y,sx,sy,distp,cam_pos[3];
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
    strcpy(strrota,"-0,4;0,4");
    prota = droppoint(strrota,&x,&y);
    printf("goal: %.2f %.2f\n",x,y);
    //markov_load();
    startOdom();
    rotaflag = 1;
    for(;rotaflag;)
    {
        distp = sqrt((x-sx)*(x-sx) + (y-sy)*(y-sy));
        if (distp < 0.1)
        {
            prota = droppoint(prota,&x,&y);
            if(!prota)
            {
                setTargetSpeed(clientID, 0, 0);
                break;
            }
        }
        //getPosition(clientID,estado);
        sx = estado[0];
        sy = estado[1];
        cam_pos[0] = x;
        cam_pos[1] = y;
        phi = -calcphi(estado,cam_pos);//atan2(y-sy,x-sx);
        v_des = 0.02;
        om_des=0.2*phi;
        d=rodasDiff;
        v_r=(v_des-d*om_des);
        v_l=(v_des+d*om_des);
        r_w=rodaRaio; ///wheel radius;
        omega_right = v_r/r_w;
        omega_left = v_l/r_w;
        setTargetSpeed(clientID, omega_left, omega_right);
    }
    stopOdom();
    //markov_free();
    //simxPauseSimulation(clientID, simx_opmode_oneshot_wait);
    simxStopSimulation(clientID,simx_opmode_oneshot_wait);
    simxFinish(clientID);
    return 0;
}

void* odom(void* arg)
{
    time_t t0;
    float tt, lcm15 = 0.0, rcm15 = 0.0;
    int clid = clientID;
    simxFloat dFiL, dFiR;
    t0 = clock();
    while (readingOdo)
    {
        //tt = CLOCKS_PER_SEC;
        //tt = (clock() - t0)/tt;
        //tt = tt * 1000;
        //if (tt > 100.0)
        //{
            //t0 = clock();
            readOdometers(clid, dFiL, dFiR);
            lcm15 += dFiL * rodaRaio;
            rcm15 += dFiR * rodaRaio;
            if (lcm15 > 0.05 || rcm15 > 0.05)
            {
                //printf("%.3f %.3f\n",lcm15,rcm15);
                markov_move(lcm15,rcm15);
                lcm15 = rcm15 = 0.0;
                //fflush(stdout);
                if (lcm15 > 1.0 || rcm15 > 1.0)
                    rotaflag = 0;
            }
        //}
    }
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
    while(!readingOdo){}
    readingOdo = 0;
}
