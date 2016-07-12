/**
 * Robô de direção diferencial
 * Disciplina de Robótica CIn/UFPE
 *
 * Pela Equipe 6 2016.1
 * @autor djfm
 *
 */

#include <iostream>
#include <signal.h>
#include <sstream>

#include <wiringPi.h>
#include "robotAPI/Pins.h"
#include "robotAPI/Sonar.h"
#include "robotAPI/Motor.h"
#include "robotAPI/Encoder.h"
#include "robotAPI/KBAsync.h"
#include "robotAPI/Odometry.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>

int readingOdo;
int clientID;
pthread_t tid1;

float vdd = 0.07, rodaRaio = 0.0325, rodasDiff = 0.15;

extern float estado[];

int rotaflag;

float smallestAngleDiff(float target, float source);

void stopOdom();
void startOdom();

void markov_load();
void markov_free();
int markov_move(double dwl, double dwr);
void markov_correct(float distF, float distL, float distR);


Motor motorL, motorR;
Sonar sonarL, sonarF, sonarR;
Encoder encoderL, encoderR;

void setTargetSpeed(float wl, float wr)
{

}

void readOdometers(double *dwl, double *dwr)
{
    (*dwl) = encoderL.getDeltaAngle();
    (*dwr) = encoderR.getDeltaAngle();
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
    float d,v_r,v_l,r_w,v_des,om_des,omega_right,omega_left,phi;
    char strrota[100], *prota;
    float x,y,sx,sy,distp,cam_pos[3];

    encoderL.setup(ENCODER_LEFT, LEFT_SIDE);
	encoderR.setup(ENCODER_RIGHT, RIGHT_SIDE);
	sonarF.setup(SONAR_FRONT_TRIGGER, SONAR_FRONT_ECHO);
	sonarL.setup(SONAR_LEFT_TRIGGER, SONAR_LEFT_ECHO);
	sonarR.setup(SONAR_RIGHT_TRIGGER, SONAR_LEFT_ECHO);
	motorL.setup(MOTOR_LEFT_A, MOTOR_LEFT_B, MOTOR_LEFT_E, &encoderL);
	motorR.setup(MOTOR_RIGHT_A, MOTOR_RIGHT_B, MOTOR_RIGHT_E, &encoderR);
	motorL.pid.setKp(0.1);
	motorR.pid.setKp(0.1);

    strcpy(strrota,"-0,4;0,2|0,0;0,0");
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
        if (distp < 0.1)
        {
            prota = droppoint(prota,&x,&y);
            if(!prota)
            {
                setTargetSpeed(0, 0);
                break;
            }
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
        setTargetSpeed(omega_left, omega_right);
    }
    stopOdom();
    markov_free();
    return 0;
}

void* odom(void* arg)
{
    int cells;
    double dwl,dwr;
    float l = rodasDiff/2, r = rodaRaio,opos[3];
	Odometry odometry(l, l, r, r, &encoderL, &encoderR);
    cells = 1;
    odometry.setPosition(estado);
    while (readingOdo)
    {
        float distF=-1, distR=-1, distL=-1;
        distF = sonarF.measureDistance();//readSonar(clid, sensorFrontHandle);
        distL = sonarL.measureDistance();//readSonar(clid, sensorLeftHandle);
        distR = sonarR.measureDistance();//readSonar(clid, sensorRightHandle);
        if (cells > 0 && distF > 0.0 && distL > 0.0 && distR > 0.0)
            markov_correct(distF,distR,distL);
        readOdometers(&dwl,&dwr);
        cells = markov_move(dwl,dwr);
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
    while(!readingOdo) {}
    readingOdo = 0;
}
