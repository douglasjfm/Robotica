#include <vector>
#include <math.h>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define pi M_PI

using namespace cv;

int nsen = 3;
float wmap = 2.2;
float hmap = 2.2;
float res = 0.05;

float Kr = 1.0;
float Kl = 1.0;

extern float vdd, rodaRaio, rodasDiff;

float pose0[] = {0.93,0.0,pi};
float estado[] = {0.0, 0.0, 0.0};

float sensorFrontPos[3] = {0.03, 0, 0};
float sensorLeftPos[3]  = {0.03, 0, M_PI/2};
float sensorRightPos[3] = {0.03, 0, -M_PI/2};

#define SEARCH_SQR 1.0
#define MINPROB 10/((wmap/res) * (hmap/res) * 360)
#define SONAR_ANGLE (7.5*pi/180)
#define SONAR_DELTA (SONAR_ANGLE*2.0/10)
#define SONAR_SIGMA 0.1

typedef struct pose_d
{
    cv::Mat s0;
    cv::Mat s1;
    cv::Mat s2;
} pose_d;

typedef struct parede
{
    float x0;
    float y0;
    float x1;
    float y1;
} parede;

typedef struct Mapa
{
    parede *wall;
    int n;
} Mapa;


Mapa mapmodel;
pose_d dist[360];
Mat bel;
int cellx,celly;

float menord(std::vector<float> v)
{
    float x = v[0];
    unsigned i;
    for (i=0; i<v.size(); i++)
        if (v[i] < x)
            x = v[i];
    return x;
}

///Realiza o incremento anti horario de 0 a 180 e de -179 a 0 graus.
int anthor(int i)
{
    int r;
    (i == 179) ? r = -180 : r = i+1;
    return r;
}

int congr(float a, float b)
{
    float sa = sin(a),ca = cos(a),sb = sin(b),cb = cos(b);
    float ds = sa - sb, dc = ca - cb;
    ds = sqrt(ds*ds);
    dc = sqrt(dc*dc);
    return ((dc < 0.0001) && (ds < 0.0001));
}

float colisao(float x, float y,int grau,Mapa *modelo)
{
    float tht;
    float x1,y1;
    std::vector<float> dis;
    int i;


    grau = grau % 360;
    tht = (grau*pi)/180.0;

    if (grau != 90 && grau != 270)
    {
        x1 = x + 1.0;
        y1 = tan(tht) + y;
    }
    else
    {
        x1 = x;
        y1 = y + 3;
    }
    for(i=0; i<modelo->n; i++)
    {
        float x2,y2,x3,y3,dem,xi,yi;
        x2 = modelo->wall[i].x0;
        y2 = modelo->wall[i].y0;
        x3 = modelo->wall[i].x1;
        y3 = modelo->wall[i].y1;

        dem = (x-x1)*(y2-y3) - (y-y1)*(x2-x3);
        if (dem > 0.0001 || dem < -0.0001)
        {
            xi = ((x*y1-y*x1)*(x2-x3)-(x-x1)*(x2*y3-y2*x3))/dem;
            yi = ((x*y1-y*x1)*(y2-y3)-(y-y1)*(x2*y3-y2*x3))/dem;
            if (((xi > x2 && xi > x3)||(yi > y2 && yi > y3))||
                    ((xi < x2 && xi < x3)||(yi < y2 && yi < y3))) continue;
            else
            {
                float d;
                float tht2 = atan2((yi-y),(xi-x));
                if (congr(tht2,tht))
                {
                    d = sqrt((xi-x)*(xi-x)+(yi-y)*(yi-y));
                    dis.push_back(d);
                }
            }
        }
    }

    if (dis.size() > 0)
        return menord(dis);
    else
        return -1.0;///Um Erro ocorreu;
}

void indexFor(float x, float y, int* i, int* j)
{
    (*i) = (hmap/2 - y)/res;
    (*j) = (x + wmap/2)/res;
}

void positionFor(int i, int j, float* x, float* y)
{
    (*x) = j*res - wmap/2;
    (*y) = hmap/2 - i*res;
}

void markov_load()
{
    FILE *f = fopen("grid.txt","r");
    int npar;
    int i,j,k;
    int g0, g1, g2;
    int sizesBel[3];
    cellx = (int)round(wmap/res);
    celly = (int)round(hmap/res);
    sizesBel[0] = celly;
    sizesBel[1] = cellx;
    sizesBel[2] = 360;
    bel = Mat(3,sizesBel,CV_32FC1);
    bel = 0.0;
    for (i=0; i<360; i++)
    {
        dist[i].s0 = Mat(celly,cellx,CV_32FC1,0.0);
        dist[i].s1 = Mat(celly,cellx,CV_32FC1,0.0);
        dist[i].s2 = Mat(celly,cellx,CV_32FC1,0.0);
    }
    fscanf(f,"%d",&npar);
    mapmodel.wall = (parede*) calloc(npar,sizeof(parede));
    mapmodel.n = npar;
    for(i=0; i<npar; i++)
    {
        float a,b,c,d;
        fscanf(f,"%f %f %f %f",&a,&b,&c,&d);
        mapmodel.wall[i].x0 = a;
        mapmodel.wall[i].y0 = b;
        mapmodel.wall[i].x1 = c;
        mapmodel.wall[i].y1 = d;
    }
    fclose(f);

    for (i=0; i<360; i++)
    {
        for(j=0; j<cellx; j++)
            for(k=0; k<celly; k++)
            {
                float xc, yc;
                positionFor(k,j,&xc,&yc);
                g0 = i;
                g1 = i+90;
                g2 = i+270;
                dist[i].s0.at<float>(k,j) = colisao(xc,yc,g0,&mapmodel);
                dist[i].s1.at<float>(k,j) = colisao(xc,yc,g1,&mapmodel);
                dist[i].s2.at<float>(k,j) = colisao(xc,yc,g2,&mapmodel);
            }
        g0 = anthor(g0);
        g1 = anthor(g1);
        g2 = anthor(g2);
    }
    indexFor(pose0[0],pose0[1],&i,&j);
    k = (int)round((pose0[2]*180/pi));
    bel.at<float>(i,j,k) = 1.0;
    estado[0] = pose0[0];
    estado[1] = pose0[1];
    estado[2] = pose0[2];
    printf("Markov Loaded: %d %d\n",celly,cellx);
}

void markov_free()
{
    int i;
    for (i=0; i<360; i++)
    {
        dist[i].s0.release();
        dist[i].s1.release();
        dist[i].s2.release();
        bel.release();
    }
    free(mapmodel.wall);
}

float ideal_dist(int s, float x ,float y, int tht)
{
    int i,j,g;
    float ret;
    indexFor(x,y,&i,&j);
    g = tht;
    switch(s)
    {
    case 0:
        ret = dist[g].s0.at<float>(i,j);
        break;
    case 1:
        ret = dist[g].s1.at<float>(i,j);
        break;
    case 2:
        ret = dist[g].s2.at<float>(i,j);
        break;
    }
    return ret;
}

float to180range(float angle)
{
    angle = fmod(angle, 2*M_PI);
    if (angle<-M_PI)
    {
        angle = angle + 2*M_PI;
    }
    else if (angle>M_PI)
    {
        angle = angle - 2*M_PI;
    }

    return angle;
}

/*!
    Returns the probability that the observed value of a standard normal
    random variable will be less than or equal to x.
*/
double z(double x) //normal pdf
{
    // constants
    double a1 =  0.254829592;
    double a2 = -0.284496736;
    double a3 =  1.421413741;
    double a4 = -1.453152027;
    double a5 =  1.061405429;
    double p  =  0.3275911;

    // Save the sign of x
    int sign = 1;
    if (x < 0)
        sign = -1;
    x = fabs(x)/sqrt(2.0);

    // A&S formula 7.1.26
    double t = 1.0/(1.0 + p*x);
    double y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*exp(-x*x);

    return 0.5*(1.0 + sign*y);
}

float pGaussian(float dist, float sigma, float step)
{
    float x = fabs(dist);
    float halfstep = step/2.0;
    float zmax = (x+halfstep)/sigma;
    float zmin = (x-halfstep)/sigma;
    if (zmax<zmin)
    {
        float swp = zmin;
        zmin = zmax;
        zmax = swp;
    }
    return z(zmax) - z(zmin);
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

float pTrvGaussian(cv::Mat &m, cv::Mat &x, cv::Mat E)
{
    /*
     * "Approximates" a trivariate gaussian by multiplying tree univariate gaussians with
     * sigma equal to the main diagonal. It seems to be fast and precise enought for our purposes.
     *
    */
    return (pGaussian(x.at<float>(0,0)-m.at<float>(0,0), E.at<float>(0,0), res)*
            pGaussian(x.at<float>(0,1)-m.at<float>(0,1), E.at<float>(1,1), res)*
            pGaussian(smallestAngleDiff(x.at<float>(0,2),m.at<float>(0,2)), E.at<float>(2,2), pi/180.0));///3;
}

void odomError(cv::Mat &FDrl, cv::Mat &Ed, cv::Mat &Ep)
{
    Ep = FDrl*Ed*FDrl.t();
}

void fdeltaRL(float theta, float ds, float dtheta, cv::Mat &FDrl)
{
    //ds = fabs(ds);
    float costdt2 = cos(theta+dtheta/2);
    float sintdt2 = sin(theta+dtheta/2);
    float b = rodasDiff;

    FDrl.at<float>(0,0) = costdt2/2 - (ds/(2*b))*sintdt2;//1
    FDrl.at<float>(0,1) = costdt2/2 + (ds/(2*b))*sintdt2;//2

    FDrl.at<float>(1,0) = sintdt2/2 + (ds/(2*b))*costdt2;//3
    FDrl.at<float>(1,1) = sintdt2/2 - (ds/(2*b))*costdt2;//4

    FDrl.at<float>(2,0) = 1/b;//5
    FDrl.at<float>(2,1) = -1/b;//6
}

int grausTo360(int g)
{
    if(g < 0 && g >= -180)
        return (360+g);
    return g;
}

int actionUpdate(float dl, float dr)
{
    int x,y,t,NUMBELX = wmap/res, NUMBELY = hmap/res;
    int graus,graus2,t1,x1,y1,cells = 0;
    char newTeta;
    float dtheta = (dr - dl)/rodasDiff,blf,ds = (dr+dl)/2.0;
    float costdt2,sintdt2,sum = 0;

    cv::Mat X1(1,3, CV_32FC1), //current position [x,y,theta]
    X0(1,3, CV_32FC1), //previous position [x,y,theta]
    M(1,3, CV_32FC1),  //new expected position [x,y,theta]
    Ed(2,2, CV_32FC1),  //covar(dsr, dsl)
    FDrl(3,2, CV_32FC1),//Jacobian
    Ep(3,3, CV_32FC1);  //motion error Sigmap

    const int belSizes[3]= {hmap/res,wmap/res,360};
    cv::Mat sumbel(3, belSizes, CV_32FC1, 0.0);

    Ed.at<float>(0,0) = Kr*fabs(dr);
    Ed.at<float>(1,1) = Kl*fabs(dl);
    Ed.at<float>(0,1) = 0;
    Ed.at<float>(1,0) = 0;

    graus = 0;
    for (t=0; t<360; t++)
    {
        newTeta = 1;
        X0.at<float>(0,2) = graus*(pi/180.0);
        for(x=0; x<NUMBELX; x++)
        {
            for(y=0; y<NUMBELY; y++)
            {
                float xp ,yp;
                positionFor(y,x,&xp,&yp);
                X0.at<float>(0,0) = xp;
                X0.at<float>(0,1) = yp;
                blf = bel.at<float>(y,x,t);
                if(blf > MINPROB)
                {
                    if (newTeta)
                    {
                        costdt2 = cos(X0.at<float>(0,2)+dtheta/2);
                        sintdt2 = sin(X0.at<float>(0,2)+dtheta/2);
                        fdeltaRL(X0.at<float>(0,2), ds, dtheta, FDrl);
                        odomError(FDrl, Ed, Ep);

                        M.at<float>(0,2) = to180range(X0.at<float>(0,2)+dtheta);
                        newTeta = 0;
                    }
                    M.at<float>(0,1) = X0.at<float>(0,1)+ds*sintdt2;
                    M.at<float>(0,0) = X0.at<float>(0,0)+ds*costdt2;

                    //Update only a rectangle around X0
                    float dsrect = SEARCH_SQR*(fabs(ds)+res);
                    float dtrect = SEARCH_SQR*(fabs(dtheta)+pi/180.0);//2*(dtheta + 1 step) since dtheta can be 0
                    int dtrect_gr = (int)(dtrect*180.0/pi);
                    float x1min[3], x1max[3];

                    //x1min
                    x1min[0] = M.at<float>(0,0)-dsrect;
                    if (x1min[0]<-2) x1min[0] = -wmap/2;
                    x1min[1] = M.at<float>(0,1)-dsrect;
                    if (x1min[1]<-2) x1min[1] = -hmap/2;
                    x1min[2] = to180range(M.at<float>(0,2)-dtrect);
                    //if (x1min[2]<-M_PI) x1min[2] = -M_PI;

                    //x1max
                    x1max[0] = M.at<float>(0,0)+dsrect;
                    if (x1max[0]>2) x1max[0] = wmap/2;
                    x1max[1] = M.at<float>(0,1)+dsrect;
                    if (x1max[1]>2) x1max[1] = hmap/2;
                    x1max[2] = to180range(M.at<float>(0,2)+dtrect);
                    //if (x1max[2]>M_PI) x1max[2] = M_PI;

                    //get map coordinates
                    int mapmin[3], mapmax[3];

                    indexFor(x1min[0],x1min[1],&(mapmin[0]),&(mapmin[1]));
                    indexFor(x1max[0],x1max[1],&(mapmax[0]),&(mapmax[1]));

                    positionFor(mapmin[0],mapmin[1],&(x1min[0]),&(x1min[1]));
                    positionFor(mapmax[0],mapmax[1],&(x1max[0]),&(x1max[1]));

                    if(mapmax[0] < mapmin[0])
                    {
                        int swp=mapmin[0];
                        mapmin[0] = mapmax[0];
                        mapmax[0] = swp;
                    }
                    if(mapmax[1] < mapmin[1])
                    {
                        int swp=mapmin[1];
                        mapmin[1] = mapmax[1];
                        mapmax[1] = swp;
                    }

                    //printf("%d %f\n",dtrect_gr,dtrect);
                    graus2 = (int)(x1min[2]*180.0/pi);
                    dtrect_gr = dtrect_gr << 1;
                    for(t1=0; t1<=dtrect_gr; t1++)  //for each new theta
                    {
                        for(x1 =  mapmin[0]; x1<=mapmax[0]; x1++)   //for each new x
                        {
                            for(y1 =  mapmin[1]; y1<=mapmax[1]; y1++)   //for each new y
                            {
                                float px1_u1x0,px1,py1;
                                positionFor(y1,x1,&px1,&py1);
                                X1.at<float>(0,0) = px1;
                                X1.at<float>(0,1) = py1;
                                X1.at<float>(0,2) = graus2*pi/180.0;
                                px1_u1x0 = pTrvGaussian(M, X1, Ep);
                                sumbel.at<float>(y1,x1,grausTo360(t1)) += blf*px1_u1x0;
                                sum += blf*px1_u1x0;
                                cells++;
//                                if (px1_u1x0>0) {
//                                    printf("px1_u1x0(%.2f,%.2f,%.2f | %.2f,%.2f,%.2f): p:%f  b:%f pb:%f\n", X1.at<float>(0,0), X1.at<float>(0,1), X1.at<float>(0,2), M.at<float>(0,0), M.at<float>(0,1), M.at<float>(0,2), px1_u1x0, b, px1_u1x0*b);
//                                }
                            }
                        }
                        graus2 = anthor(graus2);
                    }
                }
            }
        }
        graus = anthor(graus);
    }
    if(sum > 0.0)
    {
        bel/sum;
    }
    //printf("%d updated cells\n",cells);
    return cells;
}

/*!
   Markov Action Update
*/
int markov_move(float dwl, float dwr)
{
    static float prevdl = 0.0, prevdr = 0.0;
    float ds,dl = dwl*rodaRaio, dr = dwr*rodaRaio;
    float dteta;
    //int g,a,b,sigx,sigy;

    ds = (dl+dr)/2.0;
    dteta = (dr-dl)/rodasDiff;

    estado[0] += ds*cos(dteta+estado[2]);
    estado[1] += ds*sin(dteta+estado[2]);
    estado[2] += dteta;// to180range(estado[2] + dteta);

    dl = prevdl = prevdl + dl;
    dr = prevdr = prevdr + dr;

    ds = (dl+dr)/2.0;
    dteta = (dr-dl)/rodasDiff;
    //g = (int)(dteta*180.0/pi);

    //estado[0] = ds*cos(dteta+estado[2]);
    //estado[1] = ds*sin(dteta+estado[2]);
    //estado[2] += dteta;

    //printf("%.3f %.3f %.3f\n",estado[0],estado[1],estado[2]);
    //printf("%.3f %.3f\n",ds,dteta);
    if (ds > res || fabs(dteta) > pi/180.0)
    {
        //printf("->%.3f %.3f\n",ds,dteta);
        prevdl = prevdr = 0.0;
        return actionUpdate(dl,dr);
    }

    return 0;
}

float nearesPointInMap(float *p, float *npl, int s)
{
    int i,j,graus;
    indexFor(p[0],p[1],&i,&j);
    graus = (int)round(p[2]*180.0/pi);
    graus = graus < 0 ? (180 + (graus + 179)) : graus;
    return ideal_dist(s,p[0],p[1],graus);
}

void robotToWorld(float* vetin, float* robotPos, float *out)
{
    float sintheta = sin(robotPos[2]);
    float costheta = cos(robotPos[2]);

    out[0] = robotPos[0] + (vetin[0]*costheta - vetin[1]*sintheta);
    out[1] = robotPos[1] + (vetin[0]*sintheta + vetin[1]*costheta);
    out[2] = robotPos[2] + vetin[2];
}

void sensorToRobot(float dist, float* sensorPos, float *out)
{
    out[0] = sensorPos[0] + dist*cos(sensorPos[2]);
    out[1] = sensorPos[1] + dist*sin(sensorPos[2]);
}

void robotToSensorPoint(float *pRobot, float *pSensor, float dist, float* point)
{
    float pSR[2];
    sensorToRobot(dist, pSensor, pSR);
    robotToWorld(pSR, pRobot, point);
}

/*! Markov Update Action*/
void markov_correct(float distF, float distL, float distR)
{
    float robotPos[3]; //[x,y,theta]
    int x,y,t,graus=0,g15;
    float sensorPoint[3], mapPoint[3];
    float p, sum=0,b,mp=0.0,estadox = estado[0],estadoy=estado[1],estadot=estado[2];
printf("L = %.2f F = %.2f R = %.2f\n",distL,distF,distR);
printf("x = %.2f y = %.2f t = %.2f\n",estado[0],estado[1],estado[2]);
    for(x=0; x<cellx; x++)
    {
        for(y=0; y<celly; y++)
            for(t=0; t<360; t++)
            {
                robotPos[2] = graus*pi/180.0;
                graus = anthor(graus);
                positionFor(y,x,&(robotPos[0]),&(robotPos[1]));
                b = bel.at<float>(y,x,t);

                if (b>0.0)
                {
                    float stmin, stmax, d;
                    float dF=999, dL=999, dR=999;
                    float sensorDir[3];
                    int sensorDr;

//            robotToSensorPoint(robotPos, sensorFrontPos, distF, sensorPoint);
//            dF = nearesPointInMap(sensorPoint, mapPoint);

                    //sensorDir[0] = sensorFrontPos[0];
                    //sensorDir[1] = sensorFrontPos[1];
                    //stmin = sensorFrontPos[2]-SONAR_ANGLE;
                    //stmax = sensorFrontPos[2]+SONAR_ANGLE;
                    //for (sensorDir[2] = stmin; sensorDir[2]<=stmax; sensorDir[2]+=SONAR_DELTA)
                    sensorDr = robotPos[2]*180.0/pi;
                    sensorDr = grausTo360(sensorDr);
                    sensorDr -= 8;
                    sensorDr = grausTo360(sensorDr);
                    for(g15=0;g15 < 15; g15++)
                    {
                        //robotToSensorPoint(robotPos, sensorDir, distF, sensorPoint);
                        //d = nearesPointInMap(sensorPoint, mapPoint,0);
                        d = ideal_dist(0,robotPos[0],robotPos[1],sensorDr);
                        //sensorDr = grausTo360(anthor(sensorDr));
                        if (d<dF && d>0.0) dF=d;
                        d = ideal_dist(1,robotPos[0],robotPos[1],sensorDr);
                        //sensorDr = grausTo360(anthor(sensorDr));
                        if (d<dL && d>0.0) dL=d;
                        d = ideal_dist(2,robotPos[0],robotPos[1],sensorDr);
                        sensorDr = grausTo360(anthor(sensorDr));
                        if (d<dR && d>0.0) dR=d;
                    }
                    dF = pGaussian(distF-dF, SONAR_SIGMA, res);
                    dL = pGaussian(distL-dL, SONAR_SIGMA, res);
                    dR = pGaussian(distR-dR, SONAR_SIGMA, res);
                    p = dF*dL*dR*b;
                    bel.at<float>(y,x,t) = p;
                    sum+=p;
                    if(p>mp)
                    {
                        //printf("->L = %.2f F = %.2f R = %.2f\n",distL,distF,distR);
                        estadox = robotPos[0];
                        estadoy = robotPos[1];
                        estadot = robotPos[2];
                        printf("x = %.2f y = %.2f t = %.2f\n",estadox,estadoy,estadot);
                        mp = p;
                    }
                }
            }
    }

    if (sum>0)
        bel = bel/sum;
    //estado[0] = estadox;
    //estado[1] = estadoy;
    //estado[2] = estadot;
}
