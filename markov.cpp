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

float pose0[] = {0.8,0.0,pi};
float estado[] = {0.0, 0.0, 0.0};

typedef struct pose_d
{
    cv::Mat s0;
    cv::Mat s1;
    cv::Mat s2;
}pose_d;

typedef struct parede
{
    float x0;
    float y0;
    float x1;
    float y1;
}parede;

typedef struct Mapa
{
    parede *wall;
    int n;
}Mapa;


Mapa mapmodel;
pose_d dist[360], bel[360];

float menord(std::vector<float> v)
{
    float x = v[0];
    unsigned i;
    for (i=0;i<v.size();i++)
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
    for(i=0;i<modelo->n;i++)
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
    int i,j,k, cellx = wmap/res,celly = hmap/res;
    int g0, g1, g2;

    for (i=0;i<360;i++)
    {
        dist[i].s0 = Mat(celly,cellx,CV_32FC1,0.0);
        dist[i].s1 = Mat(celly,cellx,CV_32FC1,0.0);
        dist[i].s2 = Mat(celly,cellx,CV_32FC1,0.0);
        bel[i].s0 = Mat(celly,cellx,CV_32FC1,0.0);
    }
    fscanf(f,"%d",&npar);
    mapmodel.wall = (parede*) calloc(npar,sizeof(parede));
    mapmodel.n = npar;
    for(i=0;i<npar;i++)
    {
        float a,b,c,d;
        fscanf(f,"%f %f %f %f",&a,&b,&c,&d);
        mapmodel.wall[i].x0 = a;
        mapmodel.wall[i].y0 = b;
        mapmodel.wall[i].x1 = c;
        mapmodel.wall[i].y1 = d;
    }
    fclose(f);

    for (i=0;i<360;i++)
    {
        for(j=0;j<cellx;j++)
            for(k=0;k<celly;k++)
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
    bel[(int)pose0[2]].s0.at<float>(i,j) = 1.0;
    estado[0] = pose0[0];
    estado[1] = pose0[1];
    estado[2] = pose0[2];
}

void markov_free()
{
    int i;
    for (i=0;i<360;i++)
    {
        dist[i].s0.empty();
        dist[i].s1.empty();
        dist[i].s2.empty();
        bel[i].s0.empty();
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
        case 0: ret = dist[g].s0.at<float>(i,j);break;
        case 1: ret = dist[g].s1.at<float>(i,j);break;
        case 2: ret = dist[g].s2.at<float>(i,j);break;
    }
    return ret;
}

int actionUpdate(float dl, float dr)
{
    int x,y,t,NUMBELX = wmap/res, NUMBELY = hmap/res;
    int a,b,t0;
    float dteta = (dr - dl)/rodasDiff;

    cv::Mat X1(1,3, CV_32FC1), //current position [x,y,theta]
            X0(1,3, CV_32FC1), //previous position [x,y,theta]
            M(1,3, CV_32FC1),  //new expected position [x,y,theta]
            Ed(2,2, CV_32FC1),  //covar(dsr, dsl)
            FDrl(3,2, CV_32FC1),//Jacobian
            Ep(3,3, CV_32FC1);  //motion error Sigmap

    const int belSizes[3]={hmap/res,wmap/res,360};
    cv::Mat sumbel(3, belSizes, CV_32FC1, 0.0);

    Ed.at<float>(0,0) = Kr*fabs(dsr);
    Ed.at<float>(1,1) = Kl*fabs(dsl);
    Ed.at<float>(0,1) = 0;
    Ed.at<float>(1,0) = 0;

    for (t=0;t<360;t++)
    {
        for(x=0;x<NUMBELX;x++)
        {
            for(y=0;y<NUMBELY;y++)
        }
    }

}

/*!
   dl = deslocamento odometrico da roda esquerda.
   dr =      "           "      "   "   direita.
*/
int markov_move(float dwl, float dwr)
{
    static float prevdl = 0.0, prevdr = 0.0;
    float ds,dl = dwl*rodaRaio, dr = dwr*rodaRaio;
    float dteta;
    float dx,dy;
    int g,a,b,sigx,sigy;

    dl = prevdl = prevdl + dl;
    dr = prevdr = prevdr + dr;

    ds = (dl+dr)/2.0;
    dteta = (dr-dl)/rodasDiff;
    //g = (int)(dteta*180.0/pi);

    //estado[0] = ds*cos(dteta+estado[2]);
    //estado[1] = ds*sin(dteta+estado[2]);
    //estado[2] += dteta;

    //printf("x = %.3f y = %.3f\n",estado[0],estado[1]);
    if (ds > 0.05 && dteta > pi/180)
    {
        prevdl = prevdr = 0.0;
        actionUpdate(dl,dr);
    }
}

void markov_correct()
{

}
