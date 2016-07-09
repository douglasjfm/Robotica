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


#define SEARCH_SQR 1.0
#define MINPROB 10/((wmap/res) * (hmap/res) * 360)

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
pose_d dist[360], bel[360];

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
    int i,j,k, cellx = wmap/res,celly = hmap/res;
    int g0, g1, g2;

    for (i=0; i<360; i++)
    {
        dist[i].s0 = Mat(celly,cellx,CV_32FC1,0.0);
        dist[i].s1 = Mat(celly,cellx,CV_32FC1,0.0);
        dist[i].s2 = Mat(celly,cellx,CV_32FC1,0.0);
        bel[i].s0 = Mat(celly,cellx,CV_32FC1,0.0);
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
    bel[(int)pose0[2]].s0.at<float>(i,j) = 1.0;
    estado[0] = pose0[0];
    estado[1] = pose0[1];
    estado[2] = pose0[2];
}

void markov_free()
{
    int i;
    for (i=0; i<360; i++)
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
    return z(zmax) - z(zmin);
}

float to_positive_angle(float angle)
{
    angle = fmod(angle, 2 * M_PI);
    while (angle < 0) {
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

int actionUpdate(float dl, float dr)
{
    int x,y,t,NUMBELX = wmap/res, NUMBELY = hmap/res;
    int a,b,t0;
    char newTeta;
    float dtheta = (dr - dl)/rodasDiff,blf,ds = (dr+dl)/2.0;
    float costdt2,sintdt2;

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

    for (t=0; t<360; t++)
    {
        newTeta = 1;
        for(x=0; x<NUMBELX; x++)
        {
            for(y=0; y<NUMBELY; y++)
            {
                blf = bel[t].s0.at<float>(y,x);
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
                    float x1min[3], x1max[3];
                    //x1min
                    x1min[0] = M.at<float>(0,0)-dsrect;
                    if (x1min[0]<-2) x1min[0] = -2;
                    x1min[1] = M.at<float>(0,1)-dsrect;
                    if (x1min[1]<-2) x1min[1] = -2;
                    x1min[2] = M.at<float>(0,2)-dtrect;
                    if (x1min[2]<-M_PI) x1min[2] = -M_PI;

                    //x1max
                    x1max[0] = M.at<float>(0,0)+dsrect;
                    if (x1max[0]>2) x1max[0] = 2;
                    x1max[1] = M.at<float>(0,1)+dsrect;
                    if (x1max[1]>2) x1max[1] = 2;
                    x1max[2] = M.at<float>(0,2)+dtrect;
                    if (x1max[2]>M_PI) x1max[2] = M_PI;

                    //get map coordinates
                    float mapmin[3], mapmax[3];
                    worldToMap(x1min, mapmin);
                    worldToMap(x1max, mapmax);

                    indexFor(ximin

                    mapToWorld(mapmin, x1min);
                    mapToWorld(mapmax, x1max);


//                    printf("X0 = (%.2f,%.2f,%.2f) M = (%.2f,%.2f,%.2f)\n", X0.at<float>(0,0), X0.at<float>(0,1), X0.at<float>(0,2), M.at<float>(0,0), M.at<float>(0,1), M.at<float>(0,2));

                    for(X1.at<float>(0,2) = x1min[2], t1 =  mapmin[2]; t1<=mapmax[2]; X1.at<float>(0,2)+=2*M_PI/(BEL_NTHETA-1), t1++)  //for each new theta
                    {
                        for(X1.at<float>(0,0) = x1min[0], x1 =  mapmin[0]; x1<=mapmax[0]; X1.at<float>(0,0)+=4.0/(BEL_NXY-1), x1++)   //for each new x
                        {
                            for(X1.at<float>(0,1) = x1min[1], y1 =  mapmin[1]; y1<=mapmax[1]; X1.at<float>(0,1)+=4.0/(BEL_NXY-1), y1++)   //for each new y
                            {

                                float px1_u1x0 = pTrvGaussian(M, X1, Ep);
                                sumbel.at<float>(x1,y1,t1) += blf*px1_u1x0;
                                sum += blf*px1_u1x0;
                                cells++;
//                                if (px1_u1x0>0) {
//                                    printf("px1_u1x0(%.2f,%.2f,%.2f | %.2f,%.2f,%.2f): p:%f  b:%f pb:%f\n", X1.at<float>(0,0), X1.at<float>(0,1), X1.at<float>(0,2), M.at<float>(0,0), M.at<float>(0,1), M.at<float>(0,2), px1_u1x0, b, px1_u1x0*b);
//                                }
                            }
                        }
                    }
                }
            }
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
