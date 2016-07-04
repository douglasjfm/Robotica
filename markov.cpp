#include <vector>
#include <math.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_permutation.h>
#include <gsl/gsl_linalg.h>

#define pi M_PI

int nsen = 3;
float wmap = 4.0;
float hmap = 4.0;
float res = 0.05;

extern float vdd, rodaRaio, rodasDiff;

float pose0[] = {1.25,0.25,90.0}, estado[] = {0.0, 0.0, 0.0};

typedef struct pose_d
{
    gsl_matrix *s0;
    gsl_matrix *s1;
    gsl_matrix *s2;
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
        dist[i].s0 = gsl_matrix_alloc(celly,cellx);
        dist[i].s1 = gsl_matrix_alloc(celly,cellx);
        dist[i].s2 = gsl_matrix_alloc(celly,cellx);
        bel[i].s0 = gsl_matrix_alloc(celly,cellx);
        gsl_matrix_set_zero(dist[i].s0);
        gsl_matrix_set_zero(dist[i].s1);
        gsl_matrix_set_zero(dist[i].s2);
        gsl_matrix_set_zero(bel[i].s0);
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
                gsl_matrix_set(dist[i].s0,k,j,colisao(xc,yc,g0,&mapmodel));
                gsl_matrix_set(dist[i].s1,k,j,colisao(xc,yc,g1,&mapmodel));
                gsl_matrix_set(dist[i].s2,k,j,colisao(xc,yc,g2,&mapmodel));
            }
        g0 = anthor(g0);
        g1 = anthor(g1);
        g2 = anthor(g2);
    }
    indexFor(pose0[0],pose0[1],&i,&j);
    gsl_matrix_set(bel[(int)pose0[2]].s0,i,j,1.0);
}

void markov_free()
{
    int i;
    for (i=0;i<360;i++)
    {
        gsl_matrix_free(dist[i].s0);
        gsl_matrix_free(dist[i].s1);
        gsl_matrix_free(dist[i].s2);
        gsl_matrix_free(bel[i].s0);
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
        case 0: ret = gsl_matrix_get(dist[g].s0,i,j);break;
        case 1: ret = gsl_matrix_get(dist[g].s1,i,j);break;
        case 2: ret = gsl_matrix_get(dist[g].s2,i,j);break;
    }
    return ret;
}

void markov_position(float* x, float* y, int* g)
{
    int i,mi,mj,mg;
    size_t j,k;
    float maxp = 0.0, b;
    for(i=0;i<360;i++)
    {
        gsl_matrix_max_index(bel[i].s0,&j,&k);
        b = gsl_matrix_get(bel[i].s0,j,k);
        if (maxp < b)
        {
            maxp = b;
            mi = j;
            mj = k;
            mg = i;
        }
    }
    positionFor(mi,mj,x,y);
    (*g) = mg;
}

/*!
   dl = deslocamento odometrico da roda esquerda.
   dr =      "           "      "   "   direita.
*/
void markov_move(float dl, float dr)
{
    float ds = (dl+dr)/2.0;
    float dteta = (dr-dl)/rodasDiff;
    float dx = ds*cos(dteta);
    float dy = ds*sin(dteta);
    int a,b,g = (int) (dteta*180.0/pi);

    g = (g < 0) ? (360 - g) : g;
    indexFor(dx,dy,&a,&b); ///a, b e g indexam o estado de centro da crença dado que foi realizado o ultimo movimento

}
