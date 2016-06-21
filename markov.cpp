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

float colisao(float x, float y,float tht, parede *modelo)
{

}

void calcdist()
{
    FILE *f = fopen("grid.txt","r");
    parede *mapmodel;
    pose_d dist[360];
    int npar;
    int i,j,k, cellx = wmap/res,celly = hmap/res;
    for (i=0;i<360;i++)
    {
        dist[i].s0 = gsl_matrix_alloc(celly,cellx);
        dist[i].s1 = gsl_matrix_alloc(celly,cellx);
        dist[i].s2 = gsl_matrix_alloc(celly,cellx);
        gsl_matrix_set_zero(dist[i].s0);
        gsl_matrix_set_zero(dist[i].s1);
        gsl_matrix_set_zero(dist[i].s2);
    }
    fscanf(f,"%d",&npar);
    mapmodel = (parede*) calloc(npar,sizeof(parede));
    for(i=0;i<npar;i++)
    {
        float a,b,c,d;
        fscanf("%f %f %f %f",&a,&b,&c,&d);
        mapmodel[i].x0 = a;
        mapmodel[i].y0 = b;
        mapmodel[i].x1 = c;
        mapmodel[i].y1 = d;
    }
    fclose(f);
    for (i=0;i<360;i++)
        for(j=5;j<(cellx-5);j++)
            for(k=5;k<(celly-5);k++)
            {
                float xc = j*res + res/2 + 2.0,yc = k*res + res/2 + 2.0;
                gsl_matrix_set(dist[i].s0,k,j,colisao(xc,yc,i,mapmodel));
            }
}
