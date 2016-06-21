#include <math.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_permutation.h>
#include <gsl/gsl_linalg.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define pi M_PI

using namespace cv;

typedef struct WallModel
{
    Point2f dir;
    Point2f p0;
} WallModel;

///Sensores
int nsen = 3;
float asen[] = {-pi/2,0.0,pi/2};

float W = 3.0; /// Largura 3 metros - x;
float L = 4.0; /// Comprimento 4 metros - y;
float sp = 0.05; /// Precisão espacial 5 cm;
float ap = 2*M_PI/360; ///Precisão angular de 1 grau

/// Quantidade de indices a serem deslocados devido
///celula da matriz do mapa[0][0] estar na posicao (x = -1.5m, y = -2.0m)
int delta_i = W/(2*sp);
int delta_j = L/(2*sp);

gsl_matrix **mapa;

int goal[8];

int testeParede(Mat m, int x, int y)
{
    uchar cor = m.at<uchar>(y, x);
    uchar a,b,c,d,w = 255;
    if (cor == w)
    {
        a = m.at<uchar>(y-1, x);
        b = m.at<uchar>(y, x-1);
        c = m.at<uchar>(y+1, x);
        d = m.at<uchar>(y, x+1);
        if (!(a && b && c && d))
            return 1;
    }
    return 0;
}

float sensorDists(float x, float y)
{

}

void loadgrid()
{
    gsl_matrix **medidas;
    //std::vector<Point2i> celulasColisao;
    int i,j,x,y,z,a,b;
    Mat gridimg = imread("grid.bmp",CV_LOAD_IMAGE_GRAYSCALE);
    medidas = (gsl_matrix**) calloc(360,sizeof(gsl_matrix*));
    for(i=0;i<360;i++)
        medidas[i] = gsl_matrix_alloc(gridimg.cols,gridimg.rows);

    ///Realiza as medições do mapa dadas todas as poses
    for (x=0;x<gridimg.cols;x++)
        for (y=0;y<gridimg.rows;y++)
            for (z=0;z<360;z++)
            {

            }
}

/*! Dada a posição (x,y) em metros, nas coordenadas do mapa, retem-se a celula (i,j) do grid*/
void posicaoCelula(float x, float y, int *i, int *j)
{
    (*i) = x/sp + delta_i;
    (*j) = y/sp + delta_j;
}

/*! Dada a celula (i,j) do grid, retem-se a posição (x,y) do mapa real, nas coordenadas desse mapa.*/
void celulaPosicao(int i, int j, float *x, float *y)
{
    (*x) = (i - delta_i)*sp;
    (*y) = (j - delta_j)*sp;
}

/*!
    Assume-se que o robo esta na posição (1.25,0.25,PI/2)
*/
void initmapa()
{
    int w = W/sp;
    int l = L/sp;
    int i, init_i, init_j;
    mapa = (gsl_matrix**) calloc(360,sizeof(gsl_matrix*));
    for(i=0;i<360;i++)
    {
        mapa[i] = gsl_matrix_alloc(w,l);
        gsl_matrix_set_zero(mapa[i]);
    }

    ///Inicializa o mapa;
    posicaoCelula(1.25,0.25,&init_i,&init_j);
    gsl_matrix_set(mapa[90],init_i,init_j,1.0);/// P(l = (1.25m,0.25m,90grs)) = 1

    ///Inicilaiza o array com ass celulas goal
    posicaoCelula(-1.0,1.5,&(goal[0]),&(goal[1]));///Amarelo
    posicaoCelula(1.0,-1.5,&(goal[2]),&(goal[3]));///Verde
    posicaoCelula(-1.0,-1.5,&(goal[4]),&(goal[5]));///Vermelho
    posicaoCelula(1.0,1.5,&(goal[6]),&(goal[7]));///Azul
}

/*!
    Função Gaussiana com media m e desvio s;
    @param x amostra do sinal gaussiano
    @param m media da gaussiana
    @param s desvio padrão da gaussiana
    @return resultado do valor da probabilidade gaussiana no ponto x
*/
double prob_normal(double x, double m, double s)
{
    return 0.0;
}

void markov(int clientID)
{

}
