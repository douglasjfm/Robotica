#include <math.h>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

/*!
 Realiza a Transformação do sistema de coordenadas global do ponto
(x1,y1) para o sist. de coordenadas homogeneas do ponto (x,y)
com orientação dada por t rad.
  @param x Coordenada x da referencia
  @param y Coordenada y da referencia
  @param t Orientação do objeto no ponto (x,y), em radianos
  @param x1 Coordenada x do ponto a ser tranformado
  @param y1 Coordenada y do ponto a ser tranformado
  @return O angulo phi entre o vetor diretor dado por (x,y;t) e o vetor (x1-x,y1-y)

*/
float trans_get_phi (float x, float y, float t, float x1, float y1)
{
//    cv::Mat m = Mat::zeros(4,4,CV_32FC1),n(4,4,CV_32FC1);
//    cv::Mat v(4,1,CV_32FC1),r;
//    float a,b;
//
//    v.at<float>(0) = x1;
//    v.at<float>(1) = y1;
//    v.at<float>(2) = 0.0;
//    v.at<float>(3) = 0.0;
//
//    m.at<float>(0,0) = cos(t);
//    m.at<float>(1,1) = cos(t);
//    m.at<float>(0,1) = -sin(t);
//    m.at<float>(1,0) = sin(t);
//    m.at<float>(2,2) = 1.0;
//    m.at<float>(3,3) = 1.0;
//    m.at<float>(0,3) = x;
//    m.at<float>(1,3) = y;
//
//    invert(m,n,DECOMP_LU);
//    r = n * v;
//
//    //printf("%d %d\n",r.rows,r.cols);
//
//    a = r.at<float>(0);
//    b = r.at<float>(1);
//
//    m.release();
//    n.release();
//    v.release();
//    r.release();
    float vx = x1-x,vy = y1-y;
    float a = atan2(vy,vx);
    return (t-a);
}
