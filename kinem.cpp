#include <math.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_permutation.h>
#include <gsl/gsl_linalg.h>

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
    gsl_matrix *m,*n;
    gsl_vector *v,*r;
    float a,b;
    gsl_permutation *p;
    int s;

    v = gsl_vector_alloc(4);
    m = gsl_matrix_alloc(4,4);
    n = gsl_matrix_alloc(4,4);
    gsl_matrix_set_all(m,0);
    gsl_vector_set(v,0,x1);
    gsl_vector_set(v,1,y1);
    gsl_vector_set(v,2,0.0);
    gsl_vector_set(v,3,1.0);

    gsl_matrix_set(m,0,0,cos(t));
    gsl_matrix_set(m,1,1,cos(t));
    gsl_matrix_set(m,0,1,-sin(t));
    gsl_matrix_set(m,1,0,sin(t));
    gsl_matrix_set(m,2,2,1);
    gsl_matrix_set(m,3,3,1);
    gsl_matrix_set(m,0,3,x);
    gsl_matrix_set(m,1,3,y);

    p = gsl_permutation_alloc(4);
    gsl_linalg_LU_decomp (m, p, &s);
    gsl_linalg_LU_invert (m, p, n);

    r = gsl_vector_alloc(4);
    gsl_blas_dgemv(CblasNoTrans,1,n,v,0,r);

    a = gsl_vector_get(r,0);
    b = gsl_vector_get(r,1);

    gsl_vector_free(v);
    gsl_vector_free(r);
    gsl_matrix_free(m);
    gsl_matrix_free(n);
    gsl_permutation_free(p);

    //printf("p: %f %f",a,b);

    return atan2(b,a);
}
