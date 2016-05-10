#include "ImagemStr.h"

ImagemStr::ImagemStr(Mat mtx, char* nm, std::vector<KeyPoint> k, Mat d)
{
    m = mtx;
    nome = (char*) malloc(strlen(nm)*sizeof(char));
    kps = k;
    des = d;
}

ImagemStr::~ImagemStr()
{
    free(nome);
}
