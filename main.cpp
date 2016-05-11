#include "ImagemStr.h"
#define THRES_MATCHES 0.8
/**
 * @function main
 * @brief Main function
 */
int main(int argc, char** argv)
{
    std::vector<ImagemStr> treino;
    FILE *input;
    int num,i=0;
    char imgnm[50];

    Mat frame, match;
    VideoCapture cap(0);
    std::vector<KeyPoint> fkp;
    Mat fdesc;
    double good_ratio;

    int minHessian = 400;
    SurfFeatureDetector detector(minHessian);
    SurfDescriptorExtractor extractor;

    FlannBasedMatcher matcher;
    std::vector< DMatch > matches, gmatches;


    double max_dist = 0;
    double min_dist = 100;
    double mx,my;
    int tot;
    CvPoint center;

    ///Le as imagens do treino

    system("ls treino > o.txt");
    input = fopen("o.txt","r");
    strcpy(imgnm,"treino/");
    while (fscanf(input,"%s\n",imgnm+7) == 1)
    {
        Mat im = imread(imgnm,CV_LOAD_IMAGE_GRAYSCALE);
        std::vector<KeyPoint> keypoints_1;
        detector.detect(im, keypoints_1);
        //-- Step 2: Calculate descriptors (feature vectors)
        Mat descriptors_1;
        extractor.compute(im, keypoints_1, descriptors_1);
        ImagemStr *i = new ImagemStr(im,imgnm,keypoints_1,descriptors_1);
        treino.push_back(*i);
    }
    system("rm o.txt");

    ///Iteracao sobre os frames do Video
    num = treino.size();
    namedWindow("CV",1);
    for(;;)
    {
        cap >> frame;
        cvtColor(frame, frame, CV_BGR2GRAY);
        detector.detect(frame,fkp);
        extractor.compute(frame,fkp,fdesc);
        matcher.match(treino[i].des, fdesc, matches);

        //-- Quick calculation of max and min distances between keypoints
        for( int j = 0; j < treino[i].des.rows; j++ )
        {
            double dist = matches[j].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }
        mx = my = 0.0;
        tot = 0;
        for( int j = 0; j < treino[i].des.rows; j++ )
        {
            if( matches[j].distance <= max(2*min_dist, 0.02))
            {
                gmatches.push_back( matches[j]);
                mx += fkp[matches[j].trainIdx].pt.x;
                my += fkp[matches[j].trainIdx].pt.y;
                tot++;
            }
        }
        good_ratio = gmatches.size()/matches.size();
//        drawMatches( treino[i].m, treino[i].kps, frame, fkp,
//                     gmatches, match, Scalar::all(-1), Scalar::all(-1),
//                     vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        center.x = mx/tot;
        center.y = my/tot;
        circle(frame,center,50,Scalar(255,0,0),1,8,0);
        imshow("CV",frame);
        if(waitKey(30) >= 0) break;
        gmatches.clear();
        fkp.clear();
        matches.clear();
        i = (i + 1) % num;
    }

    gmatches.clear();
    matches.clear();
    treino.clear();
    fkp.clear();

    return 0;
}
