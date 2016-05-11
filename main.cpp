#include "ImagemStr.h"
#define THRES_MATCHES 0.01
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

    Mat frame, match, H;
    VideoCapture cap(0);
    std::vector<KeyPoint> fkp;
    Mat fdesc;
    double good_ratio;

    int minHessian = 400;
    SurfFeatureDetector detector(minHessian);
    SurfDescriptorExtractor extractor;

    FlannBasedMatcher matcher;
    std::vector< DMatch > matches, gmatches;

    std::vector<Point2f> obj;
    std::vector<Point2f> scene;

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
            if( matches[j].distance <= max(2.5*min_dist, 0.02))
            {
                gmatches.push_back( matches[j]);
                //mx += fkp[matches[j].trainIdx].pt.x;
                //my += fkp[matches[j].trainIdx].pt.y;
                //tot++;
            }
        }
        //good_ratio = (double)gmatches.size()/matches.size();

        //-- Localize the object

        for( int j = 0; j < matches.size(); j++ )
        {
            //-- Get the keypoints from the good matches
            obj.push_back( treino[i].kps[ matches[j].queryIdx ].pt );
            scene.push_back( fkp[ matches[j].trainIdx ].pt );
        }

        H = findHomography( obj, scene, CV_RANSAC );

        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<Point2f> obj_corners(4);
        obj_corners[0] = cvPoint(0,0);
        obj_corners[1] = cvPoint( treino[i].m.cols, 0 );
        obj_corners[2] = cvPoint( treino[i].m.cols, treino[i].m.rows );
        obj_corners[3] = cvPoint( 0, treino[i].m.rows );
        std::vector<Point2f> scene_corners(4);

        perspectiveTransform( obj_corners, scene_corners, H);

        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        line( frame, scene_corners[0] + Point2f( treino[i].m.cols, 0), scene_corners[1] + Point2f( treino[i].m.cols, 0), Scalar(0, 255, 0), 4 );
        line( frame, scene_corners[1] + Point2f( treino[i].m.cols, 0), scene_corners[2] + Point2f( treino[i].m.cols, 0), Scalar( 0, 255, 0), 4 );
        line( frame, scene_corners[2] + Point2f( treino[i].m.cols, 0), scene_corners[3] + Point2f( treino[i].m.cols, 0), Scalar( 0, 255, 0), 4 );
        line( frame, scene_corners[3] + Point2f( treino[i].m.cols, 0), scene_corners[0] + Point2f( treino[i].m.cols, 0), Scalar( 0, 255, 0), 4 );

        imshow("CV",frame);
        if(waitKey(30) >= 0) break;
        gmatches.clear();
        fkp.clear();
        matches.clear();
        obj.clear();
        scene.clear();
        i = (i + 1) % num;
    }

    gmatches.clear();
    matches.clear();
    treino.clear();
    fkp.clear();

    return 0;
}
