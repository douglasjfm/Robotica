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

    Mat frame, display, match, H;
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
    int tot,rw,rl,contaframes = 0;
    CvPoint center;

    std::vector<Point2f> trackpoints;

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
        display = frame;
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

        for( int j = 0; j < treino[i].des.rows; j++ )
        {
            if( matches[j].distance <= max(2.5*min_dist, 0.02))
            {
                gmatches.push_back( matches[j]);
            }
        }

        //-- Localiza o objeto
        for( int j = 0; j < gmatches.size(); j++ )
        {
            //-- Obtem os pontos chave dos 'bons' matches
            obj.push_back( treino[i].kps[ gmatches[j].queryIdx ].pt );
            scene.push_back( fkp[ gmatches[j].trainIdx ].pt );
        }

        if ((obj.size() > 3) && (scene.size() > 3))
            H = findHomography( obj, scene, CV_RANSAC );

        std::vector<Point2f> obj_corners(4);
        std::vector<Point2f> scene_corners(4);
        obj_corners[0] = cvPoint(0,0);
        obj_corners[1] = cvPoint( treino[i].m.cols, 0 );
        obj_corners[2] = cvPoint( treino[i].m.cols, treino[i].m.rows );
        obj_corners[3] = cvPoint( 0, treino[i].m.rows );

        perspectiveTransform( obj_corners, scene_corners, H);
        rw = scene_corners[1].x - scene_corners[0].x;
        rl = scene_corners[2].y - scene_corners[0].y;
        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        if(rw*rl > 400)
        {
            putText(display,treino[i].nome,scene_corners[0],FONT_HERSHEY_PLAIN,1.0,Scalar(0, 255, 0));
            contaframes++;
            trackpoints.clear();
            trackpoints.push_back(scene_corners[0] + Point2f( treino[i].m.cols, 0));
            trackpoints.push_back(scene_corners[1] + Point2f( treino[i].m.cols, 0));
            trackpoints.push_back(scene_corners[1] + Point2f( treino[i].m.cols, 0));
            trackpoints.push_back(scene_corners[2] + Point2f( treino[i].m.cols, 0));
            trackpoints.push_back(scene_corners[2] + Point2f( treino[i].m.cols, 0));
            trackpoints.push_back(scene_corners[3] + Point2f( treino[i].m.cols, 0));
            trackpoints.push_back(scene_corners[3] + Point2f( treino[i].m.cols, 0));
            trackpoints.push_back(scene_corners[0] + Point2f( treino[i].m.cols, 0));
        }

        if (trackpoints.size()>7)
        {
            line( display, trackpoints[0], trackpoints[1], Scalar(0, 255, 0), 4 );
            line( display,trackpoints[2], trackpoints[3], Scalar( 0, 255, 0), 4 );
            line( display, trackpoints[4], trackpoints[5], Scalar( 0, 255, 0), 4 );
            line( display, trackpoints[6], trackpoints[7], Scalar( 0, 255, 0), 4 );
        }

        imshow("CV",display);
        if(waitKey(25) >= 0) break;
        gmatches.clear();
        fkp.clear();
        matches.clear();
        obj.clear();
        scene.clear();
        obj_corners.clear();
        scene_corners.clear();
        i = (i + 1) % num;
    }

    gmatches.clear();
    matches.clear();
    treino.clear();
    fkp.clear();

    return 0;
}
