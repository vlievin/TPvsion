#include <vector>
#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/nonfree/nonfree.hpp"

using namespace std;
using namespace cv;

int main()
{
    vector<Mat> imgs;
    imgs.push_back(imread("img/IMG_6621.jpg", 0));
    imgs.push_back(imread("img/IMG_6622.jpg", 0));
    imgs.push_back(imread("img/IMG_6623.jpg", 0));
    imgs.push_back(imread("img/IMG_6624.jpg", 0));
    imgs.push_back(imread("img/IMG_6625.jpg", 0));
    imgs.push_back(imread("img/IMG_6626.jpg", 0));
    imgs.push_back(imread("img/IMG_6627.jpg", 0));

    for (int i = 0; i< imgs.size(); i++)
    {
        if (! imgs.at(i).data)
        {
            cout<< " fail to load image : "<< i << endl;
            return -1;
        }
    }

   ///-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 400;

  SurfFeatureDetector detector( minHessian );

  vector<KeyPoint>[imgs.size()][imgs.size()] keyss;


  for (int i  = O; i<imgs.size(); i++ )
  {
    for (int j  = O; j<imgs.size(); j++ )
    {
        if (i!=j)
        {
            detector.detect( imgs.at(i), keyss[i ,i] );
            detector.detect( imgs.at(j), keyss[i , j] );
        }
    }

  }


    namedWindow( "Display" , WINDOW_AUTOSIZE );
    //moveWindow("Display", 0, 0);

    imshow("Display",img);
    waitKey(0);
    return 0;
}
