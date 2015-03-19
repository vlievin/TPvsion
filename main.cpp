#include <vector>
#include <iostream>
#include <stdio.h>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

using namespace std;
using namespace cv;
/*
int mainbis() {
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

      vector<KeyPoint> keyss[imgs.size()];


      for (int i  = 0; i<imgs.size(); i++ )
      {

            detector.detect( imgs.at(i), keyss[i] );

      }

    ///-- Step 2: Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;

    Mat descriptor[imgs.size()];

    for (int i = 0 ; i < imgs.size() ; i ++)
    {
        extractor.compute( imgs.at(i), keyss[i], descriptor[i]);
    }

    ///-- Step 3: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;

    std::vector< DMatch > matches[imgs.size()][imgs.size()];

    for (int i = 0; i < imgs.size() ; i ++ )
    {
        for ( int  j = 0 ; j < imgs.size() ; j ++)
        {
            if ( i != j )
            {
                matcher.match( descriptor[i], descriptor[j] , matches[i][j] );
            }
        }

    }

    ///-- localize matches
    vector<Point2f> pts[ imgs.size() ][ imgs.size() ] ;

    for (int i = 0; i < imgs.size() ; i ++ )
    {
        for ( int  j = 0 ; j < imgs.size() ; j ++)
        {
            if ( i != j )
            {
                for (int k = 0 ; k < matches[i][j].size() ; k ++)
                {
                    pts[i][i].push_back(keyss[i][ matches[i][j].at(k).queryIdx ].pt);
                    pts[i][j].push_back(keyss[i][ matches[i][j].at(k).trainIdx ].pt);
                }
            }
        }
    }

    ///-- Step 4: Compute homographic matrix


    Mat H[ imgs.size() ] [ imgs.size() ];

    for (int i = 0; i < imgs.size() ; i ++ )
    {
        for ( int  j = 0 ; j < imgs.size() ; j ++)
        {
            if ( i != j )
            {
                H[i][j] = findHomography( pts[i][i], pts[i][j], CV_RANSAC );
            }
        }
    }

    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( imgs.at(0).cols, 0 );
    obj_corners[2] = cvPoint( imgs.at(0).cols, imgs.at(0).rows ); obj_corners[3] = cvPoint( 0, imgs.at(0).rows );
    std::vector<Point2f> scene_corners(4);

    perspectiveTransform( obj_corners, scene_corners, H[0][1]);

    //draw image match
    Mat img_matches;
    drawMatches( imgs.at(0), keyss[0], imgs.at(1), keyss[1],
               matches[0][1], img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    imshow( "Good Matches & Object detection", img_matches );




    namedWindow( "Display" , WINDOW_AUTOSIZE );
    //moveWindow("Display", 0, 0);

    //imshow("Display",img);
    waitKey(0);
    return 0;
}*/


int main( int argc, char** argv )
{
      //if( argc != 3 )
      //{ readme(); return -1; }

      Mat img_object = imread( "img/IMG_6621.jpg", 0 );
      Mat img_scene = imread( "img/IMG_6621.jpg", 0);

      if( !img_object.data || !img_scene.data )
      {
        std::cout<< "Error reading images " << std::endl;
        return -1;
      }

      //-- Step 1: Detect the keypoints using SURF Detector
      int minHessian = 400;

      SurfFeatureDetector detector( minHessian );

      std::vector<KeyPoint> keypoints_object, keypoints_scene;

      detector.detect( img_object, keypoints_object );
      detector.detect( img_scene, keypoints_scene );

      //-- Step 2: Calculate descriptors (feature vectors)
      SurfDescriptorExtractor extractor;

      Mat descriptors_object, descriptors_scene;

      extractor.compute( img_object, keypoints_object, descriptors_object );
      extractor.compute( img_scene, keypoints_scene, descriptors_scene );

      //-- Step 3: Matching descriptor vectors using FLANN matcher
      FlannBasedMatcher matcher;
      std::vector< DMatch > matches;
      matcher.match( descriptors_object, descriptors_scene, matches );

      double max_dist = 0; double min_dist = 100;

      //-- Quick calculation of max and min distances between keypoints
      for( int i = 0; i < descriptors_object.rows; i++ )
      { double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
      }

      //printf("-- Max dist : %f \n", max_dist );
      //printf("-- Min dist : %f \n", min_dist );

      //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
      std::vector< DMatch > good_matches;

      for( int i = 0; i < descriptors_object.rows; i++ )
      {
        if( matches[i].distance < 3*min_dist )
            { good_matches.push_back( matches[i]); }
      }

      Mat img_matches;
      drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
                   good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                   vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

      //-- Localize the object
      std::vector<Point2f> obj;
      std::vector<Point2f> scene;

      for( int i = 0; i < good_matches.size(); i++ )
      {
        //-- Get the keypoints from the good matches
        obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
      }

      Mat H = findHomography( obj, scene, CV_RANSAC );

      //-- Get the corners from the image_1 ( the object to be "detected" )
      std::vector<Point2f> obj_corners(4);
      obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
      obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
      std::vector<Point2f> scene_corners(4);

      perspectiveTransform( obj_corners, scene_corners, H);

      //-- Draw lines between the corners (the mapped object in the scene - image_2 )
      line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
      line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
      line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
      line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );

      //-- Show detected matches
      imshow( "Matches", img_matches );

      waitKey(0);
      return 0;
  }
