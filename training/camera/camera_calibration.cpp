#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <cassert>

using namespace std;
using namespace cv;

// size of the board: (points per row, points per colume)
// (don't count the points on the edges)
const int board_w = 9, board_h = 6;
const int board_corners_num = board_w * board_h;


// find corner of a set of chessboard images
int findCorners(vector< vector<Point2f> > &images_corners, Size &img_size) 
    // param 1: result of corners of the set of images
    // param 2: result of img_size
    // return: num of successfully dected image
{
    Size board_size(board_w, board_h);
    int success_num = 0;
    bool is_first_image = false;

    for(int image_idx = 0; image_idx < 23; image_idx++)
    {
        // read image
        string path = PROJECT_DIR"/photos/";
        path.append(to_string(image_idx));
        path.append("_orig.jpg");
        Mat src = imread(path);

        // get the size of image
        if(!is_first_image)
        {
            img_size.width = src.cols;
            img_size.height = src.rows;
        }

        // find corners of this image
        vector<Point2f> corners;
        bool isFound = findChessboardCorners(src, board_size, corners, CALIB_CB_ADAPTIVE_THRESH);
        if(isFound && corners.size() == board_corners_num)
        {
            Mat gray_img;
            cvtColor(src, gray_img, COLOR_RGB2GRAY);
            find4QuadCornerSubpix(gray_img, corners, board_size);
            images_corners.push_back(corners);
            Mat drawn_img = src.clone();
            success_num++;
            drawChessboardCorners(drawn_img, board_size, corners, isFound);
            imshow("drawn", drawn_img);
            waitKey(10);
        }
        else
        {
            cout << "Cannot fully detect corners of image " << image_idx << "." << endl;
        }
    }
    return success_num;

}

void calibrate(vector< vector<Point2f> > &images_corners, int images_num, Size &img_size)
    // param 1: corners of the set of images
    // param 2: num of images in the "images_corners"
    // param 3: size of the image
{
    Size square_size( 10, 10 ); // real world size of the square
    vector< vector < Point3f > > world_images_corners; // world coordinate of corners of images set
    vector < Point3f > world_corners; // world coordinate of corners of one image

    // generate world_corners
    for(int i = 0; i < board_w; ++i)
    {
        for(int j = 0; j < board_h; ++j)
        {
            Point3f pt;
            pt.x = square_size.width * i;
            pt.y = square_size.height * j;
            world_corners.push_back(pt);
        }
    }
    // generate world_images_corners
    for(int img_idx = 0; img_idx < images_num; img_idx++)
    {
        world_images_corners.push_back(world_corners);
    }

    // calibrate camera
    Mat camera_matrix(3, 3, CV_32FC1, Scalar::all(0));
    Mat dist_coeffs(1, 5, CV_32FC1, Scalar::all(0));
    vector< Mat > rvecs, tvecs;
    double error = calibrateCamera(world_images_corners, images_corners, img_size, camera_matrix, dist_coeffs, rvecs, tvecs);

    cout << "error: " << error << endl;
    cout << "camera matrix: " << camera_matrix << endl; 
    cout << "dist coeffs: " << dist_coeffs << endl; 

}

int main()
{
    vector< vector<Point2f> > images_corners;
    Size img_size;
    int success_num = findCorners(images_corners, img_size);
    cout << "successfully dectected " << success_num << " imgaes" <<endl;
    calibrate(images_corners, success_num, img_size);
    return 0;
}
