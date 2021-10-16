#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <cassert>

using namespace cv;
using namespace std;

int main() {

    // size of the board: (points per row, points per colume)
    // (don't count the points on the edges)
    const int board_w = 11, board_h = 8;
    const int board_n = board_w * board_h;
    Size board_size( 11, 8 );

    Mat gray_img, drawn_img;
    vector< Point2f > point_pix_pos_buf;
    vector< vector<Point2f> > point_pix_pos; // cross points positions in the pictures

    int found, successes = 0;
    Size img_size;

    int cnt = 0;
    int k = 0, n = 0;
    for (int i = 0; i < 2; i++){
        string path = PROJECT_DIR"/photos/";
        path.append(to_string(i));
        path.append(".jpg");
        Mat src0 = imread(path);
        cout << path << endl;

        assert(!src0.empty());

        // get size of src0
        if ( !cnt ) {
            img_size.width = src0.cols;
            img_size.height = src0.rows;
        }
        found = findChessboardCorners( src0, board_size, point_pix_pos_buf );
        if ( found && point_pix_pos_buf.size() == board_n ) {
            successes++;
            cvtColor( src0, gray_img, COLOR_BGR2GRAY );
            find4QuadCornerSubpix( gray_img, point_pix_pos_buf, Size( 5, 5 ) );
            point_pix_pos.push_back( point_pix_pos_buf );
            drawn_img = src0.clone();
            drawChessboardCorners( drawn_img, board_size, point_pix_pos_buf, found );
            imshow( "corners", drawn_img );
            waitKey( 1000 );
        } else
            cout << "\tbut failed to found all chess board corners in this image" << endl;
        point_pix_pos_buf.clear();
        cnt++;

    };
    cout << successes << " useful chess boards" << endl;

    Size square_size( 10, 10 );
    vector< vector< Point3f > > point_grid_pos; // cross points positions in world coordinate
    vector< Point3f > point_grid_pos_buf;
    vector< int > point_count;

    Mat camera_matrix( 3, 3, CV_32FC1, Scalar::all( 0 ) );
    Mat dist_coeffs( 1, 5, CV_32FC1, Scalar::all( 0 ) );
    vector< Mat > rvecs;
    vector< Mat > tvecs;

    for (int i = 0; i < successes; i++ ) {
        for (int j = 0; j < board_h; j++ ) {
            for (int k = 0; k < board_w; k++ ){
                Point3f pt;
                pt.x = k * square_size.width;
                pt.y = j * square_size.height;
                pt.z = 0;
                point_grid_pos_buf.push_back( pt );
            }
        }
        point_grid_pos.push_back( point_grid_pos_buf );
        point_grid_pos_buf.clear();
        point_count.push_back( board_h * board_w );
    }

    cout << calibrateCamera( point_grid_pos, point_pix_pos, img_size, camera_matrix, dist_coeffs, rvecs, tvecs ) << endl;
    cout << camera_matrix << endl << dist_coeffs << endl;
    return 0;
}
