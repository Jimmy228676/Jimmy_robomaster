#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

using namespace cv;

void doPerspectiveTransform( Mat input, Mat& output ) {
    std::vector<Point2f> srcQuad( 4 ), dstQuad( 4 );
    output = input.clone();
    srcQuad[2].x = 2265, srcQuad[3].x =  3133 , srcQuad[0].x = 2273, srcQuad[1].x = 3141;
    srcQuad[2].y = 1325, srcQuad[3].y = 1101, srcQuad[0].y = 857, srcQuad[1].y = 709;
    dstQuad[2].x = 0, dstQuad[3].x = 1200, dstQuad[0].x = 0, dstQuad[1].x = 1200;
    dstQuad[2].y = 400, dstQuad[3].y = 400, dstQuad[0].y = 0, dstQuad[1].y = 0;

    Mat warp_matrix = getPerspectiveTransform( srcQuad, dstQuad );

    warpPerspective( input, output, warp_matrix, Size( 1200, 400 ) );
}


int main() {
    Mat img = imread(PROJECT_DIR"/car.jfif");
    Mat result;

    doPerspectiveTransform( img, result );

    namedWindow( "input" );
    namedWindow( "output" );

    // imshow( "input", img );
    imshow( "output", result );

    waitKey( 0 );

    destroyWindow( "input" );
    destroyWindow( "output" );
    return 0;
}
