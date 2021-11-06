#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// 回调函数 of setMouseCallback function
void getClick(int event, int x, int y, int flags, void* srcQuad)
{
    // when press down the mouse
    if(event==EVENT_LBUTTONDOWN)
    {
        // copy the coordinate of the mouse to the srcQuad
        vector< Point2f >* pointSet = (vector< Point2f >*) srcQuad;
        pointSet->push_back(Point2f(x, y));
        cout << "get a point" << endl;
    }
}

void doPerspectiveTransform( Mat& input, Mat& output ) {
    // srcQuad: points of the picture
    // dstQuad: points of the real world
    std::vector<Point2f> srcQuad, dstQuad( 4 );
    output = input.clone();
    dstQuad[0] = Point2f(0, 0);
    dstQuad[1] = Point2f(400, 0);
    dstQuad[2] = Point2f(0, 100);
    dstQuad[3] = Point2f(400, 100);

    namedWindow( "input" );
    imshow( "input", input );

    // get input through the mouse
    setMouseCallback("input", getClick, &srcQuad);
    waitKey(0);

    assert(srcQuad.size() == 4);

    // 变换矩阵
    Mat warp_matrix = getPerspectiveTransform(srcQuad, dstQuad);
    // 根据变换矩阵生成变换后图像
    warpPerspective( input, output, warp_matrix, Size( 400, 100));
}


int main() {
    Mat img = imread(PROJECT_DIR"/car.jpg");
    Mat result;

    doPerspectiveTransform( img, result );

    namedWindow( "output" );
    imshow( "output", result );

    waitKey( 0 );
    return 0;
}
