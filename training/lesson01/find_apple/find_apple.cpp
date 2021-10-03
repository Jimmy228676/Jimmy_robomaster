#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <cassert>

// parameters
int red_h1 = 23, red_h2 = 156;
int red_s = 156, red_v = 55;
int open_size = 9;
int close_size = 9;

cv::Mat src;
char win_name[] = "set_para";

void para_track(int ,void *)
{
    // change color space from rgb to hsv
    cv::Mat hsv;
    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);

    // set red color range
    cv::Mat red_part1, red_part2;
    cv::inRange(hsv, cv::Scalar(0, red_s, red_v), cv::Scalar(red_h1, 255, 255), red_part1);
    cv::inRange(hsv, cv::Scalar(red_h2, red_s, red_v), cv::Scalar(180, 255, 255),red_part2);

    // get the union of these range
    cv::Mat ones_mat = cv::Mat::ones(cv::Size(src.cols, src.rows), CV_8UC1);
    cv::Mat result = 255 * (ones_mat - (ones_mat - red_part1 / 255).mul(ones_mat - red_part2 / 255));

    // open operation
    if(open_size>0)
    {
        cv::Mat element1 = getStructuringElement(cv::MORPH_RECT, cv::Size(open_size, open_size));
        cv::morphologyEx(result, result, cv::MORPH_OPEN, element1);
    }

    // close operation
    if(close_size>0)
    {
        cv::Mat element2 = getStructuringElement(cv::MORPH_RECT, cv::Size(close_size, close_size));
        cv::morphologyEx(result, result, cv::MORPH_CLOSE, element2);
    }

    // find & filtrate & draw contours
    std::vector< std::vector<cv::Point> > contours;
    std::vector< cv::Vec4i > hierarchy;
    cv::findContours( result, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    std::vector<cv::Rect> boundRect( contours.size() );
    cv::Mat result_src = src.clone(); // warning: should not draw on the src, because src is global valuable
    for( int i = 0; i < contours.size(); i++ ) {
        boundRect[i] = boundingRect(cv::Mat(contours[i]));
        // only draw the rectangle which is big enough
        if(boundRect[i].width > 50 && boundRect[i].height > 50)
        {
            rectangle(result_src, cv::Point(boundRect[i].x, boundRect[i].y),
                      cv::Point(boundRect[i].x + boundRect[i].width, boundRect[i].y + boundRect[i].height),
                      cv::Scalar(255, 0, 0), 2, 8);
        }
    }

    imshow(win_name,result);
    imshow("apple",result_src);
}

int main()
{
    src=cv::imread(PROJECT_DIR"/apple.png");
    assert(src.data);

    namedWindow(win_name,cv::WINDOW_AUTOSIZE);
    // trackbars for setting the parameters
    cv::createTrackbar("red_h_left：",win_name,&red_h1,50,para_track);
    cv::createTrackbar("red_h_right：",win_name,&red_h2,180,para_track);
    cv::createTrackbar("red_s：",win_name,&red_s,200,para_track);
    cv::createTrackbar("red_v：",win_name,&red_v,100,para_track);
    cv::createTrackbar("open_cal：",win_name,&open_size,20,para_track);
    cv::createTrackbar("close_cal：",win_name,&close_size,20,para_track);

    cv::waitKey(0);
    return 0;
}