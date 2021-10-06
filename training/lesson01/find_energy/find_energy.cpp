#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iostream>
#include <cassert>


// parameters
int red_h1 = 23, red_h2 = 156;
int red_s = 120, red_v = 60;
int open_size = 2;
int close_size = 6;

int num_of_son(const std::vector< std::vector<cv::Point> > &contours,
               const std::vector< cv::Vec4i > &hierarchy, const int &id)
{
    if (id == -1)
        return -1;

    int son_num = 0;
    for (int i = hierarchy[id][2]; i != -1; i = hierarchy[i][0])
    {
        cv::RotatedRect minRect;
        minRect = minAreaRect(cv::Mat(contours[i]));
        float big_length = minRect.size.height;
        float small_length = minRect.size.width;
        if(big_length<small_length)
        {
            std::swap(big_length, small_length);
        }
        if(minRect.size.area() < 100 || big_length < 25) { continue; }
        son_num++;
    }
    return son_num;
}

bool isValid(const std::vector< std::vector<cv::Point> > &contours,
             const std::vector< cv::Vec4i > &hierarchy,
             const int &id,
             float min_area=0, float big_length_min=0, float small_length_min=0, float max_area=50000,
             float ratio_max=100, float ratio_min=0)
{
    cv::RotatedRect minRect;
    minRect = minAreaRect(cv::Mat(contours[id]));
    float big_length = minRect.size.height;
    float small_length = minRect.size.width;
    float area = minRect.size.area();
    if(big_length<small_length)
    {
        std::swap(big_length, small_length);
    }
    //std::cout<<area<<' '<<big_length<<' '<<small_length<<std::endl;
    if(area<max_area && area>min_area && big_length>big_length_min && small_length>small_length_min &&
        big_length/small_length > ratio_min && big_length/small_length < ratio_max)
    {
        return true;
    }
    return false;
}

// draw the hull or ellipse of the fan or "R"
void draw_fan(const std::vector< std::vector<cv::Point> > &contours,
               const std::vector< cv::Vec4i > &hierarchy, const int &id, cv::Mat &drawing, const cv::Point &center)
{
    static cv::Scalar COLOR_LIST[3] = { {220, 20, 20}, {20, 220, 20},
                                        {20, 20, 220} };
    if(id == -1) { return; }
    int son_num = num_of_son(contours, hierarchy, id);
    if(son_num == 1)
    {
        // the thin one
        if(isValid(contours, hierarchy, id, 5000, 80, 30))
        {
            int son_id = hierarchy[id][2];
            std::vector<cv::Point> father_hull;
            // find the target point of the father hull
            convexHull(cv::Mat(contours[id]), father_hull, false);
            cv::Point target_point;
            for(auto& point : father_hull)
            {
                // selected the nearest point from the center
                if((point-center).ddot(point-center) < (target_point-center).ddot(target_point-center))
                { target_point = point; }
            }
            std::vector<std::vector<cv::Point>> result_hull(2);
            convexHull(cv::Mat(contours[son_id]), result_hull[0], false);
            result_hull[0].push_back(target_point);
            convexHull(cv::Mat(result_hull[0]), result_hull[1], false);
            cv::drawContours(drawing, result_hull,1,COLOR_LIST[1],2,8 );
        }

    }
    else if(son_num > 1)
    {
        // the fat one
        if(isValid(contours, hierarchy, id, 5000, 80, 30))
        {
            bool flag = false; // whether find the target
            int son_id = 0;
            for (int i = hierarchy[id][2]; i != -1; i = hierarchy[i][0])
            {
                if(isValid(contours, hierarchy, i, 300, 25, 15, 10000, 2.7))
                {
                    son_id = i;
                    flag = true;
                    break;
                }
            }
            if(!flag) { return; }

            std::vector<cv::Point> father_hull;
            // find the target point of the father hull
            convexHull(cv::Mat(contours[id]), father_hull, false);
            cv::Point tmp_point1=father_hull[0], tmp_point2(0,0), target_point;
            for(auto& point : father_hull)
            {
                // selected the 2 nearest point from the center
                if((point-center).ddot(point-center) < (tmp_point1-center).ddot(tmp_point1-center))
                { tmp_point1 = point; }
            }
            for(auto& point : father_hull)
            {
                if(point==tmp_point1) { continue; }
                // selected the 2 nearest point from the center
                if((point-center).ddot(point-center) < (tmp_point2-center).ddot(tmp_point2-center))
                { tmp_point2 = point; }
            }
            target_point = (tmp_point1 + tmp_point2) / 2;
            std::vector<std::vector<cv::Point>> result_hull(2);
            convexHull(cv::Mat(contours[son_id]), result_hull[0], false);
            result_hull[0].push_back(target_point);
            convexHull(cv::Mat(result_hull[0]), result_hull[1], false);
            cv::drawContours(drawing, result_hull,1,COLOR_LIST[0],2,8 );
        }
    }

}

cv::Mat src;
char win_name[] = "set_para";

void draw_all(int ,void *)
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
    cv::findContours( result, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
    cv::Mat result_src = src.clone(); // warning: should not draw on the src, because src is global valuable

    // find center
    cv::Point2f center;
    bool isFindCenter = false;
    for (int i = 0; i != -1; i = hierarchy[i][0])
    {
        if(isValid(contours, hierarchy, i, 50, 8, 8, 500, 1.5) &&
                num_of_son(contours, hierarchy, i) == 0)
        {
            float radius;
            minEnclosingCircle(contours[i],center, radius);
            circle(result_src,center,(int)radius,cv::Scalar(255,255,255),2);
            isFindCenter = true;
            break;
        }
    }
    if(!isFindCenter) { return; }


    for (int i = 0; i != -1; i = hierarchy[i][0])
    {
        draw_fan(contours, hierarchy, i, result_src, center);
    }

    //imshow(win_name,result);
    imshow("energy",result_src);
}

int main()
{
    cv::VideoCapture capture(PROJECT_DIR"/video.mp4");

    for(int i = 0; i<10; ++i)
    {
        capture.read(src);
    }

    while(capture.read(src))
    {
        draw_all(0, nullptr);
        cv::waitKey(50);
    }

    /*
    // trackbars for setting the parameters
    namedWindow(win_name,cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("red_h_left：",win_name,&red_h1,50,draw_all);
    cv::createTrackbar("red_h_right：",win_name,&red_h2,180,draw_all);
    cv::createTrackbar("red_s：",win_name,&red_s,200,draw_all);
    cv::createTrackbar("red_v：",win_name,&red_v,100,draw_all);
    cv::createTrackbar("open_cal：",win_name,&open_size,20,draw_all);
    cv::createTrackbar("close_cal：",win_name,&close_size,20,draw_all);
    */
    cv::waitKey(0);
    return 0;
}