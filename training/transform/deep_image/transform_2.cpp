#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// include of eigen must in front of opencv2/core/eigen.hpp
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

using namespace std;

int main() {
    // depth: 深度图
    cv::Mat depth = cv::imread(PROJECT_DIR"/1_dpt.tiff", cv::IMREAD_ANYDEPTH);
    const int COLS = depth.cols, ROWS = depth.rows;
    cv::Mat img = cv::imread(PROJECT_DIR"/1.jpg", 0); // must add 0
    cv::FileStorage reader(PROJECT_DIR"/data.yml", cv::FileStorage::READ);
    // c_mat: camera matrix(内参矩阵)
    // p_mat: perspective matrix(透视变换矩阵) of 3d points
    cv::Mat c_mat, p_mat;
    Eigen::Matrix<double, 3, 3> C;
    Eigen::Matrix<double, 4, 4> P;
    reader["C"] >> c_mat;
    reader["D"] >> p_mat;
    // change cv::Mat to Eigen::Matrix
    cv::cv2eigen(c_mat, C);
    cv::cv2eigen(p_mat, P);

    // 一下内容参考slam十四讲云点拼接内容
    // get coefficients of camera matrix
    double fx = C(0, 0);
    double fy = C(1, 1);
    double cx = C(0, 2);
    double cy = C(1, 2);
    cout << fx << ' ' << fy << ' ' << cx << ' ' << cy << endl;

    cv::Mat result = cv::Mat(ROWS, COLS, CV_8UC1);
    // origin image coordinate (u1,v1)
    // after transform image coordinate (u2,v2)
    for(int u1 = 0; u1 < COLS; u1++) // note: u is column, v is row
    {
        for(int v1 = 0; v1 < ROWS; v1++)
        {
            // change image coordinate to world coordinate (x1, y1, z1)

            // cannot read it as double
            double z1 = (double)depth.at<float>(v1, u1);
            double x1 = (u1 - cx) * z1 / fx;
            double y1 = (v1 - cy) * z1 / fy;

            Eigen::Vector4d q1(x1, y1, z1, 1);
            // perform perspective transform to world coordinate
            Eigen::Vector4d q2 = P * q1;
            // change world coordinate to image coordinate
            double x2 = q2(0);
            double y2 = q2(1);
            double z2 = q2(2);
            double u2 = (x2/z2)*fx + cx;
            double v2 = (y2/z2)*fy + cy;

            // load them into result
            if(u2 >= 0 && v2 >= 0 && u2 < COLS && v2 < ROWS)
            {
                // note (v,u) in cv::Mat, not (u,v)
                // v,u should be int
                result.at<uchar>((int)v2, (int)u2) = img.at<uchar>(v1, u1);
            }
        }
    }

    cv::imshow("result", result);

    cv::waitKey( 0 );
    return 0;
}
