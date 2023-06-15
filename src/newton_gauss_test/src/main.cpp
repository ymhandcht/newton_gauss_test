#include<ros/ros.h>
#include<iostream>
#include<chrono>
#include<opencv2/opencv.hpp>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<vector>

using namespace std;
using namespace cv;
using namespace Eigen;

int main(int argc,char **argv)
{
    ros::init(argc, argv, "main");
    //生成真实数据
    double ar = 1.0, br = 2.0, cr = 1.0;
    double ae = 2.0, be = -1.0, ce = 5.0;
    int N = 100;//生成100个数据点
    double sigma = 1.0;
    cv::RNG rng;//生成随机数
    vector<double> x_data, y_data;
    for (int i = 0; i < N;i++)
    {
        double x = i / 100.0; //目的生成0-1的数据
        x_data.push_back(x);
        double y = exp(ar * x * x + br * x + cr) + rng.gaussian(sigma * sigma);
        y_data.push_back(y);
    }
    //开始进行迭代
    int iterators = 100;
    double cost = 0, lastcost = 0;
    chrono::steady_clock::time_point pt1 = chrono::steady_clock::now();
    for (int it = 0; it < iterators;it++)
    {
        Matrix3d H = Matrix3d::Zero();
        Vector3d b = Vector3d::Zero();
        cost = 0;
        for (int j = 0; j < N; j++)
        {
            double xi = x_data[j], yi = y_data[j];
            double error = yi - exp(ae * xi * xi + be * xi + ce);
            Vector3d J = Vector3d::Zero();
            J[0] = -xi * xi * exp(ae * xi * xi + be * xi + ce);
            J[1] = -xi * exp(ae * xi * xi + be * xi + ce);
            J[2] = -exp(ae * xi * xi + be * xi + ce);
            H += J * J.transpose();
            cost += error * error;
            b += -error * J;
        }
        Vector3d dx = H.ldlt().solve(b);
        if(it>0&&cost>=lastcost&&cost<100){
            cout << "lastcost>cost" << endl;
            break;
        }
        ae += dx[0];
        be += dx[1];
        ce += dx[2];
        lastcost = cost;
    }
    chrono::steady_clock::time_point pt2 = chrono::steady_clock::now();
    chrono::duration<double> time = chrono::duration_cast<chrono::duration<double>>(pt2 - pt1);
    cout << "time cost = " << time.count() * 1000 << "ms" << endl;
    cout << "ae = " << ae << " be = " << be << " ce = " << ce << endl;

    return 0;
}