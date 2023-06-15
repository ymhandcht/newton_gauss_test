#include<ros/ros.h>
#include<iostream>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<ceres/ceres.h>
#include<opencv2/opencv.hpp>

using namespace std;

struct curve_fitting_cost{
    curve_fitting_cost(double x, double y):_x(x),_y(y){}
    //代价函数计算模型
    template<typename T>
    bool operator()(const T *const abc, T *residual)const //成员函数后面加const保证该成员函数不会修改成员属性的值
    {
        residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]);//我们的残差是一维的
        return true;
    }
    const double _x, _y;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ceres_test");
    //生成数据
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
    double abc[3] = {ae, be, ce};
    //构建最小二乘问题
    ceres::Problem problem;
    for (int i = 0; i < N; i++)
    {
        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<curve_fitting_cost, 1, 3>(new curve_fitting_cost(x_data[i], y_data[i])), 
        nullptr, 
        abc);
    }
    //配置求解器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    chrono::steady_clock::time_point pt1 = chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);
    chrono::steady_clock::time_point pt2 = chrono::steady_clock::now();
    chrono::duration<double> time = chrono::duration_cast<chrono::duration<double>>(pt2 - pt1);
    cout << "time cost = " << time.count() * 1000 << "ms" << endl;
    //输出结果
    cout << summary.BriefReport() << endl;
    cout << "a,b,c = " << endl;
    for(auto a:abc)
        cout << a << " ";
    cout << endl;


    return 0;
}