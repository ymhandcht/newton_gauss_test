#include<iostream>
#include<chrono>
#include<ros/ros.h>
#include<g2o/core/g2o_core_api.h>
#include<g2o/core/base_vertex.h>
#include<g2o/core/base_unary_edge.h>
#include<g2o/core/block_solver.h>


using namespace std;

int main(int argc,char **argv)
{
    ros::init(argc, argv, "g2o_test");

    return 0;
}