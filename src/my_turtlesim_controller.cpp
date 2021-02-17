#include "my_turtlesim_controller/my_turtlesim_controller.h"

MyTurtlesimController::MyTurtlesimController():private_nh("~")
{
    private_nh.param("hz",hz,{100});
    private_nh.param("polygon_num",polygon_num,{3});
    private_nh.param("turn_flag",turn_flag,{false});
    private_nh.param("one_side_length",one_side_length,{3});
    private_nh.param("moved_length",moved_length,{0.0});
    private_nh.param("integrated_theta",integrated_theta,{0.0});

    sub_pose = nh.subscribe("/turtle1/pose",10,&MyTurtlesimController::pose_callback,this);

    pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1);
}

void MyTurtlesimController::pose_callback(const turtlesim::Pose::ConstPtr &msg)
{
    past_pose = current_pose;
    current_pose = *msg;
    if(current_pose.theta*past_pose.theta < 0) integrated_theta +=0;
    else integrated_theta += fabs(current_pose.theta - past_pose.theta);

    moved_length += sqrt((current_pose.x - past_pose.x)*(current_pose.x - past_pose.x)+(current_pose.y - past_pose.y)*(current_pose.y - past_pose.y));

    if(integrated_theta > 2*M_PI/polygon_num)
    {
        turn_flag = false;
        integrated_theta = 0.0;
        moved_length = 0.0;
    }

    if(moved_length > one_side_length)
    {
        turn_flag = true;
        moved_length = 0.0;
        integrated_theta = 0.0;
    }

}

void MyTurtlesimController::go_straight()
{
    ROS_INFO_STREAM(current_pose);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.1;

    pub_cmd_vel.publish(cmd_vel);
}

void MyTurtlesimController::draw_polygon()
{
    fflush(stdout);
    printf("\rmoved_length: %f\n",moved_length);
    fflush(stdout);
    printf("\rintegrated_theta: %f\n",integrated_theta);
    fflush(stdout);
    std::cout<<"\rcurrent_pose\n"<<current_pose<<std::endl;
    geometry_msgs::Twist cmd_vel;
    if(turn_flag) cmd_vel.angular.z = 0.1;
    else cmd_vel.linear.x = 0.3;
    pub_cmd_vel.publish(cmd_vel);
}

void MyTurtlesimController::process()
{
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
        // go_straight();
        draw_polygon();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc,char** argv)
{
    ros::init(argc,argv, "my_turtlesim_controller");
    MyTurtlesimController my_turtlesim_controller;
    my_turtlesim_controller.process();
    return 0;
}
