#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <std_srvs/Empty.h>

using namespace moveit::planning_interface;
moveit_msgs::PlanningScene to_maintain_psm;
collision_detection::AllowedCollisionMatrix acm;

void planning_scene_cb(const moveit_msgs::PlanningSceneConstPtr& psm){
    acm = psm->allowed_collision_matrix;
    std::vector<std::string> acm_entry_names;
    acm.getAllEntryNames(acm_entry_names);
    std::ostream stream(nullptr);
    stream.rdbuf(std::cout.rdbuf());
//    acm.print(stream);

//    collision_detection::AllowedCollision::Type type;
//    acm.getDefaultEntry("<octomap>", type);

//    ROS_INFO_STREAM("The octomap collision type is: " << type);

//    ROS_WARN("------------------------------------------------------");
    /*
    for(size_t i = 0; i < acm_entry_names.size(); i++)
        ROS_INFO_STREAM("The Entry number: " << i << " is: " << acm_entry_names[i]);
    ROS_WARN("****************************************************");

    for(size_t i = 0; i < psm->world.collision_objects.size(); i++)
        ROS_INFO_STREAM("The Entry number: " << i << " is: " << psm->world.collision_objects[i]);
    ROS_WARN("-------------------------------------------------------");*/

}

int main(int argc, char **argv){
    ros::init(argc, argv, "test_acm_moveit_node");
    ros::NodeHandle n;

    ros::Subscriber psm_sub = n.subscribe<moveit_msgs::PlanningScene>("move_group/monitored_planning_scene", 1, planning_scene_cb);
    ros::Publisher psm_pub = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::ServiceClient get_ps = n.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");

    ros::AsyncSpinner my_spin(2);
    my_spin.start();

    MoveGroup group(MoveGroup::Options("left_arm", MoveGroup::ROBOT_DESCRIPTION, n));

    group.setPlanningTime(5);
    group.setPlannerId("RRTConnectkConfigDefault");
    bool clear_map = false;


    moveit_msgs::GetPlanningScene::Request ps_req;
    ps_req.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    moveit_msgs::GetPlanningScene::Response ps_res;
    moveit_msgs::PlanningScene ps_msg;


    double x, y, z, xmin = 0.3, xmax = 0.8, ymin = -0.2, ymax = 0.70, zmin = 0.2, zmax = 0.5;

    while(ros::ok()){
        n.getParam("/clear_octomap", clear_map);
        ROS_INFO_STREAM("The clear_octomap parameter is set to: " << clear_map);
        get_ps.call(ps_req, ps_res);
        if(clear_map){
            ps_res.scene.allowed_collision_matrix.default_entry_names.push_back("<octomap>");
            ps_res.scene.allowed_collision_matrix.default_entry_values.push_back(true);
        }
        else{
            ps_res.scene.allowed_collision_matrix.default_entry_names.clear();
            ps_res.scene.allowed_collision_matrix.default_entry_values.clear();
        }

        ROS_INFO_STREAM("From Service Response, default acm names size is: " << ps_res.scene.allowed_collision_matrix.default_entry_names.size());
        ROS_INFO_STREAM("From Service Response, acm names size is: " << ps_res.scene.allowed_collision_matrix.entry_names.size());
        ps_msg.is_diff = true;
        ps_msg.allowed_collision_matrix = ps_res.scene.allowed_collision_matrix;
        psm_pub.publish(ps_msg);

        MoveGroup::Plan plan;
        x = (xmax - xmin) * ( (double)rand() / (double)RAND_MAX ) + xmin;
        y = (ymax - ymin) * ( (double)rand() / (double)RAND_MAX ) + ymin;
        z = (zmax - zmin) * ( (double)rand() / (double)RAND_MAX ) + zmin;

        group.setPositionTarget(x, y, z);

        ROS_ERROR_STREAM("The target is set, now we plan to go to X = " << x << ", Y = " << y << " and Z = " << z);
        if(group.plan(plan)){
            ROS_ERROR_STREAM("I Found a plan I will do nothing !!!!");
            group.execute(plan);
        }
        else{
            ROS_WARN("No plan found, or Something went wrong to prevent the successful feedback of the planning");

        }
        ROS_INFO("PRESS ENTER For Another try ....");
        std::cin.ignore();
        //ros::spinOnce();
    }


    return 0;
}
