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

    ros::ServiceClient clear_octomap = n.serviceClient<std_srvs::Empty>("/dream_babbling/controller_node/clear_octomap");
    ros::Subscriber psm_sub = n.subscribe<moveit_msgs::PlanningScene>("move_group/monitored_planning_scene", 1, planning_scene_cb);
    ros::Publisher psm_pub = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::ServiceClient get_ps = n.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");

    ros::AsyncSpinner my_spin(2);
    my_spin.start();

    MoveGroup group(MoveGroup::Options("left_arm", MoveGroup::ROBOT_DESCRIPTION, n));

    //    planning_scene::PlanningScene test;
    //    test.setCurrentState();


    group.setPlanningTime(5);
    group.setPlannerId("RRTConnectkConfigDefault");
//    planning_scene_monitor::PlanningSceneMonitorPtr psm = planning_scene_monitor::PlanningSceneMonitorPtr(
//                new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

    std_srvs::Empty::Request empty_octomap_request;
    std_srvs::Empty::Response empty_octomap_response;

    bool clear_map = false;
    n.getParam("/clear_octomap", clear_map);
//    if(clear_map){
//        ROS_INFO("I am Clearing the Octomap !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
//        clear_octomap.call(empty_octomap_request, empty_octomap_response);
//    }

//    const std::string PLANNING_SCENE_SERVICE = "get_planning_scene";
//    planning_scene_monitor::PlanningSceneMonitorPtr psm =
//            boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

//    psm->requestPlanningSceneState(PLANNING_SCENE_SERVICE);

//    planning_scene_monitor::LockedPlanningSceneRW ps(psm);

//    ps->getCurrentStateNonConst().update();

    std::vector<std::string> acm_entry_names;
//    ps->getAllowedCollisionMatrixNonConst().getAllEntryNames(acm_entry_names);

//    ps->getAllowedCollisionMatrixNonConst().setEntry("<octomap>", acm_entry_names, true);

    moveit_msgs::GetPlanningScene::Request ps_req;
    moveit_msgs::GetPlanningScene::Response ps_res;

    ps_req.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;

    get_ps.call(ps_req, ps_res);

    ROS_INFO_STREAM("From Service Response, default acm names size is: " << ps_res.scene.allowed_collision_matrix.default_entry_names.size());
    ROS_INFO_STREAM("From Service Response, acm names size is: " << ps_res.scene.allowed_collision_matrix.entry_names.size());


    ps_res.scene.allowed_collision_matrix.default_entry_names.push_back("<octomap>");
    ps_res.scene.allowed_collision_matrix.default_entry_values.push_back(true);

    moveit_msgs::PlanningScene ps_msg;
    ps_msg.is_diff = true;
    ps_msg.allowed_collision_matrix = ps_res.scene.allowed_collision_matrix;

//    acm.setEntry("octomap", acm_entry_names, true);

//    ps->getAllowedCollisionMatrixNonConst().getAllEntryNames(acm_entry_names);
//    for(size_t i = 0; i < acm_entry_names.size(); i++)
//        ROS_INFO_STREAM("The Entry number: " << i << " is: " << acm_entry_names[i]);

//    std::vector<std::string> objects_ids = ps->getCollisionWorld()->getWorld()->getObjectIds();

//    ROS_WARN("****************************************************");

//    for(size_t i = 0; i < objects_ids.size(); i++)
//        ROS_INFO_STREAM("The Entry number: " << i << " is: " << objects_ids[i]);
//    ROS_WARN("-------------------------------------------------------");



//    const std::string id = "octomap";
//    collision_detection::WorldConstPtr my_world = test->getCollisionWorld()->getWorld();
//    my_world->removeObject(id);
    //ps->getCollisionWorld()->getWorld()->removeObject("<octomap>");

//    objects_ids = ps->getCollisionWorld()->getWorld()->getObjectIds();

//        ROS_WARN("****************************************************");

//        for(size_t i = 0; i < objects_ids.size(); i++)
//            ROS_INFO_STREAM("The Entry number: " << i << " is: " << objects_ids[i]);
//        ROS_WARN("-------------------------------------------------------");
    //ROS_ERROR_STREAM("Finished preparing  !!!!!!!!!!!!!!!!");

    double x, y, z, xmin = 0.3, xmax = 0.8, ymin = -0.2, ymax = 0.70, zmin = 0.2, zmax = 0.5;

    while(ros::ok()){
        if(clear_)
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
            ps_req.components.components = moveit_msgs::PlanningSceneComponents::ROBOT_STATE;
//            get_ps.call(ps_req, ps_res);


//            robot_state::RobotState current_robot_state;
//            moveit::core::robotStateMsgToRobotState(ps_res.scene.robot_state, current_robot_state);
//            collision_detection::CollisionRequest collision_request;
//            collision_detection::CollisionResult collision_result;
//            collision_request.contacts = true;
//            collision_request.max_contacts = 1000;
//            ROS_ERROR_STREAM("This failed trajectory has: " << plan.trajectory_.joint_trajectory.points.size() << " points");
//            for(size_t i = 0; i < plan.trajectory_.joint_trajectory.points.size(); i++){
//                collision_result.clear();
//                moveit::core::jointTrajPointToRobotState(plan.trajectory_.joint_trajectory, i, current_robot_state);
//                ->checkCollision(collision_request, collision_result, current_robot_state, acm);
//                ROS_INFO_STREAM("Test: Current state is "
//                                << (collision_result.collision ? "in" : "not in")
//                                << " self collision");
//                collision_detection::CollisionResult::ContactMap::const_iterator it;
//                for(it = collision_result.contacts.begin();
//                    it != collision_result.contacts.end();
//                    ++it)
//                {
//                    ROS_INFO("Contact between: %s and %s",
//                             it->first.first.c_str(),
//                             it->first.second.c_str());
//                }
//                //current_robot_state.
//            }
        }
        ROS_INFO("PRESS ENTER For Another try ....");
        std::cin.ignore();
        //ros::spinOnce();
    }


    return 0;
}
