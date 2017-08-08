#include <ros/ros.h>
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
    to_maintain_psm = *psm;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "test_acm_moveit_node");
    ros::NodeHandle n;

    ros::ServiceClient clear_octomap = n.serviceClient<std_srvs::Empty>("/dream_babbling/controller_node/clear_octomap");
    ros::Subscriber psm_sub = n.subscribe<moveit_msgs::PlanningScene>("/dream_babbling/controller_node/move_group/planning_scene", 1, planning_scene_cb);

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
    if(clear_map){
        ROS_INFO("I am Clearing the Octomap !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        clear_octomap.call(empty_octomap_request, empty_octomap_response);
    }

    const std::string PLANNING_SCENE_SERVICE = "get_planning_scene";
    planning_scene_monitor::PlanningSceneMonitorPtr psm =
            boost::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");

    psm->requestPlanningSceneState(PLANNING_SCENE_SERVICE);

    planning_scene_monitor::LockedPlanningSceneRW ps(psm);



    //ps->getCurrentStateNonConst().update();

    //planning_scene::PlanningScenePtr planning_scene_(new planning_scene::PlanningScene());
    //planning_scene_monitor::PlanningSceneMonitorPtr ps(new planning_scene_monitor::PlanningSceneMonitor());

    collision_detection::AllowedCollisionMatrix acm = ps->getAllowedCollisionMatrix();
    std::vector<std::string> acm_entry_names;
    ps->getAllowedCollisionMatrixNonConst().getAllEntryNames(acm_entry_names);

    ps->getAllowedCollisionMatrixNonConst().setEntry("<octomap>", acm_entry_names, true);
//    acm.setEntry("octomap", acm_entry_names, true);

    ps->getAllowedCollisionMatrixNonConst().getAllEntryNames(acm_entry_names);
    for(size_t i = 0; i < acm_entry_names.size(); i++)
        ROS_INFO_STREAM("The Entry number: " << i << " is: " << acm_entry_names[i]);

    std::vector<std::string> objects_ids = ps->getCollisionWorld()->getWorld()->getObjectIds();

    ROS_WARN("****************************************************");

    for(size_t i = 0; i < objects_ids.size(); i++)
        ROS_INFO_STREAM("The Entry number: " << i << " is: " << objects_ids[i]);
    ROS_WARN("-------------------------------------------------------");



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

//    double x, y, z, xmin = 0.3, xmax = 0.8, ymin = -0.2, ymax = 0.70, zmin = 0.2, zmax = 0.5;

    /*while(ros::ok()){
        planning_scene->setPlanningSceneDiffMsg(to_maintain_psm);
        ROS_ERROR_STREAM("The active collision detector name is: " << planning_scene->getActiveCollisionDetectorName());
        MoveGroup::Plan plan;
        x = (xmax - xmin) * ( (double)rand() / (double)RAND_MAX ) + xmin;
        y = (ymax - ymin) * ( (double)rand() / (double)RAND_MAX ) + ymin;
        z = (zmax - zmin) * ( (double)rand() / (double)RAND_MAX ) + zmin;

        group.setPositionTarget(x, y, z);

        ROS_ERROR_STREAM("The target is set, now we plan to go to X = " << x << ", Y = " << y << " and Z = " << z);
        if(group.plan(plan)){
            ROS_ERROR_STREAM("I Found a plan I will do nothing !!!!");
            //group.execute(plan);
        }
        else{
            ROS_WARN("No plan found, or Something went wrong to prevent the successful feedback of the planning");
            robot_state::RobotState& current_robot_state = planning_scene->getCurrentStateNonConst();
            collision_detection::CollisionRequest collision_request;
            collision_detection::CollisionResult collision_result;
            collision_request.contacts = true;
            collision_request.max_contacts = 1000;
            ROS_ERROR_STREAM("This failed trajectory has: " << plan.trajectory_.joint_trajectory.points.size() << " points");
            for(size_t i = 0; i < plan.trajectory_.joint_trajectory.points.size(); i++){
                collision_result.clear();
                moveit::core::jointTrajPointToRobotState(plan.trajectory_.joint_trajectory, i, current_robot_state);
                planning_scene->checkCollision(collision_request, collision_result, current_robot_state, acm);
                ROS_INFO_STREAM("Test: Current state is "
                                << (collision_result.collision ? "in" : "not in")
                                << " self collision");
                collision_detection::CollisionResult::ContactMap::const_iterator it;
                for(it = collision_result.contacts.begin();
                    it != collision_result.contacts.end();
                    ++it)
                {
                    ROS_INFO("Contact between: %s and %s",
                             it->first.first.c_str(),
                             it->first.second.c_str());
                }
                //current_robot_state.
            }
        }
        ROS_INFO("PRESS ENTER For Another try ....");
        std::cin.ignore();
        //ros::spinOnce();
    }*/


    return 0;
}