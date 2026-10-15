#include <plan_manage/planner_manager.h>
#include <plan_env/sdf_map.h>
#include <obj_state_msgs/ObjectsStates.h>
#include <stdexcept>
#include <iostream>
void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
int main(int argc,char** argv){
 ros::init(argc,argv,"fapp_fuel_smoke");ros::NodeHandle nh("~");
 nh.setParam("sdf_map/resolution",.1);nh.setParam("sdf_map/map_size_x",8.0);
 nh.setParam("sdf_map/map_size_y",8.0);nh.setParam("sdf_map/map_size_z",3.0);
 nh.setParam("sdf_map/ground_height",0.0);nh.setParam("sdf_map/obstacles_inflation",.1);
 nh.setParam("sdf_map/max_ray_length",5.0);nh.setParam("sdf_map/virtual_ceil_height",2.9);
 nh.setParam("manager/use_optimization",true);nh.setParam("manager/max_vel",1.0);
 nh.setParam("manager/max_acc",2.0);nh.setParam("manager/control_points_distance",.2);
 nh.setParam("manager/bspline_degree",3);
 nh.setParam("fapp/enabled",true);nh.setParam("fapp/topic","/test/fapp/states");
 nh.setParam("optimization/ld_smooth",20.0);nh.setParam("optimization/ld_dist",25.0);
 nh.setParam("optimization/ld_feasi",10.0);nh.setParam("optimization/ld_start",100.0);
 nh.setParam("optimization/ld_end",100.0);nh.setParam("optimization/ld_guide",5.0);
 nh.setParam("optimization/ld_time",1.0);nh.setParam("optimization/dist0",.3);
 nh.setParam("optimization/max_vel",1.0);nh.setParam("optimization/max_acc",2.0);
 nh.setParam("optimization/algorithm1",11);nh.setParam("optimization/algorithm2",11);
 for(int i=1;i<=4;++i){nh.setParam("optimization/max_iteration_num"+std::to_string(i),200);
 nh.setParam("optimization/max_iteration_time"+std::to_string(i),.02);}
 fast_planner::FastPlannerManager manager;manager.initPlanModules(nh);
 auto pub=nh.advertise<obj_state_msgs::ObjectsStates>("/test/fapp/states",1,true);
 for(int i=0;i<50&&pub.getNumSubscribers()==0;++i){ros::spinOnce();ros::WallDuration(.02).sleep();}
 require(pub.getNumSubscribers()>0,"FAPP subscriber missing");
 const Eigen::Vector3d a(-1.5,0,1),b(1.5,0,1);
 require(manager.planStraightTraj(a,b),"baseline planning failed");
 require(manager.isTrajectorySafe(),"baseline trajectory unsafe");
 const auto baseline=manager.local_data_.position_traj_.getControlPoint();
 const auto baseline_data=manager.local_data_;
 obj_state_msgs::ObjectsStates msg;msg.header.frame_id="UAV0/camera_init";
 obj_state_msgs::State o;o.position.z=1;o.size.x=.2;o.size.y=.2;o.size.z=.2;
 msg.states.push_back(o);
 auto send=[&](){msg.header.stamp=ros::Time::now();pub.publish(msg);
 for(int i=0;i<5;++i){ros::WallDuration(.01).sleep();ros::spinOnce();}};
 send();require(!manager.isTrajectorySafe(),"active obstacle not checked before release");
 manager.local_data_.start_time_=ros::Time::now();double distance;
 require(!manager.checkTrajCollision(distance),"active obstacle not checked in execution");
 send();require(manager.planStraightTraj(a,b),"dynamic replan failed");
 require(manager.isTrajectorySafe(),"dynamic optimized trajectory still collides");
 double deviation=0;auto& t=manager.local_data_.position_traj_;
 for(double s=0;s<t.getTimeSum();s+=.02)deviation=std::max(deviation,std::abs(t.evaluateDeBoorT(s).z()-1));
 require(deviation>.2,"dynamic trajectory did not detour");
 // A measured obstacle below the ball rules out the lower candidate.
 obj_state_msgs::State lower;lower.position.z=.45;lower.size.x=1.8;lower.size.y=.6;lower.size.z=.4;
 msg.states.push_back(lower);send();
 require(manager.planStraightTraj(a,b)&&manager.isTrajectorySafe(),"lateral detour failed");
 double lateral=0;auto& side=manager.local_data_.position_traj_;
 for(double t=0;t<side.getTimeSum();t+=.02)lateral=std::max(lateral,std::abs(side.evaluateDeBoorT(t).y()));
 require(lateral>.3,"blocked lower passage did not select a side");
 // A fully blocked route remains a native planning failure, not a colliding output.
 msg.states.resize(1);msg.states[0].size.x=8;msg.states[0].size.y=8;msg.states[0].size.z=4;send();
 require(!manager.planStraightTraj(a,b),"fully blocked route accepted");
 nh.setParam("fapp/enabled",false);
 fast_planner::FastPlannerManager disabled;disabled.initPlanModules(nh);
 require(disabled.planStraightTraj(a,b)&&disabled.isTrajectorySafe(),"disabled mode consumed obstacles");
 require(disabled.local_data_.position_traj_.getControlPoint().isApprox(baseline,1e-12),"disabled mode changed baseline");
 // Planner priority accepts a native trajectory despite current dynamic overlap.
 nh.setParam("fapp/enabled",true);nh.setParam("fapp/planner_priority",true);
 fast_planner::FastPlannerManager priority;priority.initPlanModules(nh);
 priority.local_data_=baseline_data;
 send();
 require(priority.isTrajectorySafe(),"priority mode vetoed release on FAPP overlap");
 priority.local_data_.start_time_=ros::Time::now();
 require(priority.checkTrajCollision(distance),"priority mode vetoed execution on FAPP overlap");
 // A fully overlapping prediction must not prevent the native optimizer from running.
 require(priority.planExploreTraj({a,Eigen::Vector3d(0,0,1),b},
     Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()),"priority guide still vetoed native optimization");
 // Use an ordinary target to verify that the soft FAPP cost still changes optimization.
 msg.states={o};send();
 require(priority.planExploreTraj({a,Eigen::Vector3d(0,0,1),b},
     Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()),"priority optimization failed");
 const auto with_dynamic=priority.local_data_.position_traj_.getControlPoint();
 msg.states.clear();send();
 require(priority.planExploreTraj({a,Eigen::Vector3d(0,0,1),b},
     Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()),"priority empty optimization failed");
 require(!with_dynamic.isApprox(priority.local_data_.position_traj_.getControlPoint(),1e-6),
     "priority mode removed dynamic optimization cost");
 // Static map checks remain effective before release and during execution.
 priority.local_data_=baseline_data;
 priority.edt_environment_->sdf_map_->setOccupied(Eigen::Vector3d(0,0,1));
 require(!priority.isTrajectorySafe(),"priority mode bypassed static release check");
 priority.local_data_.start_time_=ros::Time::now();
 require(!priority.checkTrajCollision(distance),"priority mode bypassed static execution check");
 std::cout<<"PASS planner priority release/execution, native optimization, retained dynamic cost and static checks"<<std::endl;
 msg.states.clear();send();require(manager.planStraightTraj(a,b)&&manager.isTrajectorySafe(),"empty update did not restore baseline");
 msg.states.push_back(o);send();ros::WallDuration(.85).sleep();
 require(manager.isTrajectorySafe(),"stale obstacle retained");
 std::cout<<"PASS FUEL subscription, release/execution checks, lower/side detours, blocked route, disabled baseline, empty update and expiry"<<std::endl;
 return 0;
}
