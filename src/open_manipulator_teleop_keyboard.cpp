#include <string>
#include <vector>
#include <iostream>
#include <cstring>

#include "open_manipulator_teleop/open_manipulator_teleop_keyboard.h"

bool stop_auto = false;
bool h_ok = false;
bool v_ok = false;
bool d_ok = false;


OpenManipulatorTeleop::OpenManipulatorTeleop()
: node_handle_(""),
  priv_node_handle_("~")
{
  /************************************************************
  ** Initialize variables
  ************************************************************/
  present_joint_angle_.resize(NUM_OF_JOINT);
  present_kinematic_position_.resize(3);

  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initSubscriber();
  initClient();

  disableWaitingForEnter();
  ROS_INFO("OpenManipulator teleoperation using keyboard start");
}

OpenManipulatorTeleop::~OpenManipulatorTeleop(){
  restoreTerminalSettings();
  ROS_INFO("Terminate OpenManipulator Joystick");
  ros::shutdown();
}

void OpenManipulatorTeleop::initClient(){
  goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
  goal_joint_space_path_from_present_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path_from_present");
  goal_task_space_path_from_present_position_only_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_from_present_position_only");
  goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
}

void OpenManipulatorTeleop::initSubscriber(){
  joint_states_sub_ = node_handle_.subscribe("joint_states", 10, &OpenManipulatorTeleop::jointStatesCallback, this);
  kinematics_pose_sub_ = node_handle_.subscribe("kinematics_pose", 10, &OpenManipulatorTeleop::kinematicsPoseCallback, this);
  center_coordinates_sub_ = node_handle_.subscribe("bbox_center", 10, &OpenManipulatorTeleop::boundingBoxCenterCallback, this); 
}

void OpenManipulatorTeleop::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg){
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);
  for (std::vector<int>::size_type i = 0; i < msg->name.size(); i ++)
  {
    if (!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;
}

void OpenManipulatorTeleop::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg){
  // std::cout << "here" << std::endl;
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);
  present_kinematic_position_ = temp_position;
}

void OpenManipulatorTeleop::boundingBoxCenterCallback(const std_msgs::String::ConstPtr &msg){
  if(!stop_auto){
    ROS_INFO(msg->data.c_str());
    if(!v_ok || !h_ok || !d_ok){
      if(v_ok) std::cout << "v_ok" << std::endl;
      if(h_ok) std::cout << "h_ok" << std::endl;
      if(d_ok) std::cout << "d_ok" << std::endl;
      moveArm(msg->data.c_str());
    }
  }
  else{
    
    ROS_INFO("Locked in on Object. Stopping Movement.");
  }
}

std::vector<double> OpenManipulatorTeleop::parseMsg(std::string msgData){
  std::vector<double> res;
  std::string temp = "";
  for(auto c : msgData){
      if(c!=',') temp += c;
      else{
          res.push_back(stod(temp));
          temp.clear();
      }
  }
  res.push_back(stod(temp));
  return res;
  //delemit information based on string
}

void OpenManipulatorTeleop::moveArm(std::string msg){
  bool of = true;
  std::vector<double> goalPose;  goalPose.resize(3, 0.0);
  //coudl try to see if the size is 1
  if(msg.size() == 1){
    of = false;
    ROS_INFO("No Object has been detected. Arm will not move");
  }
  //move to a separate function that will do this
  if(of){
    std::vector<double> splitMsg = parseMsg(msg);
    double x = splitMsg[0]; double y = splitMsg[1]; double d = splitMsg[2];
    // std::cout << "MSG: ";
    // for(int i=0; i<splitMsg.size(); i++){
    //   std::cout << splitMsg[i] << " ";
    // }
    // std::cout << std::endl << std::flush;
    //open gripper
    std::vector<double> joint_angle;
    joint_angle.push_back(0.01);
    setToolControl(joint_angle);

    bool dist_ok = true;
    
    if(d > 0.20001 && d < 0.7){
      goalPose.resize(3, 0.0);
      goalPose.at(0) = DELTA;
      setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
    //close gripper
    // std::vector<double> joint_angle;
    // joint_angle.push_back(-0.01);
    // setToolControl(joint_angle);
    }
    // else if(d <= 0.158){ //|| center_dist <= 0.158
    //   //close grabber
    //   // std::vector<double> joint_angle;
    //   // joint_angle.push_back(0.01);
    //   // setToolControl(joint_angle);

    //   if(d <= 0.155){
    //     return true;
    //   }
    // }
    else if(d < 0.2){
      dist_ok = true;
      if(d > 0.0){
        d_ok = true;
        ROS_INFO("Close Enough");
        ROS_INFO("Stopping Auto Pilot");
        stop_auto = true;
        
        acquireObject();
        return;
      }
      
      
    }
    // else{
       // dist_ok = false;
    // }
    

    if(dist_ok){
      // if(h_ok) std::cout << "h_ok" << std::endl;
      // if(v_ok) std::cout << "v_ok" << std::endl;
      if(x >= 475) {
        printf("\n<- LEFT <-\t"); //-delta y axes        
        goalPose.resize(3, 0.0);
        goalPose.at(1) = -DELTA;
        setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
      }
      else if(x <= 370) {
        printf("\n-> RIGHT ->\t"); //+deltta y axis
        goalPose.resize(3, 0.0);
        goalPose.at(1) = DELTA;
        setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
      }
      else{
        printf("\n| CENTER |\t");
        h_ok = true;
      }


      if (y <= 310){
        printf("\t + UP +\n");
        // printf("input : z \tincrease(++) z axis in task space\n");
        //we want to move the arm up increase in z direction
        goalPose.resize(3, 0.0);
        goalPose.at(2) = DELTA;
        setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
      }
      else if (y >=390){
        printf("\t - DOWN - \n");
        // printf("input : x \tdecrease(--) z axis in task space\n");
        //we want to move the arm down decrease in z direction
        goalPose.resize(3, 0.0);
        goalPose.at(2) = -DELTA;
        setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);  
      }
      else{
        // center_ok = true;
        printf("\t| CENTER |\n");
        v_ok=true;
      }
    }
  }  
  else{
    std::cout << "#" << std::endl;
  }
}

void OpenManipulatorTeleop::acquireObject(){
  ROS_INFO("Grabbing Object");
  std::vector<double> goalPose;  goalPose.resize(3, 0.0);
  std::vector<double> joint_angle;


  //move forward

  

  //move down
  // goalPose.resize(3, 0.0);
  // goalPose.at(2) = DELTA;
  // for(int i = 0; i<1; i++){
  //   setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  //   usleep(10000);
  // }

  // sleep(1);

  goalPose.resize(3, 0.0);
  goalPose.at(0) = DELTA; 

  for(int i = 0; i<130; i++){
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
    usleep(10000);
  }

  

  //turn a little to the right
  // printf("\n-> RIGHT ->\t"); //+deltta y axis
  // goalPose.resize(3, 0.0);
  // goalPose.at(1) = DELTA;
  // for(int i = 0; i<20; i++){
  //   setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  //   usleep(10000);
  // }


  //grab object
  //open grabber
  joint_angle.clear();
  joint_angle.push_back(0.01);
  setToolControl(joint_angle);
  //close grabber
  joint_angle.clear();
  joint_angle.push_back(0.00);
  setToolControl(joint_angle);  

  sleep(2);


  setGoal('2');
  ROS_INFO("Completed");
}

std::vector<double> OpenManipulatorTeleop::getPresentJointAngle(){
  return present_joint_angle_;
}

std::vector<double> OpenManipulatorTeleop::getPresentKinematicsPose(){
  return present_kinematic_position_;
}

bool OpenManipulatorTeleop::setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time){
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if (goal_joint_space_path_from_present_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time){
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if (goal_joint_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setToolControl(std::vector<double> joint_angle){
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name.push_back(priv_node_handle_.param<std::string>("end_effector_name", "gripper"));
  srv.request.joint_position.position = joint_angle;

  if (goal_tool_control_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time){
  open_manipulator_msgs::SetKinematicsPose srv;
  srv.request.planning_group = priv_node_handle_.param<std::string>("end_effector_name", "gripper");
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);
  srv.request.path_time = path_time;

  if (goal_task_space_path_from_present_position_only_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

void OpenManipulatorTeleop::printText(){
  printf("\n");
  printf("---------------------------\n");
  printf("Control Your OpenManipulator!\n");
  printf("---------------------------\n");
  printf("w : increase x axis in task space\n");
  printf("s : decrease x axis in task space\n");
  printf("a : increase y axis in task space\n");
  printf("d : decrease y axis in task space\n");
  printf("z : increase z axis in task space\n");
  printf("x : decrease z axis in task space\n");
  printf("\n");
  printf("y : increase joint 1 angle\n");
  printf("h : decrease joint 1 angle\n");
  printf("u : increase joint 2 angle\n");
  printf("j : decrease joint 2 angle\n");
  printf("i : increase joint 3 angle\n");
  printf("k : decrease joint 3 angle\n");
  printf("o : increase joint 4 angle\n");
  printf("l : decrease joint 4 angle\n");
  printf("\n");
  printf("g : gripper open\n");
  printf("f : gripper close\n");
  printf("       \n");
  printf("1 : init pose\n");
  printf("2 : home pose\n");
  printf("       \n");
  printf("q to quit\n");
  printf("---------------------------\n");
  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
         getPresentJointAngle().at(0),
         getPresentJointAngle().at(1),
         getPresentJointAngle().at(2),
         getPresentJointAngle().at(3));
  printf("Present Kinematics Position X: %.3lf Y: %.3lf Z: %.3lf\n",
         getPresentKinematicsPose().at(0),
         getPresentKinematicsPose().at(1),
         getPresentKinematicsPose().at(2));
  printf("---------------------------\n");
}

void OpenManipulatorTeleop::setGoal(char ch){
  std::vector<double> goalPose;  goalPose.resize(3, 0.0);
  std::vector<double> goalJoint; goalJoint.resize(NUM_OF_JOINT, 0.0);

  if (ch == 'w' || ch == 'W')
  {
    printf("input : w \tincrease(++) x axis in task space\n");
    goalPose.at(0) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (ch == 's' || ch == 'S')
  {
    printf("input : s \tdecrease(--) x axis in task space\n");
    goalPose.at(0) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (ch == 'a' || ch == 'A')
  {
    printf("input : a \tincrease(++) y axis in task space\n");
    goalPose.at(1) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (ch == 'd' || ch == 'D')
  {
    printf("input : d \tdecrease(--) y axis in task space\n");
    goalPose.at(1) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (ch == 'z' || ch == 'Z')
  {
    printf("input : z \tincrease(++) z axis in task space\n");
    goalPose.at(2) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (ch == 'x' || ch == 'X')
  {
    printf("input : x \tdecrease(--) z axis in task space\n");
    goalPose.at(2) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (ch == 'y' || ch == 'Y')
  {
    printf("input : y \tincrease(++) joint 1 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1"); goalJoint.at(0) = JOINT_DELTA;
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if (ch == 'h' || ch == 'H')
  {
    printf("input : h \tdecrease(--) joint 1 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1"); goalJoint.at(0) = -JOINT_DELTA;
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if (ch == 'u' || ch == 'U')
  {
    printf("input : u \tincrease(++) joint 2 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2"); goalJoint.at(1) = JOINT_DELTA;
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if (ch == 'j' || ch == 'J')
  {
    printf("input : j \tdecrease(--) joint 2 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2"); goalJoint.at(1) = -JOINT_DELTA;
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if (ch == 'i' || ch == 'I')
  {
    printf("input : i \tincrease(++) joint 3 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3"); goalJoint.at(2) = JOINT_DELTA;
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if (ch == 'k' || ch == 'K')
  {
    printf("input : k \tdecrease(--) joint 3 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3"); goalJoint.at(2) = -JOINT_DELTA;
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if (ch == 'o' || ch == 'O')
  {
    printf("input : o \tincrease(++) joint 4 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4"); goalJoint.at(3) = JOINT_DELTA;
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if (ch == 'l' || ch == 'L')
  {
    printf("input : l \tdecrease(--) joint 4 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4"); goalJoint.at(3) = -JOINT_DELTA;
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if (ch == 'g' || ch == 'G')
  {
    printf("input : g \topen gripper\n");
    std::vector<double> joint_angle;

    joint_angle.push_back(0.01);
    setToolControl(joint_angle);
  }
  else if (ch == 'f' || ch == 'F')
  {
    printf("input : f \tclose gripper\n");
    std::vector<double> joint_angle;
    joint_angle.push_back(-0.01);
    setToolControl(joint_angle);
  }
  else if (ch == '2')
  {
    printf("input : 2 \thome pose\n");
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(-1.05);
    joint_name.push_back("joint3"); joint_angle.push_back(0.35);
    joint_name.push_back("joint4"); joint_angle.push_back(0.70);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == '1')
  {
    printf("input : 1 \tinit pose\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(0.0);
    joint_name.push_back("joint3"); joint_angle.push_back(0.0);
    joint_name.push_back("joint4"); joint_angle.push_back(0.0);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
}

void OpenManipulatorTeleop::restoreTerminalSettings(void){
  tcsetattr(0, TCSANOW, &oldt_);  /* Apply saved settings */
}

void OpenManipulatorTeleop::disableWaitingForEnter(void){
  struct termios newt;

  tcgetattr(0, &oldt_);  /* Save terminal settings */
  newt = oldt_;  /* Init new settings */
  newt.c_lflag &= ~(ICANON | ECHO);  /* Change settings */
  tcsetattr(0, TCSANOW, &newt);  /* Apply settings */
}


int main(int argc, char **argv){
  // Init ROS node
  ros::init(argc, argv, "grabber");
  OpenManipulatorTeleop openManipulatorTeleop;

  // char ch;
  // openManipulatorTeleop.printText();
  if(ros::ok()) ros::spin();
  
  // while (ros::ok() && (ch = std::getchar()) != 'q')
  // {
  //   // ros::spinOnce();
  //   // openManipulatorTeleop.printText();
  //   // ros::spinOnce();
  //   openManipulatorTeleop.setGoal(ch);
  // }

  return 0;
}
