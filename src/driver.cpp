#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <soem/ethercattype.h>
#include <soem/nicdrv.h>
#include <soem/ethercatbase.h>
#include <soem/ethercatmain.h>
#include <soem/ethercatdc.h>
#include <soem/ethercatcoe.h>
#include <soem/ethercatfoe.h>
#include <soem/ethercatconfig.h>
#include <soem/ethercatprint.h>

#include <ros/ros.h>
#include "geometry_msgs/WrenchStamped.h"
#include "hirop_msgs/getOnRobotForce.h"
#include "hirop_msgs/setOnRobotForce.h"
#include <thread>
#include "unistd.h"
#include "csignal"
#define IFRAME "eth1"
#define DEBUG

using namespace std;
bool running = false;
float *buffer;
ros::Publisher state_pub;
geometry_msgs::Wrench tmp;

#define ETHERCAT_FUNCTION

boost::shared_ptr<ros::NodeHandle> n;
void runOnRobotThread()
{
    int ret = -1;
    ros::Rate loop_rate(200);
    ROS_INFO_STREAM("runOnRobotThread running...");
    while(ros::ok() && running){

#ifdef ETHERCAT_FUNCTION
        ec_send_processdata();
        ret = ec_receive_processdata(EC_TIMEOUTRET);
//        cout << "ec_receive_processdata ret = " << ret << endl;
        buffer = (float *)ec_slave[1].inputs;
    #ifdef DEBUG
//        cout << "Fx=" << buffer[0] << " || Fy= " << buffer[1] << " || Fz=" << buffer[2] << " || Tx=" << buffer[3] << " || Ty=" << buffer[4] << " || Tz=" << buffer[5] << endl;
    #endif
        tmp.force.x = buffer[0];tmp.force.y = buffer[1];tmp.force.z = buffer[2];
        tmp.torque.x = buffer[3];tmp.torque.y = buffer[4];tmp.torque.z = buffer[5];
#else
       tmp.force.x = 0; tmp.force.y = 0;tmp.force.z = 0;
       tmp.torque.x = 0;tmp.torque.y = 0;tmp.torque.z = 0;
#endif
        state_pub.publish(tmp);

        loop_rate.sleep();
    }
    ROS_INFO_STREAM("runOnRobotThread exit...");

}

void signalHandler(int signum)
{
  ROS_INFO("%s is received, Terminating the node...",strsignal(signum));
//  n->shutdown();
  exit(signum);
}

bool OnRobotEthercatServer(hirop_msgs::setOnRobotForceRequest& req,hirop_msgs::setOnRobotForceResponse& res){
    if(req.enable && running == false)
    {
        running = true;
        std::thread t1(runOnRobotThread);
        t1.detach();

    }else if(running == true && (!req.enable))
    {
        running = false;
    }
    return true;
}

int main(int argc, char **argv){

	int ret;
	char IOmap[4096];
	int chk;

    ros::init(argc, argv, "onRobot_daq_driver");
    ros::NodeHandle n;
    ROS_INFO_STREAM("init ..........");
//    n = boost::make_shared<ros::NodeHandle>();
#ifdef ETHERCAT_FUNCTION
    // 初始化soem
    ret = ec_init(IFRAME);
	if(ret < 0){
        ROS_INFO_STREAM("ec_init error, ret = " << ret );
		return -1;
	}	

	ret = ec_config_init(FALSE);	// 初始化并配置
	if(ret < 0){
        ROS_INFO_STREAM("ec_config_init error, ret = " << ret );
		return -1;	
	}

    ROS_INFO_STREAM("ec_slavecount = " << ec_slavecount );
	if(ec_slavecount > 0)
        ROS_INFO_STREAM("ec_slave0 name was: " << ec_slave[1].name );

	ec_config_map(&IOmap);
	ec_configdc();

	ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);	// 等待所有从站切换至safe OP状态
    ROS_INFO_STREAM("all slave state was safe op" );

	ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
	ec_writestate(0);

	chk = 40;
    do
    {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
    }
    while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL)); // 等待所有从站切换至OP状态

    ROS_INFO_STREAM("all slave state was op " );

    ROS_INFO_STREAM("---------------------- " << chk);
    ROS_INFO_STREAM("Obits" << ec_slave[1].Ibits );
#endif


    state_pub = n.advertise<geometry_msgs::Wrench>("daq_data", 1000);

    ros::ServiceServer service = n.advertiseService("set_onRobot_force_enable",OnRobotEthercatServer);
    ROS_INFO_STREAM( "onRobot_daq_driver start ....");
    signal(SIGINT,signalHandler);

    running = true;
    std::thread t1(runOnRobotThread);
    t1.detach();
    ros::spin();

	return 0;
}



