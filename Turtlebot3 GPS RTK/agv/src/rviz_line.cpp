//-------------------------------------------------------------------------------------------------------
//PRATICA SRL – www.praticasrl.com
//Progetto: AGV - Veicolo a guida automatica per movimentazione pallet
//Progetto cofinanziato dal Fondo POR FESR 14/20 Calabria
//Autori:
//Duardo Domenico
//Chilà Antonino
//Scimonelli Mattia
//--------------------------------------------------------------------------------------------------------

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

//#include <tf/transform_listener.h>

#include <cmath>

  
//Variabili globali
visualization_msgs::Marker line_strip_rtls, line_strip_gps, line_strip_real, line_strip_odom;
ros::Publisher marker_pub_rtls, marker_pub_gps, marker_pub_real, marker_pub_odomf;
geometry_msgs::Point p;

geometry_msgs::TransformStamped t_rtls, t_gps;

#define N 5000
//N Numero punti disegnati
  
//ricezione dati pose_rtls
void pose_rtls_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{

	//Trasformo da rtls a map
	geometry_msgs::PoseWithCovarianceStamped msg_conv;
	tf2::doTransform(msg, msg_conv, t_rtls);

	line_strip_rtls.header.stamp = ros::Time::now();
	p.x = msg_conv.pose.pose.position.x;
	p.y = msg_conv.pose.pose.position.y;
	p.z = 0;
	
	if(line_strip_rtls.points.size()>=N)
	{
		line_strip_rtls.points.erase(line_strip_rtls.points.begin());
	}

	//Append
	line_strip_rtls.points.push_back(p);
	//pubblica i marker
	marker_pub_rtls.publish(line_strip_rtls);	
	

}

//ricezione dati pose_gps
void pose_gps_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{

	//Trasformo da gps a map
	geometry_msgs::PoseWithCovarianceStamped msg_conv;
	tf2::doTransform(msg, msg_conv, t_gps);

	line_strip_gps.header.stamp = ros::Time::now();
	p.x = msg_conv.pose.pose.position.x;
	p.y = msg_conv.pose.pose.position.y;
	p.z = 0;
	
	if(line_strip_gps.points.size()>=N)
	{
		line_strip_gps.points.erase(line_strip_gps.points.begin());
	}

	//Append
	line_strip_gps.points.push_back(p);
	//pubblica i marker
	marker_pub_gps.publish(line_strip_gps);	
	

}

//ricezione dati pose_real
void pose_real_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
	line_strip_real.header.stamp = ros::Time::now();
	p.x = msg.pose.pose.position.x;
	p.y = msg.pose.pose.position.y;
	p.z = 0;

	if(line_strip_real.points.size()>=N)
	{
		line_strip_real.points.erase(line_strip_real.points.begin());
	}

	line_strip_real.points.push_back(p);
	marker_pub_real.publish(line_strip_real);	

  
}

//ricezione dati odometry_filtered
void odom_callback(const nav_msgs::Odometry& msg)
{
	line_strip_odom.header.stamp = ros::Time::now();
	p.x = msg.pose.pose.position.x;
	p.y = msg.pose.pose.position.y;
	p.z = 0;

	if(line_strip_odom.points.size()>=N)
	{
		line_strip_odom.points.erase(line_strip_odom.points.begin());
	}

	line_strip_odom.points.push_back(p);
	marker_pub_odomf.publish(line_strip_odom);	
	
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "rviz_line");
	ros::NodeHandle n;
	marker_pub_rtls = n.advertise<visualization_msgs::Marker>("vm_pose_rtls", 10);
	marker_pub_gps = n.advertise<visualization_msgs::Marker>("vm_pose_gps", 10);
	marker_pub_real = n.advertise<visualization_msgs::Marker>("vm_pose_real", 10);
	marker_pub_odomf = n.advertise<visualization_msgs::Marker>("vm_odom_filtered", 10); 
	  
	//SUBSCRIBERS
	//pose_rtls
	ros::Subscriber sub_pose_rtls= n.subscribe("pose_rtls", 100, pose_rtls_callback);
	//pose_gps
	ros::Subscriber sub_pose_gps= n.subscribe("pose_gps", 100, pose_gps_callback);
	//pose_real
	ros::Subscriber sub_pose_real = n.subscribe("pose_real", 100, pose_real_callback);
	//odometry_filtered
  	ros::Subscriber sub_odom = n.subscribe("odometry/filtered_ok", 100, odom_callback);
  
	//pose_rtls markers settings
	line_strip_rtls.header.frame_id = "map";
	line_strip_rtls.id = 1;
	line_strip_rtls.type = visualization_msgs::Marker::LINE_STRIP;
	line_strip_rtls.scale.x = 0.01;
	line_strip_rtls.color.b = 1.0;
	line_strip_rtls.color.a = 1.0;
  
	//pose_gps markers settings
	line_strip_gps.header.frame_id = "map";
	line_strip_gps.id = 4;
	line_strip_gps.type = visualization_msgs::Marker::LINE_STRIP;
	line_strip_gps.scale.x = 0.01;
	line_strip_gps.color.r = 255;
	line_strip_gps.color.g = 255;
	line_strip_gps.color.b = 0;
	line_strip_gps.color.a = 1.0;
	
	//pose_real markers settings
	line_strip_real.header.frame_id = "map";
	line_strip_real.id = 2;
	line_strip_real.type = visualization_msgs::Marker::LINE_STRIP;
	line_strip_real.scale.x = 0.01;
	line_strip_real.color.g = 1.0;
	line_strip_real.color.a = 1.0;
	
	
	//odometry_filtered markers settings
	line_strip_odom.header.frame_id = "odom";
	line_strip_odom.id = 3;
	line_strip_odom.type = visualization_msgs::Marker::LINE_STRIP;
	line_strip_odom.scale.x = 0.01;
	line_strip_odom.color.r = 1.0;
	line_strip_odom.color.a = 1.0;
	
	//Ricavo la trasformazione map rtls da ROS
	tf2_ros::Buffer tfBuffer1;
	tf2_ros::TransformListener tf2_listener1(tfBuffer1);
	try
	{
		t_rtls = tfBuffer1.lookupTransform("map", "rtls", ros::Time(0), ros::Duration(10.0) );
	}
	catch (tf2::TransformException &ex) 
	{
		ROS_WARN("rviz_line: Errore %s",ex.what());
	}

	//Ricavo la trasformazione map gps da ROS
	tf2_ros::Buffer tfBuffer2;
	tf2_ros::TransformListener tf2_listener2(tfBuffer2);
	try
	{
		t_gps = tfBuffer2.lookupTransform("map", "gps", ros::Time(0), ros::Duration(10.0) );
	}
	catch (tf2::TransformException &ex) 
	{
		ROS_WARN("rviz_line: Errore %s",ex.what());
	}

	ROS_INFO("rviz_line: Nodo attivo!");

  ros::spin();
}
