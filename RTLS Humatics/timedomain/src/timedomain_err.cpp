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
#include <ros/time.h>
#include <signal.h>

#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <errno.h>

#include <inttypes.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define MYBUFSIZE 1024
bool run_forever=true;

void mySigintHandler(int sig)
{
	run_forever=false;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
	int my_socket=0; //Socket
	struct sockaddr_in sck; 
	char buf[MYBUFSIZE];

	int ret=0; //Ritorno funzioni generale
	std::string s; //Stringa
	int count=0;

	//ROS	
	ros::init(argc, argv, "timedomain", ros::init_options::NoSigintHandler);
	ros::NodeHandle nh("~");
	signal(SIGINT, mySigintHandler);
	ros::Rate loop_rate(50);

	ros::Publisher pose_pub;
	geometry_msgs::PoseWithCovarianceStamped pose;

	nh.param("rtls_frame", pose.header.frame_id, std::string("rtls"));

	//Matrice covarianza
  	double pcov_ok[36] = { 0.09,    0,   0,   0,  0, 0,
                                  0, 0.09,   0,   0,  0, 0,
                                  0,   0, 1e6,   0,   0, 0,
                                  0,   0,   0, 1e6,   0, 0,
                                  0,   0,   0,   0, 1e6, 0,
                                  0,   0,   0,   0,   0, 1e6};
	memcpy(&(pose.pose.covariance),pcov_ok,sizeof(double)*36);

	pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose_rtls", 100);


	//Inizio configurazione TIMEDOMAIN
	ROS_INFO("Timedomain INIT...");

	//Verifica parametri (si deve passare IP)
	if (!nh.hasParam("ip"))
	{
		ROS_INFO("ERRORE parametri!");
		ret=system("pkill roslaunch");
	}
	nh.getParam("ip", s);
	ROS_INFO("Timedomain ip: %s", s.c_str());

	//Creo Socket
	if ((my_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) 
	{
		ROS_INFO("ERRORE socket!");
		ret=system("pkill roslaunch");
	}
	memset(&sck, 0, sizeof(sck));
    	sck.sin_family = AF_INET;
    	sck.sin_addr.s_addr = inet_addr(s.c_str());
    	sck.sin_port = htons(atoi("21210"));
	//Timeout 
	struct timeval timeout;      
    	timeout.tv_sec = 5;
    	timeout.tv_usec = 0;
	setsockopt (my_socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout));
	ROS_INFO("Timedomain INIT OK");

	//MODALITA' TRACKING
	ROS_INFO("Timedomain TRACKING...");
	char packet_TRACKING[] = {0x50, 0x03, 0x00, 0x02, 0x02, 0x01, 0x00, 0x00};
	if (sendto(my_socket, packet_TRACKING, 8, 0, (struct sockaddr *)&sck, sizeof(sck)) < 0) {
		ROS_INFO("ERRORE invio!");
		ret=system("pkill roslaunch");
        }

	sleep(1);
	//MODALITA' TRACKING
	if (sendto(my_socket, packet_TRACKING, 8, 0, (struct sockaddr *)&sck, sizeof(sck)) < 0) {
		ROS_INFO("ERRORE invio!");
		ret=system("pkill roslaunch");
        }
	

	ROS_INFO("Timedomain TRACKING OK");

	int32_t i32=0;
	int16_t i16=0;
	uint16_t u16=0;
	double x=0.0;
	double y=0.0;
	double x_var=0.0;
	double y_var=0.0;
	double xy_cov=0.0;

	//PRONTO A LEGGERE POSIZIONI
	while (ros::ok() && run_forever)
  	{
  		ret=recvfrom(my_socket, buf, MYBUFSIZE, 0, NULL, NULL);
		if(buf[0]==0x52 && buf[1]==0x03 && buf[7] == 0x67 && buf[8]==0x00 && buf[10]==0x00)
		{
			//Conversioni X e Y
			memcpy(&i32, &buf[20],4);
			i32 = ntohl(i32); //Funzione magica che aggiusta il formato
			x=(double)i32/1000;
			//ROS_INFO("Timedomain %" PRId32, i32);			
			memcpy(&i32, &buf[24],4);
			i32 = ntohl(i32); //Funzione magica che aggiusta il formato
			y=(double)i32/1000;

			//Conversioni var X e var Y
			memcpy(&u16, &buf[32],2);
			u16 = ntohs(u16); //Funzione magica che aggiusta il formato
			x_var=(double)u16/100000000;
			//ROS_INFO("Timedomain %" PRIu16, u16);			
			memcpy(&u16, &buf[34],2);
			u16 = ntohs(u16); //Funzione magica che aggiusta il formato
			y_var=(double)u16/100000000;

			//ROS_INFO("Timedomain X=%.3f Y=%.3f X_VAR=%f Y_VAR=%f", x, y, x_var, y_var);	

			//Conversioni cov
			//memcpy(&i16, &buf[38],2);
			//i16 = ntohl(i16); //Funzione magica che aggiusta il formato
			//xy_cov=(double)i16/10000;
			//ROS_INFO("Timedomain X=%.3f Y=%.3f X_VAR=%f Y_VAR=%f XY_COV=%f", x, y, x_var, y_var, xy_cov);	

			//Invio Topic ROS
			pose.pose.pose.position.x = x;
			pose.pose.pose.position.y = y;
			pose.pose.pose.position.z = 0;
		
			ros::Time time_now = ros::Time::now();
			pose.header.stamp = time_now;
 			pose_pub.publish(pose);
		}
  	}

	//MODALITA' IDLE
	ROS_INFO("Timedomain IDLE...");
	char packet_IDLE[] = {0x50, 0x03, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00};
	if (sendto(my_socket, packet_IDLE, 8, 0, (struct sockaddr *)&sck, sizeof(sck)) < 0) {
		ROS_INFO("ERRORE invio!");
		ret=system("pkill roslaunch");
        }

	sleep(1);
	//MODALITA' IDLE
	if (sendto(my_socket, packet_IDLE, 8, 0, (struct sockaddr *)&sck, sizeof(sck)) < 0) {
		ROS_INFO("ERRORE invio!");
		ret=system("pkill roslaunch");
        }
				
	ROS_INFO("Timedomain IDLE OK");

	return 0;
}
