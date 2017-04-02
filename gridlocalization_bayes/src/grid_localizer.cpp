#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <lab4/Motion.h>
#include <lab4/Observation.h>
#include "tf/transform_datatypes.h"
#include <algorithm>



#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#define M_PI  3.14159265358979323846 /* pi */

using namespace std;
using namespace ros;

double rot1, trans, rot2;
double rot, transO;
double threshold = 0.1;
double pos[35][35][4];
double temp_pos[35][35][4];



void initiate()
{
	for(int x = 0; x < 35; x++)
		for(int y = 0; y < 35; y++)
			for(int z = 0; z < 4; z++)
				pos[x][y][z] = 0;
}

struct Motion
{
    double rotation1, translation, rotation2;
};

void positionUpdate(Motion);

struct Observation
{
	double rotation, translation;
};

void updateObservation(int, double, double);

double tags_xPos[6] = {125,125,125,425,425,425};
double tags_yPos[6] = {525,325,125,125,325,525}; 

double getDegree(double rad)
{
	double result = rad * 180 / M_PI;
	return result;
}

double getPDF(double x, double mean, double var)
{
	return (1 / (sqrt(2 * M_PI) * var)) * pow(M_E, -1 * (((x - mean)*(x - mean))/(2 * var * var)));
}

void getPosition(int i, int j, int k, double & trans_x, double & trans_y, double & rot)
{
	trans_x = i * 20 + 10;
	trans_y = j * 20 + 10;
	rot = -180 + k * 90 + 45;
}


Observation getObservation(int i, int j, int k, int tagnum)
{
	double trans_x, trans_y, rot;
	double initial_angle;
	getPosition(i, j, k, trans_x, trans_y, rot);
	
	Observation obs;
	obs.translation = sqrt((trans_x - tags_xPos[tagnum]) * (trans_x - tags_xPos[tagnum]) + (trans_y - tags_yPos[tagnum]) * (trans_y - tags_yPos[tagnum]));
	initial_angle = getDegree(atan2(tags_yPos[tagnum]-trans_y, tags_xPos[tagnum] - trans_x));
	obs.rotation = initial_angle - rot;
	if(obs.rotation > 180)
		obs.rotation = obs.rotation - 360;
	if(obs.rotation < -180)
		obs.rotation = obs.rotation + 360;
	return obs;
}

Motion getMotionModel(int i, int j, int k, int i_t, int j_t, int k_t)
{
	double trans1_x, trans1_y, trans2_x, trans2_y;
	double rot1_1, rot2_1;
	double initial_angle;
	Motion motion;
	getPosition(i, j, k, trans1_x, trans1_y, rot1_1);
	getPosition(i_t, j_t, k_t, trans2_x, trans2_y, rot2_1);
	motion.translation = sqrt((trans1_x - trans2_x) * (trans1_x - trans2_x) + (trans1_y - trans2_y) * (trans1_y - trans2_y));
	initial_angle = getDegree(atan2(trans2_y-trans1_y, trans2_x - trans1_x));
	
	motion.rotation1 = rot1_1 - initial_angle;
	motion.rotation2 = initial_angle - rot2_1;
	if(motion.rotation1 > 180)
		motion.rotation1 = motion.rotation1 - 360;
	if(motion.rotation1 < -180)
		motion.rotation1 = motion.rotation1 + 360;
	if (motion.rotation2 > 180)
		motion.rotation2 = motion.rotation2 - 360;
	if(motion.rotation2 < -180)
		motion.rotation2 = motion.rotation2 + 360;
	return motion;
}

Publisher path;
Publisher tag_pub;
visualization_msgs::Marker marker;
visualization_msgs::Marker marker1;

void pose_publisher(int i, int j, int k)
{
	
	double index_angle, index_x, index_y;
	cout << "start pose" << endl;
	getPosition(i, j, k, index_x, index_y, index_angle);
	marker1.header.frame_id = "/world";
	marker1.header.stamp = Time(0);
	marker1.ns = "traj";
	marker1.id = 2;
	marker1.type = visualization_msgs::Marker::LINE_STRIP;
	
	marker1.scale.x = 0.1;
	marker1.scale.y = 0.0;
	marker1.scale.z = 0.0;
        
	marker1.color.r = 1.0;
	marker1.color.g = 0.0;
	marker1.color.b = 0.0;
	marker1.color.a = 1.0;
    geometry_msgs::Point p;
	p.x = index_x/100.0;
	p.y = index_y/100.0;
	p.z = 0;
	marker1.points.push_back(p);
	marker1.action = visualization_msgs::Marker::ADD;
	cout << "publish pose : " << p.x << " " << p.y << endl;
	path.publish(marker1);
    while (path.getNumSubscribers() < 1)
	 	int x = 1;	
	cout << "end pose" << endl;
	
}

void positionUpdate(Motion motion)
{
	
	Motion motionl;
	double norm_value = 0;
	double total_prob = 0;
	double trans_prb, trans_tmp;
	double probability_rot1 = 0, probability_rot2 = 0;
	double val = 0;
	double check;
	int index, index_x, index_y, index_angle;
	cout << "Starting position update " << endl;
	for(int a = 0; a < 35; a++)
	 	for(int b = 0; b < 35; b++)
	 		for(int c = 0; c < 4; c++)
	 			temp_pos[a][b][c] = pos[a][b][c];

	for(int i_t = 0; i_t < 35; i_t++)
	{
		for(int j_t = 0; j_t < 35; j_t++)
		{
			for(int k_t = 0; k_t < 4; k_t++)
			{
				check = temp_pos[i_t][j_t][k_t];
				if( check < threshold)
				{	

					for(int i = 0; i < 35; i++)
					{
						for(int j = 0; j < 35; j++)
						{
							for(int k = 0; k < 4; k++)
							{
								motionl = getMotionModel(i, j, k, i_t, j_t, k_t);
								probability_rot1 = getPDF(motionl.rotation1, rot1, 45);
								trans_prb = getPDF(motionl.translation, trans, 10);
								probability_rot2 = getPDF(motionl.rotation2, rot2, 45); 
								val = temp_pos[i_t][j_t][k_t] * trans_prb * probability_rot1 * probability_rot2;
								pos[i][j][k] = pos[i][j][k] + val;
								total_prob = total_prob + val;
							}
						}
					}
				}
			}
		}
	}			
				
	double max = 0.0;
	for(int a = 0; a < 35; a++)
		for(int b = 0; b < 35; b++)
			for(int c = 0; c < 4; c++)
			{
				pos[a][b][c] = pos[a][b][c] / total_prob;
				if(max > pos[a][b][c])
					max = a;
			}	
	index = max;
	index_angle = index % 4;
	index = index / 4;
	index_y = index % 35;
	index = index / 35;
	index_x = index % 35;
	cout << " Ending position update " << endl;
	pose_publisher(index_x, index_y, index_angle);

}

void updateObservation(int tagnum, double trans, double rot)
{
	Observation obsl;
	double norm_value = 0;
	double total_prob = 0;
	double trans_prb, rot_prb;
	double val;
	int index, index_x, index_y, index_angle;
	for(int a = 0; a < 35; a++)
		for(int b = 0; b < 35; b++)
			for(int c = 0; c < 4; c++)
				temp_pos[a][b][c] = pos[a][b][c];

	for(int i = 0; i < 35; i++)
	{
		for(int j = 0; j < 35; j++)
		{
			for(int k = 0; k < 4; k++)
			{
				obsl = getObservation(i, j, k, tagnum);
				rot_prb = getPDF(obsl.rotation, rot, 45);
				trans_prb = getPDF(obsl.translation, transO, 10);
				val = temp_pos[i][j][k] * trans_prb * rot_prb;
				pos[i][j][k] = pos[i][j][k] + val;
				total_prob = total_prob + val;
			}
		}
	}

	double max = 0.0;
	for(int a = 0; a < 35; a++)
		for(int b = 0; b < 35; b++)
			for(int c = 0; c < 4; c++)
			{
				pos[a][b][c] = pos[a][b][c] / total_prob;
				if(max > pos[a][b][c])
					max = pos[a][b][c];
			}	
	index = max;
	index_angle = index % 4;
	index = index / 4;
	index_y = index % 35;
	index = index / 35;
	index_x = index % 35;
	pose_publisher(index_x, index_y, index_angle);
}

void extract()
{
	pos[11][27][2] = 1;
	for(int i = 0; i < 6; i++)
		{
			marker.header.frame_id = "/world";
			marker.header.stamp = Time();
			marker.ns = "tag";
			marker.id = (i+1);
			marker.type = visualization_msgs::Marker::CUBE;
			marker.pose.position.x = tags_xPos[i]/100.0;
			marker.pose.position.y = tags_yPos[i]/100.0;
			marker.pose.position.z = 0;
			marker.scale.x = 0.1;
			marker.scale.y = 0.1;
			marker.scale.z = 0.1;
	        
			marker.color.r = 1.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
			marker.color.a = 1.0;
			marker.action = visualization_msgs::Marker::ADD;
			tag_pub.publish(marker);
			while (tag_pub.getNumSubscribers() < 1)
				int x = 1;	
			while (tag_pub.getNumSubscribers() < 1)
				int x = 1;
		

			for(int q = 0; q < 6; q++)
			{
				marker.header.frame_id = "/world";
				marker.header.stamp = Time();
				marker.ns = "tag";
				marker.id = (i+1);
				marker.type = visualization_msgs::Marker::CUBE;
				marker.pose.position.x = tags_xPos[5-i]/100.0;
				marker.pose.position.y = tags_yPos[5-i]/100.0;
				marker.pose.position.z = 0;
				marker.scale.x = 0.1;
				marker.scale.y = 0.1;
				marker.scale.z = 0.1;
		        
				marker.color.r = 0.0;
				marker.color.g = 1.0;
				marker.color.b = 0.0;
				marker.color.a = 1.0;
				marker.action = visualization_msgs::Marker::ADD;
				tag_pub.publish(marker);
				while (tag_pub.getNumSubscribers() < 1)
					int x = 1;	
				while (tag_pub.getNumSubscribers() < 1)
					int x = 1;
			}
		}
		Motion mot;
		mot = getMotionModel(12,28,2,11,27,3);

		rosbag::Bag bag;

		try{
			bag.open("/home/prime/vicon_ws/src/lab4/src/grid.bag", rosbag::bagmode::Read);
		} catch(rosbag::BagException  e) {
		cout << "Could not open Bag File! " << e.what() << endl;
		return ;
		}
		
		vector<std::string> topics;
		topics.push_back(std::string("Movements"));
		topics.push_back(std::string("Observations"));

		rosbag::View view(bag, rosbag::TopicQuery(topics));

		foreach(rosbag::MessageInstance const m, view)
		{

			if (m.getTopic() == "Movements") 
			{
				//cout << "Collecting Movements " << endl;
				lab4::Motion::ConstPtr msg_M = m.instantiate<lab4::Motion>(); 
				if (msg_M != NULL)
		  		{
		  			double roll, pitch, yaw1, yaw2;
		  			tf::Quaternion quat1(msg_M->rotation1.x, msg_M->rotation1.y, msg_M->rotation1.z, msg_M->rotation1.w);
		  			tf::Quaternion quat2(msg_M->rotation2.x, msg_M->rotation2.y, msg_M->rotation2.z, msg_M->rotation2.w);
	
		  			tf::Matrix3x3 m(quat1);
		  			tf::Matrix3x3 n(quat2);

    				m.getRPY(roll, pitch, yaw1);
    				n.getRPY(roll, pitch, yaw2);
    				rot1 = getDegree(yaw1);
    				rot2 = getDegree(yaw2); 
   
    				trans = msg_M->translation;
    				
    				Motion valid;
    				valid.rotation1 = rot1;
    				valid.translation = trans*100;
    				valid.rotation2 = rot2;
    				
    				//cout << valid.rotation1 << " " << valid.rotation2 << " " << valid.translation << " " << endl;
    				positionUpdate(valid);
			    }

			else if(m.getTopic() == "Observations")
			{
				//cout << "Collecting Obs " << endl;
				
				lab4::Observation::ConstPtr msg_O = m.instantiate<lab4::Observation>(); 
				if (msg_O != NULL)
				{
					float distance;
					int tag;
				    double roll, pitch, yaw3;
		  			distance = msg_O->range;
		  			tf::Quaternion quat3(msg_O->bearing.x, msg_O->bearing.y, msg_O->bearing.z, msg_O->bearing.w);
				
					tf::Matrix3x3 p(quat3);

					p.getRPY(roll, pitch, yaw3);
					rot = getDegree(yaw3);
					transO = distance;
					tag = msg_O->tagNum;
					//cout << "Obs: " << rot << " " << transO << endl;
					updateObservation(tag, transO, rot);

				}
			}

		}
	}

	bag.close();
}
	


int main(int argc, char **argv)
{
	init(argc, argv, "localizer_node");
	initiate();
	NodeHandle nh;
	tag_pub = nh.advertise<visualization_msgs::Marker>("vizmark", 1000);
	path = nh.advertise<visualization_msgs::Marker>("trajectory", 1000);
	while(ok())
	{
		extract();
		
	}
	spin();
	return 0;
}

