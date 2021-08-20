// dataset_player
#include "dataset_player/DatasetPlayer.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "datasetPlayer");
	ros::NodeHandle node;
	ros::NodeHandle privateNode("~");
	
	dataset_player::DatasetPlayer datasetPlayer;

	if (!datasetPlayer.setup(node, privateNode))
	{
		ROS_INFO_STREAM("Failed to initialize datasetplayer! Please check if the datasettype parameter is provided.");
		return -1;
	}

	datasetPlayer.spin();

	return 0;
}