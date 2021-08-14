// dataset_player
#include "dataset_player/DatasetPlayer.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "datasetPlayer");
	ros::NodeHandle node;
	ros::NodeHandle privateNode("~");
	
	dataset_player::DatasetPlayer datasetPlayer;

	if (datasetPlayer.setup(node, privateNode))
	{
		datasetPlayer.spin();
	}

	return 0;
}