#include "dataset_player/TUMDatasetPlayer.h"

namespace dataset_player
{

TUMDatasetPlayer::TUMDatasetPlayer(ros::NodeHandle & node, ros::NodeHandle & privateNode)
: BasicDatasetPlayer(node)
{
	_privateNode = privateNode;
}

bool TUMDatasetPlayer::setup()
{
	std::string sParam;
	bool        bParam;
	float       fParam;

	if (_privateNode.getParam("pathOfDataset", sParam))
	{
		ROS_INFO_STREAM("Set pathOfDataset: " << sParam);
		getConfigureParam()._pathOfDataset = sParam;
	}
	else
		ROS_INFO_STREAM("Set pathOfDataset: morenzhi");

	if (_privateNode.getParam("subDirectory", sParam))
	{
		ROS_INFO_STREAM("Set subDirectory: " << sParam);
		getConfigureParam()._subDirectory = sParam;
	}
	else
		ROS_INFO_STREAM("Set subDirectory: morenzhi");

	if (_privateNode.getParam("pubImageColor", bParam) && bParam)
	{
		ROS_INFO_STREAM("Set pubImageColor: TRUE");
		getConfigureParam()._pubImageColor = true;
	}
	else
	{
		getConfigureParam()._pubImageColor = false;
		ROS_INFO_STREAM("Set pubImageColor: FALSE");
	}

	if (_privateNode.getParam("pubImageDepth", bParam) && bParam)
	{
		ROS_INFO_STREAM("Set pubImageDepth: TRUE");
		getConfigureParam()._pubImageDepth = true;
	}
	else
	{
		getConfigureParam()._pubImageDepth = false;
		ROS_INFO_STREAM("Set pubImageDepth: FALSE");
	}

	if (getConfigureParam()._pubImageDepth || getConfigureParam()._pubImageColor)
	{
		if (_privateNode.getParam("imageTopic", sParam))
		{
			ROS_INFO_STREAM("Set imageTopic: " << sParam);
			getConfigureParam()._imageTopic = sParam;
		}
		else
			ROS_INFO_STREAM("Set imageTopic: morenzhi");
		if (_privateNode.getParam("imageColorRate", fParam))
		{
			ROS_INFO_STREAM("Set imageRate: " << fParam);
			getConfigureParam()._imageRate = fParam;
		}
		else
			ROS_INFO_STREAM("Set imageRate: morenzhi");
		if (_privateNode.getParam("imageFrameID", sParam))
		{
			ROS_INFO_STREAM("Set imageFrameID: " << sParam);
			getConfigureParam()._imageFrameID = sParam;
		}
		else
			ROS_INFO_STREAM("Set imageFrameID: morenzhi");
	}
	
	if (_privateNode.getParam("pubGroundTruth", bParam) && bParam)
	{
		ROS_INFO_STREAM("Set pubGroundTruth: TRUE");
		getConfigureParam()._pubGroundTruth = true;
		if (_privateNode.getParam("groundTruthTopic", sParam))
		{
			ROS_INFO_STREAM("Set groundTruthTopic: " << sParam);
			getConfigureParam()._groundTruthTopic = sParam;
		}
		else
			ROS_INFO_STREAM("Set groundTruthTopic: morenzhi");
		if (_privateNode.getParam("groundTruthRate", fParam))
		{
			ROS_INFO_STREAM("Set groundTruthRate: " << fParam);
			getConfigureParam()._groundTruthRate = fParam;
		}
		else
			ROS_INFO_STREAM("Set groundTruthRate: morenzhi");
		if (_privateNode.getParam("groundTruthFrameID", sParam))
		{
			ROS_INFO_STREAM("Set groundTruthFrameID: " << sParam);
			getConfigureParam()._groundTruthFrameID = sParam;
		}
		else
			ROS_INFO_STREAM("Set groundTruthFrameID: morenzhi");
	}
	else
	{
		getConfigureParam()._pubGroundTruth = false;
		ROS_INFO_STREAM("Set pubGroundTruth: FALSE");
	}
	
}

} // end namesapce dataset_player