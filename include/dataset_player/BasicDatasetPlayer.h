#pragma once

// C/C++
#include<iostream>
#include<string>

// dataset_player
#include "dataset_player/common.h"

namespace dataset_player
{

class BasicDatasetPlayer
{
public:
	BasicDatasetPlayer();

private:
	std::string directory;
}; // end class BasicDatasetPlayer

} // end namespace dataset_player