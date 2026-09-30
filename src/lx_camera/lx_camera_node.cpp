#include "lx_camera/lx_camera.h"

int main(int argc, char **argv)
{
	DcLib lib;
	if(!DynamicLink(&lib)){
		return 0;
	}
    rclcpp::init(argc, argv);
	auto node = std::make_shared<LxCamera>(&lib);
	node->Run();
	node.reset();
	DisDynamicLink(&lib);
	rclcpp::shutdown();
	return 0;
}
