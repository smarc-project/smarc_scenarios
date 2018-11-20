/* Copyright 2018 Nils Bore (nbore@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <smarc_msgs/CellOccupied.h>

using namespace std;

class PathOccupancyServer {
public:

    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::ServiceServer service;
    nav_msgs::OccupancyGrid map;
    costmap_2d::Costmap2D costmap;
    costmap_2d::Costmap2DPublisher* costmap_publisher;

    double map_height;
    double map_width;
    double map_origin_x;
    double map_origin_y;
    double map_res;
	std::string input;

    //string map_namespace;

    //vector<int> filter_indices;

    PathOccupancyServer() : n()
    {
        ros::NodeHandle pn("~");

        pn.param<double>("map_height", map_height, 2000.0);
        pn.param<double>("map_width", map_width, 2000.0);
        pn.param<double>("map_res", map_res, 1.0);
        pn.param<double>("map_origin_x", map_origin_x, -0.5*map_res*map_width);
        pn.param<double>("map_origin_y", map_origin_y, -0.5*map_res*map_height);
        pn.param<std::string>("input", input, "/lolo_auv/lolo_auv/camera/pipeline_locator");
        
        ROS_INFO_STREAM("Map width x height: " << map_width << " x " << map_height);

        costmap = costmap_2d::Costmap2D(map_width, map_height, map_res, map_origin_x, map_origin_y);

        costmap_publisher = new costmap_2d::Costmap2DPublisher(&n, &costmap, "/map", "pipe_map", true);

        ROS_INFO("DONE INITIALIZING COSTMAPS");

        sub = n.subscribe(input, 1, &PathOccupancyServer::callback, this);
        service = n.advertiseService("cell_occupied", &PathOccupancyServer::service_callback, this);//bpf_mtt::PublishGMMMap);
    }

    void publish_map(const nav_msgs::Path& path)
    {

        int map_x, map_y;
		for (const geometry_msgs::PoseStamped& p : path.poses) {
		    double x = p.pose.position.x;
		    double y = p.pose.position.y;
			costmap.worldToMapEnforceBounds(x, y, map_x, map_y);
            uint8_t cost = costmap.getCost(map_x, map_y);
			if (cost < 255) {
			    costmap.setCost(map_x, map_y, cost+1);
		    }
		}

    }

    void callback(const nav_msgs::Path::ConstPtr& path)
    {
        publish_map(*path);
        costmap_publisher->publishCostmap();
    }
    
	bool service_callback(smarc_msgs::CellOccupied::Request& req, smarc_msgs::CellOccupied::Response& res)
    {
		double x = req.x;
		double y = req.y;
        int map_x, map_y;
		costmap.worldToMapEnforceBounds(x, y, map_x, map_y);
        uint8_t cost = costmap.getCost(map_x, map_y);
		ROS_INFO("Current pos: %f, %f", x, y);
		ROS_INFO("Get service with cost: %u", cost);
		res.cost = cost;
        return true;
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pipeline_map_server");

    PathOccupancyServer pos;

    ros::spin();

    return 0;
}
