/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Brian Gerkey */

#define USAGE "\nUSAGE: map_server <map.yaml>\n" \
              "  map.yaml: map description file\n" \
              "DEPRECATED USAGE: map_server <map> <resolution>\n" \
              "  map: image file to load\n"\
              "  resolution: map resolution [meters/pixel]"

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>

#include <opencv2/opencv.hpp>

#include "ros/ros.h"
#include "ros/console.h"
#include "map_server/image_loader.h"
#include "nav_msgs/MapMetaData.h"
#include "yaml-cpp/yaml.h"

#include "common_pkg/SetVirtualWalls.h"
#include "common_pkg/map_srv.h"
#include "common_pkg/pose_init_srv.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/SetMap.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/String.h"

#ifdef HAVE_YAMLCPP_GT_0_5_0
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

class MapServer
{
  public:
    /** Trivial constructor */
    MapServer(const std::string& fname, double res) : map_has_been_loaded_(true), init_pose_has_been_set_(false),
      init_pose_(geometry_msgs::PoseWithCovarianceStamped())
    {
      ROS_INFO("[map_server] Initializing map_server node.");
      service = n.advertiseService("static_map", &MapServer::mapCallback, this);
      set_virtual_wall_service_ = n.advertiseService("/set_virtual_walls", &MapServer::HandleVirtualWalls, this);
      load_map_service_ = n.advertiseService("/load_map", &MapServer::LoadMapCallback, this);
      set_init_pose_service_ = n.advertiseService("/init_pose", &MapServer::SetInitPoseCallback, this);
      set_amcl_map_client_ = n.serviceClient<nav_msgs::SetMap>("set_map");
      metadata_pub = n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
      map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
      virtual_map_pub_ = n.advertise<nav_msgs::OccupancyGrid>("virtual_map", 1, true);
      load_map_status_pub_ = n.advertise<std_msgs::String>("/load_map_status", 1, true);
      if (!LoadMapFromYaml(fname, res) || !LoadMapFromYaml(fname, res, true)) {
        ROS_ERROR("[map_server] load map failed");
        exit(-1);
      }
    }

  private:
    bool LoadMapFromYaml(const std::string& fname) {
      return LoadMapFromYaml(fname, 0.0);
    }

    bool LoadMapFromYaml(const std::string& fname, double res) {
      return LoadMapFromYaml(fname, res, false);
    }

    bool LoadMapFromYaml(const std::string& fname, double res, const bool& map_contains_virtual_walls) {
      std::string mapfname = "";
      double origin[3];
      int negate;
      double occ_th, free_th;
      MapMode mode = TRINARY;
      std::string frame_id;
      ros::NodeHandle private_nh("~");
      private_nh.param("frame_id", frame_id, std::string("map"));
      deprecated = (res != 0);
      if (!deprecated) {
        //mapfname = fname + ".pgm";
        //std::ifstream fin((fname + ".yaml").c_str());
        std::ifstream fin(fname.c_str());
        if (fin.fail()) {
          ROS_ERROR("[map_server] Map_server could not open %s.", fname.c_str());
          return false;
        }
#ifdef HAVE_YAMLCPP_GT_0_5_0
        // The document loading process changed in yaml-cpp 0.5.
        YAML::Node doc = YAML::Load(fin);
#else
        YAML::Parser parser(fin);
        YAML::Node doc;
        parser.GetNextDocument(doc);
#endif
        try {
          doc["resolution"] >> res;
        } catch (YAML::InvalidScalar) {
          ROS_ERROR("[map_server] The map does not contain a resolution tag or it is invalid.");
          return false;
        }
        try {
          doc["negate"] >> negate;
        } catch (YAML::InvalidScalar) {
          ROS_ERROR("[map_server] The map does not contain a negate tag or it is invalid.");
          return false;
        }
        try {
          doc["occupied_thresh"] >> occ_th;
        } catch (YAML::InvalidScalar) {
          ROS_ERROR("[map_server] The map does not contain an occupied_thresh tag or it is invalid.");
          return false;
        }
        try {
          doc["free_thresh"] >> free_th;
        } catch (YAML::InvalidScalar) {
          ROS_ERROR("[map_server] The map does not contain a free_thresh tag or it is invalid.");
          return false;
        }
        try {
          std::string modeS = "";
          doc["mode"] >> modeS;

          if(modeS=="trinary")
            mode = TRINARY;
          else if(modeS=="scale")
            mode = SCALE;
          else if(modeS=="raw")
            mode = RAW;
          else{
            ROS_ERROR("[map_server] Invalid mode tag \"%s\".", modeS.c_str());
            return false;
          }
        } catch (YAML::Exception) {
          ROS_DEBUG("[map_server] The map does not contain a mode tag or it is invalid... assuming Trinary");
          mode = TRINARY;
        }
        try {
          doc["origin"][0] >> origin[0];
          doc["origin"][1] >> origin[1];
          doc["origin"][2] >> origin[2];
        } catch (YAML::InvalidScalar) {
          ROS_ERROR("[map_server] The map does not contain an origin tag or it is invalid.");
          return false;
        }
        try {
          doc["image"] >> mapfname;
          // TODO: make this path-handling more robust
          if(mapfname.size() == 0)
          {
            ROS_ERROR("[map_server] The image tag cannot be an empty string.");
            return false;
          }
          if(mapfname[0] != '/')
          {
            // dirname can modify what you pass it
            char* fname_copy = strdup(fname.c_str());
            mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
            free(fname_copy);
          }
        } catch (YAML::InvalidScalar) {
          ROS_ERROR("[map_server] The map does not contain an image tag or it is invalid.");
          return false;
        }
      } else {
        private_nh.param("negate", negate, 0);
        private_nh.param("occupied_thresh", occ_th, 0.65);
        private_nh.param("free_thresh", free_th, 0.196);
        mapfname = fname;
        origin[0] = origin[1] = origin[2] = 0.0;
      }

      try
      {
        if (map_contains_virtual_walls) {
          ROS_INFO("[map_server] Loading virtual map from image \"%s\"", mapfname.c_str());
          map_server::loadMapFromFile(&virtual_map_resp_,mapfname.c_str(),res,negate,occ_th,free_th, origin, mode);
        } else {
          ROS_INFO("[map_server] Loading map from image \"%s\"", mapfname.c_str());
          map_server::loadMapFromFile(&map_resp_,mapfname.c_str(),res,negate,occ_th,free_th, origin, mode);
          size_t position = mapfname.rfind(".");
          std::string extract_name = (std::string::npos == position)? mapfname : mapfname.substr(0, position);
          map_ = cv::imread(extract_name + ".png", cv::IMREAD_GRAYSCALE);
          cv::flip(map_, map_, 0);
        }
      }
      catch (std::runtime_error e)
      {
          ROS_ERROR("%s", e.what());
          return false;
      }
      // To make sure get a consistent time in simulation
      ros::Time::waitForValid();
      if (map_contains_virtual_walls) {
        virtual_map_resp_.map.info.map_load_time = ros::Time::now();
        virtual_map_resp_.map.header.frame_id = frame_id;
        virtual_map_resp_.map.header.stamp = ros::Time::now();
        ROS_INFO("[map_server] Read a %d X %d virtual map @ %.3lf m/cell",
                virtual_map_resp_.map.info.width,
                virtual_map_resp_.map.info.height,
                virtual_map_resp_.map.info.resolution);
        meta_data_message_ = virtual_map_resp_.map.info;
        metadata_pub.publish(meta_data_message_);
        virtual_map_pub_.publish(virtual_map_resp_.map);
      } else {
        map_resp_.map.info.map_load_time = ros::Time::now();
        map_resp_.map.header.frame_id = frame_id;
        map_resp_.map.header.stamp = ros::Time::now();
        ROS_INFO("[map_server] Read a %d X %d map @ %.3lf m/cell",
                map_resp_.map.info.width,
                map_resp_.map.info.height,
                map_resp_.map.info.resolution);
        meta_data_message_ = map_resp_.map.info;
        // Latched publisher for metadata
        metadata_pub.publish( meta_data_message_ );
        // Latched publisher for data
        map_pub.publish( map_resp_.map );
      }
      return true;
    }

    /** Callback invoked when someone requests our service */
    bool mapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res )
    {
      // request is empty; we ignore it
      // = operator is overloaded to make deep copy (tricky!)
      res = map_resp_;
      ROS_INFO("[map_server] Sending map");

      return true;
    }

    bool LoadMapCallback(common_pkg::map_srv::Request& req, common_pkg::map_srv::Response &res) {
      res.success = false;
      res.message = "Map cannot been loaded.";
      map_has_been_loaded_ = false;
      if (LoadMapFromYaml(req.path_name + req.map_name + ".yaml", 0.0)) {
        ROS_INFO("Successfully loaded map");
        res.success = true;
        res.message = "Map has been loaded successfully.";
        map_has_been_loaded_ = true;
      }
      bool ret = SetAmclMap();

      if (!LoadMapFromYaml(req.path_name + req.map_name + "_virtual.yaml", 0.0, true)) {
        ROS_INFO("Use the original map as the virtual map.");
        LoadMapFromYaml(req.path_name + req.map_name + ".yaml", 0.0, true);
      }

      if (ret) {
        std_msgs::String msg;
        msg.data = "map/" + req.map_name;
        load_map_status_pub_.publish(msg);
      }
      return true;
    }

    bool SetInitPoseCallback(common_pkg::pose_init_srv::Request& req, common_pkg::pose_init_srv::Response& res) {
      init_pose_ = req.init_pose;
      ROS_INFO("Successfully set initial pose");
      res.success = SetAmclMap();
      if (res.success) {
        res.message = "Successfully set initial pose";
      } else {
        res.message = "Failed to set initial pose";
      }
      return true;
    }

    bool SetAmclMap() {
      if (!map_has_been_loaded_) {
        ROS_INFO("Map has not been loaded.");
        return false;
      }
      nav_msgs::SetMap set_map_srv;
      set_map_srv.request.map = map_resp_.map;
      set_map_srv.request.initial_pose = init_pose_;
      set_map_srv.request.initial_pose.header.frame_id = "map";
      set_amcl_map_client_.call(set_map_srv);
      if (set_map_srv.response.success) {
        ROS_INFO("Amcl map set.");
        return true;
      } else {
        ROS_INFO("Amcl map not set.");
        return false;
      }
    }

    bool HandleVirtualWalls(common_pkg::SetVirtualWalls::Request& req, common_pkg::SetVirtualWalls::Response& res) {
      res.success = true;
      res.message.clear();
      nav_msgs::OccupancyGrid msg{virtual_map_resp_.map};
      if (map_.empty() || virtual_map_resp_.map.data.empty()) {
        ROS_ERROR("The map has not been correctly set.");
        res.message.push_back("The map has not been correctly set.");
        res.success = false;
        return true;
      }
      if (req.markers.empty()) {
        ROS_WARN("Virtual wall data is empty.");
        res.message.push_back("Virtual wall data is empty.");
        res.success = false;
        return true;
      }

      for (auto& marker : req.markers) {
        auto color{cv::Scalar(0)};
        if (marker.type >= 32) {
          color = cv::Scalar(255);
        }
        marker.type = 4;
        switch (marker.type) {
          // square
          case 1: {
            if (marker.points.size() != 2) {
              res.message.push_back("ID " + std::to_string(marker.id) + ": wrong number of points for square type virtual wall.");
              res.success = false;
              break;
            }
            cv::rectangle(map_, PointToPixel(marker.points[0]), PointToPixel(marker.points[1]), color, -1);
            res.success &= true;
            break;
          }
          // line
          case 4: {
            if (marker.points.size() < 2) {
              res.message.push_back("ID " + std::to_string(marker.id) + ": wrong number of points for line type virtual wall.");
              res.success = false;
              break;
            }
            cv::Point prev_point{PointToPixel(marker.points[0])};
            for (const auto& p : marker.points) {
              cv::Point point{PointToPixel(p)};
              if (prev_point == point) {
                continue;
              }
              cv::line(map_, prev_point, point, color, 2);
              prev_point = point;
            }
            res.success &= true;
            break;
          }
          // point
          case 8: {
            for (const auto& point : marker.points) {
              cv::circle(map_, PointToPixel(point), 2, color, -1);
            }
            res.success &= true;
            break;
          }
          default: {
            res.message.push_back("ID " + std::to_string(marker.id) + ": unknown virtual wall type.");
            break;
          }
        }
      }
      
      msg.data.clear();
      for (int i=0; i<msg.info.height; ++i) {
        for (int j=0; j<msg.info.width; ++j) {
          if (map_.at<unsigned char>(i,j) < 5) {
            msg.data.push_back(100);
          } else if (map_.at<unsigned char>(i,j) > 250) {
            msg.data.push_back(0);
          } else {
            msg.data.push_back(-1);
          }
        }
      }
      virtual_map_pub_.publish(msg);

      return true;
    }

    cv::Point PointToPixel(const geometry_msgs::Point& point) {
      return cv::Point((point.x - map_resp_.map.info.origin.position.x) / map_resp_.map.info.resolution,
        (point.y - map_resp_.map.info.origin.position.y) / map_resp_.map.info.resolution);
    }

    ros::NodeHandle n;
    ros::Publisher map_pub;
    ros::Publisher virtual_map_pub_;
    ros::Publisher load_map_status_pub_;
    ros::Publisher metadata_pub;
    ros::ServiceServer service;
    ros::ServiceServer set_virtual_wall_service_;
    ros::ServiceServer load_map_service_;
    ros::ServiceServer set_init_pose_service_;
    ros::ServiceClient set_amcl_map_client_;
    bool deprecated;
    bool map_has_been_loaded_;
    bool init_pose_has_been_set_;
    geometry_msgs::PoseWithCovarianceStamped init_pose_;

    /** The map data is cached here, to be sent out to service callers
     */
    nav_msgs::MapMetaData meta_data_message_;
    nav_msgs::GetMap::Response map_resp_;
    nav_msgs::GetMap::Response virtual_map_resp_;

    cv::Mat map_;

    /*
    void metadataSubscriptionCallback(const ros::SingleSubscriberPublisher& pub)
    {
      pub.publish( meta_data_message_ );
    }
    */

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_server", ros::init_options::AnonymousName);
  if(argc != 3 && argc != 2)
  {
    ROS_ERROR("%s", USAGE);
    exit(-1);
  }
  if (argc != 2) {
    ROS_WARN("[map_server] Using deprecated map server interface. Please switch to new interface.");
  }
  std::string fname(argv[1]);
  double res = (argc == 2) ? 0.0 : atof(argv[2]);

  try
  {
    MapServer ms(fname, res);
    ros::spin();
  }
  catch(std::runtime_error& e)
  {
    ROS_ERROR("[map_server] map_server exception: %s", e.what());
    return -1;
  }

  return 0;
}

