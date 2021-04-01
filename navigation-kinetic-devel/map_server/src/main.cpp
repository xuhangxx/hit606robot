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

#include <indoor_navigation/soc.h>
#include "ros/ros.h"
#include "ros/console.h"
#include "map_server/image_loader.h"
#include "nav_msgs/MapMetaData.h"
#include "yaml-cpp/yaml.h"
#include <gmapping/map2.h>

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))
#ifdef HAVE_YAMLCPP_GT_0_5_0
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

bool deprecated;
int flag=1;
double res;
nav_msgs::MapMetaData meta_data_message_;
nav_msgs::GetMap::Response map_resp_;
gmapping::map2 map2;   


void soc_callback(const indoor_navigation::soc& soc)
{
    ros::Duration(0.07).sleep();
    //std::cout<<res<<std::endl;
    if(soc.code == 2 || soc.code == 3 || soc.code == 4)
    {
        flag=1;
        res =  0.0 ;
    }
    else if(soc.code == 1)
    {
        flag=2;
        res =  0.0 ;
    }
}

bool mapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res )
{
      // request is empty; we ignore it

      // = operator is overloaded to make deep copy (tricky!)
    
    res = map_resp_;
    ROS_INFO("Sending map");

    return true;
}

    /** The map data is cached here, to be sent out to service callers
     */
    

    /*
    void metadataSubscriptionCallback(const ros::SingleSubscriberPublisher& pub)
    {
      pub.publish( meta_data_message_ );
    }
    */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_server", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1);
  ros::Publisher metadata_pub= n.advertise<nav_msgs::MapMetaData>("map_metadata", 1);
  ros::ServiceServer service = n.advertiseService("static_map", mapCallback);
  ros::Subscriber soc_sub = n.subscribe("/soc",1,soc_callback); 
  ros::Publisher map2_pub = n.advertise<gmapping::map2>("map2", 1, true);
  //std::cout<<argv<<std::endl;
  if(argc != 3 && argc != 2)
  {
    ROS_ERROR("%s", USAGE);
    exit(-1);
  }
  if (argc != 2) {
    ROS_WARN("Using deprecated map server interface. Please switch to new interface.");
  }
  std::string fname(argv[1]);
  //std::cout<<typeid(fname).name()<<std::endl;
  //std::cout<<fname<<std::endl;
  res = (argc == 2) ? 0.0 : atof(argv[2]);
  ros::Rate loop_rate(10);
  while(ros::ok()) 
  {
    if(flag==1)
    {

      try
      {
        std::string mapfname = "";
        double origin[3];
        int negate;
        double occ_th, free_th;
        MapMode mode = TRINARY;
        std::string frame_id;
        ros::NodeHandle private_nh("~");
        private_nh.param("frame_id", frame_id, std::string("map"));
        deprecated = (res != 0);
        if (!deprecated) 
        {
          //std::cout<<"ddd"<<std::endl;
          //mapfname = fname + ".pgm";
          //std::ifstream fin((fname + ".yaml").c_str());
          std::ifstream fin(fname.c_str());
          if (fin.fail()) 
          {
            ROS_ERROR("Map_server could not open %s.", fname.c_str());
            exit(-1);
          }
          #ifdef HAVE_YAMLCPP_GT_0_5_0
          // The document loading process changed in yaml-cpp 0.5.
          YAML::Node doc = YAML::Load(fin);
          #else
          YAML::Parser parser(fin);
          YAML::Node doc;
          parser.GetNextDocument(doc);
          #endif
          try 
          {
            doc["resolution"] >> res;
          } 
          catch (YAML::InvalidScalar) 
          {
            ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
            exit(-1);
          }
          try 
          {
            doc["negate"] >> negate;
          } 
          catch (YAML::InvalidScalar) 
          {
            ROS_ERROR("The map does not contain a negate tag or it is invalid.");
            exit(-1);
          }
          try 
          {
            doc["occupied_thresh"] >> occ_th;
          } 
          catch (YAML::InvalidScalar) 
          {
            ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
            exit(-1);
          }
          try 
          {
            doc["free_thresh"] >> free_th;
          } 
          catch (YAML::InvalidScalar) 
          {
            ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
            exit(-1);
          }
          try 
          {
            std::string modeS = "";
            doc["mode"] >> modeS;

            if(modeS=="trinary")
              mode = TRINARY;
            else if(modeS=="scale")
              mode = SCALE;
            else if(modeS=="raw")
              mode = RAW;
            else
            {
              ROS_ERROR("Invalid mode tag \"%s\".", modeS.c_str());
              exit(-1);
            }
          } 
          catch (YAML::Exception) 
          {
            ROS_DEBUG("The map does not contain a mode tag or it is invalid... assuming Trinary");
            mode = TRINARY;
          }
          try 
          {
            doc["origin"][0] >> origin[0];
            doc["origin"][1] >> origin[1];
            doc["origin"][2] >> origin[2];
          } 
          catch (YAML::InvalidScalar) {
            ROS_ERROR("The map does not contain an origin tag or it is invalid.");
            exit(-1);
          }
          try 
          {
            doc["image"] >> mapfname;
            // TODO: make this path-handling more robust
            if(mapfname.size() == 0)
            {
              ROS_ERROR("The image tag cannot be an empty string.");
              exit(-1);
            }
            if(mapfname[0] != '/')
            {
              // dirname can modify what you pass it
              char* fname_copy = strdup(fname.c_str());
              mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
              free(fname_copy);
            }
          } 
          catch (YAML::InvalidScalar) 
          {
            ROS_ERROR("The map does not contain an image tag or it is invalid.");
            exit(-1);
          }
        } 
        else 
        {
          private_nh.param("negate", negate, 0);
          private_nh.param("occupied_thresh", occ_th, 0.65);
          private_nh.param("free_thresh", free_th, 0.196);
          mapfname = fname;
          origin[0] = origin[1] = origin[2] = 0.0;
        }
        //std::cout<<mapfname<<std::endl;

        ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());
        try
        {
            map_server::loadMapFromFile(&map_resp_,mapfname.c_str(),res,negate,occ_th,free_th, origin, mode);
        }
        catch (std::runtime_error e)
        {
            ROS_ERROR("%s", e.what());
            exit(-1);
        }
        // To make sure get a consistent time in simulation
        ros::Time::waitForValid();
        map_resp_.map.info.map_load_time = ros::Time::now();
        map_resp_.map.header.frame_id = frame_id;
        map_resp_.map.header.stamp = ros::Time::now();
        ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
               map_resp_.map.info.width,
               map_resp_.map.info.height,
               map_resp_.map.info.resolution);
        meta_data_message_ = map_resp_.map.info;


        // Latched publisher for metadata
        metadata_pub.publish( meta_data_message_ );

        map_pub.publish( map_resp_.map );
      }
      catch(std::runtime_error& e)
      {
        ROS_ERROR("map_server exception: %s", e.what());
        return -1;
      }
      flag=0;
    }
    else if(flag==2)
    {

      std::string fname2="/home/x805/catkin_ws/src/indoor_navigation/maps/work0.yaml";
      try
      {
        //std::cout<<fname2<<std::endl;
        std::string mapfname = "";
        double origin[3];
        int negate;
        double occ_th, free_th;
        MapMode mode = TRINARY;
        std::string frame_id;
        ros::NodeHandle private_nh("~");
        private_nh.param("frame_id", frame_id, std::string("map"));
        deprecated = (res != 0);
        if (!deprecated) 
        {
          //std::cout<<"ddd"<<std::endl;
          //mapfname = fname + ".pgm";
          //std::ifstream fin((fname + ".yaml").c_str());
          std::ifstream fin(fname2.c_str());
          if (fin.fail()) 
          {
            ROS_ERROR("Map_server could not open %s.", fname2.c_str());
            exit(-1);
          }
          #ifdef HAVE_YAMLCPP_GT_0_5_0
          // The document loading process changed in yaml-cpp 0.5.
          YAML::Node doc = YAML::Load(fin);
          #else
          YAML::Parser parser(fin);
          YAML::Node doc;
          parser.GetNextDocument(doc);
          #endif
          try 
          {
            doc["resolution"] >> res;
          } 
          catch (YAML::InvalidScalar) 
          {
            ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
            exit(-1);
          }
          try 
          {
            doc["negate"] >> negate;
          } 
          catch (YAML::InvalidScalar) 
          {
            ROS_ERROR("The map does not contain a negate tag or it is invalid.");
            exit(-1);
          }
          try 
          {
            doc["occupied_thresh"] >> occ_th;
          } 
          catch (YAML::InvalidScalar) 
          {
            ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
            exit(-1);
          }
          try 
          {
            doc["free_thresh"] >> free_th;
          } 
          catch (YAML::InvalidScalar) 
          {
            ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
            exit(-1);
          }
          try 
          {
            std::string modeS = "";
            doc["mode"] >> modeS;

            if(modeS=="trinary")
              mode = TRINARY;
            else if(modeS=="scale")
              mode = SCALE;
            else if(modeS=="raw")
              mode = RAW;
            else
            {
              ROS_ERROR("Invalid mode tag \"%s\".", modeS.c_str());
              exit(-1);
            }
          } 
          catch (YAML::Exception) 
          {
            ROS_DEBUG("The map does not contain a mode tag or it is invalid... assuming Trinary");
            mode = TRINARY;
          }
          try 
          {
            doc["origin"][0] >> origin[0];
            doc["origin"][1] >> origin[1];
            doc["origin"][2] >> origin[2];
          } 
          catch (YAML::InvalidScalar) {
            ROS_ERROR("The map does not contain an origin tag or it is invalid.");
            exit(-1);
          }
          try 
          {
            doc["image"] >> mapfname;
            // TODO: make this path-handling more robust
            if(mapfname.size() == 0)
            {
              ROS_ERROR("The image tag cannot be an empty string.");
              exit(-1);
            }
            if(mapfname[0] != '/')
            {
              // dirname can modify what you pass it
              char* fname_copy = strdup(fname2.c_str());
              mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
              free(fname_copy);
            }
          } 
          catch (YAML::InvalidScalar) 
          {
            ROS_ERROR("The map does not contain an image tag or it is invalid.");
            exit(-1);
          }
        } 
        else 
        {
          private_nh.param("negate", negate, 0);
          private_nh.param("occupied_thresh", occ_th, 0.65);
          private_nh.param("free_thresh", free_th, 0.196);
          mapfname = fname2;
          origin[0] = origin[1] = origin[2] = 0.0;
        }
        //std::cout<<mapfname<<std::endl;

        ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());
        try
        {
            map_server::loadMapFromFile(&map_resp_,mapfname.c_str(),res,negate,occ_th,free_th, origin, mode);
        }
        catch (std::runtime_error e)
        {
            ROS_ERROR("%s", e.what());
            exit(-1);
        }
        // To make sure get a consistent time in simulation
        ros::Time::waitForValid();
        map_resp_.map.info.map_load_time = ros::Time::now();
        map_resp_.map.header.frame_id = frame_id;
        map_resp_.map.header.stamp = ros::Time::now();
        ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
               map_resp_.map.info.width,
               map_resp_.map.info.height,
               map_resp_.map.info.resolution);
        meta_data_message_ = map_resp_.map.info;


        // Latched publisher for metadata
        metadata_pub.publish( meta_data_message_ );

        map_pub.publish( map_resp_.map );
        
        map2.data.resize(map_resp_.map.info.width * map_resp_.map.info.height/4);
        int i=3;
        for(int x=0; x < map_resp_.map.info.width; x++)
        {
            for(int y=0; y < map_resp_.map.info.height; y++)
            {
                if(map_resp_.map.data[MAP_IDX(map_resp_.map.info.width, x, y)] == -1)
                {
                    map2.data[MAP_IDX(map_resp_.map.info.width, x, y)/4]+= 0;               
                }
                else if(map_resp_.map.data[MAP_IDX(map_resp_.map.info.width, x, y)] == 100)
                {
                    map2.data[MAP_IDX(map_resp_.map.info.width, x, y)/4]+= pow(4, i)*1;
                }
                else
                {
                    map2.data[MAP_IDX(map_resp_.map.info.width, x, y)/4]+= pow(4, i)*2;
                }
                i--;
                if(i==0)
                {
                  i=3;
                }
            }
        }    
        std::cout<<"time:           "<<ros::Time::now()<<std::endl;
        map2_pub.publish(map2);
      }
      catch(std::runtime_error& e)
      {
        ROS_ERROR("map_server exception: %s", e.what());
        return -1;
      }
      flag=0;
    }
    ros::spinOnce(); 
    loop_rate.sleep();
  }

  return 0;
}

