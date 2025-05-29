#include <gz/plugin/Register.hh>
#include <gz/sim/components/Physics.hh>
#include <gz/sim/components/World.hh>
#include "gz/sim/components/Pose.hh"

#include <fstream>
#include <iostream>
#include <cmath>
#include <gz/sim/System.hh>
#include <gz/common/Console.hh>
// #include <gz/transport/Node.hh>
#include <gz/math/Vector3.hh>
#include <boost/gil.hpp>
#include <boost/gil/io/detail/dynamic.hpp>
#include <boost/gil/extension/io/png/old.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <octomap/octomap.h>
// #include <rclcpp/rclcpp.hpp>
// #include <gazebo_map_creator_interface/srv/map_request.hpp>

#include "gz/sim/components/RaycastData.hh"

#define TEST

struct MapRequest
{
gz::math::Vector3d   lowerright;
gz::math::Vector3d   upperleft;
int               skip_vertical_scan;
float              resolution;
float               range_multiplier;
int                 threshold_2d;
std::string         filename;
};

class GazeboMapCreator : 
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate,
  public gz::sim::ISystemPreUpdate
{
private:
  gz::sim::EntityComponentManager* ecm_{nullptr};
  gz::sim::Entity rayEntity_;
  bool map_creator_done_ = false;
  bool ray_saved = false;
  int cnt = 0;
  MapRequest _req;

public:
  void Configure(const gz::sim::Entity&,
                const std::shared_ptr<const sdf::Element>&,
                gz::sim::EntityComponentManager& _ecm,
                gz::sim::EventManager&) override
  {
    ecm_ = &_ecm;

    // Set collision detector to bullet (supports ray intersections).
    auto worldEntity = _ecm.EntityByComponents(gz::sim::components::World());
    _ecm.CreateComponent(
      worldEntity, gz::sim::components::PhysicsCollisionDetector("bullet"));

    // Create RaycastData component for rayEntity
    rayEntity_ = _ecm.CreateEntity();
    _ecm.CreateComponent(rayEntity_, gz::sim::components::RaycastData());
    _ecm.CreateComponent(
      rayEntity_, gz::sim::components::Pose(gz::math::Pose3d(0, 0, 0, 0, 0, 0)));

     _req.lowerright = gz::math::Vector3d(-15, -13, 0.5);
    _req.upperleft = gz::math::Vector3d(25.0, 13.0, 2.0);
    _req.skip_vertical_scan = 0;
    _req.resolution = 0.06;
    _req.range_multiplier = 1;
    _req.threshold_2d = 255;
    _req.filename = "map";
  }

  void PostUpdate(const gz::sim::UpdateInfo&,
                 const gz::sim::EntityComponentManager& _ecm) override
  {
    #ifdef TEST
    if(!map_creator_done_ && ray_saved)
    {
      // Calculate the size of the cubic area
      float size_x = _req.upperleft.X() - _req.lowerright.X();
      float size_y = _req.lowerright.Y() - _req.upperleft.Y();  // size_y to be -ve
      float size_z = _req.upperleft.Z() - _req.lowerright.Z();

      // Check for coordinate validity
      if (size_x <= 0 || size_y >=0 || size_z <= 0 )
      {
          std::cout << "Invalid coordinates" << std::endl;
          return;
      }
      // Calculate the number of points in each dimension
      int num_points_x = static_cast<int>(std::abs(size_x) / _req.resolution) + 1;
      int num_points_y = static_cast<int>(std::abs(size_y) / _req.resolution) + 1;
      int num_points_z = static_cast<int>(std::abs(size_z) / _req.resolution) + 1;

      // Calculate the step size in each dimension
      float step_x = size_x / num_points_x;
      float step_y = size_y / num_points_y;
      float step_z = size_z / num_points_z;   

      int dims = 6;

      if (_req.skip_vertical_scan) 
      {
        num_points_z = 2;
        step_z = size_z;
        dims = 4;
      }
      
      auto &rays =
            _ecm.Component<gz::sim::components::RaycastData>(rayEntity_)->Data().rays;

      auto &results =
          _ecm.Component<gz::sim::components::RaycastData>(rayEntity_)->Data().results;

      if(rays.size() != results.size()) {
        return ;
      }

      std::cout << "size of rays: " << rays.size() << std::endl;
      std::cout << "size of results: " << results.size() << std::endl;

      // 地图生成逻辑（保持原有核心算法）
      // Pixed values near to 0 means occupied and 255 means empty
      boost::gil::gray8_pixel_t fill(255 - _req.threshold_2d);
      boost::gil::gray8_pixel_t blank(255);
      boost::gil::gray8_image_t image(num_points_x, num_points_y);

      // Initially fill all area with empty pixel value
      boost::gil::fill_pixels(image._view, blank);

      // Create a point cloud objects
      pcl::PointCloud<pcl::PointXYZ> cloud;
      std::string entityName;
      gz::math::Vector3d start, end;
      double dx, dy, dz, dist;
      cloud.width    = num_points_x;
      cloud.height   = num_points_y;
          
      struct PointMask {
                  int x, y, z;
              };
      PointMask directions[6] = {
                  {-1, 0, 0}, // Left
                  {1, 0, 0},  // Right
                  {0, 1, 0},  // Front
                  {0, -1, 0}, // Back
                  {0, 0, 1},  // Top
                  {0, 0, -1}  // Bottom
              };
      int index = 0;
      for (int x = 0; x < num_points_x; ++x) 
      {
          std::cout << "\rPercent complete: " << x * 100.0 / num_points_x << "%       " << std::flush;
          
          double cur_x = _req.lowerright.X() + x * step_x; 
          for (int y = 0; y < num_points_y; ++y)
          {
              double cur_y = _req.upperleft.Y() + y * step_y;    

              // Walk in z direction to check each point for any collision to it's neighbours
              for (int z = 0; z < num_points_z; ++z)
              {
                  double cur_z = _req.lowerright.Z() + z * step_z;
                  gz::math::Vector3d start(cur_x, cur_y, cur_z);

                  // Check for right, left, front, back, top and bottom points for collision.
                  // 2d mode checks for right, left, front and back.  see dims assignment.
                  for(int i = 0; i < dims; ++i) {
                      dx =  directions[i].x * step_x * _req.range_multiplier;
                      dy =  directions[i].y * step_y * _req.range_multiplier;
                      dz =  directions[i].z * step_z *_req.range_multiplier;
                      gz::math::Vector3d end(cur_x + dx, cur_y + dy, cur_z + dz);
                      
                      double expFraction = (rays[index].start - results[index].point).Length() / (rays[index].start - rays[index].end).Length();
                      index++;

                      if (!std::isnan(expFraction) && expFraction < 1)
                      { 
                        // Collision found.  Push point to cloud and set in image
                        // std::cout << "=============" << "start.x: " << start.X() << ", start.y: " << start.Y() << ", start.z: " << start.Z() << std::endl;
                        // std::cout << "=============" << "end.x: " << end.X() << ", end.y: " << end.Y() << ", end.z: " << end.Z() << std::endl;
                        // std::cout << "=============" << "result.x: " << results[index-1].point.X() << ", result.y: " << results[index-1].point.Y() << ", result.z: " << results[index-1].point.Z() << std::endl;
                        // std::cout << "=============" << "x: " << x << ", y: " << y << ", z: " << z << ", i: " << i << ", expFraction: " << expFraction << std::endl;
                        // std::cout << "\nfraction: " << expFraction << std::flush;
                        cloud.push_back(pcl::PointXYZ(cur_x, cur_y, cur_z));
                        image._view(x,y) = fill;
                        // break;
                      }            
                  }
              }
          }
      }
    
      std::cout << std::endl << "Completed calculations, writing to image" << std::endl;
      // return ;

      if (!_req.filename.empty() && cloud.size() > 0)
      { 
          // if(cloud.size() > 0)
          // {
          //   // Save pcd file
          //   pcl::io::savePCDFileASCII (_req.filename + ".pcd", cloud);
          
          //   // Save octomap file
          //   octomap::OcTree octree(_req.resolution);
          //   for (auto p:cloud.points)
          //       octree.updateNode(octomap::point3d(p.x, p.y, p.z), true );
          //   octree.updateInnerOccupancy();
          //   octree.writeBinary(_req.filename + ".bt");
          // }

          // Save png file
          boost::gil::gray8_view_t view = image._view;
          boost::gil::png_write_view(_req.filename+".png", view); 

          // Save pgm file
          pgm_write_view(_req.filename, view);

          // Write down yaml file for nav2 usage.
          std::unordered_map<std::string, std::string> yaml_dict;
          yaml_dict["image"] = _req.filename + ".pgm";
          yaml_dict["mode"] = "trinary";
          yaml_dict["resolution"] = std::to_string(_req.resolution);
          yaml_dict["origin"] =  "[" + std::to_string(_req.lowerright.X()) + std::string(", ") + std::to_string(_req.lowerright.Y()) + std::string(", 0.0]");
          yaml_dict["negate"] = "0";
          yaml_dict["occupied_thresh"] = "0.95";  // hardcoding these values since we absolutely know occupied or free
          yaml_dict["free_thresh"] = "0.90";  

          std::ofstream outputFile(_req.filename + ".yaml");
          if (outputFile.is_open()) {
              for (const auto& pair : yaml_dict) {
                  outputFile << pair.first << ": " << pair.second << std::endl;
              }
              outputFile.close();
          } else {
              std::cout << "Unable to open yaml file for writing." << std::endl;
          }

          std::cout << "Output location: " << _req.filename + "[.pcd, .bt, .pgm, .png, .yaml]" << std::endl;
          map_creator_done_ = true;
      }

      std::cout << std::endl;
    }
  #endif
  }

  void PreUpdate(const gz::sim::UpdateInfo&,
                 gz::sim::EntityComponentManager& _ecm) override
  {
    #ifdef TEST
    if(!ray_saved && cnt++ > 1000 * 20)
    {
      std::cout << "Received map creation request" << std::endl;

      // Calculate the size of the cubic area
      float size_x = _req.upperleft.X() - _req.lowerright.X();
      float size_y = _req.lowerright.Y() - _req.upperleft.Y();  // size_y to be -ve
      float size_z = _req.upperleft.Z() - _req.lowerright.Z();
      
      // Check for coordinate validity
      if (size_x <= 0 || size_y >=0 || size_z <= 0 )
      {
          std::cout << "Invalid coordinates" << std::endl;
          return;
      }
      // Calculate the number of points in each dimension
      int num_points_x = static_cast<int>(std::abs(size_x) / _req.resolution) + 1;
      int num_points_y = static_cast<int>(std::abs(size_y) / _req.resolution) + 1;
      int num_points_z = static_cast<int>(std::abs(size_z) / _req.resolution) + 1;
      
      // Calculate the step size in each dimension
      float step_x = size_x / num_points_x;
      float step_y = size_y / num_points_y;
      float step_z = size_z / num_points_z;   
  
      int dims = 6;

      if (_req.skip_vertical_scan) 
      {
        num_points_z = 2;
        step_z = size_z;
        dims = 4;
      }

      std::cout << "-----------------" << std::endl << "Area Corners: (lower right, upper left)  (" <<
        _req.lowerright.X() << ", " << _req.lowerright.Y() << ", " << _req.lowerright.Z() << "), (" <<
        _req.upperleft.X() << ", " << _req.upperleft.Y() << ", " << _req.upperleft.Z() << ") " <<  std::endl <<
        "Area size : " << size_x << " x " << size_y << " x " << size_z << " (WxLxH)" << std::endl <<
        "Step size : " << step_x << ", " << step_y << ", " << step_z << " (stepx, stepy, stepz) " << std::endl <<
        "Resolution: (" << num_points_x << ", " <<  num_points_y << ", " << num_points_z << ") - "  << _req.resolution << std::endl <<
        "Map Mode: " << (_req.skip_vertical_scan ? "Partial Scan": "Full Scan")  << std::endl << "-----------------" << std::endl ;
      
      auto &rays =
            _ecm.Component<gz::sim::components::RaycastData>(rayEntity_)->Data().rays;

      // 地图生成逻辑（保持原有核心算法）
      // Pixed values near to 0 means occupied and 255 means empty
      boost::gil::gray8_pixel_t fill(255 - _req.threshold_2d);
      boost::gil::gray8_pixel_t blank(255);
      boost::gil::gray8_image_t image(num_points_x, num_points_y);

      // Initially fill all area with empty pixel value
      boost::gil::fill_pixels(image._view, blank);

      // Create a point cloud object
      pcl::PointCloud<pcl::PointXYZ> cloud;
      std::string entityName;
      gz::math::Vector3d start, end;
      double dx, dy, dz, dist;
      cloud.width    = num_points_x;
      cloud.height   = num_points_y;
          
      struct PointMask {
                  int x, y, z;
              };
      PointMask directions[6] = {
                  {-1, 0, 0}, // Left
                  {1, 0, 0},  // Right
                  {0, 1, 0},  // Front
                  {0, -1, 0}, // Back
                  {0, 0, 1},  // Top
                  {0, 0, -1}  // Bottom
              };

      for (int x = 0; x < num_points_x; ++x) 
      {
          std::cout << "\rPercent complete: " << x * 100.0 / num_points_x << "%       " << std::flush;
          
          double cur_x = _req.lowerright.X() + x * step_x; 
          for (int y = 0; y < num_points_y; ++y)
          {
              double cur_y = _req.upperleft.Y() + y * step_y;

              // Walk in z direction to check each point for any collision to it's neighbours
              for (int z = 0; z < num_points_z; ++z)
              {
                  double cur_z = _req.lowerright.Z() + z * step_z;
                  gz::math::Vector3d start(cur_x, cur_y, cur_z);

                  // Check for right, left, front, back, top and bottom points for collision.
                  // 2d mode checks for right, left, front and back.  see dims assignment.
                  for(int i = 0; i < dims; ++i) {
                      dx =  directions[i].x * step_x * _req.range_multiplier;
                      dy =  directions[i].y * step_y * _req.range_multiplier;
                      dz =  directions[i].z * step_z *_req.range_multiplier;
                      gz::math::Vector3d end(cur_x + dx, cur_y + dy, cur_z + dz);
                      
                      gz::sim::components::RayInfo ray;
                      ray.start = start;
                      ray.end = end;
                      rays.push_back(ray);            
                  }
              }
          }
      }
    
      std::cout << std::endl << "Completed ray data pushing" << std::endl;
      auto &results =
          _ecm.Component<gz::sim::components::RaycastData>(rayEntity_)->Data().results;
      std::cout << "size of results: " << results.size() << std::endl;
      
      ray_saved = true;
    }
    #endif
  }

public: 
  void pgm_write_view(const std::string& filename, boost::gil::gray8_view_t& view)
  {
    // Write image to pgm file

    int h = view.height();
    int w = view.width();

    std::ofstream ofs;
    ofs.open(filename+".pgm");
    ofs << "P2" << '\n';          // grayscale
    ofs << w << ' ' << h << '\n'; // width and height
    ofs << 255 <<  '\n';          // max value
    for (int y = 0; y < h; ++y){
      for (int x = 0; x < w; ++x){
        // std::cout << (int)view(x, y)[0];
        ofs << (int)view(x, y)[0] << ' ';
      }
      ofs << '\n';
    }
    ofs.close();
  }
};

GZ_ADD_PLUGIN(
  GazeboMapCreator,
  gz::sim::System,
  GazeboMapCreator::ISystemConfigure,
  GazeboMapCreator::ISystemPostUpdate,
  GazeboMapCreator::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(GazeboMapCreator,"GazeboMapCreator")