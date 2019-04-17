/**************************************************************************************************
 Software License Agreement (BSD License)
 Copyright (c) 2014-2015, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
 All rights reserved.
 Redistribution and use in source and binary forms, with or without modification, are permitted
 provided that the following conditions are met:
  *Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
  *Neither the name of the University of Aveiro nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific prior written permission
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************************

/**
\file  local_path_planning.h
\brief Global include file with some classes to handle data
\author Ricardo Silva
\date   June, 2018
*/
#ifndef _LOCAL_PLANNING_H_
#define _LOCAL_PLANNING_H_

/*---General Includes---*/
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <sys/stat.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
/*---PointCould Includes---*/
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include "velodyne_pointcloud/rawdata.h"
/*---LAR TK4 Includes---*/
#include <colormap/colormap.h>
#include "aux_func.h"
#include "lidar_segmentation/clustering.h"
#include "lidar_segmentation/lidar_segmentation.h"

/*---DEFINITIONS---*/
#define RED -125
#define GREEN 125
#define BLACK 100
#define WHITE 0
#define UNKWON 50
#define YELLOW -50

/*---TYPEDEF & NAMESPACE---*/
typedef geometry_msgs::PolygonStamped polygonS;
typedef boost::shared_ptr<polygonS> polygonSPtr;
typedef sensor_msgs::PointCloud2 PCL2;
typedef boost::shared_ptr<PCL2> pcl2Ptr;
typedef pcl::PointCloud<pcl::PointXYZ> PCL;
typedef boost::shared_ptr<PCL> pclPtr;
using namespace ros;
using namespace std;
using namespace velodyne_rawdata;

/**
 * \class LidarClusters
 * Class for LIDAR Clusters
 * \author Jorge Almeida
 */
class LidarClusters
{
public:
  vector<ClusterPtr> Clusters;
};

typedef boost::shared_ptr<LidarClusters> LidarClustersPtr;

/**
 * \class Markers
 * Class to handle the visualization markers
 * \author Jorge Almeida
 */
class Markers
{
public:
  void update(visualization_msgs::Marker &marker)
  {
    for (uint i = 0; i < markers.size(); ++i)
      if (markers[i].ns == marker.ns && markers[i].id == marker.id)  // Marker found
      {
        markers.erase(markers.begin() + i);
        break;
      }
    markers.push_back(marker);
  }
  void decrement(void)
  {
    for (uint i = 0; i < markers.size(); ++i)
    {
      switch (markers[i].action)
      {
        case visualization_msgs::Marker::ADD:
          markers[i].action = visualization_msgs::Marker::DELETE;
          break;
        case visualization_msgs::Marker::DELETE:
          markers[i].action = -1;
          break;
      }
    }
  }
  void clean(void)
  {
    vector<visualization_msgs::Marker> new_markers;
    for (uint i = 0; i < markers.size(); ++i)
      if (markers[i].action != -1)
        new_markers.push_back(markers[i]);
    markers = new_markers;
  }
  vector<visualization_msgs::Marker> getOutgoingMarkers(void)
  {
    vector<visualization_msgs::Marker> m_out(markers);
    return markers;
  }

private:
  vector<visualization_msgs::Marker> markers;
};

/**
 * \class laserDataAnalise
 * Class to handle the incoming laser data from the LIDAR sensors
 * \author Diogo Correia
 */
namespace lidar_data_analise
{
class laserDataAnalise
{
public:
  laserDataAnalise(string topicName, string frame_id)
  {
    this->topicName = topicName;
    this->frameId = frame_id;
    /*----Susbcribe LaserData Topic----*/
    sub = n.subscribe(topicName, 1000, &laserDataAnalise::laserDataTreatment, this);
    ROS_INFO("Topic %s subscribed!", topicName.c_str());

    clustersPub = np.advertise<visualization_msgs::MarkerArray>("simple_clustering", 1000);
    pclPub = np.advertise<PCL2>(topicName + "_PCL", 1000);
    polygonPub = np.advertise<polygonS>(topicName + "_polygon", 1000);

    scanPcl = pcl2Ptr(new PCL2);
  }

  /**
  @brief Converts the laser scan point to an array of points
  @param[in] Laser scan of the laser data
  @param[out] Array of points to save the converted points
  @param[in] Angle between the scan plane and the horizontal plane
  @return void
  */
  void convertToXYZ(sensor_msgs::LaserScan scan, vector<PointPtr> &points, double rot)
  {
    int s = scan.ranges.size();

    for (int n = 0; n < s; n++)
    {
      double angle, d, x, y, z;
      d = scan.ranges[n] * cos(rot * M_PI / 180);
      z = scan.ranges[n] * sin(rot * M_PI / 180);
      angle = scan.angle_min + n * scan.angle_increment;

      x = d * cos(angle);
      y = d * sin(angle);

      PointPtr point(new Point);
      point->x = x;
      point->y = y;
      point->z = z;
      point->label = n;
      point->iteration = n + 1;
      point->theta = angle;
      point->range = d;
      point->cluster_id = 1;
      points.push_back(point);
    }
  }

  /**
  @brief Creats visualization markers to display the clusters resultant from the laser scan
  @param[in] Array of clusters
  @return vector<visualization_msgs::Marker> Array with the visualization markers
  */
  vector<visualization_msgs::Marker> createClutersVisualizationMarker(vector<ClusterPtr> &clusters)
  {
    static Markers marker_list;

    // Reduce the elements status, ADD to REMOVE and REMOVE to delete
    marker_list.decrement();

    class_colormap colormap("hsv", 10, 1, false);

    visualization_msgs::Marker marker_ids;
    visualization_msgs::Marker marker_clusters;

    marker_ids.header.frame_id = frameId;  // topicName;
    marker_ids.header.stamp = ros::Time::now();

    marker_clusters.header.frame_id = frameId;  // topicName;
    marker_clusters.header.stamp = marker_ids.header.stamp;

    marker_ids.ns = "ids";
    marker_ids.action = visualization_msgs::Marker::ADD;

    marker_clusters.ns = "clusters";
    marker_clusters.action = visualization_msgs::Marker::ADD;

    marker_ids.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    marker_clusters.type = visualization_msgs::Marker::SPHERE_LIST;

    marker_ids.scale.x = 0.5;
    marker_ids.scale.y = 0.5;
    marker_ids.scale.z = 0.5;

    marker_clusters.scale.x = 0.2;
    marker_clusters.scale.y = 0.2;
    marker_clusters.scale.z = 0.2;

    marker_ids.color.a = 1.0;
    marker_ids.color.r = 0.0;
    marker_ids.color.g = 0.0;
    marker_ids.color.b = 0.0;

    for (uint i = 0; i < clusters.size(); i++)  // search all clusters
    {
      ClusterPtr cluster = clusters[i];

      std_msgs::ColorRGBA color = colormap.color(i);

      // Place in the marker every point belonging to the cluster "i"
      for (uint h = 0; h < cluster->support_points.size(); h++)
      {
        geometry_msgs::Point pt;
        pt.x = cluster->support_points[h]->x;
        pt.y = cluster->support_points[h]->y;
        pt.z = 0;

        marker_clusters.points.push_back(pt);
        marker_clusters.colors.push_back(color);
      }

      marker_ids.pose.position.x = cluster->centroid->x;
      marker_ids.pose.position.y = cluster->centroid->y;
      marker_ids.pose.position.z = 0.3;

      // texto
      boost::format fm("%d");
      fm % cluster->id;

      marker_ids.text = fm.str();
      marker_ids.id = cluster->id;
      marker_list.update(marker_ids);

      marker_list.update(marker_clusters);

    }  // end for

    // Remove markers that should not be transmitted
    marker_list.clean();

    // Clean the marker_vector and put new markers in it;
    return marker_list.getOutgoingMarkers();
  }

  /**
  @brief Removes clusters with less than a minimum of points
  @param[in] Array with all the clusters
  @param[in] Minimum points to keep the clusters
  @return vector<ClusterPtr> Array with the clusters after filtering
  */
  vector<ClusterPtr> removeSmallClusters(vector<ClusterPtr> clusters, int minPoints)
  {
    vector<ClusterPtr> cleanClusters;
    int count = 0;
    for (uint i = 0; i < clusters.size(); i++)
    {
      if (clusters[i]->support_points.size() > minPoints)
      {
        ClusterPtr cluster = clusters[i];
        cluster->id = count;
        cleanClusters.push_back(cluster);
        count++;
      }
    }
    return cleanClusters;
  }

  /**
  @brief Converts a LaserScan message to a PointCloud2 message
  @param[in] LaserScan message with the data
  @param[out] Point cloud pointer to hold the data
  @return void
  */
  void scanToPcl(sensor_msgs::LaserScan scan, pcl2Ptr pclOut)
  {
    laser_geometry::LaserProjection projector;
    projector.transformLaserScanToPointCloud(scan.header.frame_id, scan, *pclOut, tf_Listener);
  }

  /**
  @brief Creates a polygon message from a point cloud
  @param[in] Pointer for the point cloud data
  @return polygonSPtr Pointer to the polygon message
  */
  static polygonSPtr getScanPolygon(pcl2Ptr scan)
  {
    polygonSPtr polygon(new (polygonS));
    pcl::PointCloud<pcl::PointXYZ> cloud;

    polygon->header = scan->header;

    geometry_msgs::Point32 point;
    point.x = 0;
    point.y = 0;
    point.z = 0;
    polygon->polygon.points.push_back(point);

    pcl::fromROSMsg(*scan, cloud);
    for (int i = 0; i < cloud.size(); i++)
    {
      geometry_msgs::Point32 point;
      pcl::PointXYZ pointPcl;
      pointPcl = cloud.points.at(i);
      point.x = pointPcl.x;
      point.y = pointPcl.y;
      point.z = pointPcl.z;
      polygon->polygon.points.push_back(point);
    }

    return polygon;
  }

  /**
  @brief Creates a polygon message from a point cloud
  @param[in] Pointer for  the point cloud data
  @param[in] Id of the reference frame for the polygon message
  @return polygonSPtr Pointer to the polygon message
  */
  static polygonSPtr getScanPolygon(pclPtr cloud_ptr, string frame_id)
  {
    polygonSPtr polygon(new (polygonS));
    pcl::PointCloud<pcl::PointXYZ> cloud(*cloud_ptr);
    std_msgs::Header h;
    h.frame_id = frame_id;
    polygon->header = h;

    //     geometry_msgs::Point32 point;
    //     point.x = 0;
    //     point.y = 0;
    //     point.z = 0;
    //     polygon->polygon.points.push_back(point);

    //     pcl::fromROSMsg(*scan, cloud);
    for (int i = 0; i < cloud.size(); i++)
    {
      geometry_msgs::Point32 point;
      pcl::PointXYZ pointPcl;
      pointPcl = cloud.points.at(i);
      point.x = pointPcl.x;
      point.y = pointPcl.y;
      point.z = 0;  // pointPcl.z;
      polygon->polygon.points.push_back(point);
    }

    return polygon;
  }

  /**
  @brief Callback used to handle the incoming laser data
  @param[in] Incoming laser scan
  @return void
  */
  void laserDataTreatment(sensor_msgs::LaserScan scan)
  {
    // scan.header.stamp = ros::Time::now();
    scanToPcl(scan, scanPcl);

    polygonSPtr polygon = getScanPolygon(scanPcl);

    /*---Publisher for the PointCould---*/
    pclPub.publish(*scanPcl);

    /*---Publisher for the Polygon---*/
    polygonPub.publish(*polygon);
  }

  /**
  @brief Gets the transformation between tow frames and applies it to the points of a point cloud
  @param[out] Point cloud pointer to assing the points
  @return bool Returns true if there if data in the scan point cloud
  */
  bool getPcl(pcl2Ptr cloud_in_ptr)
  {
    if (scanPcl->width > 0)
    {
      *cloud_in_ptr = *scanPcl;
      return true;
    }
    return false;
  }

  /**
  @brief Gets the transformation between tow frames and applies it to the points of a point cloud
  @param[in] Destination frame
  @param[in] Point cloud to apply the transformation
  @param[out] Point cloud with the transformation applied
  @return void
  */
  void transformPCL(string destFrame, pclPtr pclIn, pclPtr pclOut)
  {
    tf::StampedTransform transform;
    string oriFrame = frameId;
    try
    {
      tf_Listener.waitForTransform(destFrame, oriFrame, ros::Time(0), ros::Duration(0.1));
      tf_Listener.lookupTransform(destFrame, oriFrame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }

    Eigen::Affine3d transform_2 = Eigen::Affine3d::Identity();
    tf::transformTFToEigen(transform, transform_2);

    pcl::transformPointCloud(*pclIn, *pclOut, transform_2);
  }

  /**
  @brief Aplies a rigid body transformation to the points of a point cloud
  @param[in] Transformation to apply
  @param[in] Point cloud to apply the transformation
  @return geometry_msgs::Point Point in spheric coordinates
  */
  static void transformPCL(tf::Transform transform, pclPtr pclIn)
  {
    Eigen::Affine3d transform_2 = Eigen::Affine3d::Identity();
    tf::transformTFToEigen(transform, transform_2);

    pcl::transformPointCloud(*pclIn, *pclIn, transform_2);
  }

private:
  NodeHandle n;
  Subscriber sub;
  NodeHandle np;
  Publisher clustersPub;
  Publisher pclPub;
  Publisher polygonPub;

  double rotation;
  string topicName;
  string frameId;

  tf::TransformListener tf_Listener;
  tf::TransformBroadcaster tf_Broadcaster;

  pcl2Ptr scanPcl;
};
}

/**
 *  class ocupGrid
 * Class containing functions to create and publish an ocupation grid from point cloud data
 * \author Diogo Correia
*/
class ocupGrid
{
public:
  ocupGrid(string frameId, double xMin, double xMax, double yMin, double yMax, double cellResol)
  {
    this->cellResolution = cellResol;
    this->xMin = xMin;
    this->yMin = yMin;
    this->xMax = xMax;
    this->yMax = yMax;
    this->xCells = (int)((xMax - xMin) / cellResolution);
    this->yCells = (int)((yMax - yMin) / cellResolution);

    grid = nav_msgs::OccupancyGridPtr(new nav_msgs::OccupancyGrid);
    initGrid(frameId);

    ocGrid.assign(xCells * yCells, UNKWON);

    gridPub = np.advertise<nav_msgs::OccupancyGrid>("ocupancy_grid", 1000);
  }

  /**
  @brief Assigns assign ocupation grid size acording to a point cloud extreme points
  @param[in] Ocupation point cloud
  @return void
  */
  void getGridSize(pclPtr inPcl)
  {
    for (int i = 0; i < inPcl->points.size(); i++)
    {
      if (xMin > inPcl->points[i].x)
      {
        xMin = inPcl->points[i].x;
      }
      if (xMax < inPcl->points[i].x)
      {
        xMax = inPcl->points[i].x;
      }
      if (yMin > inPcl->points[i].y)
      {
        yMin = inPcl->points[i].y;
      }
      if (yMax < inPcl->points[i].y)
      {
        yMax = inPcl->points[i].y;
      }
    }
  }

  /**
  @brief Updates the ocupation grid parameters
  @param[in] x coordinate of origin of the grid
  @param[in] y coordinate of origin of the grid
  @return void
  */
  void updateGrid(double originX, double originY)
  {
    grid->header.seq++;
    grid->header.stamp.sec = ros::Time::now().sec;
    grid->header.stamp.nsec = ros::Time::now().nsec;
    grid->info.map_load_time = ros::Time::now();
    grid->info.resolution = cellResolution;
    grid->info.width = xCells;
    grid->info.height = yCells;
    grid->info.origin.position.x = originX;
    grid->info.origin.position.y = originY;
    grid->data = ocGrid;
  }

  /**
  @brief Assigns values to the grid's cells according to the ocupation point cloud
  @param[in] Ocupation point cloud
  @param[in] Value to assign to the UNKWON cells
  @return void
  */
  void populateMap(pclPtr inPcl, int color)
  {
    populateMap(inPcl, color, 0.0, 0.0);
  }

  /**
  @brief Assigns values to the grid's cells according to the ocupation point cloud
  @param[in] Ocupation point cloud
  @param[in] Value to assign to the UNKWON cells
  @param[in] x coordinate of origin of the point cloud
  @param[in] y coordinate of origin of the point cloud
  @return void
  */
  void populateMap(pclPtr inPcl, int color, double xOrigin, double yOrigin)
  {
    for (int i = 0; i < inPcl->points.size(); i++)
    {
      geometry_msgs::Point point;
      point.x = inPcl->points[i].x;
      double x = point.x;
      x += xOrigin;
      point.y = inPcl->points[i].y;
      double y = point.y;
      y += yOrigin;
      point.z = 0;

      if (x < xMax && x > xMin && y > yMin && y < yMax)
      {
        int xCell = (int)((x - xMin) / cellResolution);
        int yCell = (int)((y - yMin) / cellResolution);

        int idx = yCell * xCells + xCell;
        ocGrid[idx] = 100;
      }

      geometry_msgs::Point point_s = xyzTortp(point);

      for (double k = cellResolution; k < point_s.x; k += cellResolution)
      {
        geometry_msgs::Point point;
        point.x = k;
        point.y = point_s.y;
        point.z = point_s.z;
        geometry_msgs::Point point_c = rtpToxyz(point);

        double x = point_c.x;
        x += xOrigin;
        double y = point_c.y;
        y += yOrigin;

        if (x < xMax && x > xMin && y > yMin && y < yMax)
        {
          int xCell = (int)((x - xMin) / cellResolution);
          int yCell = (int)((y - yMin) / cellResolution);

          int idx = yCell * xCells + xCell;
          if (ocGrid[idx] == UNKWON)
          {
            ocGrid[idx] = color;
          }
          else if (ocGrid[idx] == RED && color != RED)
          {
            ocGrid[idx] = YELLOW;
          }
        }
      }
    }
  }

  /**
  @brief Returns the ocupeation grid matrix
  @return vector<signed char>
  */
  vector<signed char> getGrid()
  {
    return ocGrid;
  }

  /**
  @brief Resets all the cells to a specific value
  @param[in] Value to assign to the cells
  @return void
  */
  void resetGrid(int color)
  {
    ocGrid.assign(xCells * yCells, color);
  }

  /**
  @brief Assignes a matrix to the grid setting all the values and size equal to the matrix
  @param[in] Pointer to the matrix
  @return void
  */
  void assingGrid(vector<signed char> *grid)
  {
    ocGrid = *grid;
  }

  /**
  @brief Publishes the ocupation grid message
  @return void
  */
  void publish()
  {
    gridPub.publish(*grid);
  }

  /**
  @brief Changes the value of a cell were a given points is in
  @param[in] Value to assign to the cell
  @param[in] x coordinate of the point
  @param[in] y coordinate of the point
  @param[in] Discard ocupied cells
  @return void
  */
  void setValue(int color, double x, double y, bool override)
  {
    if (x < xMax && x > xMin && y > yMin && y < yMax)
    {
      int xCell = (int)((x - xMin) / cellResolution);
      int yCell = (int)((y - yMin) / cellResolution);

      int idx = yCell * xCells + xCell;
      if (ocGrid[idx] == 50 || override)
        ocGrid[idx] = color;
    }
  }

private:
  nav_msgs::OccupancyGridPtr grid;
  double cellResolution;
  double xMin, yMin, xMax, yMax;
  int xCells;
  int yCells;
  vector<signed char> ocGrid;
  ros::NodeHandle np;
  ros::Publisher gridPub;

  /**
  @brief Initiates the ocupation grid
  @param[in] Name of the reference frame for the grid
  @return void
  */
  void initGrid(string frameId)
  {
    grid->header.seq = 1;
    grid->header.frame_id = frameId;
    grid->info.origin.position.z = 0;
    grid->info.origin.orientation.w = 1;
    grid->info.origin.orientation.x = 0;
    grid->info.origin.orientation.y = 0;
    grid->info.origin.orientation.z = 0;
  }
};

/*---Prototipos---*/
// geometry_msgs::Point32 compute2DPolygonCentroid(const geometry_msgs::PolygonStamped::ConstPtr &msg);
// void mergedpolygonCallback(const geometry_msgs::PolygonStamped::ConstPtr &msg);
// void unavlpolygonCallback(const geometry_msgs::PolygonStamped::ConstPtr &msg);
// void unavrpolygonCallback(const geometry_msgs::PolygonStamped::ConstPtr &msg);
// void ocupancygridCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

#endif
