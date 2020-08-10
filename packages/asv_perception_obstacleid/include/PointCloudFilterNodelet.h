#ifndef POINTCLOUDFILTERNODELET_H
#define POINTCLOUDFILTERNODELET_H

#include <pcl_ros/pcl_nodelet.h>
#include <ros/ros.h>

namespace obstacle_id
{
  /*
    Description

    Subscriptions:
        ~input:     [sensor_msgs/PointCloud2] input pointcloud

    Publications:
        ~output:    [sensor_msgs/PointCloud2] filtered pointcloud

    Parameters:
        ~min_distance_[x,y,z]:   [float, default=0]  minimum distance filter (from origin) for dimension x/y/z, ignored if <= 0
        ~min_distance:           [float, default=0]  minimum distance filter (radius from origin), ignored if <= 0
        ~outlier_min_neighbors:  [int, default=0]    minimum number of neighbors within `outlier_radius` for a point to be excluded from removal.  0 means ignored
        ~outlier_radius:         [float, default=0]  radius for outlier removal
        ~cluster_tolerance:      [float, default=0]  cluster tolerance.  clustering is ignored if tolerance <= 0
        ~cluster_inliers:        [bool, default=true]  when filtering by clusters, only output clusters which do/do not (true/false) meet the clustering criteria
        ~cluster_size_min:       [uint, default=0]   minimum cluster size
        ~cluster_size_max:       [uint, default=uint max]  maximum cluster size
        ~cluster_area_min:       [float, default=0]  minimum 2d convex hull area
        ~cluster_area_max:       [float, default=float max]  maximum 2d convex hull area
  */
  class PointCloudFilterNodelet
    : public nodelet_topic_tools::NodeletLazy
  {
    public:

      using base_type = nodelet_topic_tools::NodeletLazy;
      
      // default constructor
      PointCloudFilterNodelet() = default;
                                      
    protected:
 
      /** \brief Nodelet initialization routine. */
      void onInit () override;

      /** \brief LazyNodelet connection routine. */
      void subscribe () override;
      void unsubscribe () override;

      // the callback function to handle input from subscription
      void sub_callback ( const sensor_msgs::PointCloud2::ConstPtr& );
      
    private:

        // parameters
        std::uint32_t 
          cluster_sz_min_ = 1
          , cluster_sz_max_ = std::numeric_limits<std::uint32_t>::max()
        ;

        float
            min_distance_x_ = 0.f
            , min_distance_y_ = 0.f
            , min_distance_z_ = 0.f
            , min_distance_ = 0.f
            , outlier_radius_ = 0.f
            , cluster_tolerance_ = 0.f
            , cluster_area_min_ = 0.f
            , cluster_area_max_ = std::numeric_limits<float>::max()
            ;

        int 
          outlier_min_neighbors_ = 0
          ;

        bool 
          cluster_inliers_ = true
          ;


        ros::Subscriber sub_;
        ros::Publisher pub_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  //#ifndef 