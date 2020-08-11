#ifndef TRIPOD_DETECTION_H
#define TRIPOD_DETECTION_H

#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <pclomp/ndt_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/conversions.h>
#include <tf/transform_broadcaster.h>

#include <boost/circular_buffer.hpp>

#define MIN_CLUSTER_SIZE 50
#define MAX_CLUSTER_SIZE 50000

namespace detection {

typedef pcl::PointXYZ VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;


/// ROI的管理，代码中有挺多的ROI
struct ROI {
    double position_x;
    double scale_x;
    double position_y;
    double scale_y;
    double position_z;
    double scale_z;
    visualization_msgs::Marker marker_;

    bool isPointInROI(VPoint p) {
        if(p.x <= (position_x + scale_x / 2) && p.x >= (position_x - scale_x / 2) &&
           p.y <= (position_y + scale_y / 2) && p.y >= (position_y - scale_y / 2) &&
           p.z <= (position_z + scale_z / 2) && p.z >= (position_z - scale_z / 2)) {
            return true;
        }
        return false;
    }

    void getRoiMarker(std::string frame_id, double r, double g, double b) {
        marker_.header.frame_id = frame_id;
        marker_.header.stamp = ros::Time();
        marker_.ns = "my_namespace";
        marker_.id = 0;
        marker_.type = visualization_msgs::Marker::CUBE;
        marker_.action = visualization_msgs::Marker::ADD;
        marker_.pose.position.x = position_x;
        marker_.pose.position.y = position_y;
        marker_.pose.position.z = position_z;
        marker_.scale.x = scale_x;
        marker_.scale.y = scale_y;
        marker_.scale.z = scale_z;
        marker_.color.a = 0.3;
        marker_.color.r = r;
        marker_.color.g = g;
        marker_.color.b = b;
    }

};


/// 聚类的结果
struct ClusterObject {
    visualization_msgs::Marker bounding_box_;

    VPointCloud cloud_;
    VPoint min_point_;
    VPoint max_point_;
    VPoint centroid_;
};


class TripodDetection {
public:
    TripodDetection(ros::NodeHandle n, ros::NodeHandle nh);

    /// 初始化参数和ROS
    void InitParameter(ros::NodeHandle nh);

    /// 左边激光的回调函数
    void LeftLidarCloudHandler(const sensor_msgs::PointCloud2ConstPtr & lidar_msg);

    /// 右边激光的回调函数
    void RightLidarCloudHandler(const sensor_msgs::PointCloud2ConstPtr & lidar_msg);

    /// 降采样
    void DownsampleCloud(VPointCloud::Ptr in_cloud, VPointCloud::Ptr out_cloud, float in_leaf_size);

    /// 聚类
    ClusterObject ClusterCloud(VPointCloud::Ptr input_cloud);

    /// 平滑位姿态
    Eigen::Matrix4f SmoothingPose(std::vector<float> weights);

    /// 处理一帧点云
    void ProcessScan();

    /// 判断初始点（开始检测）
    bool JudgeInitPoint(VPointCloud source_cloud);

    /// 判断终止点（停止检测）
    bool JudgeEndPoint(VPointCloud source_cloud);

private:
    ros::NodeHandle n_;
    ros::NodeHandle nh_;

    boost::circular_buffer<VPointCloud> left_cloud_buffer_;
    boost::circular_buffer<VPointCloud> right_cloud_buffer_;

    boost::circular_buffer<Eigen::Vector3f> pose_translation_buffer_;
    boost::circular_buffer<Eigen::Quaternionf> pose_rotation_buffer_;

    /// 均值滤波的权重
    std::vector<float> smoothing_weights_;

    /// 全局点云的变化矩阵，平面与y-z平面平行（标定给出的参数）
    Eigen::Affine3f global_transform_;

    /// 两个livox的外参（以左边的livox的中心）
    Eigen::Matrix4f right_left_transform_;

    /// 上方的初始位姿（标定给的参数）
    Eigen::Matrix4f init_guess_high_;

    /// 下方的初始位姿态（出水）（标定给的参数）
    Eigen::Matrix4f init_guess_low_;

    /// 目标点云在源点云的位姿，init_guess_.inverse()也是ndt的预测矩阵
    Eigen::Matrix4f init_guess_;

    /// 全局的ROI
    ROI global_roi_;

    /// 初始点的ROI区域，判断何时开始匹配
    ROI init_point_roi_;

    VPointCloud left_source_roi_cloud_;
    VPointCloud right_source_roi_cloud_;
    VPointCloud source_roi_cloud_;

    VPointCloud left_transform_cloud_;
    VPointCloud right_transform_cloud_;

    /// 目标点云
    VPointCloud::Ptr target_model_cloud_;

    /// NDT
    pclomp::NormalDistributionsTransform<VPoint, VPoint> ndt_;

    /// 左右两边新点云达到的flag
    bool left_start_register_flag_ = false;
    bool right_start_register_flag_ = false;

    int left_lidar_count_ = 0;
    int right_lidar_count_ = 0;
    int smoothing_count_ = 0;

    double init_point_time_ = 0.0;
    double end_point_time_ = 0.0;

    /// 聚类的结果
    ClusterObject object_after_cluster_;

    /// ros Publisher
    ros::Publisher pub_left_raw_cloud_;
    ros::Publisher pub_right_raw_cloud_;
    ros::Publisher pub_transformed_model_cloud_;
    ros::Publisher pub_bbox_;
    ros::Publisher pub_global_roi_marker_;
    ros::Publisher pub_init_roi_marker_;

    /// ros Subscriber
    ros::Subscriber sub_left_lidar_cloud_;
    ros::Subscriber sub_right_lidar_cloud_;



};

}


#endif
