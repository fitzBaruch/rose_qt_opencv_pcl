#include <qt_app/tripod_detection.h>

namespace detection {

using namespace std;

TripodDetection::TripodDetection(ros::NodeHandle n, ros::NodeHandle nh) :
    n_(n), left_cloud_buffer_(10), right_cloud_buffer_(10),
    pose_translation_buffer_(5), pose_rotation_buffer_(5),
    target_model_cloud_(new VPointCloud),
    left_start_register_flag_(false), right_start_register_flag_(false) {

    InitParameter(nh);
}


void TripodDetection::InitParameter(ros::NodeHandle nh) {
    string left_lidar_topic;
    string right_lidar_topic;
    string target_file_name;

    nh.param<std::string>("left_lidar_topic", left_lidar_topic, "/livox/lidar_1HDDH1200101761");
    nh.param<std::string>("right_lidar_topic", right_lidar_topic, "/livox/lidar_1HDDH1200103421");
    nh.param<std::string>("target_file_name", target_file_name, "/home/ubuntu/715_ws/src/sanjiaojia_detect/triangle_detect/pcd_files/sjj_dst.pcd");

    global_transform_ = Eigen::Affine3f::Identity();
//    global_transform_.translation() << 0, 0, 0;
//    global_transform_.rotate (Eigen::AngleAxisf (0.0, Eigen::Vector3f::UnitX()));
//    global_transform_.rotate (Eigen::AngleAxisf (0.0, Eigen::Vector3f::UnitY()));
//    global_transform_.rotate (Eigen::AngleAxisf (0.0, Eigen::Vector3f::UnitZ()));

    Eigen::Affine3f transform_high = Eigen::Affine3f::Identity();
    transform_high.translation() << 2.0, -0.8, 0.5;
    transform_high.rotate (Eigen::AngleAxisf (0.0, Eigen::Vector3f::UnitX()));
    transform_high.rotate (Eigen::AngleAxisf (-1.6, Eigen::Vector3f::UnitY()));
    transform_high.rotate (Eigen::AngleAxisf (0.0, Eigen::Vector3f::UnitZ()));

    /// not used yet
    Eigen::Affine3f transform_low = Eigen::Affine3f ::Identity();
    transform_low.translation() << 4.5, -1.4, 0.5;
    transform_low.rotate (Eigen::AngleAxisf (-0.2, Eigen::Vector3f::UnitX()));
    transform_low.rotate (Eigen::AngleAxisf (-1.6, Eigen::Vector3f::UnitY()));
    transform_low.rotate (Eigen::AngleAxisf (0.0, Eigen::Vector3f::UnitZ()));

    init_guess_high_ = transform_high.matrix();
    init_guess_low_ = transform_low.matrix();

    /// 左右两个激光的外参
    right_left_transform_ = Eigen::Matrix4f::Identity();
    right_left_transform_(1,3) = -0.117;

    /// set ROI
    global_roi_.position_x = 6;
    global_roi_.scale_x = 11;
    global_roi_.position_y = -1.1;
    global_roi_.scale_y = 3.0;
    global_roi_.position_z = 0.2;
    global_roi_.scale_z = 1.5;

    init_point_roi_.position_x = 1.8;
    init_point_roi_.scale_x = 0.6;
    init_point_roi_.position_y = -1.69;
    init_point_roi_.scale_y = 1.0;
    init_point_roi_.position_z = 0.28;
    init_point_roi_.scale_z = 0.6;

    /// 加载目标点云
    pcl::io::loadPCDFile(target_file_name, *target_model_cloud_);

    init_guess_ = init_guess_high_;

    /// 设置平滑权重
    smoothing_weights_ = {1.0, 2.0, 3.0, 4.0, 5.0};

    /// 初始化ros
    sub_left_lidar_cloud_ = n_.subscribe<sensor_msgs::PointCloud2>(left_lidar_topic, 10, &TripodDetection::LeftLidarCloudHandler, this);
    sub_right_lidar_cloud_ = n_.subscribe<sensor_msgs::PointCloud2>(right_lidar_topic, 10, &TripodDetection::RightLidarCloudHandler, this);

    pub_left_raw_cloud_ = n_.advertise<sensor_msgs::PointCloud2>("left_raw_pc", 10);
    pub_right_raw_cloud_ = n_.advertise<sensor_msgs::PointCloud2>("right_raw_pc", 10);
    pub_transformed_model_cloud_ = n_.advertise<sensor_msgs::PointCloud2>("transform_model_pc", 10);
    pub_bbox_ = n_.advertise<visualization_msgs::Marker>("cluster_box", 10);
    pub_global_roi_marker_ = n_.advertise<visualization_msgs::Marker>("/ROI", 1);
    pub_init_roi_marker_ = n_.advertise<visualization_msgs::Marker>("/start_ROI", 1);

}


void TripodDetection::LeftLidarCloudHandler(const sensor_msgs::PointCloud2ConstPtr &lidar_msg) {
    VPointCloud cloud, filtered_cloud;
    pcl::fromROSMsg(*lidar_msg, cloud);

    left_transform_cloud_.clear();
    pcl::transformPointCloud(cloud, left_transform_cloud_, global_transform_);

    /// ROI filter
    for(VPoint p: left_transform_cloud_.points) {
        if(global_roi_.isPointInROI(p)) {
            filtered_cloud.push_back(p);
        }
    }

    left_cloud_buffer_.push_back(filtered_cloud);
    left_lidar_count_ ++;

    if(left_lidar_count_ > 10) {
        left_source_roi_cloud_.clear();
        for(unsigned int i = 0; i < left_cloud_buffer_.size(); i++) {
          left_source_roi_cloud_ += left_cloud_buffer_.at(i);
        }
        left_start_register_flag_ = true;
    }
}


void TripodDetection::RightLidarCloudHandler(const sensor_msgs::PointCloud2ConstPtr &lidar_msg) {
    VPointCloud cloud, filtered_cloud;
    pcl::fromROSMsg(*lidar_msg, cloud);

    right_transform_cloud_.clear();
    pcl::transformPointCloud(cloud, right_transform_cloud_, (right_left_transform_ * (global_transform_.matrix())));

    /// ROI filter
    for(VPoint p: right_transform_cloud_.points) {
        if(global_roi_.isPointInROI(p)) {
            filtered_cloud.push_back(p);
        }
    }

    right_cloud_buffer_.push_back(filtered_cloud);
    right_lidar_count_ ++;

    if(right_lidar_count_ > 10) {
        right_source_roi_cloud_.clear();
        for(unsigned int i = 0; i < right_cloud_buffer_.size(); i++) {
          right_source_roi_cloud_ += right_cloud_buffer_.at(i);
        }
        right_start_register_flag_ = true;
    }
}

void TripodDetection::DownsampleCloud(VPointCloud::Ptr in_cloud, VPointCloud::Ptr out_cloud, float in_leaf_size) {
    pcl::VoxelGrid<VPoint> sor;
    sor.setInputCloud(in_cloud);
    sor.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
    sor.filter(*out_cloud);
}


ClusterObject TripodDetection::ClusterCloud(VPointCloud::Ptr input_cloud) {
    VPointCloud::Ptr downsampled_cloud(new VPointCloud);
    DownsampleCloud(input_cloud, downsampled_cloud, 0.05);
    pcl::search::KdTree<VPoint>::Ptr tree(new pcl::search::KdTree<VPoint>);
    tree->setInputCloud(downsampled_cloud);

    std::vector<pcl::PointIndices> local_indices;
    pcl::EuclideanClusterExtraction<VPoint> euclid;
    euclid.setInputCloud(downsampled_cloud);
    euclid.setClusterTolerance(0.15);
    euclid.setMinClusterSize(MIN_CLUSTER_SIZE);
    euclid.setMaxClusterSize(MAX_CLUSTER_SIZE);
    euclid.setSearchMethod(tree);
    euclid.extract(local_indices);

    ClusterObject obj_info;

    if(local_indices.size() == 0) {
        return obj_info;
    }

    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

    for (auto pit = local_indices[0].indices.begin(); pit != local_indices[0].indices.end(); ++pit)
    {
        //fill new colored cluster point by point
        VPoint p;
        p.x = downsampled_cloud->points[*pit].x;
        p.y = downsampled_cloud->points[*pit].y;
        p.z = downsampled_cloud->points[*pit].z;

        obj_info.centroid_.x += p.x;
        obj_info.centroid_.y += p.y;
        obj_info.centroid_.z += p.z;

        if (p.x < min_x)
            min_x = p.x;
        if (p.y < min_y)
            min_y = p.y;
        if (p.z < min_z)
            min_z = p.z;
        if (p.x > max_x)
            max_x = p.x;
        if (p.y > max_y)
            max_y = p.y;
        if (p.z > max_z)
            max_z = p.z;
        obj_info.cloud_.push_back(p);
    }

    obj_info.min_point_.x = min_x;
    obj_info.min_point_.y = min_y;
    obj_info.min_point_.z = min_z;

    obj_info.max_point_.x = max_x;
    obj_info.max_point_.y = max_y;
    obj_info.max_point_.z = max_z;

    if (local_indices[0].indices.size() > 0) {
        obj_info.centroid_.x /= local_indices[0].indices.size();
        obj_info.centroid_.y /= local_indices[0].indices.size();
        obj_info.centroid_.z /= local_indices[0].indices.size();
    }

    double length_ = obj_info.max_point_.x - obj_info.min_point_.x;
    double width_ = obj_info.max_point_.y - obj_info.min_point_.y;
    double height_ = obj_info.max_point_.z - obj_info.min_point_.z;

    obj_info.bounding_box_.header.frame_id = "livox_frame";
    obj_info.bounding_box_.ns = "my_namespace";
    obj_info.bounding_box_.id = 0;
    obj_info.bounding_box_.type = visualization_msgs::Marker::CUBE;
    obj_info.bounding_box_.action = visualization_msgs::Marker::ADD;

    obj_info.bounding_box_.pose.position.x = obj_info.min_point_.x + length_ / 2;
    obj_info.bounding_box_.pose.position.y = obj_info.min_point_.y + width_ / 2;
    obj_info.bounding_box_.pose.position.z = obj_info.min_point_.z + height_ / 2;

    obj_info.bounding_box_.scale.x = ((length_ < 0) ? -1 * length_ : length_);
    obj_info.bounding_box_.scale.y = ((width_ < 0) ? -1 * width_ : width_);
    obj_info.bounding_box_.scale.z = ((height_ < 0) ? -1 * height_ : height_);

    obj_info.bounding_box_.color.a = 0.5;
    obj_info.bounding_box_.color.g = 1.0;

    return obj_info;
}

Eigen::Matrix4f TripodDetection::SmoothingPose(std::vector<float> weights) {
    Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
    if(pose_translation_buffer_.size() != weights.size()) {
        cout<<"weight size wrong !!!!!"<<endl;
        return result;
    }

    std::vector<Eigen::Vector3f> vec_translation(pose_translation_buffer_.begin(), pose_translation_buffer_.end());
    std::vector<Eigen::Quaternionf> vec_quaternion(pose_rotation_buffer_.begin(), pose_rotation_buffer_.end());

    float weigth_sum = 0;
    Eigen::Vector3f trans_sum(0, 0, 0);
    /// 平滑位移
    for(size_t i = 0; i < vec_translation.size(); i++) {
        trans_sum = weights[i]*vec_translation[i] + trans_sum;
        weigth_sum += weights[i];
    }
    trans_sum = (1.0 / weigth_sum) * trans_sum;

    /// 平滑旋转
    Eigen::Matrix4f temp_rotation = Eigen::Matrix4f::Zero();
    for(size_t i = 0; i < vec_quaternion.size(); i++) {
        Eigen::Vector4f coffe(vec_quaternion[i].x(),
                              vec_quaternion[i].y(),
                              vec_quaternion[i].z(),
                              vec_quaternion[i].w());
        temp_rotation = weights[i]*(coffe * coffe.transpose()) + temp_rotation;
    }
    temp_rotation = (1.0 / weigth_sum) * temp_rotation;

    Eigen::EigenSolver<Eigen::Matrix4f> es;
    es.compute(temp_rotation,true);

    Eigen::Vector4f eigenvalueAbsVector;
    for(int i = 0; i < 4; i++) {
      eigenvalueAbsVector(i) = fabs(es.eigenvalues()(i));
    }

    int maxEigenvalueIndex = 0;
    eigenvalueAbsVector.maxCoeff(&maxEigenvalueIndex);

    auto q_coffe = es.eigenvectors().col(maxEigenvalueIndex).normalized();

    Eigen::Quaternionf average_q(q_coffe[3].real(), q_coffe[0].real(), q_coffe[1].real(), q_coffe[2].real());

    result.block<3,3>(0,0) = average_q.toRotationMatrix();
    result(0,3) = trans_sum(0);
    result(1,3) = trans_sum(1);
    result(2,3) = trans_sum(2);

    return result;
}


void TripodDetection::ProcessScan() {
    object_after_cluster_ = ClusterCloud(source_roi_cloud_.makeShared());

    VPointCloud::Ptr downsampled_source_cloud(new VPointCloud);
    DownsampleCloud(object_after_cluster_.cloud_.makeShared(), downsampled_source_cloud, 0.1);
    ndt_.setInputSource(downsampled_source_cloud);

    VPointCloud output;
    ndt_.align(output, init_guess_.inverse());

    Eigen::Matrix4f ndt_result = ndt_.getFinalTransformation();
    ndt_result = ndt_result.inverse();
    init_guess_ = ndt_result;           //更新变换

    pose_translation_buffer_.push_back(Eigen::Vector3f(init_guess_(0,3), init_guess_(1,3), init_guess_(2,3)));
    pose_rotation_buffer_.push_back(Eigen::Quaternionf(init_guess_.block<3,3>(0,0)));
    smoothing_count_ ++;
    if(smoothing_count_ > 5) {
        init_guess_ = SmoothingPose(smoothing_weights_);
    }
}


bool TripodDetection::JudgeInitPoint(VPointCloud source_cloud) {
    VPointCloud roi_cloud;
    VPointCloud::Ptr downSampledCloud(new VPointCloud);
    for(VPoint p: source_cloud.points) {
        if(init_point_roi_.isPointInROI(p)) {
            roi_cloud.push_back(p);
        }
    }
    DownsampleCloud(roi_cloud.makeShared(), downSampledCloud, 0.01);

    if(downSampledCloud->points.size() > 150) {
        double current_time = ros::Time::now().toSec();
        if(end_point_time_ == 0 || (current_time - end_point_time_) > 10) {
           init_point_time_ =  current_time;
           cout<<"Start detection !!!!"<<endl;
           return true;
        }
    }
    return false;
}


bool TripodDetection::JudgeEndPoint(VPointCloud source_cloud) {
    VPointCloud roi_cloud;
    VPointCloud::Ptr downSampledCloud(new VPointCloud);
    for(VPoint p: source_cloud.points) {
        if(init_point_roi_.isPointInROI(p)) {
            roi_cloud.push_back(p);
        }
    }
    DownsampleCloud(roi_cloud.makeShared(), downSampledCloud, 0.01);

    if(downSampledCloud->points.size() > 150) {
        double current_time = ros::Time::now().toSec();
        if(current_time - init_point_time_ > 30) {
            end_point_time_ = current_time;
            cout<<"Stop detection !!!!"<<endl;
            return true;
        }
    }
    return false;
}




}
