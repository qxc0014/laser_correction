#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/Imu.h>
#include<nav_msgs/Odometry.h>
#include<tf2_ros/transform_broadcaster.h>
#include<tf2_ros/transform_listener.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>
#include<geometry_msgs/TransformStamped.h>
#include<geometry_msgs/PointStamped.h>
#include<sophus/se3.hpp>
#include<pcl-1.9/pcl/point_cloud.h>
#include<pcl-1.9/pcl/io/pcd_io.h>
#include<pcl-1.9/pcl/filters/voxel_grid.h>
#include<pcl-1.9/pcl/point_types.h>
#include<pcl-1.9/pcl/visualization/cloud_viewer.h>
#include<pcl-1.9/pcl/common/transforms.h>

using namespace std;
using namespace Eigen;
pcl::visualization::CloudViewer g_PointCloudView("PointCloud View");
class Corrtector{
    public: 
    Corrtector(ros::NodeHandle nh,ros::NodeHandle nh_private,tf2_ros::Buffer *buffer_,tf2_ros::TransformListener *tf_);
    ~Corrtector(){}
   // void imuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
    void pose_add(vector<double> ranges,vector<double> angles,std::vector<tf2::Transform> T,std::vector<double> a_cos,std::vector<double> a_sin,vector<double>& final_ranges,vector<double>& final_angles);
    private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber imu_sub;
    ros::Subscriber scan_sub;
    ros::Publisher scan_corrector_pub;
    std::vector<double> a_sin_;
    std::vector<double> a_cos_;
    std::vector<double> b_sin_;
    std::vector<double> b_cos_;
    std::vector<tf2::Transform> T_;
    tf2_ros::Buffer* buffer_;
    tf2_ros::TransformListener* tf_;
    vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> pointcloud;
     pcl::PointCloud<pcl::PointXYZRGB> visual_cloud_;
     pcl::PointCloud<pcl::PointXYZRGB> visual_cloud_1;
};
Corrtector::Corrtector(ros::NodeHandle nh,ros::NodeHandle nh_private,tf2_ros::Buffer *buffer,tf2_ros::TransformListener *tf)
    :nh_(nh),nh_private_(nh_private),buffer_(buffer),tf_(tf){
    try{
    buffer_->lookupTransform("odom","base_footprint",ros::Time(0),ros::Duration(1.0));
    ROS_INFO("init tf success!");
    }catch(tf2::LookupException& ex){
        ROS_INFO("error!init tf false!");
    }
   // imu_sub = nh.subscribe("imu_data",1000,&Corrtector::imuCallback,this);
    scan_sub = nh.subscribe("scan",1000,&Corrtector::scanCallback,this);
    scan_corrector_pub = nh.advertise<sensor_msgs::LaserScan>("scan_correct",10);
}

void Corrtector::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg){

    pointcloud.clear();
    /*一帧激光的起始时间与结束时间*/
    ros::Time start_time,end_time;//一帧laser的起始时间
    double scan_time; //每个激光点的delta t;
    start_time = scan_msg->header.stamp;
    scan_time = scan_msg->time_increment;//= 0.1s/360
    end_time = start_time + ros::Duration(scan_time * (scan_msg->ranges.size()-1));//用Duration将float转成ros::time格式


    /*将激光数据的深度、角度装载进容器中*/
    vector<double> ranges,new_ranges,new_angles;
    vector<double> angles;
    for(unsigned int i = 0;i<scan_msg->ranges.size();++i){
        double angle = scan_msg->angle_min + i*scan_msg->angle_increment;
        ranges.push_back(scan_msg->ranges[i]);
        angles.push_back(angle);     
    }


    /*每个角度转化为三角函数用于后面坐标计算*/
    a_cos_.clear();
    a_sin_.clear();
    for(unsigned int i = 0;i<scan_msg->ranges.size();++i){
        double cos_a = cos(angles[i]);
        double sin_a = sin(angles[i]); 
        a_cos_.push_back(cos_a);
        a_sin_.push_back(sin_a);
    }

    //std::cout << "start_time =" << start_time << "scan_time"<< 
    //scan_time << "size:=" << scan_msg->ranges.size()<< std::endl;


    /*获取每一时刻的里程计位姿*/
    T_.clear();
    tf2::Transform transform_1;//激光雷达第一个点对应的位姿
    int cnt=0;
    for(ros::Time section_time = start_time;section_time < end_time + ros::Duration(scan_time*18);section_time += ros::Duration(scan_time*18)){
        geometry_msgs::TransformStamped transformStamped;  
        try{
        transformStamped = buffer_->lookupTransform("odom","base_footprint",
        ros::Time(section_time),ros::Duration(0.1));
        tf2::Quaternion q(
            transformStamped.transform.rotation.x,
            transformStamped.transform.rotation.y,
            transformStamped.transform.rotation.z,
            transformStamped.transform.rotation.w); 
        tf2::Vector3 tran(
            transformStamped.transform.translation.x,
            transformStamped.transform.translation.y,
            transformStamped.transform.translation.z);
        tf2::Transform transform_2(q,tran);
       // cout << "tf_x:" << transform_2.getOrigin().getX() << endl;
        if(cnt!=0){
        tf2::Transform transform = transform_1.inverse() * transform_2 ;
        T_.push_back(transform);
        }else{
        transform_1 = transform_2;
        T_.push_back(tf2::Transform::getIdentity());
        cnt++;
        }
        //cout << "section_time = " << section_time <<"T time= "<< transformStamped.header.stamp << endl;
        }catch(tf2::LookupException& ex){
            ROS_INFO("error!can not recieve tf!");
        }
    } 
    cout << "T's size:"<< T_.size() << endl;
    /*进行位姿插值*/
    new_ranges.clear();
    new_ranges.reserve(360);
    new_angles.clear();
    new_angles.reserve(360);
    pose_add(ranges,angles,T_,a_cos_,a_sin_,new_ranges,new_angles);
    
    /*点云显示*/
    visual_cloud_.clear();
    visual_cloud_1.clear();
    
    /*计算新的三角函数*/
    b_cos_.clear();
    b_sin_.clear();
    for(unsigned int i = 0;i<scan_msg->ranges.size();++i){
        double cos_b = cos(new_angles[i]);
        double sin_b = sin(new_angles[i]); 
        b_cos_.push_back(cos_b);
        b_sin_.push_back(sin_b);
    }
    /*发布校正后的激光雷达数据*/
    sensor_msgs::LaserScan scan_correct_msg;
    scan_correct_msg.angle_increment = scan_msg->angle_increment;
    scan_correct_msg.angle_max = scan_msg->angle_max;
    scan_correct_msg.angle_min = scan_msg->angle_min;
    scan_correct_msg.header = scan_msg->header;
    scan_correct_msg.intensities = scan_msg->intensities;
    scan_correct_msg.range_max = scan_msg->range_max;
    scan_correct_msg.range_min = scan_msg->range_min;
    scan_correct_msg.scan_time = scan_msg->scan_time;
    scan_correct_msg.time_increment = scan_msg->time_increment;
    scan_correct_msg.ranges.resize(360);                          
    for(int i = 0;i<scan_msg->ranges.size();++i){
        if(ranges[i] != INFINITY){
        pcl::PointXYZRGB pt_old;
        pt_old.x = ranges[i]*a_cos_[i];
        pt_old.y = ranges[i]*a_sin_[i];
        pt_old.z = 1;
        unsigned char r1 = 0, g1 = 255, b1 = 0;    //red color
        unsigned int rgb1 = ((unsigned int)r1 << 16 | (unsigned int)g1 << 8 | (unsigned int)b1);
        pt_old.rgb = *reinterpret_cast<float*>(&rgb1); 
        visual_cloud_.push_back(pt_old);


        pcl::PointXYZRGB pt_new;
        pt_new.x = new_ranges[i]*b_cos_[i];
        pt_new.y = new_ranges[i]*b_sin_[i];
        pt_new.z = 1;
        unsigned char r = 255, g = 0, b = 0;    //red color
        unsigned int rgb = ((unsigned int)r << 16 | (unsigned int)g << 8 | (unsigned int)b);
        pt_new.rgb = *reinterpret_cast<float*>(&rgb); 
        visual_cloud_.push_back(pt_new);
        //cout << "pt_old.x" << i << ":" << ranges[i] << "pt_new.x" << i << ":" << new_ranges[i] <<endl;
        //tf::Vector3 point(r*a_cos_[i],r*a_sin_[i],0);
        }
    }
    /*无穷点矫正后还是无穷点*/
    for(int i = 0;i < 360;i++){
        if(ranges[i] == INFINITY){
            new_ranges[i] = INFINITY;
        }
        scan_correct_msg.ranges[i] = new_ranges[i];
    }
    /*显示点云*/
    g_PointCloudView.showCloud(visual_cloud_.makeShared());
    //g_PointCloudView.showCloud(visual_cloud_1.makeShared());
   // scan_corrector_pub.publish(scan_correct_msg);
}
/*位姿插值*/
void Corrtector::pose_add(
            vector<double> ranges,
            vector<double> angles,
            std::vector<tf2::Transform> T,
            std::vector<double> a_cos,
            std::vector<double> a_sin,
            vector<double>& final_ranges,
            vector<double>& final_angles)
{
    std::vector<tf2::Vector3> allpoint;//所有激光点对应的位姿
    allpoint.clear();
    allpoint.reserve(360);
    vector<tf2::Transform> allT;
    allT.reserve(360);

    /*分为20段激光雷达数据*/
    for(int i = 0;i < T.size() - 1;i++ ){
        /*求每段开头激光束的位姿*/
        tf2::Transform beginT = T[i];
        tf2::Transform endT = T[i+1];
        allT.push_back(beginT);
                
            tf2::Vector3 beginT_tran = beginT.getOrigin();
            tf2::Quaternion beginT_q = beginT.getRotation();
            tf2::Vector3 endT_tran =  endT.getOrigin();
            tf2::Quaternion endT_q = endT.getRotation();

                 for(int index = 1;index < 18;index++){
                   tf2::Vector3 midT_tran = beginT_tran.lerp(endT_tran,index/18.00);
                   tf2::Quaternion midT_q = beginT_q.slerp(endT_q,index/18.00);
                   tf2::Transform mid_T(midT_q,midT_tran);
                   allT.push_back(mid_T);
                 }
    }
    for(int i = 0;i < ranges.size();i++){
        if(ranges[i] != Infinity){
            tf2::Vector3 mid_point(ranges[i]*a_cos_[i],ranges[i]*a_sin_[i],0);
            tf2::Vector3 new_point = allT[i] * mid_point;
            final_ranges[i] = std::sqrt(new_point[0]*new_point[0]+new_point[1]*new_point[1]);
          //  cout << "第" << i << "个激光点的原始夹角："<< atan2(mid_point[1],mid_point[0])
           //      << "第" << i << "个激光点的新的夹角："<< atan2(new_point[1],new_point[0])
            //     << endl;
            final_angles.push_back(atan2(new_point[1],new_point[0]));
        }
    }
    
    cout <<  "一共存了" << allT.size()<<"个位姿" << endl;
    for(int i=0;i < allpoint.size();i++)
    cout << allpoint[i][0]<<","<<allpoint[i][1] << endl;
}
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"laser_corrector");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    tf2_ros::Buffer tfBuffer(ros::Duration(20.0));
    tf2_ros::TransformListener tf(tfBuffer);
    Corrtector s1(nh,nh_private,&tfBuffer,&tf);
    ros::spin();
    return 0;
}
