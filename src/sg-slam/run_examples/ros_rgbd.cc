// -----------------------------------------------------------------------------
// SG‑SLAM RGB‑D wrapper for Intel RealSense (D435/D435i)
// -----------------------------------------------------------------------------
//  * Subscribes to two image topics (configurable via ROS params)
//  * Feeds them to ORB‑SLAM2 System in RGB‑D mode
//  * Publishes camera pose as /Camera_Pose (PoseStamped)
// -----------------------------------------------------------------------------

#include <System.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Core>

// -----------------------------------------------------------------------------
using sync_pol = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>;

static geometry_msgs::PoseStamped g_pose_msg;
static tf::TransformBroadcaster *g_tf_pub{nullptr};

// -----------------------------------------------------------------------------
void publishPose(const cv::Mat &Tcw)
{
    if (Tcw.empty()) return;

    cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc * Tcw.rowRange(0,3).col(3);

    Eigen::Matrix3d R;
    R << Rwc.at<float>(0,0), Rwc.at<float>(0,1), Rwc.at<float>(0,2),
         Rwc.at<float>(1,0), Rwc.at<float>(1,1), Rwc.at<float>(1,2),
         Rwc.at<float>(2,0), Rwc.at<float>(2,1), Rwc.at<float>(2,2);
    Eigen::Quaterniond q(R);

    tf::Transform tf_cam;
    tf_cam.setOrigin(tf::Vector3(twc.at<float>(2), -twc.at<float>(0), -twc.at<float>(1)));
    tf_cam.setRotation(tf::Quaternion(q.z(), -q.x(), -q.y(), q.w()));

    ros::Time stamp = ros::Time::now();
    if (!g_tf_pub) g_tf_pub = new tf::TransformBroadcaster;
    g_tf_pub->sendTransform(tf::StampedTransform(tf_cam, stamp, "map", "camera"));

    g_pose_msg.header.stamp = stamp;
    g_pose_msg.header.frame_id = "map";
    tf::pointTFToMsg(tf_cam.getOrigin(), g_pose_msg.pose.position);
    tf::quaternionTFToMsg(tf_cam.getRotation(), g_pose_msg.pose.orientation);
}

// -----------------------------------------------------------------------------
class ImageGrabber
{
public:
    explicit ImageGrabber(ORB_SLAM2::System &sys) : slam_(sys)
    {
        ros::NodeHandle nh("~");
        pub_ = nh.advertise<geometry_msgs::PoseStamped>("/Camera_Pose", 1);

        std::string topic_rgb, topic_depth;
        nh.param<std::string>("topic_rgb",   topic_rgb,   "/camera/color/image_raw");
        nh.param<std::string>("topic_depth", topic_depth, "/camera/aligned_depth_to_color/image_raw");

        rgb_sub_.subscribe(nh,   topic_rgb,   30);
        depth_sub_.subscribe(nh, topic_depth, 30);
        sync_.reset(new message_filters::Synchronizer<sync_pol>(sync_pol(10), rgb_sub_, depth_sub_));
        sync_->registerCallback(boost::bind(&ImageGrabber::callback, this, _1, _2));
    }

private:
    void callback(const sensor_msgs::ImageConstPtr &rgb_msg,
                  const sensor_msgs::ImageConstPtr &depth_msg)
    {
        // RGB
        cv_bridge::CvImageConstPtr cv_rgb;
        try { cv_rgb = cv_bridge::toCvShare(rgb_msg, "rgb8"); }
        catch (cv_bridge::Exception &e) { ROS_ERROR("cv_bridge: %s", e.what()); return; }

        // depth (native 16UC1)
        cv_bridge::CvImageConstPtr cv_d;
        try { cv_d = cv_bridge::toCvShare(depth_msg); }
        catch (cv_bridge::Exception &e) { ROS_ERROR("cv_bridge: %s", e.what()); return; }

        cv::Mat Tcw = slam_.TrackRGBD(cv_rgb->image, cv_d->image, rgb_msg->header.stamp.toSec());
        publishPose(Tcw);
        pub_.publish(g_pose_msg);
    }

    ORB_SLAM2::System &slam_;
    ros::Publisher pub_;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    std::shared_ptr< message_filters::Synchronizer<sync_pol> > sync_;
};

// -----------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sg_slam_ros_rgbd");
    if (argc != 3)
    {
        std::cerr << "Usage: rosrun sg_slam_ros_rgbd path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD, true);
    ImageGrabber grabber(SLAM);

    ros::spin();
    SLAM.Shutdown();
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    return 0;
}
