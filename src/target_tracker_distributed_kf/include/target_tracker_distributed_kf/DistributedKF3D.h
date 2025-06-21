//
// ROS2 migration of DistributedKF3D
// Originally created by glawless on 23.05.17.
// Migrated to ROS2 maintaining exact functionality
//

#ifndef TARGET_TRACKER_DISTRIBUTED_KF_DISTRIBUTEDKF3D_H
#define TARGET_TRACKER_DISTRIBUTED_KF_DISTRIBUTEDKF3D_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <uav_msgs/msg/uav_pose.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdDeque>

#include <pose_cov_ops/pose_cov_ops.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <deque>
#include <string>
#include <memory>
#include <mutex>

namespace target_tracker_distributed_kf {

using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::TwistWithCovarianceStamped;
using geometry_msgs::msg::PoseWithCovariance;
using uav_msgs::msg::UAVPose;
using namespace std;
using namespace Eigen;

typedef struct CacheElement_s{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    rclcpp::Time stamp;
    string frame_id;
    VectorXd state;
    MatrixXd cov;
    vector<shared_ptr<PoseWithCovariance>> measurements;
    bool isSelfRobot;
    int robot;

    CacheElement_s() = delete; // at least use constructor with stamp
    CacheElement_s(const std_msgs::msg::Header& h, const int vecSize, const bool selfRobotFlag, const int robotID) :
        stamp(h.stamp), frame_id(h.frame_id), state(VectorXd::Zero(vecSize)), cov(MatrixXd::Zero(vecSize, vecSize)), measurements(0), isSelfRobot(selfRobotFlag), robot(robotID){
            measurements.reserve(5);
        };
    CacheElement_s(const int vecSize, const PoseWithCovarianceStamped& m, const bool selfRobotFlag, const int robotID) :
        stamp(m.header.stamp), frame_id(m.header.frame_id), state(VectorXd::Zero(vecSize)), cov(MatrixXd::Zero(vecSize, vecSize)), measurements(0), isSelfRobot(selfRobotFlag), robot(robotID){
            measurements.reserve(5);
            auto ptr = shared_ptr<PoseWithCovariance>(new PoseWithCovariance);
            ptr->pose = m.pose.pose;
            ptr->covariance = m.pose.covariance;
            measurements.push_back(ptr);
        };

}CacheElement;

class Cache : public deque<CacheElement, aligned_allocator<CacheElement> >{

    private:
        size_t cache_size_{0};

    public:
        Cache() : cache_size_(0) {};

        void set_cache_size(size_t sz) {
            this->cache_size_ = sz;
        }

        deque<CacheElement>::iterator insert_ordered(const CacheElement& elem){
            // Sanity check
            if(cache_size_ <= 0)
                return end();

            // First element, special case
            if(empty())
                return insert(begin(), elem);

            // If full and time lower than the first in deque, then return nullptr to signalize that it failed to insert
            if(size() == cache_size_ && begin()->stamp.nanoseconds() > elem.stamp.nanoseconds())
                return end();

            // Pop enough elements to keep at max size
            while(!empty() && (size() + 1 > cache_size_))
                pop_front();

            if(empty())
                return insert(begin(), elem);

            // Always start from end, can't however use rbegin since we need to insert
            auto it = end();

            // Look for the stamp just after elem's stamp
            for(; it != begin(); --it) {
                // Use nanoseconds for comparison to avoid clock source issues
                if(it->stamp.nanoseconds() == elem.stamp.nanoseconds() && it->robot==elem.robot){
                    // Two measurements for the same time and robot, in this case we don't insert and instead add to the vector of measurements
                    if(!elem.measurements.empty()) {
                        auto ptr = shared_ptr<PoseWithCovariance>(new PoseWithCovariance);
                        ptr->pose = elem.measurements[0]->pose;
                        ptr->covariance = elem.measurements[0]->covariance;
                        it->measurements.push_back(ptr);
                    }
                    return it;
                }
                else if((it-1)->stamp.nanoseconds() < elem.stamp.nanoseconds()) {
                    // Check against previous one because insertion iterator should point to position after where we want to insert
                    break;
                }
            }

            // Insert is done before the it position
            it = insert(it, elem);

            // Return the iterator to inserted position
            return it;
        }

        void print(std::ostream &stream) const{
            stream << "Cache at time (current node time)" << std::endl;
            if(empty())
                stream << "Empty" << std::endl;
            else {
                stream << "Total of " << size() << " elements" << std::endl;
                stream << "\tStamp \t\tRobotID\t#Measurements\tFrame\tSelf\tState" << std::endl;
            }

            IOFormat NoEndLineOnRowSep(StreamPrecision,0," ", " ");
            for(auto it=begin(); it != end(); ++it){
                stream << "\t" << it->stamp.nanoseconds() * 1e-9 << "\t" << it->robot << "\t" << it->measurements.size() << "\t\t" << it->frame_id << "\t" << it->isSelfRobot << it->state.format(NoEndLineOnRowSep) << std::endl;
            }
        }
};

inline std::ostream& operator<< (std::ostream& stream, const Cache& cache) {
    cache.print(stream);
    return stream;
}

// Forward declaration
class DistributedKF3D;

class Callbackhandler {
public:
    DistributedKF3D *parent;
    Callbackhandler(DistributedKF3D *myparent, bool willbeself, int robot) {
        parent = myparent;
        isSelf = willbeself;
        robotID = robot;
    }
    int robotID;
    bool isSelf;
    void callback(const PoseWithCovarianceStamped::SharedPtr msg);
};

class DistributedKF3D : public rclcpp::Node {

private:
    // Subscribers
    std::vector<rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr> other_subs_;
    std::unique_ptr<Callbackhandler> selfcallbackhandler;
    std::vector<std::shared_ptr<Callbackhandler>> callbackhandlers;
    rclcpp::Subscription<UAVPose>::SharedPtr pose_sub_;
    rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr self_sub_;
    
    // Publishers
    rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr targetPub_;
    rclcpp::Publisher<TwistWithCovarianceStamped>::SharedPtr targetVelPub_;
    rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr offsetPub_;
    
    // Cache and thread safety
    Cache state_cache_;
    // std::mutex cache_mutex_;
    
    // Messages for publishing
    PoseWithCovarianceStamped msg_;
    TwistWithCovarianceStamped velMsg_;
    
    // Parameter callback
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    // Time tracking for backwards jump detection
    rclcpp::Time last_time_;
    
    // Private methods
    void initializeFilter();
    void setUnknownInitial(CacheElement&);
    bool predict(const CacheElement&, CacheElement&);
    bool update(CacheElement&);
    void initializeStaticMatrices();
    void populateJacobianG(MatrixXd &G, const double deltaT);
    void populateJacobianQ(MatrixXd &Q, const PoseWithCovariance& pcov);
    bool detectBackwardsTimeJump();
    
    void predictAndPublish(const UAVPose::SharedPtr msg);
    void publishStateAndCov(const CacheElement&);
    
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> & parameters);

protected:
    static constexpr auto state_size = 9;
    static constexpr auto measurement_state_size = 6;
    double time_threshold{10.0};

public:
    DistributedKF3D();
    
    // Method to be called after construction for shared_from_this
    void initialize();
    
    void measurementsCallback(const PoseWithCovarianceStamped::SharedPtr msg, const bool isSelf, const int robotID);
    
    // Static matrices
    MatrixXd I, Hself, Hother, R;
    
    // Initial values for uncertainty on reset
    double  initialUncertaintyPosXY{100},
            initialUncertaintyPosZ{10},
            initialUncertaintyVelXY{1},
            initialUncertaintyVelZ{0.5},
            initialUncertaintyOffsetXY{1},
            initialUncertaintyOffsetZ{3};
    
    // Noises
    double  noisePosXVar{0.0},
            noiseVelXVar{0.5},
            noiseOffXVar{0.02},
            noisePosYVar{0.0},
            noiseVelYVar{0.5},
            noiseOffYVar{0.02},
            noisePosZVar{0.0},
            noiseVelZVar{0.1},
            noiseOffZVar{0.02};
    
    double  posGlobalOffsetBiasX{0.0},
            posGlobalOffsetBiasY{0.0},
            posGlobalOffsetBiasZ{0.0};
    
    // Decay of velocity
    double velocityDecayTime{6.0};
    double offsetDecayTime{30.0};
    
    // FalsePositiveThreshhold x in x*sigma
    double falsePositiveThresholdSigma{2.0};
    
    void initializeSubscribers();
};

} // namespace target_tracker_distributed_kf

#endif //TARGET_TRACKER_DISTRIBUTED_KF_DISTRIBUTEDKF3D_H