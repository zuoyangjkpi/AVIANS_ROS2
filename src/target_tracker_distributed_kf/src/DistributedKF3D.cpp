//
// Created by glawless on 23.05.17.
// Migrated to ROS2
// FIXED: Added proper clock synchronization and timestamp validation
//

#include <mrpt/math/distributions.h>
#include <target_tracker_distributed_kf/DistributedKF3D.h>
#include <ros2_utils/clock_sync.hpp>
#include <cmath>

namespace target_tracker_distributed_kf {

    static const std::string world_frame{"world"};

    DistributedKF3D::DistributedKF3D() : Node("distributed_kf_3d"),
    I((int) state_size, (int) state_size),
    Hself((int) measurement_state_size, (int) state_size),
    Hother((int) measurement_state_size, (int) state_size),
    R((int) state_size, (int) state_size) {
        
        // CRITICAL: Declare use_sim_time parameter FIRST before anything else
        if (!this->has_parameter("use_sim_time")) {
            this->declare_parameter("use_sim_time", true);
        }
        
        // Log sim time status for debugging
        bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
        if (use_sim_time) {
            RCLCPP_INFO(this->get_logger(), "Using simulation time");
        } else {
            RCLCPP_INFO(this->get_logger(), "Using system time");
        }
        
        // Declare all parameters with default values
        this->declare_parameter("initialUncertaintyPosXY", initialUncertaintyPosXY);
        this->declare_parameter("initialUncertaintyPosZ", initialUncertaintyPosZ);
        this->declare_parameter("initialUncertaintyVelXY", initialUncertaintyVelXY);
        this->declare_parameter("initialUncertaintyVelZ", initialUncertaintyVelZ);
        this->declare_parameter("initialUncertaintyOffsetXY", initialUncertaintyOffsetXY);
        this->declare_parameter("initialUncertaintyOffsetZ", initialUncertaintyOffsetZ);
        this->declare_parameter("falsePositiveThresholdSigma", falsePositiveThresholdSigma);
        this->declare_parameter("pub_topic", std::string("target_tracker/pose"));
        this->declare_parameter("velPub_topic", std::string("target_tracker/twist"));
        this->declare_parameter("offset_topic", std::string("target_tracker/offset"));
        this->declare_parameter("reset_time_threshold", time_threshold);
        this->declare_parameter("cache_size", 20);
        this->declare_parameter("robotID", 0);
        this->declare_parameter("numRobots", 0);
        this->declare_parameter("pose_topic", std::string("pose"));
        this->declare_parameter("measurement_topic_suffix_self", std::string("/nonono"));
        this->declare_parameter("measurement_topic_suffix", std::string("/nonono"));
        
        // Noise parameters
        this->declare_parameter("noisePosXVar", noisePosXVar);
        this->declare_parameter("noiseVelXVar", noiseVelXVar);
        this->declare_parameter("noiseOffXVar", noiseOffXVar);
        this->declare_parameter("noisePosYVar", noisePosYVar);
        this->declare_parameter("noiseVelYVar", noiseVelYVar);
        this->declare_parameter("noiseOffYVar", noiseOffYVar);
        this->declare_parameter("noisePosZVar", noisePosZVar);
        this->declare_parameter("noiseVelZVar", noiseVelZVar);
        this->declare_parameter("noiseOffZVar", noiseOffZVar);
        this->declare_parameter("posGlobalOffsetBiasX", posGlobalOffsetBiasX);
        this->declare_parameter("posGlobalOffsetBiasY", posGlobalOffsetBiasY);
        this->declare_parameter("posGlobalOffsetBiasZ", posGlobalOffsetBiasZ);
        this->declare_parameter("velocityDecayTime", velocityDecayTime);
        this->declare_parameter("offsetDecayTime", offsetDecayTime);

        // Get parameters
        initialUncertaintyPosXY = this->get_parameter("initialUncertaintyPosXY").as_double();
        initialUncertaintyPosZ = this->get_parameter("initialUncertaintyPosZ").as_double();
        initialUncertaintyVelXY = this->get_parameter("initialUncertaintyVelXY").as_double();
        initialUncertaintyVelZ = this->get_parameter("initialUncertaintyVelZ").as_double();
        initialUncertaintyOffsetXY = this->get_parameter("initialUncertaintyOffsetXY").as_double();
        initialUncertaintyOffsetZ = this->get_parameter("initialUncertaintyOffsetZ").as_double();
        falsePositiveThresholdSigma = this->get_parameter("falsePositiveThresholdSigma").as_double();
        time_threshold = this->get_parameter("reset_time_threshold").as_double();
        
        // Get noise parameters
        noisePosXVar = this->get_parameter("noisePosXVar").as_double();
        noiseVelXVar = this->get_parameter("noiseVelXVar").as_double();
        noiseOffXVar = this->get_parameter("noiseOffXVar").as_double();
        noisePosYVar = this->get_parameter("noisePosYVar").as_double();
        noiseVelYVar = this->get_parameter("noiseVelYVar").as_double();
        noiseOffYVar = this->get_parameter("noiseOffYVar").as_double();
        noisePosZVar = this->get_parameter("noisePosZVar").as_double();
        noiseVelZVar = this->get_parameter("noiseVelZVar").as_double();
        noiseOffZVar = this->get_parameter("noiseOffZVar").as_double();
        posGlobalOffsetBiasX = this->get_parameter("posGlobalOffsetBiasX").as_double();
        posGlobalOffsetBiasY = this->get_parameter("posGlobalOffsetBiasY").as_double();
        posGlobalOffsetBiasZ = this->get_parameter("posGlobalOffsetBiasZ").as_double();
        velocityDecayTime = this->get_parameter("velocityDecayTime").as_double();
        offsetDecayTime = this->get_parameter("offsetDecayTime").as_double();

        // Set up parameter callback for dynamic reconfigure equivalent
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&DistributedKF3D::parametersCallback, this, std::placeholders::_1));

        // Create publishers
        std::string pub_topic = this->get_parameter("pub_topic").as_string();
        std::string velPub_topic = this->get_parameter("velPub_topic").as_string();
        std::string offset_topic = this->get_parameter("offset_topic").as_string();
        
        targetPub_ = this->create_publisher<PoseWithCovarianceStamped>(pub_topic, 10);
        targetVelPub_ = this->create_publisher<TwistWithCovarianceStamped>(velPub_topic, 10);
        offsetPub_ = this->create_publisher<PoseWithCovarianceStamped>(offset_topic, 10);

        RCLCPP_INFO_STREAM(this->get_logger(), "Publishing to " << pub_topic);
        RCLCPP_INFO_STREAM(this->get_logger(), "Offset Publishing to " << offset_topic);

        // Cache definition
        int cache_size = this->get_parameter("cache_size").as_int();
        assert(cache_size > 0);
        state_cache_.set_cache_size((std::size_t) cache_size);

        // Initialize static matrices first
        initializeStaticMatrices();
        
        // Initialize the filter
        initializeFilter();
        initializeSubscribers();
        
        RCLCPP_INFO(this->get_logger(), "DistributedKF3D constructor completed - ready for clock sync");
    }

    void DistributedKF3D::initializeSubscribers() {
        // Self and other robots info
        int robotID = this->get_parameter("robotID").as_int();
        int numRobots = this->get_parameter("numRobots").as_int();

        // Pose subscriber
        std::string pose_topic = this->get_parameter("pose_topic").as_string();
        pose_sub_ = this->create_subscription<uav_msgs::msg::UAVPose>(
            pose_topic, 300, 
            [this](uav_msgs::msg::UAVPose::ConstSharedPtr msg) {
                this->predictAndPublish(msg);
            });

        // Measurement subscribers
        string measurement_suffix_self = this->get_parameter("measurement_topic_suffix_self").as_string();
        string measurement_suffix = this->get_parameter("measurement_topic_suffix").as_string();

        selfcallbackhandler = std::make_unique<Callbackhandler>(this, true, robotID);
        self_sub_ = this->create_subscription<PoseWithCovarianceStamped>(
            measurement_suffix_self, 50,
            [this](const PoseWithCovarianceStamped::SharedPtr msg) {
                this->selfcallbackhandler->callback(msg);
            });
        RCLCPP_INFO_STREAM(this->get_logger(), "Registered self measurement subscriber for topic " << measurement_suffix_self);

        for (int robot = 1; robot <= numRobots; robot++) {
            if (robot == robotID)
                continue;

            std::shared_ptr<Callbackhandler> cb(new Callbackhandler(this, false, robot));
            callbackhandlers.emplace_back(cb);
            const auto other_topic = "/machine_" + to_string(robot) + '/' + measurement_suffix;
            
            auto sub = this->create_subscription<PoseWithCovarianceStamped>(
                other_topic, 50,
                [cb](const PoseWithCovarianceStamped::SharedPtr msg) {
                    cb->callback(msg);
                });
            other_subs_.emplace_back(std::move(sub));

            RCLCPP_INFO_STREAM(this->get_logger(),
                    "Registered other robot's measurements subscriber for topic " << other_topic);
        }
    }

    void DistributedKF3D::initializeFilter() {
        // If there is a last element, grab its frame id; if not, use default world_frame
        std::string frame_id{world_frame};
        if (!state_cache_.empty())
            frame_id = state_cache_.back().frame_id;

        // Reset the cache
        state_cache_.clear();

        // Put an initial unknown estimate in the cache
        std_msgs::msg::Header h;
        h.frame_id = frame_id;
        h.stamp = this->get_clock()->now();
        CacheElement first_element(h, state_size, true, 0);
        setUnknownInitial(first_element);
        first_element.frame_id = frame_id;
        state_cache_.insert_ordered(first_element);

        RCLCPP_INFO(this->get_logger(), "The filter was (re)initialized");
    }

    void DistributedKF3D::measurementsCallback(const PoseWithCovarianceStamped::SharedPtr msg, const bool isSelf, const int robot) {
        
        // CRITICAL: Validate timestamp before processing
        if (!ros2_utils::ClockSynchronizer::validateTimestamp(shared_from_this(), rclcpp::Time(msg->header.stamp))) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                 "Invalid timestamp in measurement message from robot %d", robot);
            return;
        }
        
        if (detectBackwardsTimeJump()) {
            RCLCPP_WARN(this->get_logger(), "Backwardstimejump in cache - ignoring update");
            return;
        }

        if (state_cache_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Cache is empty - ignoring update");
            return;
        }

        // Create a new element for the cache
        CacheElement new_element(state_size, *msg, isSelf, robot);

        // Insert this element into cache, which returns the iterator at insert position
        auto it = state_cache_.insert_ordered(new_element);

        // Check if failure to insert - this would be due to a very old message
        if (it == state_cache_.end()) {
            RCLCPP_WARN_STREAM(this->get_logger(),
                    "Trying to insert a measurement that is too old! This is its stamp " << msg->header.stamp.sec << "." << msg->header.stamp.nanosec << std::endl
                    << "Did you forget to reiniatilize the node after going back to the past e.g. stop and restart playback?");
            return;
        }

        // Rare, but may occur
        if(it == state_cache_.begin())
            ++it;

        // In a loop until we go through the whole cache, keep predicting and updating
        for (; it != state_cache_.end(); ++it) {
            if (!predict(*(it - 1), *it)) {
                RCLCPP_WARN(this->get_logger(), "Prediction step failed!");
                return;
            }
            if (it->measurements.size() > 0) {
                if (!update(*it)) {
                    RCLCPP_WARN(this->get_logger(), "Rewind/Update failed!");
                    return;
                }
            }
        }
    }

    void DistributedKF3D::setUnknownInitial(CacheElement &elem) {
        elem.cov << initialUncertaintyPosXY, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, initialUncertaintyPosXY, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, initialUncertaintyPosZ, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, initialUncertaintyVelXY, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, initialUncertaintyVelXY, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, initialUncertaintyVelZ, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, initialUncertaintyOffsetXY, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, initialUncertaintyOffsetXY, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, initialUncertaintyOffsetZ;
    }

    bool DistributedKF3D::predict(const CacheElement &in, CacheElement &out) {
        // Easy access
        const VectorXd &ins = in.state;
        VectorXd &outs = out.state;

        // Time past from one to next
        const double deltaT = (out.stamp - in.stamp).seconds();

        if (deltaT > time_threshold) {
            RCLCPP_WARN_STREAM(this->get_logger(), "It's been a long time since there was an update (" << deltaT
                    << " seconds). Resetting the filter to be safe... position(0,0,0) and high uncertainty");
            initializeFilter();
            return false;
        }

        const static double velocityDecayTo = 0.1;
        const double velocityDecayAlpha = pow(velocityDecayTo, 1.0 / velocityDecayTime);
        const double velocityDecayFactor = pow(velocityDecayAlpha, deltaT);
        const double velocityIntegralFactor = (velocityDecayFactor - 1) / log(velocityDecayAlpha);

        // Decreasing velocity model
        outs(0) = ins(0) + ins(3) * velocityIntegralFactor;
        outs(1) = ins(1) + ins(4) * velocityIntegralFactor;
        outs(2) = ins(2) + ins(5) * velocityIntegralFactor;

        outs(3) = ins(3) * velocityDecayFactor;
        outs(4) = ins(4) * velocityDecayFactor;
        outs(5) = ins(5) * velocityDecayFactor;

        const static double offsetDecayTo = 0.1;
        const double offsetDecayAlpha = pow(offsetDecayTo, 1.0 / offsetDecayTime);
        const double offsetDecayFactor = pow(offsetDecayAlpha, deltaT);

        outs(6) = posGlobalOffsetBiasX + ((ins(6) - posGlobalOffsetBiasX) * offsetDecayFactor);
        outs(7) = posGlobalOffsetBiasY + ((ins(7) - posGlobalOffsetBiasY) * offsetDecayFactor);
        outs(8) = posGlobalOffsetBiasZ + ((ins(8) - posGlobalOffsetBiasZ) * offsetDecayFactor);

        // Construct jacobian G based on deltaT
        MatrixXd G((int) state_size, (int) state_size);
        populateJacobianG(G, deltaT);

        // Update covariance from one to next
        out.cov = MatrixXd((G * in.cov * G.transpose()) + (deltaT / 1.0) * R);

        return true;
    }

    bool DistributedKF3D::update(CacheElement &elem) {
        if (elem.measurements.empty()) {
            RCLCPP_WARN(this->get_logger(), "Tried to perform update step with no measurements in element. Returning without doing anything");
            return false;
        }

        // Find out closest measurement to current state estimate and use that one
        int closest_idx = -1;
        double min_error{DBL_MAX};
        for (size_t i = 0; i < elem.measurements.size(); ++i) {
            const auto measurement = elem.measurements[i];

            double difference[3]{measurement->pose.position.x - elem.state(0),
                measurement->pose.position.y - elem.state(1),
                measurement->pose.position.z - elem.state(2)};

            double sqr_error{
                sqrt(difference[0] * difference[0] + difference[1] * difference[1] + difference[2] * difference[2])};

            if (sqr_error < min_error) {
                min_error = sqr_error;
                closest_idx = i;
            }
        }

        if (closest_idx < 0 || closest_idx > (int) elem.measurements.size()) {
            RCLCPP_ERROR(this->get_logger(), "Something went wrong, couldn't didn't find the closest measurement");
            return false;
        }

        const auto closest_measurement = elem.measurements[closest_idx];
        bool isSelf = elem.isSelfRobot;

        if (isSelf) {
            // check if measurement is a false positive. False positives are allowed to be fused for person estimate, but hazardous for self pose estimate
            // we set isSelf to false if we think we had a false positive

            // the state is the current person estimate
            PoseWithCovariance state;
            state.pose.position.x = elem.state(0);
            state.pose.position.y = elem.state(1);
            state.pose.position.z = elem.state(2);
            state.pose.orientation.w = 1;
            state.pose.orientation.x = 0;
            state.pose.orientation.y = 0;
            state.pose.orientation.z = 0;
            state.covariance = { elem.cov(0*9+0), elem.cov(0*9+1), elem.cov(0*9+2), 0, 0, 0,  elem.cov(1*9+0), elem.cov(1*9+1), elem.cov(1*9+2), 0, 0, 0,  elem.cov(2*9+0), elem.cov(2*9+1), elem.cov(2*9+2), 0, 0, 0 };

            // the measurement distribution is the distribution of the measurement with zero mean
            PoseWithCovariance distribution(*closest_measurement);
            distribution.pose.position.x = 0;
            distribution.pose.position.y = 0;
            distribution.pose.position.z = 0;
            distribution.pose.orientation = state.pose.orientation;

            // state merged with measurement distribution gives the expected mean distribution for new measurements under current state and measurement covariance
            PoseWithCovariance merged;
            pose_cov_ops::compose(state,distribution,merged);

            // calculate normalized expectance density at the mean of the observed measurement
            Eigen::Matrix<double,3,1> statemeanE,measurementmeanE;
            Eigen::Matrix<double,3,3> expectanceE;
            statemeanE << elem.state(0), elem.state(1), elem.state(2);
            measurementmeanE << closest_measurement->pose.position.x, closest_measurement->pose.position.y, closest_measurement->pose.position.z;
            expectanceE << merged.covariance[0*6+0], merged.covariance[0*6+1], merged.covariance[0*6+2], merged.covariance[1*6+0], merged.covariance[1*6+1], merged.covariance[1*6+2], merged.covariance[2*6+0], merged.covariance[2*6+1], merged.covariance[2*6+2];
            mrpt::math::CMatrixDouble31 statemean(statemeanE),measurementmean(measurementmeanE);
            mrpt::math::CMatrixDouble33 expectance(expectanceE);
            double density = mrpt::math::normalPDF(measurementmean, statemean, expectance) / mrpt::math::normalPDF(statemean, statemean, expectance);

            // normalizeed density function with sigma=1 and mu=0:  e^(-1/2 * x^2 )
            // then x = sqrt(-2*log(density))
            double x = sqrt(-2*log(density));

            // ignore measurement for self pose estimation if it is less likely to be a true positive than the prior likelihood for false positives.
            if ( x > falsePositiveThresholdSigma ) {
                RCLCPP_INFO_STREAM(this->get_logger(), "Person Measurement likelihood " << x << " * sigma beyond threshold of " << falsePositiveThresholdSigma << " * sigma. Discarding!");
		        return true;
            }
        }

        const auto &H = isSelf ? Hself : Hother;

        MatrixXd Q((int) measurement_state_size, (int) measurement_state_size);
        Q <<  closest_measurement->covariance[0] , closest_measurement->covariance[1] , closest_measurement->covariance[2], 0.0 , 0.0 , 0.0
            , closest_measurement->covariance[6] , closest_measurement->covariance[7] , closest_measurement->covariance[8], 0.0 , 0.0 , 0.0
            , closest_measurement->covariance[12] , closest_measurement->covariance[13] , closest_measurement->covariance[14], 0.0 , 0.0 , 0.0
            , 0.0 , 0.0 , 0.0, elem.cov(0) + closest_measurement->covariance[0] , elem.cov(1) + closest_measurement->covariance[1] , elem.cov(2) + closest_measurement->covariance[2]
            , 0.0 , 0.0 , 0.0, elem.cov(9) + closest_measurement->covariance[6] , elem.cov(10) + closest_measurement->covariance[7] , elem.cov(11) + closest_measurement->covariance[8]
            , 0.0 , 0.0 , 0.0, elem.cov(18) + closest_measurement->covariance[12] , elem.cov(19) + closest_measurement->covariance[13] , elem.cov(20) + closest_measurement->covariance[14];

        MatrixXd K = elem.cov * H.transpose() * (H * elem.cov * H.transpose() + Q).inverse();

        VectorXd e_measurement((int) measurement_state_size);

        // we aren't really measuring the offset, we can only measure the difference between observed and predicted target, which should be offset corrected already

        double measured_offset_x = elem.state(6) - (closest_measurement->pose.position.x - elem.state(0));
        double measured_offset_y = elem.state(7) - (closest_measurement->pose.position.y - elem.state(1));
        double measured_offset_z = elem.state(8) - (closest_measurement->pose.position.z - elem.state(2));

        e_measurement
            << closest_measurement->pose.position.x, closest_measurement->pose.position.y, closest_measurement->pose.position.z, measured_offset_x, measured_offset_y, measured_offset_z;

        VectorXd e_predicted((int) measurement_state_size);
        e_predicted << elem.state(0), elem.state(1), elem.state(2), elem.state(6), elem.state(7), elem.state(8);

        // Update
        elem.state = elem.state + K * (e_measurement - e_predicted);
        elem.cov = (I - K * H) * elem.cov;

        return true;
    }

    void DistributedKF3D::predictAndPublish(uav_msgs::msg::UAVPose::ConstSharedPtr pose) {
        
        // CRITICAL: Validate timestamp before processing
        if (!ros2_utils::ClockSynchronizer::validateTimestamp(shared_from_this(), rclcpp::Time(pose->header.stamp))) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                 "Invalid timestamp in UAV pose message");
            return;
        }
        
        if (state_cache_.empty())
            return;

        // Always self robot because predict is only called for self poses
        CacheElement tmp_element(pose->header, state_size, true, 0);

        const auto last = state_cache_.back();
        if (!predict(last, tmp_element))
            return;

        publishStateAndCov(tmp_element);
    }

    void DistributedKF3D::initializeStaticMatrices() {
        I << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

        Hself << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

        Hother << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        R << noisePosXVar, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, noisePosYVar, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, noisePosZVar, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, noiseVelXVar, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, noiseVelYVar, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, noiseVelZVar, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, noiseOffXVar, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, noiseOffYVar, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, noiseOffZVar;
    }

    void DistributedKF3D::populateJacobianG(MatrixXd &G, const double deltaT) {
        // offset assumed independent from target detection
        G << 1.0, 0.0, 0.0, deltaT, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 1.0, 0.0, 0.0, deltaT, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 1.0, 0.0, 0.0, deltaT, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    }

    void DistributedKF3D::populateJacobianQ(MatrixXd &Q, const PoseWithCovariance &pcov) {
        Q << pcov.covariance[0], pcov.covariance[1], pcov.covariance[2], 0.0, 0.0, 0.0
            , pcov.covariance[6], pcov.covariance[7], pcov.covariance[8], 0.0, 0.0, 0.0
            , pcov.covariance[12], pcov.covariance[13], pcov.covariance[14], 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, pcov.covariance[0], pcov.covariance[1], pcov.covariance[2]
            , 0.0, 0.0, 0.0, pcov.covariance[6], pcov.covariance[7], pcov.covariance[8]
            , 0.0, 0.0, 0.0, pcov.covariance[12], pcov.covariance[13], pcov.covariance[14];
    }

    void DistributedKF3D::publishStateAndCov(const CacheElement &elem) {
        msg_.header.frame_id = elem.frame_id;
        msg_.header.stamp = elem.stamp;

        msg_.pose.pose.position.x = elem.state[0];
        msg_.pose.pose.position.y = elem.state[1];
        msg_.pose.pose.position.z = elem.state[2];
        msg_.pose.covariance[0 * 6 + 0] = elem.cov(0 * 9 + 0);
        msg_.pose.covariance[0 * 6 + 1] = elem.cov(0 * 9 + 1);
        msg_.pose.covariance[0 * 6 + 2] = elem.cov(0 * 9 + 2);
        msg_.pose.covariance[1 * 6 + 0] = elem.cov(1 * 9 + 0);
        msg_.pose.covariance[1 * 6 + 1] = elem.cov(1 * 9 + 1);
        msg_.pose.covariance[1 * 6 + 2] = elem.cov(1 * 9 + 2);
        msg_.pose.covariance[2 * 6 + 0] = elem.cov(2 * 9 + 0);
        msg_.pose.covariance[2 * 6 + 1] = elem.cov(2 * 9 + 1);
        msg_.pose.covariance[2 * 6 + 2] = elem.cov(2 * 9 + 2);

        msg_.pose.pose.orientation.w = 1.0;

        targetPub_->publish(msg_);

        velMsg_.header.frame_id = elem.frame_id;
        velMsg_.header.stamp = elem.stamp;
        velMsg_.twist.twist.linear.x = elem.state[3];
        velMsg_.twist.twist.linear.y = elem.state[4];
        velMsg_.twist.twist.linear.z = elem.state[5];
        velMsg_.twist.covariance[ 0 * 6 + 0] = elem.cov(0 * 9 + 3);
        velMsg_.twist.covariance[ 0 * 6 + 1] = elem.cov(0 * 9 + 4);
        velMsg_.twist.covariance[ 0 * 6 + 2] = elem.cov(0 * 9 + 5);
        velMsg_.twist.covariance[ 1 * 6 + 0] = elem.cov(1 * 9 + 3);
        velMsg_.twist.covariance[ 1 * 6 + 1] = elem.cov(1 * 9 + 4);
        velMsg_.twist.covariance[ 1 * 6 + 2] = elem.cov(1 * 9 + 5);
        velMsg_.twist.covariance[ 2 * 6 + 0] = elem.cov(2 * 9 + 3);
        velMsg_.twist.covariance[ 2 * 6 + 1] = elem.cov(2 * 9 + 4);
        velMsg_.twist.covariance[ 2 * 6 + 2] = elem.cov(2 * 9 + 5);

        targetVelPub_->publish(velMsg_);

        // Publish offset
        msg_.pose.pose.position.x = elem.state[6];
        msg_.pose.pose.position.y = elem.state[7];
        msg_.pose.pose.position.z = elem.state[8];
        msg_.pose.covariance[0 * 6 + 0] = elem.cov(6 * 9 + 6);
        msg_.pose.covariance[0 * 6 + 1] = elem.cov(6 * 9 + 7);
        msg_.pose.covariance[0 * 6 + 2] = elem.cov(6 * 9 + 8);
        msg_.pose.covariance[1 * 6 + 0] = elem.cov(7 * 9 + 6);
        msg_.pose.covariance[1 * 6 + 1] = elem.cov(7 * 9 + 7);
        msg_.pose.covariance[1 * 6 + 2] = elem.cov(7 * 9 + 8);
        msg_.pose.covariance[2 * 6 + 0] = elem.cov(8 * 9 + 6);
        msg_.pose.covariance[2 * 6 + 1] = elem.cov(8 * 9 + 7);
        msg_.pose.covariance[2 * 6 + 2] = elem.cov(8 * 9 + 8);

        offsetPub_->publish(msg_);
    }

    rcl_interfaces::msg::SetParametersResult DistributedKF3D::parametersCallback(const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        
        bool params_changed = false;
        
        for (const auto & param : parameters) {
            if (param.get_name() == "noisePosXVar") {
                noisePosXVar = param.as_double();
                params_changed = true;
            } else if (param.get_name() == "noiseVelXVar") {
                noiseVelXVar = param.as_double();
                params_changed = true;
            } else if (param.get_name() == "noiseOffXVar") {
                noiseOffXVar = param.as_double();
                params_changed = true;
            } else if (param.get_name() == "noisePosYVar") {
                noisePosYVar = param.as_double();
                params_changed = true;
            } else if (param.get_name() == "noiseVelYVar") {
                noiseVelYVar = param.as_double();
                params_changed = true;
            } else if (param.get_name() == "noiseOffYVar") {
                noiseOffYVar = param.as_double();
                params_changed = true;
            } else if (param.get_name() == "noisePosZVar") {
                noisePosZVar = param.as_double();
                params_changed = true;
            } else if (param.get_name() == "noiseVelZVar") {
                noiseVelZVar = param.as_double();
                params_changed = true;
            } else if (param.get_name() == "noiseOffZVar") {
                noiseOffZVar = param.as_double();
                params_changed = true;
            } else if (param.get_name() == "posGlobalOffsetBiasX") {
                posGlobalOffsetBiasX = param.as_double();
                params_changed = true;
            } else if (param.get_name() == "posGlobalOffsetBiasY") {
                posGlobalOffsetBiasY = param.as_double();
                params_changed = true;
            } else if (param.get_name() == "posGlobalOffsetBiasZ") {
                posGlobalOffsetBiasZ = param.as_double();
                params_changed = true;
            } else if (param.get_name() == "velocityDecayTime") {
                velocityDecayTime = param.as_double();
                params_changed = true;
            } else if (param.get_name() == "offsetDecayTime") {
                offsetDecayTime = param.as_double();
                params_changed = true;
            } else if (param.get_name() == "falsePositiveThresholdSigma") {
                falsePositiveThresholdSigma = param.as_double();
                params_changed = true;
            }
        }
        
        if (params_changed) {
            // Reinitialize matrices
            initializeStaticMatrices();
            RCLCPP_INFO_STREAM(this->get_logger(), "Process noise matrix updated" << std::endl << R);

            // Reinitialize filter
            initializeFilter();
        }
        
        return result;
    }

    bool DistributedKF3D::detectBackwardsTimeJump() {
        // Check if using sim time by checking the use_sim_time parameter
        bool using_sim_time = false;
        try {
            using_sim_time = this->get_parameter("use_sim_time").as_bool();
        } catch (const rclcpp::exceptions::ParameterNotDeclaredException&) {
            // Parameter not set, default to false
            using_sim_time = false;
        }

        if (!using_sim_time)
            return false;

        static auto time = this->get_clock()->now();

        if (this->get_clock()->now() < time) {
            // Jump backwards detected, reset interface
            RCLCPP_WARN(this->get_logger(), "Backwards jump in time detected, performing reset");
            initializeFilter();
            time = this->get_clock()->now();
            return true;
        }
        time = this->get_clock()->now();
        return false;
    }

}