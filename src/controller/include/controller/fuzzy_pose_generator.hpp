#pragma once
#include <chrono>
#include <random>
#include <cmath>
#include "moveit/task_constructor/stages.h"
#include "moveit/task_constructor/storage.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace mtc = moveit::task_constructor;

class FuzzyPoseGenerator : public mtc::stages::GeneratePose{
public:
    FuzzyPoseGenerator(const std::string& name) : mtc::stages::GeneratePose(name), pos_tol_(0.1), ori_tol_(0.1), sample_count_(10) {}

    void setTolerance(double position_tolerance, double orientation_tolerance){
        pos_tol_ = position_tolerance;
        ori_tol_ = orientation_tolerance;
    }

    void setSampleCount(int count){
        sample_count_ = count;
    }

    void compute() override{
        if (!properties().hasProperty("pose")){
            throw std::runtime_error("Pose property not set for FuzzyPoseGenerator");
        }

        const geometry_msgs::msg::PoseStamped& pose = properties().get<geometry_msgs::msg::PoseStamped>("pose");
        std::normal_distribution<double> pos_dist(0.0, pos_tol_);
        std::normal_distribution<double> ori_dist(0.0, ori_tol_);

        // Process each upstream solution
        for (const auto* upstream_solution : upstream_solutions_) {
            // Get the planning scene from the upstream solution
            const planning_scene::PlanningSceneConstPtr& scene = upstream_solution->end()->scene();
            
            // Generate multiple fuzzy pose samples
            for (int i = 0; i < sample_count_; ++i){
                geometry_msgs::msg::PoseStamped fuzzy_pose = pose;

                // Add position noise
                fuzzy_pose.pose.position.x += pos_dist(generator_);
                fuzzy_pose.pose.position.y += pos_dist(generator_);
                fuzzy_pose.pose.position.z += pos_dist(generator_);

                // Add orientation noise
                double d_roll = ori_dist(generator_);
                double d_pitch = ori_dist(generator_);
                double d_yaw = ori_dist(generator_);

                tf2::Quaternion q_orig, q_delta, q_new;
                tf2::fromMsg(fuzzy_pose.pose.orientation, q_orig);
                q_delta.setRPY(d_roll, d_pitch, d_yaw);
                q_new = q_delta * q_orig;
                q_new.normalize();
                fuzzy_pose.pose.orientation = tf2::toMsg(q_new);

                // Create an InterfaceState with the scene from upstream
                mtc::InterfaceState state(scene);
                state.properties().set("target_pose", fuzzy_pose);

                // Calculate cost based on deviation from original pose
                double pos_deviation = std::sqrt(
                    std::pow(fuzzy_pose.pose.position.x - pose.pose.position.x, 2) +
                    std::pow(fuzzy_pose.pose.position.y - pose.pose.position.y, 2) +
                    std::pow(fuzzy_pose.pose.position.z - pose.pose.position.z, 2)
                );
                double ori_deviation = std::abs(d_roll) + std::abs(d_pitch) + std::abs(d_yaw);
                double cost = pos_deviation + ori_deviation;
                        
                // Spawn the state with calculated cost
                spawn(std::move(state), cost);
            }
        }
    }
    
protected:
    void onNewSolution(const mtc::SolutionBase& s) override {
        // Call parent class implementation to handle upstream solution tracking
        mtc::stages::GeneratePose::onNewSolution(s);
    }

private:
    double pos_tol_;
    double ori_tol_;
    int sample_count_;

    std::default_random_engine generator_;
};