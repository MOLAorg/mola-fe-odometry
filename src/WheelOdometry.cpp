/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   WheelOdometry.cpp
 * @brief  Simple SLAM FrontEnd for wheel odometry: creates SE(3) edges from
 * @author Jose Luis Blanco Claraco
 * @date   Jun 13, 2019
 */

/** \defgroup mola_fe_odometry_grp mola-fe-odometry.
 * Simple SLAM FrontEnd for wheel odometry: creates SE(3) edges from odometry
 * readings.
 *
 */

#include <mola-fe-odometry/WheelOdometry.h>
#include <mola-kernel/yaml_helpers.h>
#include <mrpt/core/initializer.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/containers/yaml.h>

using namespace mola;

// arguments: class_name, parent_class, class namespace
IMPLEMENTS_MRPT_OBJECT(WheelOdometry, FrontEndBase, mola)

MRPT_INITIALIZER(do_register_WheelOdometry)
{
    // Register MOLA modules:
    MOLA_REGISTER_MODULE(WheelOdometry);

    // Register serializable classes:
    // (None)
}

WheelOdometry::WheelOdometry() = default;

void WheelOdometry::initialize(const std::string& cfg_block)
{
    MRPT_TRY_START

    // Load params:
    auto c   = mrpt::containers::yaml::FromText(cfg_block);
    auto cfg = c["params"];
    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << cfg);

    YAML_LOAD_REQ(params_, min_dist_xyz_between_keyframes, double);
    YAML_LOAD_OPT_DEG(params_, min_rotation_between_keyframes, double);

    YAML_LOAD_OPT(params_.noise_model_planar, a1, double);
    YAML_LOAD_OPT(params_.noise_model_planar, a2, double);
    YAML_LOAD_OPT(params_.noise_model_planar, a3, double);
    YAML_LOAD_OPT(params_.noise_model_planar, a4, double);
    YAML_LOAD_OPT(params_.noise_model_planar, minStdXY, double);
    YAML_LOAD_OPT_DEG(params_.noise_model_planar, minStdPHI, double);

    MRPT_TRY_END
}
void WheelOdometry::spinOnce()
{
    MRPT_TRY_START

    ProfilerEntry tleg(profiler_, "spinOnce");

    //
    MRPT_TRY_END
}

void WheelOdometry::reset() { state_ = MethodState(); }

void WheelOdometry::onNewObservation(CObservation::Ptr& o)
{
    MRPT_TRY_START
    ProfilerEntry tleg(profiler_, "onNewObservation");

    // Only process "my" sensor source:
    ASSERT_(o);
    if (o->sensorLabel != raw_sensor_label_) return;

    // Enqueue task:
    worker_pool_.enqueue(&WheelOdometry::doProcessNewObservation, this, o);

    MRPT_TRY_END
}

// here happens the main stuff:
void WheelOdometry::doProcessNewObservation(CObservation::Ptr& o)
{
    // All methods that are enqueued into a thread pool should have its own
    // top-level try-catch:
    try
    {
        ProfilerEntry tleg(profiler_, "doProcessNewObservation");

        ASSERT_(o);

        mrpt::obs::CObservationOdometry::Ptr obs;
        obs = std::dynamic_pointer_cast<mrpt::obs::CObservationOdometry>(o);

        if (!obs)
        {
            MRPT_LOG_WARN_STREAM(
                "Observation of type `"
                << o->GetRuntimeClass()->className
                << "` could not be converted into CObservationOdometry. Doing "
                   "nothing.");
            return;
        }

        // Compute odometry increment:
        const auto cur_abs_odo    = obs->odometry;
        const auto last_abs_odo   = state_.last_absolute_odom;
        state_.last_absolute_odom = cur_abs_odo;

        // First odometry?
        if (!state_.last_absolute_odom_valid)
        {
            state_.last_absolute_odom_valid = true;
            return;
        }

        // Accumulate:
        const auto odo_incr = mrpt::poses::CPose3D(cur_abs_odo - last_abs_odo);
        state_.accum_since_last_kf = state_.accum_since_last_kf + odo_incr;
        MRPT_LOG_DEBUG_STREAM(
            "cur_abs_odo: " << cur_abs_odo.asString()
                            << " last_abs_odo: " << last_abs_odo.asString()
                            << " accum_since_last_kf: "
                            << state_.accum_since_last_kf.asString());

        // Create a new KF if the distance since the last one is large
        // enough:
        const double dist_eucl_since_last = state_.accum_since_last_kf.norm();
        const double rot_since_last =
            mrpt::poses::Lie::SE<3>::log(state_.accum_since_last_kf)
                .blockCopy<3, 1>(3, 0)
                .norm();

        MRPT_LOG_DEBUG_FMT(
            "Since last KF: dist=%5.03f m rotation=%.01f deg",
            dist_eucl_since_last, mrpt::RAD2DEG(rot_since_last));

        if (dist_eucl_since_last > params_.min_dist_xyz_between_keyframes ||
            rot_since_last > params_.min_rotation_between_keyframes)
        {
            // Yes: create new KF
            // 1) New KeyFrame
            BackEndBase::ProposeKF_Input kf;

            kf.timestamp = o->timestamp;
            {
                mrpt::obs::CSensoryFrame& sf = kf.observations.emplace();
                sf.push_back(o);
            }

            profiler_.enter("doProcessNewObservation.addKeyFrame");

            std::future<BackEndBase::ProposeKF_Output> kf_out_fut;
            kf_out_fut = slam_backend_->addKeyFrame(kf);

            // Wait until it's executed:
            auto kf_out = kf_out_fut.get();

            ASSERT_(kf_out.success);
            ASSERT_(kf_out.new_kf_id);

            const mola::id_t new_kf_id = kf_out.new_kf_id.value();
            ASSERT_(new_kf_id != mola::INVALID_ID);

            profiler_.leave("doProcessNewObservation.addKeyFrame");

            MRPT_LOG_INFO_STREAM("New KF: ID=" << new_kf_id);

            // 2) New SE(3) constraint between consecutive Keyframes:
            if (state_.last_kf != mola::INVALID_ID)
            {
                std::future<BackEndBase::AddFactor_Output> factor_out_fut;
                // Important: The "constant velocity model" factor is
                // automatically added by the SLAM module (if applicable). Here,
                // all we need to tell it is the SE(3) constraint, and the
                // KeyFrame timestamp:
                mola::FactorRelativePose3 fPose3(
                    state_.last_kf, new_kf_id,
                    state_.accum_since_last_kf.asTPose());

                MRPT_TODO("Use params_ for custom uncertainty covariance");
                fPose3.noise_model_diag_xyz_ = 0.10;
                fPose3.noise_model_diag_rot_ = mrpt::DEG2RAD(1.0);

                mola::Factor f = std::move(fPose3);
                factor_out_fut = slam_backend_->addFactor(f);

                // Wait until it's executed:
                auto factor_out = factor_out_fut.get();
                ASSERT_(factor_out.success);
                ASSERT_(
                    factor_out.new_factor_id &&
                    factor_out.new_factor_id != mola::INVALID_FID);

                MRPT_LOG_DEBUG_STREAM(
                    "New FactorRelativePose3: #"
                    << state_.last_kf << " <=> #" << new_kf_id
                    << ". rel_pose=" << state_.accum_since_last_kf.asString());
            }

            // Reset accumulators:
            state_.accum_since_last_kf = mrpt::poses::CPose3D();
            state_.last_kf             = new_kf_id;
        }  // end done add a new KF

        // In any case, publish to the SLAM BackEnd what's our **current**
        // vehicle pose, no matter if it's a keyframe or not:
        {
            ProfilerEntry tle(
                profiler_,
                "doProcessNewObservation.advertiseUpdatedLocalization");

            BackEndBase::AdvertiseUpdatedLocalization_Input new_loc;
            new_loc.timestamp    = o->timestamp;
            new_loc.reference_kf = state_.last_kf;
            new_loc.pose         = state_.accum_since_last_kf.asTPose();

            std::future<void> adv_pose_fut =
                slam_backend_->advertiseUpdatedLocalization(new_loc);
        }
    }
    catch (const std::exception& e)
    {
        MRPT_LOG_ERROR_STREAM("Exception:\n" << mrpt::exception_to_str(e));
    }
}
