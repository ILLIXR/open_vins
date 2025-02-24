#pragma once

#include "core/VioManager.h"
#include "state/State.h"

#include "illixr/data_format/imu.hpp"
#include "illixr/data_format/opencv_data_types.hpp"
#include "illixr/data_format/pose.hpp"
#include "illixr/phonebook.hpp"
#include "illixr/plugin.hpp"
#include "illixr/relative_clock.hpp"
#include "illixr/switchboard.hpp"

namespace ILLIXR {
// Comment in if using ZED instead of offline_imu_cam
// TODO: Pull from config YAML file

duration from_seconds(double seconds) { return duration{long(seconds * 1e9L)}; }

class slam2 : public plugin {
public:
    /* Provide handles to slam2 */
    [[maybe_unused]] slam2(const std::string &name_, phonebook *pb_);

    void start() override;

    void feed_imu_cam(const switchboard::ptr<const data_format::imu_type> &datum,
                      [[maybe_unused]] std::size_t iteration_no);

    ~slam2() override = default;

private:
    const std::shared_ptr<switchboard> switchboard_;
    std::shared_ptr<relative_clock> clock_;
    switchboard::writer<data_format::pose_type> pose_;
    switchboard::writer<data_format::imu_integrator_input> imu_integrator_input_;
    ov_msckf::State *state_{};

    switchboard::ptr<const data_format::binocular_cam_type> cam_buffer_;
    switchboard::buffered_reader<data_format::binocular_cam_type> cam_;

    ov_msckf::VioManagerOptions manager_params_;
    ov_msckf::VioManager open_vins_estimator_;
};

} // namespace ILLIXR
