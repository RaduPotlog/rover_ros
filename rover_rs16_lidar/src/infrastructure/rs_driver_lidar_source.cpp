// Copyright 2026 Mechatronics Academy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rover_rs16_lidar/infrastructure/rs_driver_lidar_source.hpp"

#include <atomic>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "rs_driver/api/lidar_driver.hpp"
#include "rs_driver/msg/point_cloud_msg.hpp"
#include "rs_driver/utility/sync_queue.hpp"

namespace rover_rs16_lidar::infrastructure
{

namespace
{

using RsPointCloud = PointCloudT<PointXYZI>;
using RsPointCloudPtr = std::shared_ptr<RsPointCloud>;

/// How long the worker waits for a revolution before re-checking the exit flag.
constexpr unsigned int kQueueWaitUs = 100000;  // 100 ms

}  // namespace

struct RsDriverLidarSource::Impl
{
    explicit Impl(domain::SensorSettings settings) : settings(std::move(settings)) {}

    domain::SensorSettings settings;

    FrameCallback on_frame;
    ErrorCallback on_error;

    robosense::lidar::LidarDriver<RsPointCloud> driver;
    // Buffers cycle between the two: filled ones come back on `filled`, are converted, then
    // returned to `free` for the decoder to reuse.
    robosense::lidar::SyncQueue<RsPointCloudPtr> free_clouds;
    robosense::lidar::SyncQueue<RsPointCloudPtr> filled_clouds;

    std::thread worker;
    std::atomic<bool> stopping{false};
    bool started{false};

    /// Called by rs_driver when it needs somewhere to decode the next revolution into.
    RsPointCloudPtr acquireCloud()
    {
        auto cloud = free_clouds.pop();
        if (cloud) {
            return cloud;
        }
        return std::make_shared<RsPointCloud>();
    }

    /// Called by rs_driver once a revolution is complete. Must not block the decoder.
    void submitCloud(RsPointCloudPtr cloud) { filled_clouds.push(std::move(cloud)); }

    void reportError(const robosense::lidar::Error & error)
    {
        if (on_error) {
            on_error(error.toString());
        }
    }

    void run()
    {
        domain::PointCloudFrame frame;

        while (!stopping.load(std::memory_order_relaxed)) {
            auto cloud = filled_clouds.popWait(kQueueWaitUs);
            if (!cloud) {
                continue;
            }

            convert(*cloud, frame);

            if (on_frame) {
                on_frame(frame);
            }

            cloud->points.clear();
            free_clouds.push(std::move(cloud));
        }
    }

    /// `frame` is reused across revolutions so its point buffer keeps its capacity.
    static void convert(const RsPointCloud & cloud, domain::PointCloudFrame & frame)
    {
        frame.width = cloud.width;
        frame.height = cloud.height;
        frame.timestamp_s = cloud.timestamp;
        frame.is_dense = cloud.is_dense;
        frame.seq = cloud.seq;

        frame.points.resize(cloud.points.size());
        for (std::size_t i = 0; i < cloud.points.size(); ++i) {
            const auto & in = cloud.points[i];
            auto & out = frame.points[i];
            out.x = in.x;
            out.y = in.y;
            out.z = in.z;
            // rs_driver reports intensity as a uint8; the PointCloud2 field is a float32.
            out.intensity = static_cast<float>(in.intensity);
        }
    }
};

RsDriverLidarSource::RsDriverLidarSource(domain::SensorSettings settings)
: impl_(std::make_unique<Impl>(std::move(settings)))
{
}

RsDriverLidarSource::~RsDriverLidarSource()
{
    stop();
}

void RsDriverLidarSource::setFrameCallback(FrameCallback callback)
{
    impl_->on_frame = std::move(callback);
}

void RsDriverLidarSource::setErrorCallback(ErrorCallback callback)
{
    impl_->on_error = std::move(callback);
}

void RsDriverLidarSource::start()
{
    if (impl_->started) {
        return;
    }

    const auto & settings = impl_->settings;

    robosense::lidar::RSDriverParam param;
    param.lidar_type = robosense::lidar::LidarType::RS16;
    param.input_type = settings.input_type == domain::LidarInputType::Pcap
                           ? robosense::lidar::InputType::PCAP_FILE
                           : robosense::lidar::InputType::ONLINE_LIDAR;
    // The frame id is stamped by the ROS adapter, not here.
    param.input_param.msop_port = settings.msop_port;
    param.input_param.difop_port = settings.difop_port;
    param.input_param.host_address = settings.host_address;
    param.input_param.group_address = settings.group_address;
    param.input_param.pcap_path = settings.pcap_path;
    param.input_param.pcap_repeat = settings.pcap_repeat;
    param.input_param.pcap_rate = settings.pcap_rate;

    param.decoder_param.min_distance = settings.min_distance;
    param.decoder_param.max_distance = settings.max_distance;
    param.decoder_param.use_lidar_clock = settings.use_lidar_clock;
    param.decoder_param.dense_points = settings.dense_points;
    param.decoder_param.ts_first_point = settings.ts_first_point;
    param.decoder_param.wait_for_difop = settings.wait_for_difop;
    param.decoder_param.start_angle = settings.start_angle;
    param.decoder_param.end_angle = settings.end_angle;

    impl_->driver.regPointCloudCallback(
        [impl = impl_.get()] { return impl->acquireCloud(); },
        [impl = impl_.get()](RsPointCloudPtr cloud) { impl->submitCloud(std::move(cloud)); });
    impl_->driver.regExceptionCallback(
        [impl = impl_.get()](const robosense::lidar::Error & error) { impl->reportError(error); });

    if (!impl_->driver.init(param)) {
        throw std::runtime_error(
            "rs_driver failed to initialise. Check that UDP ports " +
            std::to_string(settings.msop_port) + "/" + std::to_string(settings.difop_port) +
            " are free and, for pcap input, that the file exists.");
    }

    impl_->stopping.store(false, std::memory_order_relaxed);
    impl_->worker = std::thread([impl = impl_.get()] { impl->run(); });

    impl_->driver.start();
    impl_->started = true;
}

void RsDriverLidarSource::stop()
{
    if (!impl_ || !impl_->started) {
        return;
    }

    // Stop producing first, then drain, so the worker cannot outlive the publishers it calls.
    impl_->driver.stop();
    impl_->stopping.store(true, std::memory_order_relaxed);
    if (impl_->worker.joinable()) {
        impl_->worker.join();
    }
    impl_->started = false;
}

}  // namespace rover_rs16_lidar::infrastructure
