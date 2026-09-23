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

#include "rover_twist_mux/domain/command_freshness_filter.hpp"

#include <algorithm>
#include <limits>

namespace rover_twist_mux::domain
{

namespace
{

// The most traffic time one command can credit towards baseline creep [s]. Commands flow at
// 10 Hz; a longer gap is silence, and silence must not loosen the baseline.
constexpr double kMaxCreditedGapS = 1.0;

}  // namespace

CommandFreshnessFilter::CommandFreshnessFilter(const CommandFreshnessConfig & config)
: config_(config)
{
}

FreshnessVerdict CommandFreshnessFilter::accept(const double stamp_s, const double received_s)
{
    const double gap_s = last_received_s_.has_value()
        ? received_s - *last_received_s_
        : std::numeric_limits<double>::infinity();
    last_received_s_ = received_s;

    if (!(stamp_s > 0.0)) {
        return reject(FreshnessVerdict::Unstamped, 0.0, received_s);
    }

    const double latency_s = received_s - stamp_s;

    if (!baseline_s_.has_value()) {
        return acceptWithBaseline(latency_s, latency_s, received_s);
    }

    // The baseline may have drifted up since it was last set, by at most max_clock_drift per
    // second of traffic. `reference` is where it would be now if the latency justifies it.
    const double credited_s = std::clamp(received_s - baseline_updated_s_, 0.0, kMaxCreditedGapS);
    const double ceiling_s = *baseline_s_ + config_.max_clock_drift * credited_s;
    const double reference_s = std::min(latency_s, ceiling_s);
    const double excess_s = latency_s - reference_s;

    if (excess_s <= config_.max_delay_s) {
        return acceptWithBaseline(latency_s, reference_s, received_s);
    }

    if (!run_.has_value()) {
        // Only a run that follows silence may resync; mid-stream lateness is a backlog.
        if (gap_s >= config_.resync_gap_s) {
            run_ = RejectedRun{received_s, latency_s, latency_s};
        }
        return reject(FreshnessVerdict::Stale, excess_s, received_s);
    }

    run_->min_latency_s = std::min(run_->min_latency_s, latency_s);
    run_->max_latency_s = std::max(run_->max_latency_s, latency_s);

    if (run_->max_latency_s - run_->min_latency_s > config_.max_delay_s) {
        // Latency jumping around is not a clock step. Give up until the next silence.
        run_.reset();
        return reject(FreshnessVerdict::Stale, excess_s, received_s);
    }

    if (received_s - run_->started_s < config_.resync_time_s) {
        return reject(FreshnessVerdict::Stale, excess_s, received_s);
    }

    baseline_s_ = run_->min_latency_s;
    baseline_updated_s_ = received_s;
    run_.reset();

    ++stats_.accepted;
    ++stats_.resyncs;
    stats_.baseline_s = baseline_s_;
    stats_.last_excess_delay_s = 0.0;
    stats_.last_verdict = FreshnessVerdict::Resynced;
    return FreshnessVerdict::Resynced;
}

FreshnessVerdict CommandFreshnessFilter::acceptWithBaseline(
    const double latency_s, const double reference_s, const double received_s)
{
    baseline_s_ = reference_s;
    baseline_updated_s_ = received_s;
    run_.reset();

    ++stats_.accepted;
    stats_.baseline_s = baseline_s_;
    stats_.last_excess_delay_s = std::max(0.0, latency_s - reference_s);
    stats_.last_verdict = FreshnessVerdict::Fresh;
    return FreshnessVerdict::Fresh;
}

FreshnessVerdict CommandFreshnessFilter::reject(
    const FreshnessVerdict verdict, const double excess_s, const double received_s)
{
    ++stats_.rejected;
    stats_.last_excess_delay_s = excess_s;
    stats_.last_verdict = verdict;
    stats_.last_rejected_at_s = received_s;
    return verdict;
}

}  // namespace rover_twist_mux::domain
