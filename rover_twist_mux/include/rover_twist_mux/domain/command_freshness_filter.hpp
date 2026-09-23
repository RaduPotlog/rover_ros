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

#ifndef ROVER_TWIST_MUX_DOMAIN_COMMAND_FRESHNESS_FILTER_HPP_
#define ROVER_TWIST_MUX_DOMAIN_COMMAND_FRESHNESS_FILTER_HPP_

#include <cstdint>
#include <optional>

namespace rover_twist_mux::domain
{

struct CommandFreshnessConfig
{
    /// How much later than the best recent delivery a command may arrive and still be driven on [s].
    double max_delay_s{0.3};

    /// How fast the baseline may rise, [s of latency per s of traffic]. Covers clock drift
    /// between sender and rover (NTP-synced clocks: well under 1e-4). Anything growing faster -
    /// a backlog building up in a TCP buffer - eventually exceeds max_delay_s and is dropped.
    double max_clock_drift{0.001};

    /// A run of rejected commands re-baselines the filter after this long [s], if it started
    /// after a silence of at least resync_gap_s and its latency stayed within max_delay_s of
    /// itself. Recovers from the sender's clock stepping backwards.
    double resync_time_s{2.0};

    /// Silence [s] that must precede a run of rejected commands for it to be allowed to resync.
    double resync_gap_s{1.0};
};

enum class FreshnessVerdict
{
    Fresh,      ///< Arrived within max_delay_s of the baseline: pass it on.
    Resynced,   ///< Re-baselined on this command (sender clock step): pass it on.
    Stale,      ///< Arrived too late - it sat in a buffer: drop it.
    Unstamped,  ///< No sender timestamp, so its age cannot be judged: drop it.
};

inline bool isAccepted(FreshnessVerdict verdict)
{
    return verdict == FreshnessVerdict::Fresh || verdict == FreshnessVerdict::Resynced;
}

struct FreshnessStats
{
    std::uint64_t accepted{0};
    std::uint64_t rejected{0};
    std::uint64_t resyncs{0};

    /// Smallest recent (receive time - stamp): the sender's clock offset plus the best-case
    /// transport delay. nullopt until the first stamped command.
    std::optional<double> baseline_s;

    /// How far the last command arrived beyond the baseline [s]; 0 when it was fresh.
    double last_excess_delay_s{0.0};

    std::optional<FreshnessVerdict> last_verdict;

    /// Receive time of the last dropped command, for "recently dropping" diagnostics.
    std::optional<double> last_rejected_at_s;
};

/**
 * @brief Drops velocity commands that arrive late, without trusting the sender's clock.
 * @details A browser stamps its commands with its own clock, which can be seconds away from the
 *          rover's, so an absolute age check (now - stamp) is meaningless. Instead the filter
 *          tracks the baseline: the smallest recent latency (receive time - stamp), which is the
 *          clock offset plus the best-case transport delay. A command whose latency exceeds the
 *          baseline by more than max_delay_s sat in a buffer on the way - typically the websocket
 *          (TCP) holding commands through a Wi-Fi stall and then delivering them in a burst - and
 *          is dropped instead of replaying old motion.
 *
 *          - The first stamped command sets the baseline and is accepted.
 *          - A lower latency lowers the baseline at once. The baseline rises only while commands
 *            are accepted, and by at most max_clock_drift per second of traffic, so silence never
 *            loosens it: a burst after a long stall is still judged against the pre-stall
 *            baseline.
 *          - Rejected commands never move the baseline, so a stale burst cannot re-baseline
 *            itself; its tail, sent just before delivery resumed, passes on its own merits.
 *          - A sender clock stepping backwards makes every command look late. Resync: rejected
 *            commands that follow a silence of resync_gap_s and stay within max_delay_s of each
 *            other for resync_time_s re-baseline the filter. A stale burst arrives within
 *            milliseconds, so it never spans resync_time_s; a backlog growing while commands
 *            flow has no preceding silence. Neither can resync.
 *
 *          Times are seconds on any fixed epoch per side: `stamp_s` on the sender's clock,
 *          `received_s` on a local wall clock (not sim time - it must advance with the sender's).
 */
class CommandFreshnessFilter
{
public:
    explicit CommandFreshnessFilter(const CommandFreshnessConfig & config);

    FreshnessVerdict accept(double stamp_s, double received_s);

    const FreshnessStats & stats() const { return stats_; }

    const CommandFreshnessConfig & config() const { return config_; }

private:
    struct RejectedRun
    {
        double started_s;
        double min_latency_s;
        double max_latency_s;
    };

    FreshnessVerdict acceptWithBaseline(double latency_s, double reference_s, double received_s);

    FreshnessVerdict reject(FreshnessVerdict verdict, double excess_s, double received_s);

    CommandFreshnessConfig config_;

    std::optional<double> baseline_s_;
    double baseline_updated_s_{0.0};

    std::optional<double> last_received_s_;

    // Only a run that began after a silence of resync_gap_s; see the class comment.
    std::optional<RejectedRun> run_;

    FreshnessStats stats_;
};

}  // namespace rover_twist_mux::domain

#endif  // ROVER_TWIST_MUX_DOMAIN_COMMAND_FRESHNESS_FILTER_HPP_
