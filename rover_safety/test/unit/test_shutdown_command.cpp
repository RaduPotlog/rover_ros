// Copyright 2025 Mechatronics Academy
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


#include <gtest/gtest.h>

#include <array>
#include <cstdio>
#include <memory>
#include <string>

#include "rover_safety/infrastructure/shutdown_command.hpp"

using rover_safety::infrastructure::buildShutdownCommand;
using rover_safety::infrastructure::shellQuote;

namespace
{

/** Runs a bash command and returns its stdout. */
std::string runBash(const std::string & command)
{
    const std::string wrapped = "bash -c " + shellQuote(command);
    std::unique_ptr<FILE, int (*)(FILE *)> pipe(popen(wrapped.c_str(), "r"), pclose);
    std::string output;
    std::array<char, 256> buffer{};
    while (pipe && fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        output += buffer.data();
    }
    return output;
}

}  // namespace

TEST(ShutdownCommand, QuotesPlainAndSpecialCharacters)
{
    EXPECT_EQ(shellQuote("abc"), "'abc'");
    EXPECT_EQ(shellQuote(""), "''");
    EXPECT_EQ(shellQuote("it's"), "'it'\\''s'");
}

TEST(ShutdownCommand, ReasonReachesTheCommandVerbatim)
{
    const std::string reason = "Battery at 61.5 C; $(echo injected) `echo injected` \"quoted\" it's";
    const auto command = buildShutdownCommand("printf '%s' \"$ROVER_SHUTDOWN_REASON\"", reason);

    EXPECT_EQ(runBash(command), reason);
}
