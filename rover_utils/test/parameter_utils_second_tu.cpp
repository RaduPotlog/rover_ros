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

// A second translation unit that includes parameter_utils.hpp, linked into the same test as
// test_parameter_utils.cpp: if a helper in the header loses `inline`, the test fails to link
// with a multiple-definition error.

#include "rover_utils/parameter_utils.hpp"

rcl_interfaces::msg::ParameterDescriptor describeFromSecondTranslationUnit()
{
    return rover_utils::ros::describePositive("second", 2.0);
}
