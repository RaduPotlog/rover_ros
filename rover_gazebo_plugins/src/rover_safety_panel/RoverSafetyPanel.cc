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

#include "RoverSafetyPanel.hh"

#include <QDateTime>
#include <QMetaObject>

#include <gz/common/Console.hh>
#include <gz/msgs/empty.pb.h>
#include <gz/plugin/Register.hh>

#include <functional>

namespace rover_gazebo_plugins
{

namespace
{
constexpr int kHwButtonRepublishMs = 1000;
constexpr int kStaleCheckMs = 500;
// The PLC node publishes at 5 Hz; a few missed periods means it is gone.
constexpr qint64 kStaleAfterMs = 2000;
}  // namespace

RoverSafetyPanel::RoverSafetyPanel()
{
  connect(&hw_button_timer_, &QTimer::timeout, this, &RoverSafetyPanel::PublishHwButton);
  connect(&stale_timer_, &QTimer::timeout, this, &RoverSafetyPanel::CheckStale);
}

RoverSafetyPanel::~RoverSafetyPanel() = default;

void RoverSafetyPanel::LoadConfig(const tinyxml2::XMLElement * plugin_elem)
{
  if (this->title.empty()) {
    this->title = "Rover Safety";
  }

  if (plugin_elem) {
    if (auto * ns_elem = plugin_elem->FirstChildElement("namespace"); ns_elem && ns_elem->GetText()) {
      namespace_ = ns_elem->GetText();
    }
  }
  // Normalise to "/ns" (or "" for no namespace) so topics come out as "/ns/sim_safety/...".
  while (!namespace_.empty() && namespace_.back() == '/') {
    namespace_.pop_back();
  }
  if (!namespace_.empty() && namespace_.front() != '/') {
    namespace_.insert(0, "/");
  }

  hw_button_pub_ = node_.Advertise<gz::msgs::Boolean>(Topic("sim_safety/hw_e_stop_button"));
  sw_set_pub_ = node_.Advertise<gz::msgs::Empty>(Topic("sim_safety/sw_e_stop_set"));
  sw_reset_pub_ = node_.Advertise<gz::msgs::Empty>(Topic("sim_safety/sw_e_stop_reset"));
  latch_reset_pub_ = node_.Advertise<gz::msgs::Empty>(Topic("sim_safety/latch_reset"));

  const auto subscribe_bool = [this](const std::string & name, bool * field) {
      std::function<void(const gz::msgs::Boolean &)> callback =
        [this, field](const gz::msgs::Boolean & msg) {OnBool(field, msg);};
      if (!node_.Subscribe(Topic(name), callback)) {
        gzerr << "RoverSafetyPanel: failed to subscribe to " << Topic(name) << std::endl;
      }
    };
  subscribe_bool("sim_safety/sw_e_stop", &sw_e_stop_);
  subscribe_bool("sim_safety/latch_active", &latch_active_);
  subscribe_bool("sim_safety/contactor_engaged", &contactor_engaged_);
  subscribe_bool("motion_lock", &motion_locked_);

  std::function<void(const gz::msgs::StringMsg &)> result_callback =
    [this](const gz::msgs::StringMsg & msg) {OnResult(msg);};
  node_.Subscribe(Topic("sim_safety/result"), result_callback);

  hw_button_timer_.start(kHwButtonRepublishMs);
  stale_timer_.start(kStaleCheckMs);

  gzmsg << "RoverSafetyPanel: using " << Topic("sim_safety/") << "*" << std::endl;
}

std::string RoverSafetyPanel::Topic(const std::string & name) const
{
  return namespace_ + "/" + name;
}

void RoverSafetyPanel::ToggleHwEStop()
{
  hw_pressed_ = !hw_pressed_;
  emit HwPressedChanged();
  PublishHwButton();
}

void RoverSafetyPanel::SwEStopSet() {PublishEmpty(sw_set_pub_);}

void RoverSafetyPanel::SwEStopReset() {PublishEmpty(sw_reset_pub_);}

void RoverSafetyPanel::LatchReset() {PublishEmpty(latch_reset_pub_);}

void RoverSafetyPanel::PublishHwButton()
{
  gz::msgs::Boolean msg;
  msg.set_data(hw_pressed_);
  hw_button_pub_.Publish(msg);
}

void RoverSafetyPanel::PublishEmpty(gz::transport::Node::Publisher & publisher)
{
  if (!publisher.Publish(gz::msgs::Empty())) {
    result_ = "failed to publish - is the gz bridge running?";
    emit ResultChanged();
  }
}

void RoverSafetyPanel::OnBool(bool * field, const gz::msgs::Boolean & msg)
{
  const bool value = msg.data();
  const bool from_plc = field != &motion_locked_;
  QMetaObject::invokeMethod(
    this, [this, field, value, from_plc]() {
      *field = value;
      if (from_plc) {
        MarkFresh();
      }
      emit StateChanged();
    }, Qt::QueuedConnection);
}

void RoverSafetyPanel::OnResult(const gz::msgs::StringMsg & msg)
{
  const QString text = QString::fromStdString(msg.data());
  QMetaObject::invokeMethod(
    this, [this, text]() {
      result_ = text;
      emit ResultChanged();
    }, Qt::QueuedConnection);
}

void RoverSafetyPanel::MarkFresh()
{
  last_state_ms_ = QDateTime::currentMSecsSinceEpoch();
  state_fresh_ = true;
}

void RoverSafetyPanel::CheckStale()
{
  if (state_fresh_ && QDateTime::currentMSecsSinceEpoch() - last_state_ms_ > kStaleAfterMs) {
    state_fresh_ = false;
    emit StateChanged();
  }
}

}  // namespace rover_gazebo_plugins

GZ_ADD_PLUGIN(rover_gazebo_plugins::RoverSafetyPanel, gz::gui::Plugin)
