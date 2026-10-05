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

#ifndef ROVER_GAZEBO_PLUGINS__ROVER_SAFETY_PANEL_HH_
#define ROVER_GAZEBO_PLUGINS__ROVER_SAFETY_PANEL_HH_

#include <QString>
#include <QTimer>

#include <gz/gui/Plugin.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>

#include <string>

namespace rover_gazebo_plugins
{

// Gazebo GUI panel for the simulated safety chain (rover_gazebo/scripts/sim_safety_plc.py).
//
// Speaks gz-transport only; ros_gz_bridge carries the topics to and from ROS (see
// rover_gazebo/config/gz_bridge.yaml). Commands, all under /<namespace>/sim_safety/:
//   hw_e_stop_button  gz.msgs.Boolean  maintained HW E-Stop button, on change and every second;
//                                      releasing it also resets the latch
//   sw_e_stop_set     gz.msgs.Empty    SW user E-Stop
//   sw_e_stop_reset   gz.msgs.Empty    release the SW user E-Stop (refused while wheels move)
//   latch_reset       gz.msgs.Empty    reset the E-Stop latch (no effect while a stop is asserted)
// State: sim_safety/{sw_e_stop,latch_active,contactor_engaged} and /<namespace>/motion_lock
// (gz.msgs.Boolean), sim_safety/result (gz.msgs.StringMsg).
class RoverSafetyPanel : public gz::gui::Plugin
{
  Q_OBJECT

  Q_PROPERTY(bool hwPressed READ HwPressed NOTIFY HwPressedChanged)
  Q_PROPERTY(bool swEStop READ SwEStop NOTIFY StateChanged)
  Q_PROPERTY(bool latchActive READ LatchActive NOTIFY StateChanged)
  Q_PROPERTY(bool contactorEngaged READ ContactorEngaged NOTIFY StateChanged)
  Q_PROPERTY(bool motionLocked READ MotionLocked NOTIFY StateChanged)
  Q_PROPERTY(bool stateFresh READ StateFresh NOTIFY StateChanged)
  Q_PROPERTY(QString result READ Result NOTIFY ResultChanged)
  Q_PROPERTY(QString robotNamespace READ RobotNamespace CONSTANT)

public:
  RoverSafetyPanel();
  ~RoverSafetyPanel() override;

  void LoadConfig(const tinyxml2::XMLElement * plugin_elem) override;

  bool HwPressed() const {return hw_pressed_;}
  bool SwEStop() const {return sw_e_stop_;}
  bool LatchActive() const {return latch_active_;}
  bool ContactorEngaged() const {return contactor_engaged_;}
  bool MotionLocked() const {return motion_locked_;}
  bool StateFresh() const {return state_fresh_;}
  QString Result() const {return result_;}
  QString RobotNamespace() const {return QString::fromStdString(namespace_);}

  Q_INVOKABLE void ToggleHwEStop();
  Q_INVOKABLE void SwEStopSet();
  Q_INVOKABLE void SwEStopReset();
  Q_INVOKABLE void LatchReset();

signals:
  void HwPressedChanged();
  void StateChanged();
  void ResultChanged();

private:
  std::string Topic(const std::string & name) const;
  void PublishHwButton();
  void PublishEmpty(gz::transport::Node::Publisher & publisher);

  // gz-transport callbacks run on its own threads; they only queue work onto the Qt thread.
  void OnBool(bool * field, const gz::msgs::Boolean & msg);
  void OnResult(const gz::msgs::StringMsg & msg);
  void MarkFresh();
  void CheckStale();

  gz::transport::Node node_;
  gz::transport::Node::Publisher hw_button_pub_;
  gz::transport::Node::Publisher sw_set_pub_;
  gz::transport::Node::Publisher sw_reset_pub_;
  gz::transport::Node::Publisher latch_reset_pub_;

  // Republishes the maintained HW button, so the PLC node picks it up after a (re)start.
  QTimer hw_button_timer_;
  // Greys the lamps out when the PLC node stops answering.
  QTimer stale_timer_;
  qint64 last_state_ms_{0};

  std::string namespace_;
  bool hw_pressed_{false};
  bool sw_e_stop_{false};
  bool latch_active_{false};
  bool contactor_engaged_{false};
  bool motion_locked_{true};
  bool state_fresh_{false};
  QString result_;
};

}  // namespace rover_gazebo_plugins

#endif  // ROVER_GAZEBO_PLUGINS__ROVER_SAFETY_PANEL_HH_
