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

import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
  id: safetyPanel
  color: "transparent"
  Layout.minimumWidth: 290
  Layout.minimumHeight: 420
  anchors.fill: parent

  readonly property color stopRed: "#c62828"
  readonly property color okGreen: "#2e7d32"
  readonly property color warnAmber: "#ef6c00"
  readonly property color staleGrey: "#9e9e9e"

  component Lamp: RowLayout {
    property string label
    // true = the "bad" (stopping) state is on.
    property bool active
    property string onText: "ON"
    property string offText: "OFF"
    spacing: 8
    Rectangle {
      width: 16
      height: 16
      radius: 8
      color: !_RoverSafetyPanel.stateFresh ? staleGrey : (active ? stopRed : okGreen)
      border.color: "#424242"
    }
    Label {
      text: label
      Layout.fillWidth: true
    }
    Label {
      text: !_RoverSafetyPanel.stateFresh ? "?" : (active ? onText : offText)
      font.bold: true
    }
  }

  component ActionButton: Button {
    property color tint
    Layout.fillWidth: true
    Layout.preferredHeight: 40
    font.bold: true
    contentItem: Label {
      text: parent.text
      font: parent.font
      color: "white"
      horizontalAlignment: Text.AlignHCenter
      verticalAlignment: Text.AlignVCenter
    }
    background: Rectangle {
      radius: 4
      color: parent.down ? Qt.darker(tint, 1.3) : tint
    }
  }

  ColumnLayout {
    anchors.fill: parent
    anchors.margins: 10
    spacing: 8

    // Maintained mushroom button: stays pressed until clicked again, like the real one.
    Button {
      id: hwButton
      Layout.fillWidth: true
      Layout.preferredHeight: 64
      font.bold: true
      font.pixelSize: 16
      onClicked: _RoverSafetyPanel.ToggleHwEStop()
      contentItem: Label {
        text: _RoverSafetyPanel.hwPressed ? "HW E-STOP: PRESSED\n(click to release)"
                                          : "HW E-STOP"
        font: hwButton.font
        color: "white"
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
      }
      background: Rectangle {
        radius: 6
        color: _RoverSafetyPanel.hwPressed ? Qt.darker(stopRed, 1.6) : stopRed
        border.width: _RoverSafetyPanel.hwPressed ? 4 : 1
        border.color: _RoverSafetyPanel.hwPressed ? "#ffeb3b" : "#424242"
      }
      ToolTip.visible: hovered
      ToolTip.text: "Physical E-Stop button (maintained). Pressing sets the latch; releasing resets it, unless the SW E-Stop is still set."
    }

    ActionButton {
      text: "SW E-STOP"
      tint: stopRed
      onClicked: _RoverSafetyPanel.SwEStopSet()
      ToolTip.visible: hovered
      ToolTip.text: "hardware_interface/sw_user_e_stop_set"
    }

    RowLayout {
      Layout.fillWidth: true
      ActionButton {
        text: "SW RESET"
        tint: warnAmber
        onClicked: _RoverSafetyPanel.SwEStopReset()
        ToolTip.visible: hovered
        ToolTip.text: "hardware_interface/sw_user_e_stop_reset - refused while the wheels turn"
      }
      ActionButton {
        text: "RESET LATCH"
        tint: warnAmber
        onClicked: _RoverSafetyPanel.LatchReset()
        ToolTip.visible: hovered
        ToolTip.text: "hardware_interface/sw_e_stop_latch_reset - no effect while a stop is asserted"
      }
    }

    MenuSeparator { Layout.fillWidth: true }

    Lamp { label: "SW E-Stop"; active: _RoverSafetyPanel.swEStop; onText: "SET"; offText: "CLEAR" }
    Lamp { label: "Latch"; active: _RoverSafetyPanel.latchActive; onText: "LATCHED"; offText: "CLEAR" }
    Lamp { label: "Contactor"; active: !_RoverSafetyPanel.contactorEngaged; onText: "OPEN"; offText: "CLOSED" }
    Lamp { label: "Motion lock"; active: _RoverSafetyPanel.motionLocked; onText: "LOCKED"; offText: "FREE" }

    Label {
      Layout.fillWidth: true
      wrapMode: Text.WordWrap
      font.italic: true
      text: _RoverSafetyPanel.stateFresh
            ? (_RoverSafetyPanel.result.length > 0 ? _RoverSafetyPanel.result : " ")
            : "No state from " + _RoverSafetyPanel.robotNamespace + "/sim_safety_plc"
    }

    Label {
      Layout.fillWidth: true
      wrapMode: Text.WordWrap
      color: staleGrey
      text: "After SW E-STOP: SW RESET, then RESET LATCH.\nAfter HW E-STOP: release it (resets the latch)."
    }

    Item { Layout.fillHeight: true }
  }
}
