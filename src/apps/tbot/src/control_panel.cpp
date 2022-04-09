/* 
 * control_panel.cpp
 *
 * Created on 4/3/22 11:12 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "tbot/control_panel.hpp"

namespace robosw {
ControlPanel::ControlPanel(swviz::Viewer *parent, TbotContext &ctx) :
    Panel("ControlPanel", parent), ctx_(ctx) {

}

void ControlPanel::Draw() {
  Begin(NULL, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar);

  ImGui::Indent(10);

  ImGui::Dummy(ImVec2(0.0f, 5.0f));
  {
    ImGui::Text("PWM Control");
    ImGui::Separator();
    ImGui::Dummy(ImVec2(0.0f, 5.0f));

    static int pwm_left = 0, pwm_right = 0;

    ImGui::Text("Left:");
    ImGui::SameLine(60);
    ImGui::SliderInt("##slider_pwm_left", &pwm_left, -100, 100, "%d");
    ImGui::SameLine();
    if (ImGui::Button("Reset##Left")) { pwm_left = 0; }

    ImGui::Text("Right:");
    ImGui::SameLine(60);
    ImGui::SliderInt("##slider_pwm_right", &pwm_right, -100, 100, "%d");
    ImGui::SameLine();
    if (ImGui::Button("Reset##Right")) { pwm_right = 0; }

    if (ctx_.msger->IsStarted()) {
      ctx_.msger->SendPwmCommand(pwm_left, pwm_right);
    }
  }

  ImGui::Dummy(ImVec2(0.0f, 5.0f));
  {
    ImGui::Text("Motor Control");
    ImGui::Separator();
    ImGui::Dummy(ImVec2(0.0f, 5.0f));
  }

  ImGui::Unindent(10);

  End();
}
}