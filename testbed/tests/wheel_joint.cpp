// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "settings.h"
#include "test.h"
#include "imgui/imgui.h"

// Test the wheel joint with motor, spring, and limit options.
class WheelJoint : public Test
{
public:
	WheelJoint()
	{
		b2Body* ground = NULL;
		{
			b2BodyDef bd;
			ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
			ground->CreateFixture(&shape, 0.0f);
		}

		{
			b2PolygonShape shape;
			shape.SetAsBox(2.0f, 2.0f);

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(0.0f, 10.0f);
			bd.angle = 0.5f * b2_pi;
			bd.allowSleep = false;
			b2Body* body = m_world->CreateBody(&bd);
			body->CreateFixture(&shape, 5.0f);

			float mass = body->GetMass();
			float hertz = 1.0f;
			float dampingRatio = 0.7f;
			float omega = 2.0f * b2_pi * hertz;

			b2WheelJointDef jd;

			// Horizontal
			jd.Initialize(ground, body, bd.position, b2Vec2(0.0f, 1.0f));

			jd.motorSpeed = 10.0f;
			jd.maxMotorTorque = 10000.0f;
			jd.enableMotor = false;
			jd.stiffness = mass * omega * omega;
			jd.damping = 2.0f * mass * dampingRatio * omega;
			jd.lowerTranslation = -3.0f;
			jd.upperTranslation = 3.0f;
			jd.enableLimit = true;

			m_joint = (b2WheelJoint*)m_world->CreateJoint(&jd);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(200.0f, 700.0f));
		ImGui::Begin("Tuning", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		ImGui::Separator();

		ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.5f);

		const ImGuiComboFlags comboFlags = 0;
		const char* bendModels[] = { "Spring", "PBD Ang", "XPBD Ang", "PBD Dist", "PBD Height" };
		const char* stretchModels[] = { "PBD", "XPBD" };

		ImGui::Text("Rope 1");
		static int bendModel1 = m_tuning1.bendingModel;
		if (ImGui::BeginCombo("Bend Model##1", bendModels[bendModel1], comboFlags))
		{
			for (int i = 0; i < IM_ARRAYSIZE(bendModels); ++i)
			{
				bool isSelected = (bendModel1 == i);
				if (ImGui::Selectable(bendModels[i], isSelected))
				{
					bendModel1 = i;
					m_tuning1.bendingModel = b2BendingModel(i);
				}

				if (isSelected)
				{
					ImGui::SetItemDefaultFocus();
				}
			}
			ImGui::EndCombo();
		}

		ImGui::SliderFloat("Damping##B1", &m_tuning1.bendDamping, 0.0f, 4.0f, "%.1f");
		ImGui::SliderFloat("Hertz##B1", &m_tuning1.bendHertz, 0.0f, 60.0f, "%.0f");
		ImGui::SliderFloat("Stiffness##B1", &m_tuning1.bendStiffness, 0.0f, 1.0f, "%.1f");

		ImGui::Checkbox("Isometric##1", &m_tuning1.isometric);
		ImGui::Checkbox("Fixed Mass##1", &m_tuning1.fixedEffectiveMass);
		ImGui::Checkbox("Warm Start##1", &m_tuning1.warmStart);

		static int stretchModel1 = m_tuning1.stretchingModel;
		if (ImGui::BeginCombo("Stretch Model##1", stretchModels[stretchModel1], comboFlags))
		{
			for (int i = 0; i < IM_ARRAYSIZE(stretchModels); ++i)
			{
				bool isSelected = (stretchModel1 == i);
				if (ImGui::Selectable(stretchModels[i], isSelected))
				{
					stretchModel1 = i;
					m_tuning1.stretchingModel = b2StretchingModel(i);
				}

				if (isSelected)
				{
					ImGui::SetItemDefaultFocus();
				}
			}
			ImGui::EndCombo();
		}

		ImGui::SliderFloat("Damping##S1", &m_tuning1.stretchDamping, 0.0f, 4.0f, "%.1f");
		ImGui::SliderFloat("Hertz##S1", &m_tuning1.stretchHertz, 0.0f, 60.0f, "%.0f");
		ImGui::SliderFloat("Stiffness##S1", &m_tuning1.stretchStiffness, 0.0f, 1.0f, "%.1f");

		ImGui::SliderInt("Iterations##1", &m_iterations1, 1, 100, "%d");

		ImGui::Separator();

		ImGui::Text("Rope 2");
		static int bendModel2 = m_tuning2.bendingModel;
		if (ImGui::BeginCombo("Bend Model##2", bendModels[bendModel2], comboFlags))
		{
			for (int i = 0; i < IM_ARRAYSIZE(bendModels); ++i)
			{
				bool isSelected = (bendModel2 == i);
				if (ImGui::Selectable(bendModels[i], isSelected))
				{
					bendModel2 = i;
					m_tuning2.bendingModel = b2BendingModel(i);
				}

				if (isSelected)
				{
					ImGui::SetItemDefaultFocus();
				}
			}
			ImGui::EndCombo();
		}

		ImGui::SliderFloat("Damping##B2", &m_tuning2.bendDamping, 0.0f, 4.0f, "%.1f");
		ImGui::SliderFloat("Hertz##B2", &m_tuning2.bendHertz, 0.0f, 60.0f, "%.0f");
		ImGui::SliderFloat("Stiffness##B2", &m_tuning2.bendStiffness, 0.0f, 1.0f, "%.1f");

		ImGui::Checkbox("Isometric##2", &m_tuning2.isometric);
		ImGui::Checkbox("Fixed Mass##2", &m_tuning2.fixedEffectiveMass);
		ImGui::Checkbox("Warm Start##2", &m_tuning2.warmStart);

		static int stretchModel2 = m_tuning2.stretchingModel;
		if (ImGui::BeginCombo("Stretch Model##2", stretchModels[stretchModel2], comboFlags))
		{
			for (int i = 0; i < IM_ARRAYSIZE(stretchModels); ++i)
			{
				bool isSelected = (stretchModel2 == i);
				if (ImGui::Selectable(stretchModels[i], isSelected))
				{
					stretchModel2 = i;
					m_tuning2.stretchingModel = b2StretchingModel(i);
				}

				if (isSelected)
				{
					ImGui::SetItemDefaultFocus();
				}
			}
			ImGui::EndCombo();
		}

		ImGui::SliderFloat("Damping##S2", &m_tuning2.stretchDamping, 0.0f, 4.0f, "%.1f");
		ImGui::SliderFloat("Hertz##S2", &m_tuning2.stretchHertz, 0.0f, 60.0f, "%.0f");
		ImGui::SliderFloat("Stiffness##S2", &m_tuning2.stretchStiffness, 0.0f, 1.0f, "%.1f");

		ImGui::SliderInt("Iterations##2", &m_iterations2, 1, 100, "%d");

		ImGui::Separator();

		ImGui::SliderFloat("Speed", &m_speed, 10.0f, 100.0f, "%.0f");

		if (ImGui::Button("Reset"))
		{
			m_position1.Set(-5.0f, 15.0f);
			m_position2.Set(5.0f, 15.0f);
			m_rope1.Reset(m_position1);
			m_rope2.Reset(m_position2);
		}

		ImGui::PopItemWidth();

		ImGui::End();
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_L:
			m_joint->EnableLimit(!m_joint->IsLimitEnabled());
			break;

		case GLFW_KEY_M:
			m_joint->EnableMotor(!m_joint->IsMotorEnabled());
			break;

		case GLFW_KEY_S:
			m_joint->SetMotorSpeed(-m_joint->GetMotorSpeed());
			break;
		}
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);
		g_debugDraw.DrawString(5, m_textLine, "Keys: (l) limits, (m) motors, (s) speed");
		m_textLine += m_textIncrement;
		float torque = m_joint->GetMotorTorque(settings.m_hertz);
		g_debugDraw.DrawString(5, m_textLine, "Motor Torque = %4.0f", (float)torque);
		m_textLine += m_textIncrement;
	}

	static Test* Create()
	{
		return new WheelJoint;
	}

	b2WheelJoint* m_joint;
};

static int testIndex = RegisterTest("Joints", "Wheel", WheelJoint::Create);
