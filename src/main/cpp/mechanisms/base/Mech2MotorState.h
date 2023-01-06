
//====================================================================================================================================================
/// Copyright 2022 Lake Orion Robotics FIRST Team 302 
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#pragma once
#include <string>

#include <State.h>

// forward declare
class ControlData;
class Mech2IndMotors;

class Mech2MotorState : public State
{
    public:

        Mech2MotorState
        (
            Mech2IndMotors*                 mechanism,
            std::string                     stateName,
            int                             stateId,
            ControlData*                    control,
            ControlData*                    control2,
            double                          primaryTarget,
            double                          secondaryTarget
        );
        Mech2MotorState() = delete;
        ~Mech2MotorState() = default;

        void Init() override;
        void Run() override;
        void Exit() override;
        bool AtTarget() const override;

        void LogInformation() const override;

        double GetPrimaryTarget() const {return m_primaryTarget;}
        double GetSecondaryTarget() const {return m_secondaryTarget;}
        double GetPrimaryRPS() const;
        double GetSecondaryRPS() const;
        ControlData* GetPrimaryControlData() const {return m_control;}
        ControlData* GetSecondaryControlData() const {return m_control2;}

    private:

        Mech2IndMotors*                 m_mechanism;
        ControlData*                    m_control;
        ControlData*                    m_control2;
        double                          m_primaryTarget;
        double                          m_secondaryTarget;
        bool                            m_positionBased;
        bool                            m_speedBased;
};
