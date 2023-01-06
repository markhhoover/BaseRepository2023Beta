
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

// C++ Includes
#include <memory>
#include <string>

// FRC includes

// Team 302 includes
#include <State.h>
#include <mechanisms/base/Mech1IndMotorSolenoidState.h>
#include <mechanisms/controllers/ControlData.h>
#include <mechanisms/controllers/MechanismTargetData.h>
#include <mechanisms/base/Mech1IndMotorSolenoid.h>
#include <utils/Logger.h>

#include <TeleopControl.h>

// Third Party Includes

using namespace std;

/// @class Mech1IndMotorSolenoidState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
Mech1IndMotorSolenoidState::Mech1IndMotorSolenoidState
(
    Mech1IndMotorSolenoid*          mechanism,
    string                          stateName,
    int                             stateId,
    ControlData*                    control,
    double                          target,
    MechanismTargetData::SOLENOID   solState

) : State(stateName, stateId),
    m_mechanism( mechanism ),
    m_motorState(make_shared<Mech1MotorState>(mechanism->Get1IndMotorMech(), stateName, stateId, control, target)),
    m_solenoidState(make_shared<MechSolenoidState>(mechanism->GetSolenoidMech(), stateName, stateId, solState))
{
    if ( control == nullptr )
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, mechanism->GetNetworkTableName(), ("Mech1IndMotorSolenoidState::Mech1IndMotorSolenoidState"), string("no control data"));
    }

    if ( mechanism == nullptr )
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("Bad Pointer"), string("Mech1IndMotorSolenoidState::Mech1IndMotorSolenoidState"), string("no mechanism"));
    }    
}

void Mech1IndMotorSolenoidState::Init()
{
    m_motorState.get()->Init();
    m_solenoidState.get()->Init();
}


void Mech1IndMotorSolenoidState::Run()           
{
    m_motorState.get()->Run();
    m_solenoidState.get()->Run();

}

void Mech1IndMotorSolenoidState::Exit() 
{
}

bool Mech1IndMotorSolenoidState::AtTarget() const
{
    return m_motorState.get()->AtTarget();
}

double Mech1IndMotorSolenoidState::GetTarget() const
{
    return m_motorState.get()->GetTarget();
}

double Mech1IndMotorSolenoidState::GetRPS() const
{
    return m_motorState.get()->GetRPS();
}

void Mech1IndMotorSolenoidState::LogInformation() const
{
    m_motorState.get()->LogInformation();
    m_solenoidState.get()->LogInformation();
}