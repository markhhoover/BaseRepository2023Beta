
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
#include <mechanisms/base/Mech1MotorState.h>
#include <mechanisms/controllers/ControlData.h>
#include <mechanisms/controllers/MechanismTargetData.h>
#include <mechanisms/base/Mech1IndMotor.h>
#include <utils/Logger.h>

#include <TeleopControl.h>

// Third Party Includes

using namespace std;

/// @class Mech1MotorState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
Mech1MotorState::Mech1MotorState
(
    Mech1IndMotor*                  mechanism,
    string                          stateName,
    int                             stateId,
    ControlData*                    control,
    double                          target
) : State(stateName, stateId),
    m_mechanism( mechanism ),
    m_control( control ),
    m_target( target ),
    m_positionBased( false ),
    m_speedBased( false )
{
    auto ntName = string("Mech1MotorState");
    if ( mechanism == nullptr )
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, ntName, ("Mech1MotorState::Mech1MotorState"), string("no mechanism"));
    }    
    else 
    {
        ntName = mechanism->GetNetworkTableName();
    }
    
    if ( control == nullptr )
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, ntName, string("Mech1MotorState::Mech1MotorState"), string("no control data"));
    }
    else
    {
        auto mode = control->GetMode();
        switch (mode)
        {
            case ControlModes::CONTROL_TYPE::PERCENT_OUTPUT:
                m_positionBased = false;
                m_speedBased = false;
                break;

            case ControlModes::CONTROL_TYPE::VOLTAGE:
                m_positionBased = false;
                m_speedBased = false;
                break;

            case ControlModes::CONTROL_TYPE::POSITION_DEGREES:
            case ControlModes::CONTROL_TYPE::POSITION_INCH:
                m_positionBased = true;
                m_speedBased = false;
                break;
            
            case ControlModes::CONTROL_TYPE::VELOCITY_DEGREES:
            case ControlModes::CONTROL_TYPE::VELOCITY_INCH:
            case ControlModes::CONTROL_TYPE::VELOCITY_RPS:
                m_positionBased = false;
                m_speedBased = true;
                break;

            case ControlModes::CONTROL_TYPE::CURRENT:
                m_positionBased = false;
                m_speedBased = false;
                break;

            case ControlModes::CONTROL_TYPE::MOTION_PROFILE:
                m_positionBased = false;
                m_speedBased = false;
                break;

            case ControlModes::CONTROL_TYPE::MOTION_PROFILE_ARC:
                m_positionBased = false;
                m_speedBased = false;
                break;

            case ControlModes::CONTROL_TYPE::TRAPEZOID:
                m_positionBased = false;
                m_speedBased = false;
                break;

            default:
                m_positionBased = false;
                m_speedBased = false;
                break;
        }
    }
    
}

void Mech1MotorState::Init()
{
    if ( m_mechanism != nullptr && m_control != nullptr )
    {
        m_mechanism->SetControlConstants( 0, m_control );
        m_mechanism->UpdateTarget( m_target );
    }
}


void Mech1MotorState::Run()           
{
    if ( m_mechanism != nullptr)
    {
        m_mechanism->Update();
    }
}

void Mech1MotorState::Exit() 
{
}

bool Mech1MotorState::AtTarget() const
{
    auto same = true;
    if ( m_mechanism != nullptr )
    {
        if ( m_positionBased && !m_speedBased )
        {
            same = ( abs( m_target - m_mechanism->GetPosition())  < 1.0 );
        }
        else if ( !m_positionBased && m_speedBased )
        {
            same = ( abs( m_target - m_mechanism->GetSpeed()) < 1.0 );
        }
        else if ( m_positionBased && m_speedBased )
        {
            same = ( ( abs( m_target - m_mechanism->GetPosition())  < 1.0 ) ||
                     ( abs( m_target - m_mechanism->GetSpeed())     < 1.0 ) );
        }
    }
    return same;
}

void Mech1MotorState::LogInformation() const
{
    if ( m_mechanism != nullptr)
    {
        auto ntName = m_mechanism->GetNetworkTableName();
        auto statename = GetStateName();
        auto idStatename = string("Mech1MotorState") + to_string(GetStateId()) + string(" - ") + statename;
        auto idStatenameTarget = idStatename + string(" - Target");
        auto idStatenameSpeed = idStatename + string(" - Speed");

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, ntName, idStatename, GetStateName());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, ntName, idStatenameTarget, GetTarget());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, ntName, idStatenameSpeed, GetRPS());
    }
}