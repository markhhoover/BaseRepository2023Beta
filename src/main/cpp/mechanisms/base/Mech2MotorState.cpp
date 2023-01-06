
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
#include <mechanisms/base/Mech2MotorState.h>
#include <mechanisms/controllers/ControlData.h>
#include <mechanisms/controllers/MechanismTargetData.h>
#include <mechanisms/base/Mech2IndMotors.h>
#include <utils/Logger.h>

#include <TeleopControl.h>

// Third Party Includes

using namespace std;

/// @class Mech2MotorState
/// @brief information about the control (open loop, closed loop position, closed loop velocity, etc.) for a mechanism state
Mech2MotorState::Mech2MotorState
(
    Mech2IndMotors*                 mechanism,
    string                          stateName,
    int                             stateId,
    ControlData*                    control,
    ControlData*                    control2,
    double                          primaryTarget,
    double                          secondaryTarget
) : State(stateName, stateId),
    m_mechanism( mechanism ),
    m_control( control ),
    m_control2( control2 ),
    m_primaryTarget( primaryTarget ),
    m_secondaryTarget( secondaryTarget ),
    m_positionBased( false ),
    m_speedBased( false )
{
    if ( mechanism == nullptr )
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("Mech2MotorState"), string("Mech2MotorState"), string("no mechanism"));
    }    
    else if ( control == nullptr )
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, mechanism->GetNetworkTableName(), ("Mech2MotorState::Mech2MotorState"), string("no control data"));
    }    
    else if ( control2 == nullptr )
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, mechanism->GetNetworkTableName(), ("Mech2MotorState::Mech2MotorState"), string("no control2 data"));
    }
    else
    {
        auto mode = control->GetMode();
        auto mode2 = control2->GetMode();
        if ( mode == mode2)
        {
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
        else
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, mechanism->GetNetworkTableName(), ("Mech2MotorState::Mech2MotorState"), string("inconsistent control modes"));
        }
        
    }
    
}

void Mech2MotorState::Init()
{
    if ( m_mechanism != nullptr && m_control != nullptr && m_control2 != nullptr )
    {
        m_mechanism->SetControlConstants( 0, m_control );
        m_mechanism->SetSecondaryControlConstants( 0, m_control2 );
        m_mechanism->UpdateTargets( m_primaryTarget, m_secondaryTarget );
    }
}


void Mech2MotorState::Run()           
{
    if ( m_mechanism != nullptr )
    {
        m_mechanism->Update();
    }
}

void Mech2MotorState::Exit() 
{
}

bool Mech2MotorState::AtTarget() const
{
    auto same = true;
    if ( m_mechanism != nullptr )
    {
        if ( m_positionBased && !m_speedBased )
        {
            same = ( abs( m_primaryTarget - m_mechanism->GetPrimaryPosition())     < 1.0 &&
                     abs( m_secondaryTarget - m_mechanism->GetSecondaryPosition()) < 1.0 );
        }
        else if ( !m_positionBased && m_speedBased )
        {
            same = ( abs( m_primaryTarget - m_mechanism->GetPrimarySpeed())     < 1.0 &&
                     abs( m_secondaryTarget - m_mechanism->GetSecondarySpeed()) < 1.0 );
        }
        else if ( m_positionBased && m_speedBased )
        {
            same = ( ( abs( m_primaryTarget - m_mechanism->GetPrimaryPosition())     < 1.0 &&
                       abs( m_secondaryTarget - m_mechanism->GetSecondaryPosition()) < 1.0 ) ||
                     ( abs( m_primaryTarget - m_mechanism->GetPrimarySpeed())     < 1.0 &&
                       abs( m_secondaryTarget - m_mechanism->GetSecondarySpeed()) < 1.0 ) );
        }
    }
    return same;
}

double Mech2MotorState::GetPrimaryRPS() const
{
    return m_mechanism->GetPrimarySpeed();
}

double Mech2MotorState::GetSecondaryRPS() const
{
    return m_mechanism->GetSecondarySpeed();
}


void Mech2MotorState::LogInformation() const
{
    if ( m_mechanism != nullptr )
    {
        auto ntName = m_mechanism->GetNetworkTableName();
        auto id = GetStateName();
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, ntName, string("state name"), id);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, ntName, string("target1"), m_primaryTarget);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, ntName, string("target2"), m_secondaryTarget);
    }

}