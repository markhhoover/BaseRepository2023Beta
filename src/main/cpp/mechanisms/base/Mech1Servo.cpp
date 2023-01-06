
//====================================================================================================================================================
// Copyright 2022 Lake Orion Robotics FIRST Team 302 
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================


// C++ Includes
#include <memory>
#include <string>

// FRC includes

// Team 302 includes
#include <hw/DragonServo.h>
#include <mechanisms/base/Mech.h>
#include <mechanisms/base/Mech1Servo.h>
#include <utils/Logger.h>

// Third Party Includes

using namespace std;

/// @brief Create a generic mechanism wiht 1 servo 
/// @param [in] std::shared_ptr<DragonServo> servo used by this mechanism
Mech1Servo::Mech1Servo
(
    MechanismTypes::MECHANISM_TYPE              type,
    std::string                                 controlFileName,
    std::string                                 networkTableName,
    DragonServo*                                servo
) : Mech(type, controlFileName, networkTableName),
    m_servo(servo)
{
    if (m_servo == nullptr )
    {
        Logger::GetLogger()->LogData( LOGGER_LEVEL::ERROR_ONCE, networkTableName, string( "Mech1Servo constructor" ), string( "servo is nullptr" ) );
    }
}


/// @brief      Move servo to the desired angle
/// @param [in] double angle: Target angle in degrees
/// @return     void
void Mech1Servo::SetAngle
(
    double angle       
) 
{
    if ( m_servo != nullptr )
    {
        m_servo->SetAngle( angle );
    }
}

double Mech1Servo::GetAngle() const
{
    if ( m_servo != nullptr )
    {
        return m_servo->GetAngle();
    }
    return 0.0;
}



/// @brief log data to the network table if it is activated and time period has past
void Mech1Servo::LogInformation() const
{
}



