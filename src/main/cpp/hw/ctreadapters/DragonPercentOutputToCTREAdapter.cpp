//====================================================================================================================================================
// IDragonAnglePositionSensor.h
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

// FRC includes

// Team 302 includes
#include <hw/ctreadapters/DragonControlToCTREAdapter.h>
#include <hw/ctreadapters/DragonPercentOutputToCTREAdapter.h>
#include <mechanisms/controllers/ControlData.h>
#include <mechanisms/controllers/ControlModes.h>

// Third Party Includes
#include <ctre/phoenix/motorcontrol/ControlMode.h>
#include <ctre/phoenix/motorcontrol/can/WPI_BaseMotorController.h>

DragonPercentOutputToCTREAdapter::DragonPercentOutputToCTREAdapter
(
    std::string                                                     networkTableName,
    int                                                             controllerSlot, 
    ControlData*                                                    controlInfo,          
    DistanceAngleCalcStruc                                          calcStruc,
    ctre::phoenix::motorcontrol::can::WPI_BaseMotorController*      controller   
) : DragonControlToCTREAdapter(networkTableName, controllerSlot, controlInfo, calcStruc, controller)
{

}
void DragonPercentOutputToCTREAdapter::Set
(
    double          value
)
{
    m_controller->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, value);
}

void DragonPercentOutputToCTREAdapter::SetControlConstants
(
    int             controlSlot, 
    ControlData*    controlInfo
) 
{
	SetPeakAndNominalValues(m_networkTableName, controlInfo);

}



