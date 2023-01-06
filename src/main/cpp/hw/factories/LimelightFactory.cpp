
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

#include <map>

#include "hw/factories/LimelightFactory.h"
#include <hw/DragonLimelight.h>


using namespace std;

LimelightFactory* LimelightFactory::m_limelightFactory = nullptr;

LimelightFactory* LimelightFactory::GetLimelightFactory()
{
    if(m_limelightFactory == nullptr)
    {
        m_limelightFactory = new LimelightFactory();
    }
    return m_limelightFactory;
}

LimelightFactory::LimelightFactory() : m_limelight( nullptr )
{
}

DragonLimelight* LimelightFactory::CreateLimelight
(
    string                      tableName, 
    units::length::inch_t       mountingHeight,             /// <I> - mounting height of the limelight
    units::length::inch_t       mountingHorizontalOffset,   /// <I> - mounting horizontal offset from the middle of the robot
    units::angle::degree_t      rotation,                   /// <I> - clockwise rotation of limelight
    units::angle::degree_t      mountingAngle,              /// <I> - mounting angle of the camera
    units::length::inch_t       targetHeight,               /// <I> - height the target
    units::length::inch_t       targetHeight2,               /// <I> - height of second target
    DragonLimelight::LED_MODE       ledMode,
    DragonLimelight::CAM_MODE       camMode,
    DragonLimelight::STREAM_MODE    streamMode,
    DragonLimelight::SNAPSHOT_MODE  snapMode,
    double                          defaultXHairX,
    double                          defaultXHairY,
    double                          secXHairX,
    double                          secXHairY
)
{
    if ( m_limelight == nullptr )
    {
        m_limelight = new DragonLimelight(tableName, 
                                          mountingHeight, 
                                          mountingHorizontalOffset, 
                                          rotation, 
                                          mountingAngle, 
                                          targetHeight, 
                                          targetHeight2);
        /**
        m_limelight->SetLEDMode( ledMode );
        m_limelight->SetCamMode( camMode );
        m_limelight->SetStreamMode( streamMode );
        m_limelight->ToggleSnapshot( snapMode );
        if ( defaultXHairX > -1.5 && defaultXHairY > -1.5 )
        {
            // m_limelight->  todo add method
        }        
        if ( secXHairX > -1.5 && secXHairY > -1.5 )
        {
            // m_limelight->  todo add method
        }
        **/
    }
    return m_limelight;
}

DragonLimelight* LimelightFactory::GetLimelight()
{
    return m_limelight;
}

