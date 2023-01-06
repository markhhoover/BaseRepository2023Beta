
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

/// @class ChassisXmlParser. 
/// @brief Create a chassis from an XML definition.   CHASSIS_TYPE (ChassisFactory.h) determines the type of 
///        chassis to create.   WheelBase is the front to back distance between the wheel centers.   Track 
///        is the left to right distance between the wheels.

// C++ includes
#include <map>
#include <memory>
#include <string>
#include <utility>


// FRC includes
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>


// Team302 includes
#include <chassis/ChassisFactory.h>
#include <chassis/ChassisSpeedCalcEnum.h>
#include <chassis/IChassis.h>
#include <chassis/PoseEstimatorEnum.h>
#include <hw/interfaces/IDragonMotorController.h>
#include <hw/usages/IDragonMotorControllerMap.h>
#include <utils/Logger.h>
#include <chassis/ChassisXmlParser.h>
#include <hw/xml/MotorXmlParser.h>
#include <chassis/swerve/SwerveModuleXmlParser.h>

// Third Party includes
#include <pugixml/pugixml.hpp>


using namespace frc;
using namespace pugi;
using namespace std;



/// @brief  Parse the chassie element (and it children).  When this is done a IChassis object exists.
///		   It can be retrieved from the factory.
/// @param [in]  pugi::xml_node the chassis element in the XML document
/// @return void

IChassis* ChassisXmlParser::ParseXML
(
	xml_node      chassisNode
)
{
    IChassis* chassis = nullptr;
    // initialize the attributes to the default values
    ChassisFactory::CHASSIS_TYPE type = ChassisFactory::CHASSIS_TYPE::TANK_CHASSIS;
    units::length::inch_t wheelDiameter(0.0);
    units::length::inch_t wheelBase(0.0);
    units::length::inch_t track(0.0);
    double odometryComplianceCoefficient = 1.0;
    units::velocity::meters_per_second_t maxVelocity(0.0);
    units::radians_per_second_t maxAngularSpeed(0.0);
    units::acceleration::meters_per_second_squared_t maxAcceleration(0.0);
    units::angular_acceleration::radians_per_second_squared_t maxAngularAcceleration(0.0);
    string networkTableName;
    string controlFileName;

    //ChassisSpeedCalcEnum speedCalcOption = ChassisSpeedCalcEnum::ETHER;
    PoseEstimatorEnum poseEstOption   = PoseEstimatorEnum::EULER_AT_CHASSIS;

    bool hasError = false;

    // process attributes
    for (xml_attribute attr = chassisNode.first_attribute(); attr && !hasError; attr = attr.next_attribute())
    {
        string attrName (attr.name());
        if ( attrName.compare("type") == 0 )
        {
            auto val = string( attr.value() );
            if ( val.compare( "MECANUM") == 0 )
            {
                type = ChassisFactory::CHASSIS_TYPE::MECANUM_CHASSIS;
            }
            else if ( val.compare( "TANK" ) == 0 )
            {
                type = ChassisFactory::CHASSIS_TYPE::TANK_CHASSIS;
            }
            else if (val.compare("SWERVE") == 0)
            {
                type = ChassisFactory::CHASSIS_TYPE::SWERVE_CHASSIS;
            }
            else
            {
                string msg = "Unknown Chassis Type";
                msg += attr.value();
                Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("ChasssiXmlParser"), string( "ParseXML" ), msg );
            }
        }
        else if (  attrName.compare("wheelBase") == 0 )
        {
        	wheelBase = units::length::inch_t(attr.as_double());
        }
        else if (  attrName.compare("track") == 0 )
        {
        	track = units::length::inch_t(attr.as_double());
        }
        else if (  attrName.compare("maxVelocity") == 0 )
        {
            units::velocity::feet_per_second_t fps(attr.as_double()/12.0);
        	maxVelocity = units::velocity::meters_per_second_t(fps);
        }
        else if (  attrName.compare("maxAngularVelocity") == 0 )
        {
            units::degrees_per_second_t degreesPerSec(attr.as_double());
        	maxAngularSpeed = units::radians_per_second_t(degreesPerSec);
        }
        else if (  attrName.compare("maxAcceleration") == 0 )
        {
            maxAcceleration = units::feet_per_second_t(attr.as_double()/12.0) / 1_s;
        }
        else if (  attrName.compare("maxAngularAcceleration") == 0 )
        {
            maxAngularAcceleration = units::degrees_per_second_t(attr.as_double()) / 1_s;
        }
        else if (  attrName.compare("wheelDiameter") == 0 )
        {
        	wheelDiameter = units::length::inch_t(attr.as_double());
        }
        else if ( attrName.compare("odometryComplianceCoefficient") == 0 )
        {
            odometryComplianceCoefficient = attr.as_double();
        }
        else if (attrName.compare("networkTable") == 0)
        {
            networkTableName = attr.as_string();
        }
        else if (attrName.compare("controlFile") == 0)
        {
            controlFileName = attr.as_string();
        }
        /** TODO: remove this is unused **/
        else if (attrName.compare("wheelSpeedCalcOption") ==0)
        {
            /**
            auto val = string( attr.value() );
            if (val.compare( "WPI") == 0)
            {
                speedCalcOption = ChassisSpeedCalcEnum::WPI_METHOD;
            }
            else if (val.compare("ETHER") == 0)
            {
                speedCalcOption = ChassisSpeedCalcEnum::ETHER;
            }
            else if (val.compare("ETHER") == 0)
            {
                speedCalcOption = ChassisSpeedCalcEnum::ETHER;
            }
            else
            {
                string msg = "unknown Chassis Speed Calc Option ";
                msg += val;
                Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("ChassisXmlParser::ParseXML"), msg );
                hasError = true;
            }
            **/
        }
        /** **/
        else if (attrName.compare("poseEstimationOption") ==0)
        {
            auto val = string( attr.value() );
            if (val.compare( "WPI") == 0)
            {
                poseEstOption = PoseEstimatorEnum::WPI;
            }
            else if (val.compare("EULERCHASSIS") == 0)
            {
                poseEstOption = PoseEstimatorEnum::EULER_AT_CHASSIS;
            }
            else if (val.compare("EULERWHEEL") == 0)
            {
                poseEstOption = PoseEstimatorEnum::EULER_USING_MODULES;
            }
            else if (val.compare("POSECHASSIS") == 0)
            {
                poseEstOption = PoseEstimatorEnum::POSE_EST_AT_CHASSIS;
            }
            else if (val.compare("POSEWHEEL") == 0)
            {
                poseEstOption = PoseEstimatorEnum::POSE_EST_USING_MODULES;
            }
            else
            {
                string msg = "unknown Chassis Pose Estimation Option ";
                msg += val;
                Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("ChassisXmlParser"),string("ParseXML"), msg );
                hasError = true;
            }
        }
        else   // log errors
        {
            string msg = "unknown attribute ";
            msg += attr.name();
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("ChassisXmlParser"), string("ParseXML"), msg );
            hasError = true;
        }
    }


    // Process child element nodes
    IDragonMotorControllerMap motors;
    unique_ptr<MotorXmlParser> motorXML = make_unique<MotorXmlParser>();
    
    shared_ptr<SwerveModule> lfront;
    shared_ptr<SwerveModule> rfront;
    shared_ptr<SwerveModule> lback;
    shared_ptr<SwerveModule> rback;
    unique_ptr<SwerveModuleXmlParser> moduleXML = make_unique<SwerveModuleXmlParser>();


    for (xml_node child = chassisNode.first_child(); child; child = child.next_sibling())
    {
        string childName (child.name());
      	if (childName.compare("motor") == 0)
    	{
            auto motor = motorXML.get()->ParseXML(networkTableName, child);
            if ( motor.get() != nullptr )
            {
                motors[ motor.get()->GetType() ] =  motor ;
            }
    	}        
    	else if (childName.compare("swervemodule") == 0)
    	{
            shared_ptr<SwerveModule> module = moduleXML.get()->ParseXML(networkTableName, child);
            switch ( module.get()->GetType() )
            {
                case SwerveModule::ModuleID::LEFT_FRONT:
                    lfront = module;
                    break;

                case SwerveModule::ModuleID::LEFT_BACK:
                    lback = module;
                    break;

                case SwerveModule::ModuleID::RIGHT_FRONT:
                    rfront = module;
                    break;
                
                case SwerveModule::ModuleID::RIGHT_BACK:
                    rback = module;
                    break;

                default:
                    break;
            }
    	}
    	else  // log errors
    	{
            string msg = "unknown child ";
            msg += child.name();
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("SwerveChassisXmlParser"), string("ParseXML"), msg );
    	}
    }


    // create chassis instance
    if ( !hasError )
    {
        auto factory = ChassisFactory::GetChassisFactory();
        if ( factory != nullptr )
        {
            chassis = factory->CreateChassis( type, 
                                              networkTableName,
                                              controlFileName,
                                              wheelDiameter, 
                                              wheelBase, 
                                              track, 
                                              maxVelocity,
                                              maxAngularSpeed,
                                              maxAcceleration,
                                              maxAngularAcceleration,
                                              motors,
                                              lfront,
                                              rfront,
                                              lback,
                                              rback,
                                              //speedCalcOption,
                                              poseEstOption, 
                                              odometryComplianceCoefficient );
        }
        else  // log errors
        {
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("ChassisXmlParser"), string("ParseXML"), string("unable to create chassis") );
        }
    }
    return chassis;
}

