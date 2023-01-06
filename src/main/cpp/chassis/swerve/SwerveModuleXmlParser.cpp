
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

/// @class SwerveModuleXmlParser. 
/// @brief Create a chassis from an XML definition.   CHASSIS_TYPE (ChassisFactory.h) determines the type of 
///        chassis to create.   WheelBase is the front to back distance between the wheel centers.   Track 
///        is the left to right distance between the wheels.

// C++ includes
#include <map>
#include <memory>
#include <string>
#include <utility>


// FRC includes

// Team302 includes
#include <chassis/ChassisFactory.h>
#include <chassis/swerve/SwerveModule.h>
#include <hw/DragonCanCoder.h>
#include <hw/interfaces/IDragonMotorController.h>
#include <hw/usages/IDragonMotorControllerMap.h>
#include <utils/Logger.h>
#include <hw/xml/CancoderXmlParser.h>
#include <hw/xml/MotorXmlParser.h>
#include <chassis/swerve/SwerveModuleXmlParser.h>

// Third Party includes
#include <pugixml/pugixml.hpp>


using namespace frc;
using namespace pugi;
using namespace std;



/// @brief  Parse the chassie element (and it children).  When this is done a SwerveModule object exists.
///		   It can be retrieved from the factory.
/// @param [in]  pugi::xml_node the chassis element in the XML document
/// @return void shared_ptr<SwerveModule> 
std::shared_ptr<SwerveModule> SwerveModuleXmlParser::ParseXML
(
    string          baseNetworkTableName,
	xml_node        SwerveModuleNode
)
{
   auto hasError = false;
   std::shared_ptr<SwerveModule> module;

    // initialize the attributes to the default values
    SwerveModule::ModuleID position = SwerveModule::ModuleID::LEFT_FRONT;
    double turnP = 0.0;
    double turnI = 0.0;
    double turnD = 0.0;
    double turnF = 0.0;
    double turnNominalVal = 0.0;
    double turnPeakVal = 1.0;
    double turnMaxAcc = 0.0;
    double turnCruiseVel = 0.0;
    double countsOnTurnEncoderPerDegreesOnAngleSensor = 1.0;
    auto networkTableName = baseNetworkTableName;

    // process attributes
    for (xml_attribute attr = SwerveModuleNode.first_attribute(); attr && !hasError; attr = attr.next_attribute())
    {
        string attrName (attr.name());
        if ( attrName.compare("type") == 0 )
        {
            auto thisPosition = string( attr.value() );
            if ( thisPosition.compare("LEFT_FRONT") == 0 )
            {
                position = SwerveModule::ModuleID::LEFT_FRONT;
                networkTableName  += " - Left_Front Module";
            }
            else if ( thisPosition.compare("RIGHT_FRONT") == 0  )
            {
                position = SwerveModule::ModuleID::RIGHT_FRONT;
                networkTableName  += " - Right_Front Module";
            }
            else if ( thisPosition.compare("LEFT_BACK") == 0  )
            {
                position = SwerveModule::ModuleID::LEFT_BACK;
                networkTableName  += " - Left_Back Module";
            }
            else if ( thisPosition.compare("RIGHT_BACK") == 0  )
            {
                position = SwerveModule::ModuleID::RIGHT_BACK;
                networkTableName  += " - Right_Back Module";
            }
            else 
            {
                string msg = "unknown position ";
                msg += attr.name();
                Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("SwerveChassisXmlParser"),string("ParseXML"), msg );
                hasError = true;
            }
        }
        else if (  attrName.compare("turn_p") == 0 )
        {
        	turnP = attr.as_double();
        }
        else if (  attrName.compare("turn_i") == 0 )
        {
        	turnI = attr.as_double();
        }
        else if (  attrName.compare("turn_d") == 0 )
        {
        	turnD = attr.as_double();
        }
        else if (  attrName.compare("turn_f") == 0 )
        {
        	turnF = attr.as_double();
        }
        else if (  attrName.compare("turn_nominal_val") == 0 )
        {
        	turnNominalVal = attr.as_double();
        }
        else if (  attrName.compare("turn_peak_val") == 0 )
        {
        	turnPeakVal = attr.as_double();
        }
        else if (  attrName.compare("turn_max_acc") == 0 )
        {
        	turnMaxAcc = attr.as_double();
        }
        else if (  attrName.compare("turn_cruise_vel") == 0 )
        {
        	turnCruiseVel = attr.as_double();
        }
        else if (attrName.compare("countsOnTurnEncoderPerDegreesOnAngleSensor") == 0)
        {
            countsOnTurnEncoderPerDegreesOnAngleSensor = attr.as_double();
        }
        else   // log errors
        {
            string msg = "unknown attribute ";
            msg += attr.name();
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("SwerveChassisXmlParser"), string("ParseXML"), msg );
            hasError = true;
        }
    }


    // Process child element nodes
    DragonCanCoder* turnsensor = nullptr;
    IDragonMotorControllerMap motors;

    unique_ptr<MotorXmlParser> motorXML = make_unique<MotorXmlParser>();
    unique_ptr<CancoderXmlParser> cancoderXML = make_unique<CancoderXmlParser>();

    for (xml_node child = SwerveModuleNode.first_child(); child; child = child.next_sibling())
    {
        string childName (child.name());
    	if ( childName.compare("motor") == 0 )
    	{
            auto motor = motorXML.get()->ParseXML(networkTableName, child);
            if ( motor.get() != nullptr )
            {
                motors[ motor.get()->GetType() ] =  motor ;
            }
    	}
    	else if ( childName.compare("cancoder") == 0 )
    	{
            turnsensor = cancoderXML.get()->ParseXML(networkTableName, child);
    	}
    	else  // log errors
    	{
            string msg = "unknown child ";
            msg += child.name();
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("SwerveModuleXmlParser"), string("ParseXML"), msg );
            hasError = true;
    	}
    }


    // create chassis instance
    if ( !hasError )
    {
        module = ChassisFactory::GetChassisFactory()->CreateSwerveModule(position, 
                                                                         motors, 
                                                                         turnsensor, 
                                                                         turnP,
                                                                         turnI,
                                                                         turnD,
                                                                         turnF,
                                                                         turnNominalVal,
                                                                         turnPeakVal,
                                                                         turnMaxAcc,
                                                                         turnCruiseVel,
                                                                         countsOnTurnEncoderPerDegreesOnAngleSensor );
    }
    return module;
}

