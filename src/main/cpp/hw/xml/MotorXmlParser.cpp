
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

/// @brief XML parsing  leverages the 3rd party Open Source Pugixml library (https://pugixml.org/).

// C++ Includes
#include <memory>
#include <string>

// FRC includes

// Team 302 includes
#include <hw/DragonTalonSRX.h>
#include <hw/factories/DragonMotorControllerFactory.h>
#include <hw/interfaces/IDragonMotorController.h>
#include <utils/HardwareIDValidation.h>
#include <utils/Logger.h>
#include <hw/xml/MotorXmlParser.h>
#include <hw/DistanceAngleCalcStruc.h>


// Third Party Includes
#include <pugixml/pugixml.hpp>

using namespace frc;
using namespace pugi;
using namespace std;

/// @brief: Parse a motor XML element and create a motor controller from its definition.
/// @param [in] xml_node motor node in the xml file
/// @return shared_ptr<IDragonMotorController> motor controller or nullptr if there is an error in the definition
shared_ptr<IDragonMotorController> MotorXmlParser::ParseXML
(
    string              networkTableName,
    xml_node            motorNode
)
{
    // initialize the output
    shared_ptr<IDragonMotorController> controller;

    // initialize attributes to default values
    int canID = 0;
	int pdpID = -1;
	string usage;
    bool inverted = false;
    bool sensorInverted = false;
    ctre::phoenix::motorcontrol::FeedbackDevice  feedbackDevice = ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder;
    
    DistanceAngleCalcStruc calcStruc;
    calcStruc.countsPerRev = 0;
    calcStruc.countsPerInch = 0;
    calcStruc.countsPerDegree = 0;
    calcStruc.gearRatio = 1.0;
    calcStruc.diameter = 1.0;

    bool brakeMode = false;
    int follow = -1;
    int peakCurrentDuration = 0;
    int continuousCurrentLimit = 0;
    int peakCurrentLimit = 0;
    bool enableCurrentLimit = false;
    bool forwardLimitSwitch = false;
    bool forwardLimitSwitchNormallyOpen = false;
    bool reverseLimitSwitch = false;
    bool reverseLimitSwitchNormallyOpen = false;
    double voltageCompensationSaturation = 12.0;
    bool enableVoltageCompensation = false;
    IDragonMotorController::MOTOR_TYPE motortype = IDragonMotorController::NONE;
    std::string canBusName("rio");


    string mtype;


    bool hasError = false;

    for (xml_attribute attr = motorNode.first_attribute(); attr && !hasError; attr = attr.next_attribute())
    {
        if ( strcmp( attr.name(), "usage" ) == 0 )
        {
             usage = attr.value();
        }
		// CAN ID 0 thru 62 are valid
        else if ( strcmp( attr.name(), "canId" ) == 0 )
        {
            canID = attr.as_int();
            hasError = HardwareIDValidation::ValidateCANID( canID, string( "MotorXmlParser::ParseXML" ) );
        }
        else if (strcmp(attr.name(), "canbus") == 0)
        {
            canBusName = attr.as_string();
        }
		// PDP ID 0 thru 15 are valid
        else if ( strcmp( attr.name(), "pdpID" ) == 0 )
        {
            pdpID = attr.as_int();
            hasError = HardwareIDValidation::ValidatePDPID( pdpID, string( "MotorXmlParser::ParseXML" ) );
        }
		// type:  cantalon, sparkmax_brushless and sparkmax_brushed are valid
        else if ( strcmp( attr.name(), "type" ) == 0 )
        {
            mtype = attr.value();
        }
		// inverted
        else if ( strcmp( attr.name(), "inverted" ) == 0 )
        {
            inverted = attr.as_bool();
        }
		// sensor inverted
        else if ( strcmp( attr.name(), "sensorInverted" ) == 0 )
        {
            sensorInverted = attr.as_bool();
        }
		else if ( strcmp( attr.name(), "feedbackDevice" ) == 0 )
        {
            auto val = string( attr.value() );
            if ( val.compare( "QUADENCODER") == 0 )
            {
                feedbackDevice = ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder;
            }
            else if ( val.compare( "ANALOG") == 0 )
            {
                feedbackDevice = ctre::phoenix::motorcontrol::FeedbackDevice::Analog;
            }
            else if ( val.compare( "TACHOMETER") == 0 )
            {
                feedbackDevice = ctre::phoenix::motorcontrol::FeedbackDevice::Tachometer;
            }
            else if ( val.compare( "PULSEWIDTHENCODERPOSITION") == 0 )
            {
                feedbackDevice = ctre::phoenix::motorcontrol::FeedbackDevice::PulseWidthEncodedPosition;
            }
            else if ( val.compare( "SENSORSUM") == 0 )
            {
                feedbackDevice = ctre::phoenix::motorcontrol::FeedbackDevice::SensorSum;
            }
            else if ( val.compare( "SENSORDIFFERENCE") == 0 )
            {
                feedbackDevice = ctre::phoenix::motorcontrol::FeedbackDevice::SensorDifference;
            }
            else if ( val.compare( "REMOTESENSOR0") == 0 )
            {
                feedbackDevice = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0;
            }
            else if ( val.compare( "REMOTESENSOR1") == 0 )
            {
                feedbackDevice = ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor1;
            }
            else if ( val.compare( "SOFTWAREEMULATEDSENSOR") == 0 )
            {
                feedbackDevice = ctre::phoenix::motorcontrol::FeedbackDevice::SoftwareEmulatedSensor;
            }
            else if ( val.compare( "CTRE_MAGENCODER_ABSOLUTE") == 0 )
            {
                feedbackDevice = ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Absolute;
            }
            else if ( val.compare( "CTRE_MAGENCODER_RELATIVE") == 0 )
            {
                feedbackDevice = ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative;
            }
            else if ( val.compare( "INTERNAL") == 0 )
            {
                feedbackDevice = ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor;
            }
            else if ( val.compare( "NONE") == 0 )
            {
                feedbackDevice = ctre::phoenix::motorcontrol::FeedbackDevice::None;
            }
            else 
            {
                string msg = "Invalid feedback device ";
                msg += val;
                Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("MotorXmlParser "), string("ParseXML "), msg );
            }
        }
        else if ( strcmp( attr.name(), "motorType" ) == 0 )
        {
            auto val = string( attr.value() );
            if ( val.compare( "FALCON500") == 0 )
            {
                motortype = IDragonMotorController::FALCON500;
            }
            else if ( val.compare( "NEOMOTOR") == 0 )
            {
                motortype = IDragonMotorController::NEOMOTOR;
            }
            else if ( val.compare( "NEO500MOTOR") == 0 )
            {
                motortype = IDragonMotorController::NEO500MOTOR;
            }
            else if ( val.compare( "CLIMMOTOR") == 0 )
            {
                motortype = IDragonMotorController::CIMMOTOR;
            }
            else if ( val.compare( "MINICIMMOTOR") == 0 )
            {
                motortype = IDragonMotorController::MINICIMMOTOR;
            }
            else if ( val.compare( "BAGMOTOR") == 0 )
            {
                motortype = IDragonMotorController::BAGMOTOR;
            }
            else if ( val.compare( "PRO775") == 0 )
            {
                motortype = IDragonMotorController::PRO775;
            }
            else if ( val.compare( "ANDYMARK9015") == 0 )
            {
                motortype = IDragonMotorController::ANDYMARK9015;
            }
            else if ( val.compare( "ANDYMARKNEVEREST") == 0 )
            {
                motortype = IDragonMotorController::ANDYMARKNEVEREST;
            }
            else if ( val.compare( "ANDYMARKRS775125") == 0 )
            {
                motortype = IDragonMotorController::ANDYMARKRS775125;
            }
            else if ( val.compare( "TETRIXMAXTORQUENADOMOTOR") == 0 )
            {
                motortype = IDragonMotorController::TETRIXMAXTORQUENADOMOTOR;
            }
            else if ( val.compare( "ANDYMARKREDLINEA") == 0 )
            {
                motortype = IDragonMotorController::ANDYMARKREDLINEA;
            }
            else if ( val.compare( "REVROBOTICSHDHEXMOTOR") == 0 )
            {
                motortype = IDragonMotorController::REVROBOTICSHDHEXMOTOR;
            }
            else if ( val.compare( "BANEBOTSRS77518V") == 0 )
            {
                motortype = IDragonMotorController::BANEBOTSRS77518V;
            }
            else if ( val.compare( "BANEBOTSRS550") == 0 )
            {
                motortype = IDragonMotorController::BANEBOTSRS550;
            }
            else if ( val.compare( "MODERNROBOTICS12VDCMOTOR") == 0 )
            {
                motortype = IDragonMotorController::MODERNROBOTICS12VDCMOTOR;
            }
            else if ( val.compare( "JOHNSONELECTRICALGEARMOTOR") == 0 )
            {
                motortype = IDragonMotorController::JOHNSONELECTRICALGEARMOTOR;
            }
            else if ( val.compare( "NONE") == 0 )
            {
                motortype = IDragonMotorController::NONE;
            }
            else 
            {
                string msg = "Invalid motor type";
                msg += val;
                Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("MotorXmlParser "), string("ParseXML "), msg );
            }
        }

		// counts per revolution
        else if ( strcmp( attr.name(), "countsPerRev" ) == 0 )
        {
            calcStruc.countsPerRev = attr.as_int();
        }
        else if ( strcmp( attr.name(), "countsPerInch" ) == 0 )
        {
            calcStruc.countsPerInch = attr.as_double();
        }
        else if ( strcmp( attr.name(), "countsPerDegree" ) == 0 )
        {
            calcStruc.countsPerDegree = attr.as_double();
        }
		// gear ratio
        else if ( strcmp( attr.name(), "gearRatio" ) == 0 )
        {
            calcStruc.gearRatio = attr.as_float();
        }

		// brake mode (or coast)
        else if ( strcmp( attr.name(), "brakeMode" ) == 0 )
        {
            brakeMode = attr.as_bool();
        }
		// follow (existing CAN id of the master motor)
        else if ( strcmp( attr.name(), "follow") == 0 )
        {
            follow = attr.as_int();
        }
		// peak current duration (cantalon)
        else if ( strcmp( attr.name(), "peakCurrentDuration") == 0 )
        {
            peakCurrentDuration = attr.as_int();
        }
		// continuous current duration (cantalon)
        else if ( strcmp( attr.name(), "continuousCurrentLimit") == 0 )
        {
            continuousCurrentLimit = attr.as_int();
        }
		// peak current limit (cantalon)
        else if ( strcmp( attr.name(), "peakCurrentLimit") == 0 )
        {
            peakCurrentLimit = attr.as_int();
        }
		// enable current limiting
        else if ( strcmp( attr.name(), "currentLimiting") == 0 )
        {
            enableCurrentLimit = attr.as_bool();
        }
        else if ( strcmp( attr.name(), "forwardlimitswitch") == 0 )
        {
            forwardLimitSwitch = attr.as_bool();
        }
        else if ( strcmp( attr.name(), "forwardlimitswitchopen") == 0 )
        {
            forwardLimitSwitchNormallyOpen = attr.as_bool();
        }        
        else if ( strcmp( attr.name(), "reverselimitswitch") == 0 )
        {
            reverseLimitSwitch = attr.as_bool();
        }
        else if ( strcmp( attr.name(), "reverselimitswitchopen") == 0 )
        {
            reverseLimitSwitchNormallyOpen = attr.as_bool();
        }
        else if (strcmp( attr.name(), "voltageCompensationSaturation"))
        {
            voltageCompensationSaturation = attr.as_double();
        }
        else if (strcmp( attr.name(), "voltageCompensationEnable"))
        {
            enableVoltageCompensation = attr.as_bool();
        }
        else
        {
            string msg = "unknown attribute ";
            msg += attr.name();
            Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("MotorXmlParser"), string("ParseXML"), msg );
            hasError = true;
        }
    }

    if ( !hasError )
    {
		pdpID = ( pdpID < 0 ) ? canID : pdpID;
        controller = DragonMotorControllerFactory::GetInstance()->CreateMotorController( networkTableName,
                                                                                         mtype,
                                                                                         canID,
                                                                                         canBusName,
                                                                                         pdpID,
                                                                                         usage,
                                                                                         inverted,
                                                                                         sensorInverted,
                                                                                         feedbackDevice,
                                                                                         calcStruc,
                                                                                         brakeMode,
                                                                                         follow,
                                                                                         peakCurrentDuration,
                                                                                         continuousCurrentLimit,
                                                                                         peakCurrentLimit,
                                                                                         enableCurrentLimit,
                                                                                         forwardLimitSwitch,
                                                                                         forwardLimitSwitchNormallyOpen,
                                                                                         reverseLimitSwitch,
                                                                                         reverseLimitSwitchNormallyOpen,
                                                                                         voltageCompensationSaturation,
                                                                                         enableVoltageCompensation,
                                                                                         motortype);
    }
    return controller;
}


