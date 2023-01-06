
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

// C++ Includes
#include <array>
#include <map>
#include <memory>
#include <string>

// Team 302 includes
#include <hw/interfaces/IDragonMotorController.h>
#include <hw/DistanceAngleCalcStruc.h>

// Third Party Includes
#include <ctre/phoenix/MotorControl/FeedbackDevice.h>

/// @brief  This is a singleton that creates motor motorControllers (IDragonMotorController).  This 
/// @brief  allows us to interact with motor motorControllers such as TalonSRX, Rev Spark Max without
///	@brief	actually knowing what type is actually being used.
class DragonMotorControllerFactory
{
    public:
		/// @brief 	This indicates the type of motor controller.  Most of our 
		/// @brief	code doesn't need/care about this, however since this is the
		///	@brief	factory, it is needed when constructing the object.
	    enum MOTOR_TYPE
    	{
        	TALONSRX,				/// Controller is a Cross the Road Electronics (CTRE) Talon SRX on the CAN network
			FALCON
    	};


        static DragonMotorControllerFactory* GetInstance();

		/// @brief  Create a motor controller from the inputs
		std::shared_ptr<IDragonMotorController> CreateMotorController
		(
			std::string										networkTableName,
			std::string                             		mtype,					/// Controller Type
			int 											canID,					/// CAN ID for the controller
			std::string	     								canBusName,
			int 											pdpID,					/// PDP slot the motor is on
			std::string                                     usage,					/// Motor usage (e.g. Front Left Drive Motor)
			bool 											inverted, 				/// Motor is inverted (positive values make the motor turn in reverse)or not
			bool 											sensorInverted,			/// Sensor direction matches motor direction or not 
			ctre::phoenix::motorcontrol::FeedbackDevice  	feedbackDevice,			/// Sensor type
			DistanceAngleCalcStruc							calcStruc,
			bool 											brakeMode,				/// brake mode using back emf to resist motion when power is not applied
			int 											followMotor,				/// CAN ID of the "master" motor controller if this is a follower motor controller (-1 indicates it is a master)
			int 											peakCurrentDuration,	/// peak current limit
			int 											continuousCurrentLimit,	/// continuous current limit
			int 											peakCurrentLimit,		/// amount of time the peak current can be achieved before limiting to the continuous current limit
			bool 											enableCurrentLimit, 	/// enable current limiting or not
			bool											forwardLimitSwitch,
			bool											forwardLimitSwitchNormallyOpen,
			bool											reverseLimitSwitch,
			bool											reverseLimitSwitchNormallyOpen,
			double											voltageCompensationSaturation,
			bool											enableVoltageCompensation,
			IDragonMotorController::MOTOR_TYPE                       				motorType
		);

	private:

		/// @brief		return motor controller
		/// @returns 	IDragonMotorController* 	may be nullptr if there isn't a controller with this usage.
		std::shared_ptr<IDragonMotorController> GetController
		(
			int													canID		/// Motor Controller CAN ID
		) const;

		void CreateTypeMap();

        DragonMotorControllerFactory();
        ~DragonMotorControllerFactory() = default;

        static DragonMotorControllerFactory*                                    m_instance;

		std::array<std::shared_ptr<IDragonMotorController>,63>				    m_canmotorControllers;
        std::map<std::string, DragonMotorControllerFactory::MOTOR_TYPE>         m_typeMap;


};

