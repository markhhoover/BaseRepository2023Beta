
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

#pragma once
#include <memory>
#include <string>

#include <frc/BuiltInAccelerometer.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModuleState.h>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>


#include <chassis/DragonTargetFinder.h>
#include <chassis/IChassis.h>
#include <chassis/PoseEstimatorEnum.h>
#include <chassis/swerve/SwerveModule.h>
#include <hw/DragonLimelight.h>
#include <hw/DragonPigeon.h>
#include <hw/factories/PigeonFactory.h>

class SwerveChassis : public IChassis
{
    public:

        /// @brief Construct a swerve chassis
        /// @param [in] std::shared_ptr<SwerveModule>           frontleft:          front left swerve module
        /// @param [in] std::shared_ptr<SwerveModule>           frontright:         front right swerve module
        /// @param [in] std::shared_ptr<SwerveModule>           backleft:           back left swerve module
        /// @param [in] std::shared_ptr<SwerveModule>           backright:          back right swerve module
        /// @param [in] units::length::inch_t                   wheelDiameter:      Diameter of the wheel
        /// @param [in] units::length::inch_t                   wheelBase:          distance between the front and rear wheels
        /// @param [in] units::length::inch_t                   track:              distance between the left and right wheels
        /// @param [in] units::velocity::meters_per_second_t    maxSpeed:           maximum linear speed of the chassis 
        /// @param [in] units::radians_per_second_t             maxAngularSpeed:    maximum rotation speed of the chassis 
        /// @param [in] double                                  maxAcceleration:    maximum acceleration in meters_per_second_squared
        SwerveChassis
        ( 
			std::shared_ptr<SwerveModule>                               frontLeft, 
			std::shared_ptr<SwerveModule>                               frontRight,
			std::shared_ptr<SwerveModule>                               backLeft, 
			std::shared_ptr<SwerveModule>                               backRight, 
            units::length::inch_t                                       wheelDiameter,
			units::length::inch_t                                       wheelBase,
			units::length::inch_t                                       track,
            double                                                      odometryComplianceCoefficient,
			units::velocity::meters_per_second_t                        maxSpeed,
			units::radians_per_second_t                                 maxAngularSpeed,
			units::acceleration::meters_per_second_squared_t            maxAcceleration,
			units::angular_acceleration::radians_per_second_squared_t   maxAngularAcceleration,
            std::string                                                 networkTableName,
            std::string                                                 controlFileName
        );

	    ~SwerveChassis() noexcept override = default;


        /// @brief Align all of the swerve modules to point forward
        void ZeroAlignSwerveModules();

        /// @brief Drive the chassis
        /// @param [in] double  drivePercent:   forward/reverse percent output (positive is forward)
        /// @param [in] double  steerPercent:   left/right percent output (positive is left)
        /// @param [in] double  rotatePercent:  Rotation percent output around the vertical (Z) axis; (positive is counter clockwise)
        /// @param [in] bool    fieldRelative:  true: movement is based on the field (e.g., push it goes away from the driver regardless of the robot orientation),
        ///                                     false: direction is based on robot front/back
        /// @param [in] bool    useTargetHeading:  true: constrain the heading based on the stored target heading,
        ///                                     false: don't contrain the heading
        void Drive
        (
            double drivePercent, 
            double steerPercent, 
            double rotatePercent, 
            CHASSIS_DRIVE_MODE  mode,
            HEADING_OPTION      headingOption
        );

        /// @brief Drive the chassis
        /// @param [in] frc::ChassisSpeeds  speeds:         kinematics for how to move the chassis
        /// @param [in] bool                fieldRelative:  true: movement is based on the field (e.g., push it goes away from the driver regardless of the robot orientation),
        ///                                                 false: direction is based on robot front/back
        /// @param [in] bool                useTargetHeading:  true: constrain the heading based on the stored target heading,
        ///                                                 false: don't contrain the heading
        void Drive
        (
            frc::ChassisSpeeds speeds, 
            CHASSIS_DRIVE_MODE  mode,
            HEADING_OPTION      headingOption
        ) override;
        void Drive
        (
            frc::ChassisSpeeds            chassisSpeeds
        ) override;

        /// @brief update the chassis odometry based on current states of the swerve modules and the pigeon
        void UpdateOdometry();

        /// @brief Provide the current chassis speed information
        frc::ChassisSpeeds GetChassisSpeeds() const;

        /// @brief Sets of the motor encoders to zero
        void SetEncodersToZero();

        /// @brief Get encoder values
        double GetEncoderValues(std::shared_ptr<SwerveModule> motor);

        /// @brief Reset the current chassis pose based on the provided pose and rotation
        /// @param [in] const Pose2d&       pose        Current XY position
        /// @param [in] const Rotation2d&   angle       Current rotation angle
        void ResetPose
        ( 
            const frc::Pose2d&       pose,
            const frc::Rotation2d&   angle
        );

        /// @brief Reset the current chassis pose based on the provided pose (the rotation comes from the Pigeon)
        /// @param [in] const Pose2d&       pose        Current XY position
        void ResetPose
        ( 
            const frc::Pose2d&       pose
        ) override;

        //static constexpr auto MaxSpeed = 3.0_mps; 
        //static constexpr units::angular_velocity::radians_per_second_t MaxAngularSpeed{wpi::numbers::pi};

        units::length::inch_t GetWheelDiameter() const {return m_wheelDiameter; }  
        units::length::inch_t GetWheelBase() const {return m_wheelBase; }  
        units::length::inch_t GetTrack() const {return m_track;}
        units::velocity::meters_per_second_t GetMaxSpeed() const {return m_maxSpeed;}
        units::radians_per_second_t GetMaxAngularSpeed() const {return m_maxAngularSpeed;}
        units::acceleration::meters_per_second_squared_t GetMaxAcceleration() const { return m_maxAcceleration; }
        units::angular_acceleration::radians_per_second_squared_t GetMaxAngularAcceleration() const { return m_maxAngularAcceleration; }
        std::shared_ptr<SwerveModule> GetFrontLeft() const { return m_frontLeft;}
        std::shared_ptr<SwerveModule> GetFrontRight() const { return m_frontRight;}
        std::shared_ptr<SwerveModule> GetBackLeft() const { return m_backLeft;}
        std::shared_ptr<SwerveModule> GetBackRight() const { return m_backRight;}
        //frc::SwerveDrivePoseEstimator<4> GetPoseEst() const { return m_poseEstimator; }  
        frc::Pose2d GetPose() const;
        units::angle::degree_t GetYaw() const override;

        //Dummy functions for IChassis Implementation
        inline IChassis::CHASSIS_TYPE GetType() const override {return IChassis::CHASSIS_TYPE::SWERVE;};
        inline void Initialize() override {};

        void RunWPIAlgorithm(bool runWPI ) { m_runWPI = runWPI; }
        void SetPoseEstOption(PoseEstimatorEnum opt ) { m_poseOpt = opt; }
        double GetodometryComplianceCoefficient() const { return m_odometryComplianceCoefficient; }
        void SetTargetHeading(units::angle::degree_t targetYaw) override;

        void ReZero();

        void DriveHoldPosition();

    private:
        frc::ChassisSpeeds GetFieldRelativeSpeeds
        (
            units::meters_per_second_t xSpeed,
            units::meters_per_second_t ySpeed,
            units::radians_per_second_t rot        
        );

        units::angular_velocity::degrees_per_second_t CalcHeadingCorrection
        (
            units::angle::degree_t  targetAngle,
            double                  kP
        );

        void CalcSwerveModuleStates
        (
            frc::ChassisSpeeds 
        );

        void AdjustRotToMaintainHeading
        (
            units::meters_per_second_t&  xspeed,
            units::meters_per_second_t&  yspeed,
            units::radians_per_second_t& rot 
        );        

        void AdjustRotToPointTowardGoal
        (
            frc::Pose2d                  robotPose,
            units::radians_per_second_t& rot
        );

        void DriveToPointTowardGoal
        (
            frc::Pose2d              robotPose,
            frc::Pose2d              goalPose, 
            units::meters_per_second_t&  xspeed,
            units::meters_per_second_t&  yspeed,
            units::radians_per_second_t& rot
            
        );
        units::angle::degree_t UpdateForPolarDrive
        (
            frc::Pose2d              robotPose,
            frc::Pose2d              goalPose,
            frc::Transform2d       wheelLoc,
            frc::ChassisSpeeds       speeds
        );

        std::shared_ptr<SwerveModule>                               m_frontLeft;
        std::shared_ptr<SwerveModule>                               m_frontRight;
        std::shared_ptr<SwerveModule>                               m_backLeft;
        std::shared_ptr<SwerveModule>                               m_backRight;

        frc::SwerveModuleState                                      m_flState;
        frc::SwerveModuleState                                      m_frState;
        frc::SwerveModuleState                                      m_blState;
        frc::SwerveModuleState                                      m_brState;
        
        units::length::inch_t                                       m_wheelDiameter;       
        units::length::inch_t                                       m_wheelBase;       
        units::length::inch_t                                       m_track;
        double                                                      m_odometryComplianceCoefficient;
        units::velocity::meters_per_second_t                        m_maxSpeed;
        units::radians_per_second_t                                 m_maxAngularSpeed;
        units::acceleration::meters_per_second_squared_t            m_maxAcceleration;
        units::angular_acceleration::radians_per_second_squared_t   m_maxAngularAcceleration;

        DragonPigeon*                                               m_pigeon;
        frc::BuiltInAccelerometer                                   m_accel;
        bool                                                        m_runWPI;
        PoseEstimatorEnum                                           m_poseOpt;
        frc::Pose2d                                                 m_pose;
        units::angle::degree_t                                      m_offsetPoseAngle;
        units::velocity::meters_per_second_t                        m_drive;
        units::velocity::meters_per_second_t                        m_steer;
        units::angular_velocity::radians_per_second_t               m_rotate;

        const double                                                m_deadband = 0.0;
        const units::angular_velocity::radians_per_second_t         m_angularDeadband = units::angular_velocity::radians_per_second_t(0.00);
        
        frc::Translation2d m_frontLeftLocation{0.381_m, 0.381_m};
        frc::Translation2d m_frontRightLocation{0.381_m, -0.381_m};
        frc::Translation2d m_backLeftLocation{-0.381_m, 0.381_m};
        frc::Translation2d m_backRightLocation{-0.381_m, -0.381_m};
        frc::SwerveDriveKinematics<4> m_kinematics{m_frontLeftLocation, 
                                                   m_frontRightLocation, 
                                                   m_backLeftLocation, 
                                                   m_backRightLocation};


        // Gains are for example purposes only - must be determined for your own robot!
        frc::SwerveDrivePoseEstimator<4> m_poseEstimator{ m_kinematics,
                                                          frc::Rotation2d{},
                                                          {m_frontLeft.get()->GetPosition(), m_frontRight.get()->GetPosition(), m_backLeft.get()->GetPosition(), m_backRight.get()->GetPosition()},
                                                          frc::Pose2d(),
                                                          {0.1, 0.1, 0.1},
                                                          {0.1, 0.1, 0.1}};

        const double kPMaintainHeadingControl = 1.5; //4.0, 3.0
        const double kPAutonSpecifiedHeading = 3.0;  // 4.0
        const double kPAutonGoalHeadingControl = 5.0;  // 2.0
        const double kPGoalHeadingControl = 6.0; //10.0, 7.0
        const double kPDistance = 10.0; //10.0, 7.0
        const double kIHeadingControl = 0.0; //not being used
        const double kDHeadingControl = 0.0; //not being used
        const double kFHeadingControl = 0.0; //not being used
        bool m_hold = false;
        units::angle::degree_t m_storedYaw;
        units::angular_velocity::degrees_per_second_t m_yawCorrection;

        DragonTargetFinder m_targetFinder;
        units::angle::degree_t m_targetHeading;
        DragonLimelight*        m_limelight;

        std::string             m_networkTableName;
        std::string             m_controlFileName;

        const units::length::inch_t m_shootingDistance = units::length::inch_t(105.0); // was 105.0


};