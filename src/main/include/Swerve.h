#include "Robot.h"

//Find offsets with bevel gears on the right side of the drive train
#define FL_WHEEL_OFFSET 0.474 // FB 0.11475   
#define FR_WHEEL_OFFSET 0.426 // FB 0.082
#define BR_WHEEL_OFFSET 0.865 // FB 0.841
#define BL_WHEEL_OFFSET 0.147 // FB 0.0312

//Assuming a rectangular drive train (input distance between center of wheels)
#define DRIVE_LENGTH 0.5906_m 
#define DRIVE_WIDTH 0.489_m 

//Encoder constants
#define DRIVE_MOTOR_GEAR_RATIO 7.36 
#define DRIVE_MOTOR_CIRCUMFERENCE 0.10469983 * M_PI
#define SPIN_MOTOR_GEAR_RATIO 15.43

//Numerical Constants TODO CHANGE
#define SWERVE_DRIVE_MAX_MPS 4.247 // unneccesary
#define SWERVE_DRIVE_MAX_ACCELERATION 3 // unneccesary
#define MAX_RADIAN_PER_SECOND 4 // unneccesary

// Swerve Module Wheel Spin PID Values
#define WHEEL_SPIN_KP 1
#define WHEEL_SPIN_KI 0
#define WHEEL_SPIN_KI_MAX 0.03

//PID VALUES FOR DRIVE TO POSE ODOMETRY
#define O_TRANSLATION_KP 1.1
#define O_TRANSLATION_KI 0.02
#define O_TRANSLATION_KI_MAX 0.03
#define O_TRANSLATION_MAX_SPEED 0.2
#define O_TRANSLATION_MAX_ACCEL 3
#define O_SPIN_KP 0.5
#define O_SPIN_KI 0.01
#define O_SPIN_KI_MAX 0.05
#define O_SPIN_MAX_SPEED 0.4
#define O_SPIN_MAX_ACCEL 1.5  
#define O_ALLOWABLE_ERROR_TRANSLATION 0.05
#define O_ALLOWABLE_ERROR_ROTATION 0.05

//PID VALUES FOR SPLINE
#define S_TRANSLATION_KP 6
#define S_TRANSLATION_KI 0
#define S_TRANSLATION_KD 0
#define S_TRANSLATION_KI_MAX 0.1
#define S_TRANSLATION_MAX_SPEED 1.0
#define S_TRANSLATION_MAX_ACCEL 1.5
#define S_SPIN_KP 4.4
#define S_SPIN_KI 0.0
#define S_SPIN_KI_MAX 0.0
#define S_SPIN_KD 0
#define S_SPIN_MAX_SPEED 4.5
#define S_SPIN_MAX_ACCEL 1.5
#define S_ALLOWABLE_ERROR_TRANSLATION 0.05
#define S_ALLOWABLE_ERROR_ROTATION 0.1

//PID VALUES FOR TURN TO Pole WITH LIMELIGHT
#define P_STRAFE_KP 0.6
#define P_STRAFE_KI 0.04
#define P_STRAFE_KI_MAX 0.06
#define P_STRAFE_MAX_SPEED 0.3
#define P_STRAFE_MAX_ACCEL 3
#define P_ALLOWABLE_ERROR_STRAFE 0.05 // in pixel values from -1 to 1
#define P_TRANS_KP 3.5
#define P_TRANS_KI 0.06
#define P_TRANS_KI_MAX 0.06
#define P_TRANS_MAX_SPEED 0.2
#define P_TRANS_MAX_ACCEL 3 
#define P_ALLOWABLE_ERROR_TRANS 0.01
#define P_SPIN_KP 2.7
#define P_SPIN_KI 0
#define P_SPIN_KI_MAX 0.1
#define P_SPIN_MAX_SPEED 0.2
#define P_SPIN_MAX_ACCEL 0.6
#define P_ALLOWABLE_ERROR_ROTATION 0.05 // in pixel values from -1 to 1


// Going to an April Tag
#define A_ALLOWABLE_ERROR_TRANSLATION 0.10
#define A_TRANSLATION_KP 0.75
#define A_TRANSLATION_KI 0.03
#define A_TRANSLATION_KI_MAX 0.03
#define A_TRANSLATION_MAX_SPEED 0.2
#define A_TRANSLATION_MAX_ACCEL 3
// #define A_STRAFE_KP 0.7
// #define A_STRAFE_KI 0
// #define A_STRAFE_KI_MAX 0.03
// #define A_STRAFE_MAX_SPEED 0.2
// #define A_STRAFE_MAX_ACCEL 3

// Grabbing a Cone
#define C_ALLOWABLE_ERROR_TRANSLATION 0.05
#define C_TRANSLATION_KP 1.1
#define C_TRANSLATION_KI 0.02
#define C_TRANSLATION_KI_MAX 0.03
#define C_TRANSLATION_MAX_SPEED 0.3
#define C_TRANSLATION_MAX_ACCEL 3
#define C_STRAFE_KP 0.7
#define C_STRAFE_KI 0
#define C_STRAFE_KI_MAX 0.03
#define C_STRAFE_MAX_SPEED 0.2
#define C_STRAFE_MAX_ACCEL 3
#define C_SPIN_KP 0.56
#define C_SPIN_KI 0.01
#define C_SPIN_KD 0.01
#define C_SPIN_KI_MAX 0.05
#define C_SPIN_MAX_SPEED 0.4
#define C_SPIN_MAX_ACCEL 3
#define C_ALLOWABLE_ERROR_ROTATION 0.03


/**
 * A Swerve Drive has 4 Swerve Modules, each of which is defined as an object of the class SwerveModule
 * In general, this class has functions allowing us to control each swerve module individually.
 */
class SwerveModule
{
    private:
        // Instance Variables for each swerve module
        ctre::phoenix::motorcontrol::can::TalonFX *driveMotor; /* The motor responsible for actually driving the wheel*/
        rev::CANSparkMax *spinMotor; /* The motor responsible for "spinning" the wheel left to right to change direction*/
        rev::SparkMaxRelativeEncoder *spinRelativeEncoder; /* The relative encoder built into the spinMotor */
        double encoderOffset;       /* Offset in magnetic encoder from 0 facing the front of the robot */
        double driveEncoderInitial; /* Used to computer the change in encoder tics, aka motor rotation */
        double spinEncoderInitialHeading; /* Initial Heading of Relative Spin Encoder used for zeroing*/
        double spinEncoderInitialValue; /* Initial Value of Relative Spin Encoder used for zeroing*/
        double runningIntegral = 0; /* Running sum of errors for integral in PID */

    public:
        frc::DutyCycleEncoder *magEncoder;

        SwerveModule(ctre::phoenix::motorcontrol::can::TalonFX *driveMotor_,
                    rev::CANSparkMax *spinMotor_, frc::DutyCycleEncoder *magEncoder_,
                    double encoderOffset_);

        double GetModuleHeading();
        void ResetEncoders();
        double GetDriveEncoderMeters();
        double GetDriveVelocity();
        double GetSpinEncoderRadians();
        void StopSwerveModule();
        SwerveModuleState GetSwerveModuleState();
        SwerveModulePosition GetSwerveModulePosition();
        void DriveSwerveModulePercent(double driveSpeed, double targetAngle);
        void DriveSwerveModuleMeters(double driveSpeed, double targetAngle);
};

/**
 * This class has functions to control all the autonomous and drive control functionality of a swerve drive
 * 
 * IMPORTANT: This class uses 3 different types of odometry. Each odometry is tracked separately and is used in different scenarios.
 * odometry: Basic odometry reading that only uses wheel encoders. As the match continues it slowly drifts and becomes less accurate.
 * coneOdometry: Returns position on the field in relation to the nearest cone in vision. 
 * tagOdometry: Returns position on the field in relation to the nearest aprilTag in vision.
 */
class SwerveDrive
{
    private:
        Pigeon2 *pigeonIMU;
        Translation2d m_frontLeft, m_frontRight, m_backLeft, m_backRight; /* Location of each wheel in relation to the center of the robot */
        SwerveDriveKinematics<4> kinematics; /* A WPI struct which contains all wheels of a swerve drive*/
        SwerveDriveOdometry<4> *odometry, *coneOdometry, *tagOdometry; /* An odometry class which returns the position of the robot using the method specified*/
        std::queue<pathplanner::PathPlannerTrajectory> trajectoryList; /* A list of all trajectories to be run in an auton*/
        pathplanner::PathPlannerTrajectory trajectory; /* The curent trajectory*/

        bool isRedAlliance = true; 

        //Variables for PID
        double lastX = 0; /* The speed of the Swerve in the X direction last update loop*/
        double lastY = 0;
        double lastSpin = 0;

        double runningIntegralX = 0; /* The running tally of error in the X direction, aka the Integral used for pId */
        double runningIntegralY = 0;
        double runningIntegralSpin = 0;

        //Variables for Balancing
        double lastGyroRot = 0; 
        double lastGyroVel = 0;
        int currentBalanceDrivingDirection = 1;
        int balanceFacingDirection = 1;
        bool waitingOnPlatform = false;

    public:
        SwerveModule *FLModule, *FRModule, *BRModule, *BLModule;
        double pigeon_initial;

        SwerveDrive(ctre::phoenix::motorcontrol::can::TalonFX *_FLDriveMotor,
            rev::CANSparkMax *_FLSpinMotor, frc::DutyCycleEncoder *_FLMagEncoder,
            double _FLEncoderOffset, ctre::phoenix::motorcontrol::can::TalonFX *_FRDriveMotor,
            rev::CANSparkMax *_FRSpinMotor, frc::DutyCycleEncoder *_FRMagEncoder,
            double _FREncoderOffset, ctre::phoenix::motorcontrol::can::TalonFX *_BRDriveMotor,
            rev::CANSparkMax *_BRSpinMotor, frc::DutyCycleEncoder *_BRMagEncoder,
            double _BREncoderOffset, ctre::phoenix::motorcontrol::can::TalonFX *_BLDriveMotor,
            rev::CANSparkMax *_BLSpinMotor, frc::DutyCycleEncoder *_BLMagEncoder,
            double _BLEncoderOffset, Pigeon2 *_pigeonIMU, double robotStartingRadian);

        double VelocityToPercent(double velocity);
        double PercentToVelocity(double percent);
        double AngularVelocityToPercent(double velocity);
        double AngularPercentToVelocity(double percent);
        double GetIMURadians();
        void ResetOdometry();
        void ResetOdometry(Pose2d position);
        void ResetConeOdometry();
        void ResetConeOdometry(Pose2d position);
        void ResetTagOdometry();
        void ResetTagOdometry(Pose2d position);
        void UpdateOdometry(units::second_t currentTime);
        void UpdateConeOdometry();
        void UpdateTagOdometry();
        void AddPositionEstimate(Translation2d poseEstimate, units::second_t timeOfEstimate);
        void AddPositionEstimate(Pose2d poseEstimate, units::second_t timeOfEstimate);
        Pose2d GetPose();
        Pose2d GetConeOdometryPose();
        Pose2d GetTagOdometryPose();
        void SetModuleStates(std::array<SwerveModuleState, 4> states);
        void DriveSwervePercentNonFieldOriented(double STRAFE_Drive_Speed, double FWD_Drive_Speed, double Turn_Speed);
        void DriveSwervePercent(double STRAFE_Drive_Speed, double FWD_Drive_Speed, double Turn_Speed);
        void DriveSwerveMetersAndRadians(double STRAFE_Drive_Speed, double FWD_Drive_Speed, double Turn_Speed);
        void BeginPIDLoop();
        bool DriveToPose(Pose2d target, double elapsedTime);
        bool DriveToPoseTag(Pose2d target, double elapsedTime);
        bool DriveToPoseConeOdometry(Pose2d target, double elapsedTime);
        bool DriveToPose(Pose2d current, Pose2d target, double elapsedTime,
                        double translationMaxSpeed, double translationMaxAccel, double allowableErrorTranslation,
                        double translationP, double translationI, double translationIMaxEffect,
                        double rotationMaxSpeed, double rotationMaxAccel, double allowableErrorRotation,
                        double rotationP, double rotationI, double rotationIMaxEffect, double useWeirdMinSpeedThing);
        double TurnToPointDesiredSpin(Translation2d point, double elapsedTime, double allowableErrorRotation, double spinMaxSpeed, double spinMaxAccel, double spinP, double spinI);
        void ResetTrajectoryList();
        void SetAllianceColorRed();
        void SetAllianceColorBlue();
        void InitializeTrajectory(string trajectoryString);
        void InitializeTrajectory(string trajectoryString, units::meters_per_second_t velocity, units::meters_per_second_squared_t acceleration);
        void SetNextTrajectory();
        bool FollowTrajectory(units::second_t time, double elapsedTime);
        bool TurnToPixel(double offset, double elapsedTime);
        bool TurnToPixelCone(double offset, double elapsedTime);
        bool StrafeToPole(double offsetX, double offsetY, double xGoal, double yGoal, double elapsedTime);
        bool StrafeToPoleFast(double offsetX, double offsetY, double xGoal, double yGoal, double elapsedTime);
        void StartBalance();
        double GetIMURoll();
        bool BalanceOnCharger(double elapsedTime);
        void StrafeLock();
};