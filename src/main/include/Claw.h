#include "Robot.h"
#include "PicoColorSensor.h"

#define CLAWKP 0.1
#define CLAWKI 0.1
#define CLAWKIMAX 0.3
#define ALLOWABLE_ERROR_CLAW 1
#define CLAWMAX_SPEED 0.8
#define CLAWMAX_ACCELERATION 5

#define WRISTFF 0
#define WRISTKP 0.4
#define WRISTKI 0
#define WRISTKIMAX 0.1
#define ALLOWABLE_ERROR_WRIST 0.05  
#define WRISTMAX_SPEED 0.5
#define WRISTMAX_ACCELERATION 3

class Claw
{
    private:
        double runningClawIntegral = 0;
        double lastClawSpeed = 0;
        double runningWristIntegral = 0;
        double lastWristSpeed = 0;
        double initalClawPIDTime = 0;
        bool usingBeamBreaks = true;

    public:
        rev::CANSparkMax *wristMotor;
        rev::CANSparkMax clawMotor;
        rev::SparkMaxRelativeEncoder *wristEncoder, *clawEncoder; 
        frc::DigitalInput *regualarBreambreak;
        frc::DigitalInput *substationBreambreak;
        rev::SparkMaxAbsoluteEncoder *magEncoder;
        rev::SparkMaxLimitSwitch closedLimit, openLimit;
        pico::ColorSensor *colorSensor;

        Claw(rev::CANSparkMax *wrist);
        
        double WristEncoderReading();
        double MagEncoderReading();
        void MoveWristPercent(double percent);
        bool PIDWrist(double point, double elapsedTime);
        bool PIDWristDown(double elapsedTime);
        bool PIDWristUp(double elapsedTime);
        double ClawEncoderReading();
        void MoveClawPercent(double percent);
        void BeginClawPID();
        bool PIDClaw(double point, double elapsedTime);
        void ResetClawEncoder();
        void ResetClawEncoder(double val);
        bool OpenClaw(double elapsedTime);
        bool CloseClaw(double elapsedTime);
        bool GetUsingBeamBreaks();
        void SetUsingBeamBreaks(bool shouldUse);
        bool ObjectInClaw();
        bool ObjectInClawSubstation();
        bool IsObjectCone();
};