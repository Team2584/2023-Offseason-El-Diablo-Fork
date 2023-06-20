#include "Robot.h"

//Holding Elevator at Position PID
#define ELEVHOLDFF 0
#define ELEVKP 0.08
#define ELEVKI 0
#define ELEVKIMAX 0.1
#define ALLOWABLE_ERROR_ELEV 0.6
#define ELEVMAX_SPEED 0.9
#define ELEVMAX_ACCELERATION 5

class ElevatorLift
{
    private:
        double runningIntegral = 0;
        double lastSpeed = 0;
        double lastHeight = 0;

    public:
        rev::CANSparkMax *winchL, *winchR;
        rev::SparkMaxRelativeEncoder *winchEncoder;
        TimeOfFlight *tofSensor;

        ElevatorLift(rev::CANSparkMax *winchL_, rev::CANSparkMax *winchR_, TimeOfFlight *tofSensor_);
        void ResetElevatorEncoder();
        double winchEncoderReading();
        double TOFSReading();
        double TOFSElevatorHeight();
        void StopElevator();
        void MoveElevatorPercent(double percent);
        void StartPIDLoop();
        bool SetElevatorHeightPID(double height, double elapsedTime);
};