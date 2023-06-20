#include "Robot.h"

class PID
{
    private:
        double maximumAllowableError;
        double minVelocity; 
        double maxVelocity;
        double minAcceleration;
        double maxAcceleration; 
        double kP;
        double kI;
        double kIMaxEffect;
        double lastVal = 0;
        double runningIntegral = 0;

    public:
        PID(double maximumAllowableError_, double minVelocity_, double maxVelocity_, double minAcceleration_, double maxAcceleration_, double kP_, double kI_, double kIMaxEffect_);
        double Calculate(double error, double elapsedTime);
};