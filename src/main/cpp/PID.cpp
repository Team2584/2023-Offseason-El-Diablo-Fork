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
    /**
    * Instantiates PID with various settings
    * @param maximumAllowableError_ if the current error is less than the allowable error, the PID will return 0 because it is "done"
    * @param minVelocity_ the minum value returned 
    * @param maxVelocity_ the maximum value returned
    * @param minAcceleration_ the maximum amount the value returned will DECREASE per second
    * @param maxAcceleration_ the maximum amount the value returned will INCREASE per second
    * @param kP_ the P value in PID
    * @param kI_ the I value in PID
    * @param kIMaxEffect_ the maximum effect the I value can have on your result (normally keep low)
    */
    PID(double maximumAllowableError_, double minVelocity_, double maxVelocity_, double minAcceleration_, double maxAcceleration_, double kP_, double kI_, double kIMaxEffect_)
    {
        maximumAllowableError = maximumAllowableError_;
        minVelocity = -1 * fabs(minVelocity_);
        maxVelocity = maxVelocity_;
        minAcceleration = -1 * fabs(minAcceleration_);
        maxAcceleration = maxAcceleration_;
        kP = kP_;
        kI = kI_;
        kIMaxEffect = kIMaxEffect_;
    }

    /**
    * Calculate the best velocity to minimize error
    * @param error the distance between your goal and current (DO NOT ABS VALUE YOUR ERROR)
    * @param elapedTimethe time since our last iteration of the PID loop
    */
    double Calculate(double error, double elapsedTime)
    {
        // If we are within our allowable error, stop moving in the x direction
        if (fabs(error) < maximumAllowableError)
        {
            error = 0;
        }

        // calculate our P in PID
        double intendedP = kP * error;

        // calculate our I in PID and clamp it between our maximum I effects
        double intendedI = std::clamp(kI * runningIntegral, -1 * kIMaxEffect, kIMaxEffect);

        // Clamp our intended velocity to our maximum and minimum velocity to prevent the robot from going too fast
        double intendedVelocity = std::clamp(intendedP + intendedI, -1 * maxVelocity, maxVelocity);

        // Make sure our change in velocity from the last loop is not going above our maximum acceleration
        lastVal += std::clamp(intendedVelocity - lastVal, minAcceleration * elapsedTime,
                            maxAcceleration * elapsedTime);

        return lastVal;
    }
};