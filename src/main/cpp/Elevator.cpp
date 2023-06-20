#include "Elevator.h"

/**
 * Instantiates a two motor elevator lift
 */
ElevatorLift::ElevatorLift(rev::CANSparkMax *winchL_, rev::CANSparkMax *winchR_, TimeOfFlight *tofSensor_)
{
  winchL = winchL_;
  winchR = winchR_;
  winchR->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  winchL->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  winchEncoder = new rev::SparkMaxRelativeEncoder(winchL->GetEncoder());
  winchEncoder->SetPosition(0.0);
  tofSensor = tofSensor_;
}

void ElevatorLift::ResetElevatorEncoder()
{
  winchEncoder->SetPosition(0.0);
}

double ElevatorLift::winchEncoderReading()
{
  return -winchEncoder->GetPosition();
}

double ElevatorLift::TOFSReading()
{
  return (tofSensor->GetRange()) / 100;
}

double ElevatorLift::TOFSElevatorHeight()
{
  if (!tofSensor->IsRangeValid())
      return lastHeight;
  return ((tofSensor->GetRange() - 30) * 0.70710678 + 260) / 1000;
}

void ElevatorLift::StopElevator()
{
  MoveElevatorPercent(0);
}

/**
 * Moves the elevator at a percent speed, positive being upwards
 */
void ElevatorLift::MoveElevatorPercent(double percent)
{
  winchR->Set(percent);
  winchL->Set(-percent);   
}

void ElevatorLift::StartPIDLoop()
{
  runningIntegral = 0;
  lastSpeed = 0;
}

bool ElevatorLift::SetElevatorHeightPID(double height, double elapsedTime)
{
  double error = height - winchEncoderReading();

    if (fabs(error) < ALLOWABLE_ERROR_ELEV)
  {
    runningIntegral = 0;
    MoveElevatorPercent(0);
    return true;
  }

  // calculate our I in PID and clamp it between our maximum I effects
  double intendedI = std::clamp(ELEVKI * runningIntegral, -1 * ELEVKIMAX, ELEVKIMAX);

  // Clamp our intended velocity to our maximum and minimum velocity to prevent the robot from going too fast
  double intendedVelocity = std::clamp(ELEVKP * error + intendedI, -1 * ELEVMAX_SPEED, ELEVMAX_SPEED);

  // Make sure our change in velocity from the last loop is not going above our maximum acceleration
  lastSpeed += std::clamp(intendedVelocity - lastSpeed, -1 * ELEVMAX_ACCELERATION * elapsedTime,
                      ELEVMAX_ACCELERATION * elapsedTime);

  runningIntegral += error;

  if (lastSpeed < -0.8)
    lastSpeed = -0.8;

  if (lastSpeed < -0.3 && winchEncoderReading() < 8)
    lastSpeed = -0.3;

  MoveElevatorPercent(lastSpeed + ELEVHOLDFF);
  return false;
}