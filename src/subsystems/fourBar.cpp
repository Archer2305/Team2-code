#include "main.h"

using namespace okapi;

Motor fourBarLift(fourBarPort, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

int liftState;
double fourBarPower;
double error;
auto fourBarController = IterativeControllerFactory::posPID(0.5, 0, 0.001);//1.4, 0, 0.003

double fourBarMacros(double setpoint)
{
  fourBarController.setOutputLimits(-100, 100);
  error = setpoint - fourBarLift.getPosition();
  fourBarPower = fourBarController.step(error);

  return (-fourBarPower);
}

void updateFourBar()
{
  fourBarLift.setBrakeMode(AbstractMotor::brakeMode::hold);

  if (controller.getDigital(ControllerDigital::L1) == 1)
  {
    liftState = 1;
  }
  else if (controller.getDigital(ControllerDigital::L2) == 1)
  {
    liftState = 2;
  }
  else if (controller.getDigital(ControllerDigital::A) == 1)
  {
    liftState = 3;
  }


  switch (liftState)
  {
    case 1:
      fourBarLift.moveVelocity(fourBarMacros(600));
      break;
    case 2:
      fourBarLift.moveVelocity(fourBarMacros(-10));
      break;
    case 3:
      fourBarLift.moveVelocity(fourBarMacros(250));
      break;
  }
}
