#include "main.h"

using namespace okapi;

Motor rightFront(rightFrontPort, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor rightTop(rightTopPort, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor rightBottom(rightBottomPort, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

Motor leftFront(leftFrontPort, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor leftTop(leftTopPort, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor leftBottom(leftBottomPort, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

typedef struct PID pid;
pid park;
pid rotate1;
pid translate1;
int timer1;

std::shared_ptr<ChassisController> drive =
  ChassisControllerBuilder()
  .withMotors({leftFront, leftTop, leftBottom}, {rightFront, rightTop, rightBottom})
  .withDimensions(AbstractMotor::gearset::green, {{4_in, 13.7_in}, imev5greenTPR})
  .build();

void updateDrive(){
  drive -> getModel() -> tank(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightY));
  if (controller.getDigital(ControllerDigital::left) == 1)
  {
    leftFront.setBrakeMode(AbstractMotor::brakeMode::hold);
    leftTop.setBrakeMode(AbstractMotor::brakeMode::hold);
    leftBottom.setBrakeMode(AbstractMotor::brakeMode::hold);

    rightFront.setBrakeMode(AbstractMotor::brakeMode::hold);
    rightTop.setBrakeMode(AbstractMotor::brakeMode::hold);
    rightBottom.setBrakeMode(AbstractMotor::brakeMode::hold);
  }
  else if (controller.getDigital(ControllerDigital::right) == 1)
  {
    leftFront.setBrakeMode(AbstractMotor::brakeMode::coast);
    leftTop.setBrakeMode(AbstractMotor::brakeMode::coast);
    leftBottom.setBrakeMode(AbstractMotor::brakeMode::coast);

    rightFront.setBrakeMode(AbstractMotor::brakeMode::coast);
    rightTop.setBrakeMode(AbstractMotor::brakeMode::coast);
    rightBottom.setBrakeMode(AbstractMotor::brakeMode::coast);
  }
}
