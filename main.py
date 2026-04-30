from stepper_motors_juanmf1.AccelerationStrategy import (
    DynamicDelayPlanner,
    ExponentialAcceleration,
)
from stepper_motors_juanmf1.Controller import BipolarStepperMotorDriver
from stepper_motors_juanmf1.Navigation import DynamicNavigation
from stepper_motors_juanmf1.StepperMotor import GenericStepper

from l298n_motor_driver import L298NMotorDriver

# L298N input GPIO order is IN1, IN3, IN2, IN4 so the driver sequence
# matches the motor table order: Pin 1, Pin 3, Pin 2, Pin 4.
LEFT_PINS = (18, 23, 24, 25)
RIGHT_PINS = (14, 15, 18, 12)


def check_direction():
    # -1 left 0 straight 1 right
    return 0  # placeholder


class Car:
    # using a class because it would be messy to try and keep track of everything otherwise
    def __init__(self):
        self.left = Car.setupDriver(inputPins=LEFT_PINS)
        self.leftPos = 0
        self.right = Car.setupDriver(inputPins=RIGHT_PINS)
        self.rightPos = 0

    def leftPosListener(self, currentPosition, targetPosition, direction, multiprocessObserver=None):
        self.leftPos = currentPosition

    def rightPosListener(self, currentPosition, targetPosition, direction, multiprocessObserver=None):
        self.rightPos = currentPosition

    def move(self, leftDelta, rightDelta):
        self.left.signedSteps(leftDelta, fn=self.leftPosListener)
        self.right.signedSteps(rightDelta, fn=self.rightPosListener)

    @staticmethod
    def setupDriver(*, inputPins):
        stepperMotor = GenericStepper(
            maxPps=2000,
            minPps=1600,
            minSleepTime=1 / 200,
            maxSleepTime=1 / 10,
        )  # We'll need to test this but I just added something reasonable.
        delayPlanner = DynamicDelayPlanner()
        navigation = DynamicNavigation()

        acceleration = ExponentialAcceleration(
            stepperMotor=stepperMotor,
            delayPlanner=delayPlanner,
            steppingModeMultiple=BipolarStepperMotorDriver.RESOLUTION_MULTIPLE[
                BipolarStepperMotorDriver.DEFAULT_STEPPING_MODE
            ],
        )
        return L298NMotorDriver(
            stepperMotor=stepperMotor,
            accelerationStrategy=acceleration,
            stepGpioPin=inputPins,
            navigation=navigation,
        )

def main():
    car = Car()
    car.move(100, 100)
    # while True:
    #     direction = check_direction()
    #     if direction == -1:
    #         car.move(300, 0)
    #     if direction == 0:
    #         car.move(300, 300)
    #     if direction == 1:
    #         car.move(0, 300)


if __name__ == "__main__":
    main()