from stepper_motors_juanmf1.AccelerationStrategy import (
    DynamicDelayPlanner,
    ExponentialAcceleration,
)
from stepper_motors_juanmf1.Controller import BipolarStepperMotorDriver
from stepper_motors_juanmf1.Navigation import DynamicNavigation
from stepper_motors_juanmf1.StepperMotor import GenericStepper

from l298n_motor_driver import L298NMotorDriver

# L298N input GPIO order is IN1, IN2, IN3, IN4.
LEFT_PINS = (18, 23, 24, 25)
RIGHT_PINS = (14, 15, 19, 12)
L298N_STEPPING_MODE = "Half"


def validate_motor_pins(*pin_groups):
    pins = [pin for group in pin_groups for pin in group]
    duplicates = {pin for pin in pins if pins.count(pin) > 1}
    if duplicates:
        raise ValueError(f"GPIO pins cannot be shared between motors: {sorted(duplicates)}")


def check_direction():
    # -1 left 0 straight 1 right
    return 0  # placeholder


class Car:
    # using a class because it would be messy to try and keep track of everything otherwise
    def __init__(self):
        validate_motor_pins(LEFT_PINS, RIGHT_PINS)
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
            maxPps=100,
            minPps=20,
            minSleepTime=1 / 100,
            maxSleepTime=1 / 20,
        )  # We'll need to test this but I just added something reasonable.
        delayPlanner = DynamicDelayPlanner()
        navigation = DynamicNavigation()

        acceleration = ExponentialAcceleration(
            stepperMotor=stepperMotor,
            delayPlanner=delayPlanner,
            steppingModeMultiple=BipolarStepperMotorDriver.RESOLUTION_MULTIPLE[
                L298N_STEPPING_MODE
            ],
        )
        return L298NMotorDriver(
            stepperMotor=stepperMotor,
            accelerationStrategy=acceleration,
            stepGpioPin=inputPins,
            stepsMode=L298NMotorDriver.SEQUENCE,
            navigation=navigation,
        )

def main():
    car = Car()
    car.move(2000, 2000)
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