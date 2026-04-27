from stepper_motors_juanmf1.AccelerationStrategy import (
    DynamicDelayPlanner,
    ExponentialAcceleration,
)
from stepper_motors_juanmf1.Controller import (
    BipolarStepperMotorDriver,
    DRV8825MotorDriver,  # equivalent to a4988
)
from stepper_motors_juanmf1.Navigation import DynamicNavigation
from stepper_motors_juanmf1.StepperMotor import GenericStepper

LEFT_DIR = 23
LEFT_STEP = 24
RIGHT_DIR = 14
RIGHT_STEP = 15


def check_direction():
    # -1 left 0 straight 1 right
    return 0  # placeholder


class Car:
    # using a class because it would be messy to try and keep track of everything otherwise
    def __init__(self):
        self.left = Car.setupDriver(directionGpioPin=LEFT_DIR, stepGpioPin=LEFT_STEP)
        self.leftPos = 0
        self.right = Car.setupDriver(directionGpioPin=RIGHT_DIR, stepGpioPin=RIGHT_STEP)
        self.rightPos = 0
        
    def leftPosListener(self, currentPosition, targetPosition, direction, multiprocessObserver=None):
        self.leftPos = currentPosition

    def rightPosListener(self, currentPosition, targetPosition, direction, multiprocessObserver=None):
        self.rightPos = currentPosition

    def move(self, leftDelta, rightDelta):
        self.left.signedSteps(leftDelta, fn=self.leftPosListener)
        self.right.signedSteps(rightDelta, fn=self.rightPosListener)

    @staticmethod
    def setupDriver(*, directionGpioPin, stepGpioPin):
        stepperMotor = GenericStepper(
            maxPps=200,
            minPps=10,
            minSleepTime=1 / 200,
            maxSleepTime=1 / 10,
        )  # We'll need to test this but I just added somethign resonable
        delayPlanner = DynamicDelayPlanner()
        navigation = DynamicNavigation()
        
        acceleration = ExponentialAcceleration(
            stepperMotor=stepperMotor,
            delayPlanner=delayPlanner,
            steppingModeMultiple=BipolarStepperMotorDriver.RESOLUTION_MULTIPLE[
                BipolarStepperMotorDriver.DEFAULT_STEPPING_MODE
            ],
        )
        return DRV8825MotorDriver(
            stepperMotor=stepperMotor,
            accelerationStrategy=acceleration,
            directionGpioPin=directionGpioPin,
            stepGpioPin=stepGpioPin,
            navigation=navigation,
        )

def main():
    car = Car()
    while True:
        direction = check_direction()
        if direction == -1:
            car.move(300, 0)
        if direction == 0:
            car.move(300, 300)
        if direction == 1:
            car.move(0, 300)


if __name__ == "__main__":
    main()