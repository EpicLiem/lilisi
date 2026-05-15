import math
from contextlib import suppress
from importlib import import_module

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
RIGHT_PINS = (22, 13, 5, 27)
MAGNET_PINS = LEFT_PINS + RIGHT_PINS
L298N_STEPPING_MODE = "Half"

WHEEL_DIAMETER_CM = 6.7
WHEEL_BASE_CM = 35
CAR_WEIGHT_LB = 3
MOTOR_STEPS_PER_REVOLUTION = 200
PATH_SAMPLE_CM = 5
RIGHT_DISTANCE_SENSOR_PINS = {"echo": 19, "trigger": 6}
LEFT_DISTANCE_SENSOR_PINS = {"echo": 16, "trigger": 12}

DEFAULT_CALIBRATION = {
    "left_steps_per_cm_scale": 1.0,
    "right_steps_per_cm_scale": 1.0,
    "alignment_trim_per_cm": 0.0,
    "wheel_base_cm": WHEEL_BASE_CM,
}

# The provided torque chart is at full-step PPS. With 30 cm wheels, 100 full
# steps/sec is about 47 cm/sec, so keep this conservative for initial testing.
MIN_FULL_STEP_PPS = 40
MAX_FULL_STEP_PPS = 120

# Conservative full-step torque curve for a 3 lb car on an L298N. The library
# uses this to ramp speed without jumping too aggressively under load.
TORQUE_CURVE = [
    (MIN_FULL_STEP_PPS, 20),
    (60, 20),
    (80, 15),
    (95, 10),
    (105, 5),
    (MAX_FULL_STEP_PPS, 0),
]


def validate_motor_pins(*pin_groups):
    pins = [pin for group in pin_groups for pin in group]
    duplicates = {pin for pin in pins if pins.count(pin) > 1}
    if duplicates:
        raise ValueError(f"GPIO pins cannot be shared between motors: {sorted(duplicates)}")


def turn_all_magnets_off():
    try:
        GPIO = import_module("RPi.GPIO")
    except (ImportError, RuntimeError):
        return

    with suppress(RuntimeError):
        GPIO.setwarnings(False)
        if GPIO.getmode() is None:
            GPIO.setmode(GPIO.BCM)

    for pin in MAGNET_PINS:
        with suppress(RuntimeError):
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)


def normalize_angle_radians(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def distance_between(start, end):
    return math.hypot(end[0] - start[0], end[1] - start[1])


def quadratic_bezier_point(start, control, end, amount):
    return (
        (1 - amount) ** 2 * start[0]
        + 2 * (1 - amount) * amount * control[0]
        + amount**2 * end[0],
        (1 - amount) ** 2 * start[1]
        + 2 * (1 - amount) * amount * control[1]
        + amount**2 * end[1],
    )


def cubic_bezier_point(start, control1, control2, end, amount):
    return (
        (1 - amount) ** 3 * start[0]
        + 3 * (1 - amount) ** 2 * amount * control1[0]
        + 3 * (1 - amount) * amount**2 * control2[0]
        + amount**3 * end[0],
        (1 - amount) ** 3 * start[1]
        + 3 * (1 - amount) ** 2 * amount * control1[1]
        + 3 * (1 - amount) * amount**2 * control2[1]
        + amount**3 * end[1],
    )


def samples_for_path_length(*points):
    control_polygon_length = sum(
        distance_between(start, end) for start, end in zip(points, points[1:])
    )
    return max(1, math.ceil(control_polygon_length / PATH_SAMPLE_CM))


class Car:
    # using a class because it would be messy to try and keep track of everything otherwise
    def __init__(self, calibration=None):
        validate_motor_pins(LEFT_PINS, RIGHT_PINS)
        self.calibration = {**DEFAULT_CALIBRATION, **(calibration or {})}
        self.left = Car.setupDriver(inputPins=LEFT_PINS)
        self.leftPos = 0
        self.right = Car.setupDriver(inputPins=RIGHT_PINS)
        self.rightPos = 0
        self.xCm = 0
        self.yCm = 0
        self.headingRadians = 0

    def leftPosListener(self, currentPosition, targetPosition, direction, multiprocessObserver=None):
        self.leftPos = currentPosition

    def rightPosListener(self, currentPosition, targetPosition, direction, multiprocessObserver=None):
        self.rightPos = currentPosition

    def turnMagnetsOff(self):
        for driver in (self.left, self.right):
            for methodName in ("release", "disable", "stop", "cleanup"):
                method = getattr(driver, methodName, None)
                if callable(method):
                    with suppress(Exception):
                        method()
                    break

        turn_all_magnets_off()

    def move(self, leftDelta, rightDelta):
        self.left.signedSteps(leftDelta, fn=self.leftPosListener)
        self.right.signedSteps(rightDelta, fn=self.rightPosListener)

    @property
    def stepsPerCm(self):
        return MOTOR_STEPS_PER_REVOLUTION / (math.pi * WHEEL_DIAMETER_CM)

    @property
    def wheelBaseCm(self):
        return self.calibration["wheel_base_cm"]

    def cmToSteps(self, distanceCm, *, side):
        scale = self.calibration[f"{side}_steps_per_cm_scale"]
        return round(distanceCm * self.stepsPerCm * scale)

    def moveCm(self, leftCm, rightCm):
        averageDistanceCm = (leftCm + rightCm) / 2
        alignmentTrimCm = averageDistanceCm * self.calibration["alignment_trim_per_cm"]
        leftCm += alignmentTrimCm
        rightCm -= alignmentTrimCm
        self.move(
            self.cmToSteps(leftCm, side="left"),
            self.cmToSteps(rightCm, side="right"),
        )

    def driveDistanceCm(self, distanceCm):
        self.moveCm(distanceCm, distanceCm)
        self.xCm += math.cos(self.headingRadians) * distanceCm
        self.yCm += math.sin(self.headingRadians) * distanceCm

    def turnRadians(self, angleRadians):
        wheelDistanceCm = (self.wheelBaseCm * angleRadians) / 2
        self.moveCm(-wheelDistanceCm, wheelDistanceCm)
        self.headingRadians = normalize_angle_radians(self.headingRadians + angleRadians)

    def turnDegrees(self, angleDegrees):
        self.turnRadians(math.radians(angleDegrees))

    def avoidObstacles(
        self,
        *,
        stopDistanceCm=10,
        avoidDistanceCm=30,
        forwardStepCm=8,
        reverseStepCm=6,
        turnAngleDegrees=35,
        sampleDelaySeconds=0.05,
    ):
        from gpiozero import DistanceSensor  # pyright: ignore[reportMissingImports]
        from time import sleep

        rightSensor = DistanceSensor(**RIGHT_DISTANCE_SENSOR_PINS)
        leftSensor = DistanceSensor(**LEFT_DISTANCE_SENSOR_PINS)

        try:
            while True:
                rightDistanceCm = rightSensor.distance * 100
                leftDistanceCm = leftSensor.distance * 100

                if rightDistanceCm < stopDistanceCm and leftDistanceCm < stopDistanceCm:
                    self.turnMagnetsOff()
                    sleep(sampleDelaySeconds)
                elif rightDistanceCm < avoidDistanceCm and leftDistanceCm < avoidDistanceCm:
                    self.driveDistanceCm(-reverseStepCm)
                    self.turnDegrees(turnAngleDegrees)
                elif rightDistanceCm < avoidDistanceCm:
                    self.turnDegrees(turnAngleDegrees)
                elif leftDistanceCm < avoidDistanceCm:
                    self.turnDegrees(-turnAngleDegrees)
                else:
                    self.driveDistanceCm(forwardStepCm)

                sleep(sampleDelaySeconds)
        finally:
            rightSensor.close()
            leftSensor.close()
            self.turnMagnetsOff()

    def followSegmentCm(self, distanceCm, headingDeltaRadians):
        startingHeading = self.headingRadians
        leftCm = distanceCm - (self.wheelBaseCm * headingDeltaRadians) / 2
        rightCm = distanceCm + (self.wheelBaseCm * headingDeltaRadians) / 2
        self.moveCm(leftCm, rightCm)

        self.headingRadians = normalize_angle_radians(self.headingRadians + headingDeltaRadians)
        averageHeading = startingHeading + headingDeltaRadians / 2
        self.xCm += math.cos(averageHeading) * distanceCm
        self.yCm += math.sin(averageHeading) * distanceCm

    def tracePoints(self, points):
        if len(points) < 2:
            return

        for start, end in zip(points, points[1:]):
            segmentDistance = distance_between(start, end)
            if segmentDistance == 0:
                continue

            segmentHeading = math.atan2(end[1] - start[1], end[0] - start[0])
            headingDelta = normalize_angle_radians(segmentHeading - self.headingRadians)
            self.followSegmentCm(segmentDistance, headingDelta)

    def traceLine(self, start, end):
        self.tracePoints([start, end])

    def tracePolyline(self, points):
        self.tracePoints(points)

    def traceCircle(self, radiusCm, samples=None):
        circumference = 2 * math.pi * radiusCm
        samples = samples if samples else max(12, math.ceil(circumference / PATH_SAMPLE_CM)) # use at most 12 samples or less if circumference is small enough
        points = [
            (
                radiusCm * math.sin(2 * math.pi * sample / samples), # y component of the point
                radiusCm * (1 - math.cos(2 * math.pi * sample / samples)), # x component of the point
            )
            for sample in range(samples + 1) # (first point doesn't really count bc it's 0,0)
        ]
        self.tracePoints(points)

    def traceFigureEight(self, radiusCm, samples=None):
        widthCm = 2 * radiusCm
        path_length_guess = 4 * math.pi * radiusCm
        samples = samples if samples else max(24, math.ceil(path_length_guess / PATH_SAMPLE_CM))
        points = [
            (
                widthCm * math.sin(2 * math.pi * sample / samples),
                radiusCm * math.sin(4 * math.pi * sample / samples),
            )
            for sample in range(samples + 1)
        ]
        self.tracePoints(points)

    def traceQuadraticBezier(self, start, control, end, samples=None):
        samples = samples if samples else samples_for_path_length(start, control, end)
        points = [
            quadratic_bezier_point(start, control, end, sample / samples)
            for sample in range(samples + 1)
        ]
        self.tracePoints(points)

    def traceCubicBezier(self, start, control1, control2, end, samples=None):
        samples = samples if samples else samples_for_path_length(start, control1, control2, end)
        points = [
            cubic_bezier_point(start, control1, control2, end, sample / samples)
            for sample in range(samples + 1)
        ]
        self.tracePoints(points)

    @staticmethod
    def setupDriver(*, inputPins):
        stepperMotor = GenericStepper(
            maxPps=MAX_FULL_STEP_PPS,
            minPps=MIN_FULL_STEP_PPS,
            minSleepTime=1 / MAX_FULL_STEP_PPS,
            maxSleepTime=1 / MIN_FULL_STEP_PPS,
            spr=MOTOR_STEPS_PER_REVOLUTION,
            torqueCurve=TORQUE_CURVE,
        )
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
