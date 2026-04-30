from stepper_motors_juanmf1.UnipolarController import UnipolarMotorDriver


class L298NMotorDriver(UnipolarMotorDriver):
    """Drive a four-wire bipolar stepper through an L298N IN1-IN4 interface."""

    SEQUENCE = "L298N-Half"

    if SEQUENCE not in UnipolarMotorDriver.Sequence.SUPPORTED_TYPES:
        UnipolarMotorDriver.Sequence.SUPPORTED_TYPES.append(SEQUENCE)
    if SEQUENCE not in UnipolarMotorDriver.Sequence.PINS_USED_2_TYPES_MAP[4]:
        UnipolarMotorDriver.Sequence.PINS_USED_2_TYPES_MAP[4].append(SEQUENCE)
    UnipolarMotorDriver.Sequence.SEQUENCE_2_MICROSTEPPING_MODE_MAP[SEQUENCE] = "Full"
    UnipolarMotorDriver.Sequence.STEP_SEQUENCE[SEQUENCE] = [
        # Motor datasheet order: Pin 1, Pin 3, Pin 2, Pin 4.
        # Wire GPIOs as L298N inputs for those outputs: IN1, IN3, IN2, IN4.
        (1, 0, 0, 0),
        (1, 1, 0, 0),
        (0, 1, 0, 0),
        (0, 1, 1, 0),
        (0, 0, 1, 0),
        (0, 0, 1, 1),
        (0, 0, 0, 1),
        (1, 0, 0, 1),
    ]

    def __init__(self, **kwargs):
        kwargs.setdefault("directionGpioPin", None)
        kwargs.setdefault("stepsMode", self.SEQUENCE)
        super().__init__(**kwargs)
