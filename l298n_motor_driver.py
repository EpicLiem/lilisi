from stepper_motors_juanmf1.UnipolarController import UnipolarMotorDriver


class L298NMotorDriver(UnipolarMotorDriver):
    """Drive a four-wire bipolar stepper through an L298N IN1-IN4 interface."""

    def __init__(self, **kwargs):
        kwargs.setdefault("directionGpioPin", None)
        kwargs.setdefault("stepsMode", UnipolarMotorDriver.Sequence.FULL)
        super().__init__(**kwargs)
