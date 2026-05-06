import json
import math
import signal
import sys
from pathlib import Path

from car import DEFAULT_CALIBRATION, WHEEL_BASE_CM, Car, turn_all_magnets_off

CHECKPOINT_PATH = Path("calibration_checkpoints.json")
TRAINING_GAIN = 0.35
STRAIGHT_TRAINING_DISTANCE_CM = 200
CIRCLE_TRAINING_RADIUS_CM = 60
FIGURE_8_TRAINING_RADIUS_CM = 45


def load_training_state(checkpointPath=CHECKPOINT_PATH):
    if not checkpointPath.exists():
        return {"calibration": DEFAULT_CALIBRATION, "checkpoints": []}

    with checkpointPath.open() as checkpointFile:
        state = json.load(checkpointFile)

    return {
        "calibration": {**DEFAULT_CALIBRATION, **state.get("calibration", {})},
        "checkpoints": state.get("checkpoints", []),
    }


def save_training_state(state, checkpointPath=CHECKPOINT_PATH):
    with checkpointPath.open("w") as checkpointFile:
        json.dump(state, checkpointFile, indent=2)
        checkpointFile.write("\n")


def benchmark_circle(radiusCm=60):
    car = Car(calibration=load_training_state()["calibration"])
    car.traceCircle(radiusCm)


def benchmark_figure_8(radiusCm=45):
    car = Car(calibration=load_training_state()["calibration"])
    car.traceFigureEight(radiusCm)


def benchmark_spin():
    car = Car(calibration=load_training_state()["calibration"])
    car.turnDegrees(360)


def benchmark_straight(distanceCm=STRAIGHT_TRAINING_DISTANCE_CM):
    car = Car(calibration=load_training_state()["calibration"])
    car.driveDistanceCm(distanceCm)


def run_benchmark(benchmark, *, calibration=None):
    car = Car(calibration=calibration)
    if benchmark == "straight":
        car.driveDistanceCm(STRAIGHT_TRAINING_DISTANCE_CM)
    elif benchmark == "circle":
        car.traceCircle(CIRCLE_TRAINING_RADIUS_CM)
    elif benchmark == "figure8":
        car.traceFigureEight(FIGURE_8_TRAINING_RADIUS_CM)
    elif benchmark == "spin":
        car.turnDegrees(360)
    else:
        raise ValueError("Expected benchmark: straight, circle, figure8, or spin")
    return car


def benchmark_path_length_cm(benchmark):
    if benchmark == "straight":
        return STRAIGHT_TRAINING_DISTANCE_CM
    if benchmark == "circle":
        return 2 * math.pi * CIRCLE_TRAINING_RADIUS_CM
    if benchmark == "figure8":
        return 4 * math.pi * FIGURE_8_TRAINING_RADIUS_CM
    if benchmark == "spin":
        return math.pi * WHEEL_BASE_CM
    raise ValueError("Expected benchmark: straight, circle, figure8, or spin")


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def closed_path_calibration(calibration, benchmark, measuredXCm, measuredYCm):
    pathLengthCm = benchmark_path_length_cm(benchmark)
    nextCalibration = dict(calibration)
    forwardError = measuredXCm / pathLengthCm
    lateralError = measuredYCm / pathLengthCm
    nextCalibration["alignment_trim_per_cm"] += clamp(
        -lateralError * TRAINING_GAIN,
        -0.01,
        0.01,
    )
    scaleCorrection = clamp(-forwardError * TRAINING_GAIN, -0.03, 0.03)
    nextCalibration["left_steps_per_cm_scale"] *= 1 + scaleCorrection
    nextCalibration["right_steps_per_cm_scale"] *= 1 + scaleCorrection
    return nextCalibration


def straight_calibration(calibration, measuredXCm, measuredYCm, headingErrorDegrees):
    nextCalibration = dict(calibration)
    actualDistanceCm = measuredXCm if measuredXCm != 0 else STRAIGHT_TRAINING_DISTANCE_CM
    distanceRatio = STRAIGHT_TRAINING_DISTANCE_CM / actualDistanceCm
    scaleCorrection = clamp((distanceRatio - 1) * TRAINING_GAIN, -0.08, 0.08)
    driftCorrection = clamp((measuredYCm / STRAIGHT_TRAINING_DISTANCE_CM) * TRAINING_GAIN, -0.03, 0.03)
    headingCorrection = clamp((headingErrorDegrees / 90) * TRAINING_GAIN, -0.03, 0.03)

    nextCalibration["left_steps_per_cm_scale"] *= 1 + scaleCorrection + headingCorrection
    nextCalibration["right_steps_per_cm_scale"] *= 1 + scaleCorrection - headingCorrection
    nextCalibration["alignment_trim_per_cm"] += driftCorrection
    return nextCalibration


def spin_calibration(calibration, headingErrorDegrees):
    nextCalibration = dict(calibration)
    actualDegrees = 360 + headingErrorDegrees
    if actualDegrees == 0:
        return nextCalibration
    wheelBaseRatio = 360 / actualDegrees
    correction = clamp((wheelBaseRatio - 1) * TRAINING_GAIN, -0.08, 0.08)
    nextCalibration["wheel_base_cm"] *= 1 + correction
    return nextCalibration


def trained_calibration(calibration, benchmark, measuredXCm, measuredYCm, headingErrorDegrees):
    if benchmark == "straight":
        return straight_calibration(calibration, measuredXCm, measuredYCm, headingErrorDegrees)
    if benchmark == "spin":
        return spin_calibration(calibration, headingErrorDegrees)
    if benchmark in {"circle", "figure8"}:
        return closed_path_calibration(calibration, benchmark, measuredXCm, measuredYCm)
    raise ValueError("Expected benchmark: straight, circle, figure8, or spin")


def parse_offset(offsetText):
    xText, yText = offsetText.split(",", maxsplit=1)
    return float(xText), float(yText)


def parse_heading_error(headingText):
    return float(headingText) if headingText else 0.0


def prompt_measurement(benchmark):
    if benchmark == "straight":
        prompt = (
            "Measured final x,y cm from start "
            f"(target {STRAIGHT_TRAINING_DISTANCE_CM},0; blank to stop): "
        )
    else:
        prompt = "Measured final x,y cm from start (blank to stop): "

    positionText = input(prompt).strip()
    if not positionText:
        return None

    headingText = input("Heading error in degrees, CCW positive (blank for 0): ").strip()
    measuredXCm, measuredYCm = parse_offset(positionText)
    return measuredXCm, measuredYCm, parse_heading_error(headingText)


def train(benchmark, initialMeasurement=None):
    state = load_training_state()
    iteration = len(state["checkpoints"]) + 1
    pendingMeasurement = initialMeasurement

    while True:
        print(f"Training iteration {iteration}: running {benchmark}")
        run_benchmark(benchmark, calibration=state["calibration"])

        if pendingMeasurement is None:
            measurement = prompt_measurement(benchmark)
            if measurement is None:
                break
        else:
            measurement = pendingMeasurement
            pendingMeasurement = None

        measuredXCm, measuredYCm, headingErrorDegrees = measurement

        nextCalibration = trained_calibration(
            state["calibration"],
            benchmark,
            measuredXCm,
            measuredYCm,
            headingErrorDegrees,
        )
        checkpoint = {
            "iteration": iteration,
            "benchmark": benchmark,
            "measured_position_cm": {"x": measuredXCm, "y": measuredYCm},
            "heading_error_degrees": headingErrorDegrees,
            "calibration_before": state["calibration"],
            "calibration_after": nextCalibration,
        }
        state["checkpoints"].append(checkpoint)
        state["calibration"] = nextCalibration
        save_training_state(state)
        print(f"Saved checkpoint {iteration} to {CHECKPOINT_PATH}")
        iteration += 1


def main():
    benchmark = sys.argv[1] if len(sys.argv) > 1 else "circle"
    if benchmark == "straight":
        benchmark_straight()
    elif benchmark == "circle":
        benchmark_circle()
    elif benchmark == "figure8":
        benchmark_figure_8()
    elif benchmark == "spin":
        benchmark_spin()
    elif benchmark == "train":
        trainingBenchmark = sys.argv[2] if len(sys.argv) > 2 else "circle"
        if len(sys.argv) > 3:
            measuredXCm, measuredYCm = parse_offset(sys.argv[3])
            headingErrorDegrees = parse_heading_error(sys.argv[4]) if len(sys.argv) > 4 else 0
            initialMeasurement = (measuredXCm, measuredYCm, headingErrorDegrees)
        else:
            initialMeasurement = None
        train(trainingBenchmark, initialMeasurement=initialMeasurement)
    else:
        raise ValueError("Expected benchmark: straight, circle, figure8, spin, or train")


def handle_interrupt(signalNumber, frame):
    turn_all_magnets_off()
    raise KeyboardInterrupt


if __name__ == "__main__":
    signal.signal(signal.SIGINT, handle_interrupt)
    try:
        main()
    except KeyboardInterrupt:
        turn_all_magnets_off()
        raise
