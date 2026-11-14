// Mega 2560: X STEP/DIR/EN=2/3/4, Y=5/6/7. Too large for Uno/Nano.
// Home X/Y=8/9, shared emergency input=12. Switches to GND.
// Serial 115200: h home, p route, r resume after a stop point,
// s controlled stop (discard route), b brake, d disable, c clear fault.
#include <AxisStepper.h>
#include <AxisPlanner.h>

AxisStepper<STEPPER2WIRE> x(2, 3, 4), y(5, 6, 7);
AxisPlanner<2, 8> motion;  // Number of axes, number of queued segments.

void report(axis::Result result) {
    if (Serial.availableForWrite() >= 32) Serial.println(axis::toString(result));
}

axis::Result queueRoute() {
    if (motion.isBusy() || x.isBusy() || y.isBusy()) return axis::Result::BUSY;
    if (!x.isHomed() || !y.isHomed()) return axis::Result::NOT_HOMED;
    axis::Result result = motion.clearBuffer();
    const double points[][2] = {{10, 5}, {20, 10}, {20, 20}, {0, 0}};
    for (uint8_t n = 0; n < 4 && result == axis::Result::OK; ++n)
        result = motion.addTarget(points[n], n == 2 || n == 3);
    if (result != axis::Result::OK) { motion.clearBuffer(); return result; }
    return motion.start();
}

void setup() {
    Serial.begin(115200);
    report(x.setStepsPerUnit(100)); report(y.setStepsPerUnit(100));
    axis::Pins xp, yp;
    xp.home = axis::InputPin(8); yp.home = axis::InputPin(9);
    xp.emergency = yp.emergency = axis::InputPin(12);
    report(x.setPins(xp)); report(y.setPins(yp));
    axis::Config config;
    config.maxJerk = 0;  // GyverPlanner2-style trapezoid, acceleration jumps.
    config.maxSpeed = 3;
    config.acceleration = config.deceleration = 10;
    config.home.searchSpeed = 1; config.home.latchSpeed = 0.2;
    report(x.begin(config)); report(y.begin(config));
    report(motion.addAxis(0, x)); report(motion.addAxis(1, y));
    report(motion.setMaxSpeed(3));
    report(motion.setAcceleration(10)); report(motion.setDeceleration(10));
    // Exact stops at non-collinear corners; collinear points preserve feed.
    report(motion.setDtA(0));
}

void loop() {
    motion.tick();  // Also services homing/holding when the route is inactive.
    if (motion.ready() && Serial.availableForWrite() >= 32) Serial.println("Point reached; r resumes");
    if (!Serial.available()) return;
    switch (Serial.read()) {
        case 'h':
            if (!motion.isBusy() && !x.isBusy() && !y.isBusy()) {
                motion.clearBuffer(); report(x.home()); report(y.home());
            }
            break;
        case 'p': report(queueRoute()); break;
        case 'r': report(motion.resume()); break;
        case 's': report(motion.stop()); break;
        case 'b': motion.brake(); break;
        case 'd': motion.disable(); break;
        case 'c': report(motion.clearFault()); break;
    }
}
