// Mega 2560: X stepper STEP/DIR/EN=5/6/7; Y DC DIR/PWM=4/9.
// Y encoder A/B=2/3, X/Y home=22/23, shared emergency=28.
// Serial: h home, p queued path, r resume, s stop, d disable, c clear fault.
#include <Axis.h>
#include <AxisStepper.h>
#include <AxisPlanner.h>
#include <util/atomic.h>

AxisStepper<STEPPER2WIRE> x(5, 6, 7);
Axis<GM2::DIR_PWM> y(4, 9);
axis::Quadrature encoder;
AxisPlanner<2, 8> motion;

void encoderISR() { encoder.update(digitalRead(2), digitalRead(3)); }
uint32_t readEncoder(void*) {
    uint32_t count;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { count = encoder.readUnsafe(); }
    return count;
}
void report(axis::Result result) {
    if (Serial.availableForWrite() >= 32) Serial.println(axis::toString(result));
}
axis::Result queueRoute() {
    if (motion.isBusy() || x.isBusy() || y.isBusy()) return axis::Result::BUSY;
    if (!x.isHomed() || !y.isHomed()) return axis::Result::NOT_HOMED;
    axis::Result result = motion.clearBuffer();
    const double points[][2] = {{5, 10}, {10, 20}, {15, 20}};
    for (uint8_t n = 0; n < 3 && result == axis::Result::OK; ++n)
        result = motion.addTarget(points[n], n == 2);
    if (result != axis::Result::OK) { motion.clearBuffer(); return result; }
    return motion.start();
}
void setup() {
    Serial.begin(115200);
    pinMode(2, INPUT_PULLUP); pinMode(3, INPUT_PULLUP);
    encoder.begin(digitalRead(2), digitalRead(3));
    attachInterrupt(digitalPinToInterrupt(2), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(3), encoderISR, CHANGE);
    report(y.attachEncoder(readEncoder));
    report(x.setStepsPerUnit(100));
    axis::Pins xp, yp;
    xp.home = axis::InputPin(22); yp.home = axis::InputPin(23);
    xp.emergency = yp.emergency = axis::InputPin(28);
    report(x.setPins(xp)); report(y.setPins(yp));
    axis::Config config;
    config.maxJerk = 0;
    config.maxSpeed = 3; config.acceleration = config.deceleration = 10;
    config.home.searchSpeed = 1; config.home.latchSpeed = 0.2;
    report(x.begin(config));
    // 2000 encoder counts per screw revolution, 5 mm per revolution.
    config.encoder = axis::EncoderScale::rotary(2000, 5);
    // Tune these gains/feedforward for the actual DC drive before use.
    config.velocityFeedforward = 0.1;
    report(y.begin(config));
    report(motion.addAxis(0, x)); report(motion.addAxis(1, y));
    report(motion.setMaxSpeed(3));
    report(motion.setDtA(0));
}
void loop() {
    motion.tick();
    if (motion.ready() && Serial.availableForWrite() >= 32) Serial.println("Route reached");
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
        case 'd': motion.disable(); break;
        case 'c': report(motion.clearFault()); break;
    }
}
