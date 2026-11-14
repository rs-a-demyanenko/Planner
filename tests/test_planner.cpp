#include <Axis.h>
#include <AxisStepper.h>
#include <AxisPlanner.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits>

uint32_t mockMicros = 0;
int mockDigital[256] = {};
MockSerial Serial;
static unsigned checks = 0;
#define CHECK(expr) do { ++checks; if (!(expr)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); } } while (0)
#define NEAR(a, b, eps) CHECK(fabs((a) - (b)) <= (eps))
using axis::Result;

// Actual Axis Core, with encoder quantization and a separately committed motor.
// A stepper follows the requested pulse velocity; a DC motor has first-order
// mechanical lag. Neither plant copies the planner reference into feedback.
struct Plant {
    axis::Core core;
    axis::Inputs inputs;
    axis::Output applied;
    double position = 2.0, velocity = 0.0;
    uint32_t now = 1000;
    unsigned driveWrites = 0;
    bool brokenRead = false, frozen = false;
    explicit Plant(axis::Control control = axis::Control::STEPPER, double maxSpeed = 8.0,
                   uint32_t period = 1000, double jerk = 0.0) {
        axis::Config c;
        c.control = control; c.encoder = axis::EncoderScale::linear(10000);
        c.maxSpeed = maxSpeed; c.acceleration = 20; c.deceleration = 12; c.maxJerk = jerk;
        c.controlPeriodUs = period; c.velocityFilterTime = 0.005;
        c.velocitySetpointWeight = 1; c.velocityKp = 0.12; c.velocityKi = 0.8;
        c.velocityFeedforward = 0.05; c.positionTolerance = 0.02; c.velocityTolerance = 0.04;
        c.settleTimeUs = 10000; c.softMax = 100;
        c.home.searchSpeed = 1; c.home.latchSpeed = 0.4; c.home.backoff = 0.2;
        c.maxFollowingError = 2; c.stallOutput = 0;
        CHECK(core.begin(c, now, count(), inputs) == Result::OK);
        applied = core.getOutput();
    }
    uint32_t count() const { return static_cast<uint32_t>(static_cast<int64_t>(llround(position * 10000.0))); }
    void advance(uint32_t us = 1000) {
        double demand = applied.action == axis::OutputAction::DRIVE ?
                        (core.getConfig().control == axis::Control::STEPPER ? applied.velocity : applied.duty * 20.0) : 0.0;
        if (frozen) velocity = 0.0;
        else if (core.getConfig().control == axis::Control::STEPPER) velocity = demand;
        else velocity += (demand - velocity) * (1.0 - exp(-static_cast<double>(us) * 1e-6 / 0.04));
        position += velocity * us * 1e-6;
        now += us;
        inputs.home = position <= 0;
    }
    static bool read(void* context, uint32_t& count, axis::Inputs& inputs) {
        Plant& self = *static_cast<Plant*>(context);
        count = self.count(); inputs = self.inputs; return !self.brokenRead;
    }
    static void write(void* context) {
        Plant& self = *static_cast<Plant*>(context);
        self.applied = self.core.getOutput();
        if (self.applied.action == axis::OutputAction::DRIVE) ++self.driveWrites;
    }
    axis::AxisPort port() {
        axis::AxisPort p; p.core = &core; p.context = this; p.read = read; p.apply = write; return p;
    }
    void home() {
        CHECK(core.home() == Result::OK);
        for (unsigned n = 0; n < 60000 && core.isBusy(); ++n) {
            advance(); port().update(now); port().commit();
        }
        if (core.getFault() != axis::Fault::NONE) fprintf(stderr,
            "home fault %s pos=%f ref=%f v=%f phase=%u control=%u jerk=%f\n",
            axis::toString(core.getFault()), position, core.getReferenceState().position,
            velocity, unsigned(core.getHomePhase()), unsigned(core.getConfig().control), core.getConfig().maxJerk);
        CHECK(core.isHomed()); CHECK(!core.isBusy());
        core.disable(); port().commit();
    }
};

template <uint8_t N, uint8_t B>
void cycle(planner::Planner<N, B>& p, Plant& x, Plant& y, uint32_t us = 1000) {
    x.advance(us); y.advance(us); CHECK(x.now == y.now); p.tickAt(x.now);
}
static void align(Plant& x, Plant& y) {
    x.now = y.now = (x.now > y.now ? x.now : y.now) + 1000;
    x.port().update(x.now); y.port().update(y.now); x.port().commit(); y.port().commit();
}
template <uint8_t N, uint8_t B>
void finish(planner::Planner<N, B>& p, Plant& x, Plant& y) {
    for (unsigned n = 0; n < 100000 && p.isBusy(); ++n) cycle(p, x, y);
    if (p.getError() != planner::Error::NONE || p.isBusy())
        fprintf(stderr, "planner status=%u error=%u x=%f/%f y=%f/%f faults=%s/%s\n",
                unsigned(p.getStatus()), unsigned(p.getError()), x.core.getPosition(), p.getReference(0).position,
                y.core.getPosition(), p.getReference(1).position,
                axis::toString(x.core.getFault()), axis::toString(y.core.getFault()));
    CHECK(p.getError() == planner::Error::NONE); CHECK(!p.isBusy());
}

static void arguments() {
    planner::Planner<2, 2> p;
    const double a[] = {1, 2};
    CHECK(p.startAt(0) == Result::NOT_INITIALIZED);
    CHECK(p.addTarget(a) == Result::NOT_INITIALIZED);
    Plant x, y;
    CHECK(p.addAxis(2, x.port()) == Result::INVALID_ARGUMENT);
    CHECK(p.addAxis(0, axis::AxisPort()) == Result::INVALID_ARGUMENT);
    CHECK(p.addAxis(0, x.port()) == Result::OK);
    CHECK(p.addAxis(1, x.port()) == Result::INVALID_ARGUMENT);
    CHECK(p.addAxis(1, y.port()) == Result::OK);
    CHECK(p.startAt(x.now) == Result::INVALID_ARGUMENT);
    CHECK(p.setMaxSpeed(0) == Result::INVALID_ARGUMENT);
    CHECK(p.setAcceleration(-1) == Result::INVALID_ARGUMENT);
    CHECK(p.setDtA(1.01) == Result::INVALID_ARGUMENT);
    const double nan[] = {std::numeric_limits<double>::quiet_NaN(), 1};
    const double overflow[] = {101, 1};
    const double duplicate[] = {0, 0};
    CHECK(p.addTarget(nan) == Result::INVALID_ARGUMENT);
    CHECK(p.addTarget(overflow) == Result::OUT_OF_RANGE);
    CHECK(p.addTarget(duplicate) == Result::INVALID_ARGUMENT);
    CHECK(p.queued() == 0);
    CHECK(p.addTarget(a) == Result::OK);
    CHECK(p.setMaxSpeed(1) == Result::BUSY);
    CHECK(p.addAxis(1, y.port()) == Result::BUSY);
    CHECK(p.addTarget(a) == Result::INVALID_ARGUMENT);
    const double relative[] = {1, 1};
    CHECK(p.addTarget(relative, true, planner::Coordinates::RELATIVE) == Result::OK);
    CHECK(!p.available()); CHECK(p.freeSlots() == 0);
    CHECK(p.addTarget(a) == Result::BUSY);
    CHECK(p.startAt(x.now) == Result::NOT_HOMED);
    CHECK(p.clearBuffer() == Result::OK); CHECK(p.available());

    Plant jerk(axis::Control::STEPPER, 8, 1000, 100);
    jerk.home();
    planner::Planner<1> boundedJerk;
    CHECK(boundedJerk.addAxis(0, jerk.port()) == Result::OK);
    const double t[] = {2};
    CHECK(boundedJerk.addTarget(t) == Result::OK);
    CHECK(boundedJerk.startAt(jerk.now) == Result::UNSUPPORTED);
    CHECK(!boundedJerk.isBusy()); CHECK(!jerk.core.isBusy());
}

static void mixedMotion() {
    Plant x(axis::Control::STEPPER, 8, 1000), y(axis::Control::DC, 2, 5000);
    x.home(); y.home(); align(x, y);
    planner::Planner<2, 8> p;
    CHECK(p.addAxis(0, x.port()) == Result::OK); CHECK(p.addAxis(1, y.port()) == Result::OK);
    const double sx = x.core.getPosition(), sy = y.core.getPosition();
    const double a[] = {sx + 2, sy + 4}, b[] = {sx + 4, sy + 8};
    CHECK(p.addTarget(a) == Result::OK); CHECK(p.addTarget(b, true) == Result::OK);
    CHECK(p.startAt(x.now) == Result::OK);
    CHECK(p.getPeriod() == 5000);
    CHECK(x.core.moveBy(1) == Result::BUSY);
    CHECK(p.clearBuffer() == Result::BUSY);
    bool through = false;
    for (unsigned n = 0; n < 20000 && p.isBusy(); ++n) {
        cycle(p, x, y);
        CHECK(p.getError() == planner::Error::NONE);
        axis::MotionState xr = p.getReference(0), yr = p.getReference(1);
        NEAR(yr.position - sy, 2.0 * (xr.position - sx), 1e-9);
        NEAR(yr.velocity, 2.0 * xr.velocity, 1e-9);
        CHECK(fabs(xr.velocity) <= 1.000001); CHECK(fabs(yr.velocity) <= 2.000001);
        CHECK(fabs(yr.acceleration) <= 20.000001);
        CHECK(x.core.getSampleTimeUs() == y.core.getSampleTimeUs());
        if (p.queued() == 1 && p.getPathSpeed() > 0.5) through = true;
    }
    CHECK(through); CHECK(!p.isBusy()); CHECK(p.ready()); CHECK(!p.ready());
    NEAR(x.core.getPosition(), b[0], 0.02); NEAR(y.core.getPosition(), b[1], 0.02);
    CHECK(x.core.getCompletion() == axis::Completion::REACHED);
    CHECK(y.core.getCompletion() == axis::Completion::REACHED);
}

static void streamingAndStops() {
    Plant x, y; x.home(); y.home(); align(x, y);
    planner::Planner<2, 2> p;
    CHECK(p.addAxis(0, x.port()) == Result::OK); CHECK(p.addAxis(1, y.port()) == Result::OK);
    const double a[] = {2, 2}, b[] = {4, 4}, delta[] = {2, 2};
    CHECK(p.addTarget(a) == Result::OK); CHECK(p.addTarget(b) == Result::OK);
    CHECK(p.startAt(x.now) == Result::OK);
    while (p.queued() == 2 && p.isBusy()) cycle(p, x, y);
    CHECK(p.isBusy()); CHECK(p.available());
    double speed = p.getPathSpeed();
    CHECK(p.addTarget(delta, true, planner::Coordinates::RELATIVE) == Result::OK);
    NEAR(p.getPathSpeed(), speed, 1e-12);  // Active profile was not changed.
    finish(p, x, y); CHECK(p.ready());
    NEAR(x.core.getPosition(), 6, 0.02); NEAR(y.core.getPosition(), 6, 0.02);

    // A mandatory stop holds a queued continuation until explicit resume.
    const double c[] = {8, 6}, d[] = {8, 8};
    CHECK(p.addTarget(c, true) == Result::OK); CHECK(p.addTarget(d, true) == Result::OK);
    CHECK(p.resumeAt(x.now) == Result::OK); finish(p, x, y);
    CHECK(p.queued() == 1); CHECK(p.ready()); NEAR(x.core.getPosition(), 8, 0.02);
    NEAR(y.core.getPosition(), 6, 0.02);
    CHECK(p.resumeAt(x.now) == Result::OK); finish(p, x, y);
    NEAR(y.core.getPosition(), 8, 0.02);

    // Reverse direction forces zero junction speed even without a stop flag.
    const double e[] = {10, 8}, f[] = {8, 8};
    CHECK(p.addTarget(e) == Result::OK); CHECK(p.addTarget(f) == Result::OK);
    CHECK(p.startAt(x.now) == Result::OK); finish(p, x, y);
    NEAR(x.core.getPosition(), 8, 0.02);
}

static void stopping() {
    Plant x, y; x.home(); y.home(); align(x, y);
    planner::Planner<2, 4> p;
    CHECK(p.addAxis(0, x.port()) == Result::OK); CHECK(p.addAxis(1, y.port()) == Result::OK);
    CHECK(p.setMaxSpeed(4) == Result::OK);
    const double a[] = {10, 10}; CHECK(p.addTarget(a) == Result::OK);
    CHECK(p.startAt(x.now) == Result::OK);
    for (unsigned n = 0; n < 400; ++n) cycle(p, x, y);
    double previousSpeed = p.getPathSpeed(), before = p.getReference(0).position;
    CHECK(previousSpeed > 0); CHECK(p.stop() == Result::OK);
    CHECK(p.stop() == Result::OK);
    CHECK(!p.available()); CHECK(p.addTarget(a) == Result::BUSY);
    for (unsigned n = 0; n < 10000 && p.isBusy(); ++n) {
        cycle(p, x, y); CHECK(p.getPathSpeed() <= previousSpeed + 1e-9); previousSpeed = p.getPathSpeed();
    }
    CHECK(p.getError() == planner::Error::NONE); CHECK(!p.isBusy()); CHECK(!p.ready());
    CHECK(p.queued() == 0); CHECK(p.getReference(0).position > before);
    CHECK(p.getReference(0).position < 10);
    CHECK(x.core.getCompletion() == axis::Completion::STOPPED);
    CHECK(y.core.getCompletion() == axis::Completion::STOPPED);

    // Request braking just before a collinear junction, while stopping distance
    // is greater than the remaining first line. It must continue smoothly.
    double start = x.core.getPosition();
    const double b[] = {start + 1, y.core.getPosition() + 1};
    const double c[] = {start + 4, y.core.getPosition() + 4};
    CHECK(p.addTarget(b) == Result::OK); CHECK(p.addTarget(c) == Result::OK);
    CHECK(p.startAt(x.now) == Result::OK);
    while (p.isBusy() && p.getReference(0).position < start + 0.9) cycle(p, x, y);
    CHECK(p.queued() == 2); CHECK(p.stop() == Result::OK);
    finish(p, x, y); CHECK(p.getReference(0).position > b[0]);
    CHECK(p.getReference(0).position < c[0]);
    CHECK(x.core.getCompletion() == axis::Completion::STOPPED);
}

static void faultsAndOwnership() {
    Plant x, y; x.home(); y.home(); align(x, y);
    planner::Planner<2> p;
    CHECK(p.addAxis(0, x.port()) == Result::OK); CHECK(p.addAxis(1, y.port()) == Result::OK);
    const double a[] = {5, 5}; CHECK(p.addTarget(a) == Result::OK);
    CHECK(p.startAt(x.now) == Result::OK);
    for (unsigned n = 0; n < 50; ++n) cycle(p, x, y);
    unsigned writes = x.driveWrites;
    y.inputs.maxLimit = true;
    cycle(p, x, y, 1);  // Fault between control cycles, second member.
    CHECK(p.getError() == planner::Error::AXIS_FAULT);
    CHECK(x.driveWrites == writes);
    CHECK(x.applied.action != axis::OutputAction::DRIVE);
    CHECK(y.applied.action != axis::OutputAction::DRIVE);
    CHECK(x.core.getFault() == axis::Fault::SYNC_FAULT);
    CHECK(y.core.getFault() == axis::Fault::MAX_LIMIT);
    CHECK(p.queued() == 0); CHECK(p.clearFaultAt(x.now) == Result::FAULT_ACTIVE);
    y.inputs.maxLimit = false;
    for (unsigned n = 0; n < 100; ++n) cycle(p, x, y);
    CHECK(p.clearFaultAt(x.now) == Result::OK);
    CHECK(p.getError() == planner::Error::NONE);
    CHECK(p.addTarget(a) == Result::OK); CHECK(p.startAt(x.now) == Result::OK);
    cycle(p, x, y, 50001);
    CHECK(p.getError() == planner::Error::TIMING);
    CHECK(!p.isBusy());
    for (unsigned n = 0; n < 100; ++n) cycle(p, x, y);
    CHECK(p.clearFaultAt(x.now) == Result::OK);
    CHECK(p.addTarget(a) == Result::OK); CHECK(p.startAt(x.now) == Result::OK);
    y.brokenRead = true; cycle(p, x, y);
    CHECK(p.getError() == planner::Error::AXIS_FAULT);
    CHECK(x.applied.action != axis::OutputAction::DRIVE);
    CHECK(p.clearFaultAt(x.now) == Result::FAULT_ACTIVE);
    y.brokenRead = false; CHECK(p.clearFaultAt(x.now) == Result::OK);

    CHECK(p.addTarget(a) == Result::OK); CHECK(p.startAt(x.now) == Result::OK);
    y.core.disable(); cycle(p, x, y);
    CHECK(p.getError() == planner::Error::INTERRUPTED);
    CHECK(p.clearFaultAt(x.now) == Result::OK);
    CHECK(p.addTarget(a) == Result::OK); CHECK(p.startAt(x.now) == Result::OK);
    p.brake(); CHECK(!p.isBusy()); CHECK(!x.core.isBusy()); CHECK(!y.core.isBusy());
    CHECK(p.queued() == 0);
}

static void shortSegmentsAndWrap() {
    Plant x, y; x.home(); y.home(); align(x, y);
    x.now = y.now = UINT32_MAX - 3000;
    x.port().update(x.now); y.port().update(y.now);
    planner::Planner<2, 16> p;
    CHECK(p.addAxis(0, x.port()) == Result::OK); CHECK(p.addAxis(1, y.port()) == Result::OK);
    for (unsigned n = 1; n <= 16; ++n) {
        const double point[] = {n * 0.001, n * 0.002}; CHECK(p.addTarget(point) == Result::OK);
    }
    CHECK(p.startAt(x.now) == Result::OK); finish(p, x, y);
    NEAR(p.getReference(0).position, 0.016, 1e-12);
    NEAR(p.getReference(1).position, 0.032, 1e-12);
    CHECK(p.ready()); CHECK(x.now < 1000000);
}

static int steps = 0;
static bool freezeEncoder = false;
static uint32_t frozenCount = 0;
static void onStep(uint8_t positive) { steps += positive ? 1 : -1; }
static uint32_t externalEncoder(void*) {
    return freezeEncoder ? frozenCount : static_cast<uint32_t>(steps * 10);
}
static axis::Inputs driverInputs(void*) {
    axis::Inputs inputs;
    // Emulated home switch tied to issued physical steps, not a trajectory.
    inputs.home = steps <= 0;
    return inputs;
}
static void realAdapters() {
    AxisStepper<STEPPER2WIRE, STEPPER_VIRTUAL> motor;
    CHECK(motor.attachStep(onStep) == Result::OK);
    CHECK(motor.setStepsPerUnit(100) == Result::OK);
    CHECK(motor.attachInputs(driverInputs) == Result::OK);
    axis::Config c; c.maxJerk = 0; c.maxSpeed = 2;
    CHECK(motor.begin(c) == Result::OK);
    AxisPlanner<1> p;
    CHECK(p.addAxis(0, motor) == Result::OK);
    const double point[] = {1}; CHECK(p.addTarget(point) == Result::OK);
    CHECK(p.start() == Result::NOT_HOMED);
    CHECK(p.clearBuffer() == Result::OK);
    CHECK(motor.home() == Result::OK);
    for (unsigned n = 0; n < 600000 && motor.isBusy(); ++n) {
        mockMicros += 50; p.tick();
    }
    CHECK(motor.isHomed()); CHECK(!motor.isBusy());
    CHECK(p.addTarget(point) == Result::OK); CHECK(p.start() == Result::OK);
    unsigned serviceCalls = 0;
    for (; serviceCalls < 600000 && p.isBusy(); ++serviceCalls) { mockMicros += 50; p.tick(); }
    CHECK(!p.isBusy()); CHECK(p.getError() == planner::Error::NONE);
    CHECK(p.ready()); NEAR(motor.getPosition(), 1, c.positionTolerance);
    CHECK(steps > 0); CHECK(serviceCalls > 0);

    // The same real stepper adapter with a separately scaled encoder. Freeze
    // feedback after homing: issuing pulses must not count as measured travel.
    motor.disable();
    CHECK(motor.attachEncoder(externalEncoder) == Result::OK);
    c.encoder = axis::EncoderScale::linear(1000);
    c.maxFollowingError = 0.3; c.followingErrorTimeUs = 50000;
    CHECK(motor.begin(c) == Result::OK); CHECK(motor.home() == Result::OK);
    for (unsigned n = 0; n < 600000 && motor.isBusy(); ++n) { mockMicros += 50; p.tick(); }
    CHECK(motor.isHomed()); CHECK(!motor.isBusy());
    frozenCount = externalEncoder(nullptr); freezeEncoder = true;
    const double further[] = {3};
    CHECK(p.addTarget(further) == Result::OK); CHECK(p.start() == Result::OK);
    int beforeSteps = steps;
    for (unsigned n = 0; n < 600000 && p.isBusy(); ++n) { mockMicros += 50; p.tick(); }
    CHECK(p.getError() == planner::Error::AXIS_FAULT);
    CHECK(motor.getFault() == axis::Fault::FOLLOWING_ERROR);
    CHECK(steps > beforeSteps); CHECK(motor.getPosition() < 0.1);
    freezeEncoder = false;
    Axis<GM2::DIR_PWM> dc(2, 3);
    AxisPlanner<2> mixed;
    CHECK(mixed.addAxis(0, motor) == Result::OK);
    CHECK(mixed.addAxis(1, dc) == Result::OK);
    mixed.disable();
}

static void fourAxes() {
    Plant a, b(axis::Control::DC), c, d(axis::Control::DC);
    a.home(); b.home(); c.home(); d.home();
    uint32_t now = a.now;
    if (b.now > now) now = b.now;
    if (c.now > now) now = c.now;
    if (d.now > now) now = d.now;
    Plant* motors[] = {&a, &b, &c, &d};
    planner::Planner<4, 4> p;
    for (uint8_t i = 0; i < 4; ++i) {
        motors[i]->now = now;
        motors[i]->port().update(now);
        CHECK(p.addAxis(i, motors[i]->port()) == Result::OK);
    }
    const double target[] = {2, 4, 1, d.core.getPosition()};
    CHECK(p.addTarget(target, true) == Result::OK); CHECK(p.startAt(now) == Result::OK);
    for (unsigned n = 0; n < 20000 && p.isBusy(); ++n) {
        for (uint8_t i = 0; i < 4; ++i) motors[i]->advance();
        now += 1000; p.tickAt(now);
        CHECK(p.getError() == planner::Error::NONE);
        NEAR(p.getReference(3).velocity, 0.0, 1e-12);
    }
    CHECK(!p.isBusy()); CHECK(p.ready());
    for (uint8_t i = 0; i < 4; ++i) NEAR(motors[i]->core.getPosition(), target[i], 0.02);
}

static void cornersAndDeceleration() {
    for (unsigned mode = 0; mode < 2; ++mode) {
        Plant x, y; x.home(); y.home(); align(x, y);
        planner::Planner<2, 8> p;
        CHECK(p.addAxis(0, x.port()) == Result::OK); CHECK(p.addAxis(1, y.port()) == Result::OK);
        CHECK(p.setMaxSpeed(3) == Result::OK);
        CHECK(p.setAcceleration(7) == Result::OK); CHECK(p.setDeceleration(3) == Result::OK);
        CHECK(p.setDtA(mode ? 0.3 : 0.0) == Result::OK);
        const double route[][2] = {{2, 0.1}, {2, 2}, {4, 2}, {4.001, 2.001}, {4.002, 2.002}, {6, 4}, {1, 4}};
        for (unsigned n = 0; n < 7; ++n) CHECK(p.addTarget(route[n]) == Result::OK);
        CHECK(p.startAt(x.now) == Result::OK);
        double speedBefore = 0.0;
        uint8_t previousCount = p.queued();
        bool crossed = false;
        for (unsigned n = 0; n < 30000 && p.isBusy(); ++n) {
            cycle(p, x, y);
            CHECK(p.getError() == planner::Error::NONE);
            CHECK(p.getPathSpeed() <= 3.000001);
            axis::MotionState xr = p.getReference(0), yr = p.getReference(1);
            CHECK(fabs(xr.acceleration) <= 7.000001); CHECK(fabs(yr.acceleration) <= 7.000001);
            if (p.queued() != previousCount && previousCount == 7) {
                crossed = true;
                if (mode) CHECK(speedBefore > 0.1);
                else CHECK(speedBefore < 0.05);
            }
            speedBefore = p.getPathSpeed(); previousCount = p.queued();
        }
        CHECK(crossed); CHECK(!p.isBusy()); CHECK(p.ready());
        NEAR(x.core.getPosition(), 1, 0.02); NEAR(y.core.getPosition(), 4, 0.02);
    }
}

int main() {
    arguments(); mixedMotion(); streamingAndStops(); stopping();
    faultsAndOwnership(); shortSegmentsAndWrap(); realAdapters(); fourAxes(); cornersAndDeceleration();
    printf("PASS: %u checks\n", checks);
}
