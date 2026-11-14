#pragma once

// Buffered line planner inspired by GyverPlanner2, adapted to Axis::syncPort().
// Copyright (c) 2021 AlexGyver (original planning algorithm)
// Copyright (c) 2026 Planner contributors. MIT; see LICENSE.
#include <AxisSync.h>
#include <math.h>
#include <float.h>

namespace planner {

enum class Coordinates : uint8_t { ABSOLUTE, RELATIVE };
enum class Status : uint8_t { IDLE, RUNNING, SETTLING, STOPPING, FAULT };
enum class Error : uint8_t {
    NONE, AXIS_FAULT, INTERRUPTED, REFERENCE_LIMIT, TIMING, NUMERIC
};

// Physical units, one control task, no heap allocation. The same port interface
// supports DC axes, steppers with/without encoders, and mixtures of those axes.
template <uint8_t Axes, uint8_t Buffer = 8>
class Planner {
    static_assert(Axes > 0, "At least one axis is required");
    static_assert(Buffer > 0, "At least one buffered target is required");
 public:
    typedef axis::Result Result;
    Planner() {}
    Planner(const Planner&) = delete;
    Planner& operator=(const Planner&) = delete;

    Result addAxis(uint8_t index, axis::AxisPort port) {
        if (isBusy() || _count) return Result::BUSY;
        if (index >= Axes || !port.valid()) return Result::INVALID_ARGUMENT;
        for (uint16_t i = 0; i < Axes; ++i)
            if (i != index && _axes[i].core == port.core) return Result::INVALID_ARGUMENT;
        _axes[index] = port;
        return Result::OK;
    }
    template <class AxisType>
    Result addAxis(uint8_t index, AxisType& motor) { return addAxis(index, motor.syncPort()); }

    Result setMaxSpeed(double value) { return setting(_speed, value); }
    Result setAcceleration(double value) { return setting(_acceleration, value); }
    Result setDeceleration(double value) { return setting(_deceleration, value); }
    Result setDtA(double value) {
        if (isBusy() || _count) return Result::BUSY;
        if (!isfinite(value) || value < 0.0 || value > 1.0) return Result::INVALID_ARGUMENT;
        _dtA = value; return Result::OK;
    }

    // Relative coordinates are relative to the last QUEUED target.
    // A stop point holds until resumeAt(); a buffer tail always ends at zero speed.
    Result addTarget(const double (&target)[Axes], bool stopPoint = false,
                     Coordinates coordinates = Coordinates::ABSOLUTE) {
        if (_error != Error::NONE) return Result::FAULT_ACTIVE;
        if (_status == Status::STOPPING || (_status == Status::SETTLING && _stopped)) return Result::BUSY;
        if (!complete()) return Result::NOT_INITIALIZED;
        if (_count == Buffer) return Result::BUSY;
        if (coordinates != Coordinates::ABSOLUTE && coordinates != Coordinates::RELATIVE)
            return Result::INVALID_ARGUMENT;
        double start[Axes];
        for (uint16_t i = 0; i < Axes; ++i)
            start[i] = _count ? segment(_count - 1).target[i] :
                       (isBusy() ? _position[i] : _axes[i].core->getPosition());
        Segment next;
        next.stop = stopPoint;
        for (uint16_t i = 0; i < Axes; ++i) {
            next.target[i] = target[i] + (coordinates == Coordinates::RELATIVE ? start[i] : 0.0);
            const axis::Config& c = _axes[i].core->getConfig();
            if (!isfinite(next.target[i])) return Result::INVALID_ARGUMENT;
            if (c.softLimitsEnabled && (next.target[i] < c.softMin || next.target[i] > c.softMax))
                return Result::OUT_OF_RANGE;
        }
        Result result = geometry(next, start);
        if (result != Result::OK) return result;
        // Reject duplicates explicitly; otherwise zero-length vectors would
        // make corner angles undefined and could erase a requested stop.
        if (next.length == 0.0) return Result::INVALID_ARGUMENT;
        segment(_count) = next; ++_count;
        replan(isBusy() && _status == Status::RUNNING ? 1 : 0);
        return Result::OK;
    }

    Result startAt(uint32_t now) {
        if (_error != Error::NONE) return Result::FAULT_ACTIVE;
        if (isBusy()) return Result::BUSY;
        if (!complete()) return Result::NOT_INITIALIZED;
        if (!_count) return Result::INVALID_ARGUMENT;
        // Do not take ownership from another coordinator or local motion.
        for (uint16_t i = 0; i < Axes; ++i)
            if (_axes[i].core->isBusy()) return Result::BUSY;
        _period = _settleLimit = 0;
        for (uint16_t i = 0; i < Axes; ++i) {
            _axes[i].update(now);
            _axes[i].commit();
            Result result = _axes[i].core->canSynchronize();
            if (result != Result::OK) return result;
            const axis::Config& c = _axes[i].core->getConfig();
            // A second-order profile cannot satisfy a finite jerk bound.
            if (c.maxJerk != 0.0) return Result::UNSUPPORTED;
            _position[i] = _axes[i].core->getPosition();
            if (c.controlPeriodUs > _period) _period = c.controlPeriodUs;
            if (c.settleTimeUs > _settleLimit) _settleLimit = c.settleTimeUs;
        }
        for (uint16_t i = 0; i < Axes; ++i)
            if (_axes[i].core->getConfig().maxTickGapUs < _period) return Result::INVALID_ARGUMENT;
        // Positions/limits may have changed since enqueue, e.g. after homing.
        for (uint16_t n = 0; n < _count; ++n) {
            Segment& s = segment(n);
            const double* origin = n ? segment(n - 1).target : _position;
            for (uint16_t i = 0; i < Axes; ++i) {
                const axis::Config& c = _axes[i].core->getConfig();
                if (c.softLimitsEnabled && (s.target[i] < c.softMin || s.target[i] > c.softMax))
                    return Result::OUT_OF_RANGE;
            }
            Result result = geometry(s, origin);
            if (result != Result::OK) return result;
        }
        replan(0);
        for (uint16_t i = 0; i < Axes; ++i) {
            Result result = _axes[i].core->beginSynchronization();
            if (result != Result::OK) {
                for (uint16_t j = 0; j < i; ++j) _axes[j].core->releaseSynchronization(false);
                commit(); return result;
            }
        }
        _status = Status::RUNNING; _last = now; _settle = 0; _ready = false;
        beginSegment();
        return _status == Status::FAULT ? Result::FAULT_ACTIVE : Result::OK;
    }
    Result resumeAt(uint32_t now) { return startAt(now); }

    // Call frequently, including between control cycles: AxisStepper emits
    // pulses in commit(), and Axis scans limits on every input update.
    bool tickAt(uint32_t now) {
        if (!isBusy()) {
            for (uint16_t i = 0; i < Axes; ++i) if (_axes[i].valid()) {
                if (_axes[i].core->getMode() == axis::Mode::SYNCHRONIZED) continue;
                _axes[i].update(now); _axes[i].commit();
            }
            return false;
        }
        for (uint16_t i = 0; i < Axes; ++i) {
            if (_axes[i].core->getMode() != axis::Mode::SYNCHRONIZED) {
                fail(_axes[i].core->getFault() == axis::Fault::NONE ? Error::INTERRUPTED : Error::AXIS_FAULT);
                return false;
            }
            if (static_cast<uint32_t>(now - _last) > _axes[i].core->getConfig().maxTickGapUs) {
                fail(Error::TIMING); return false;
            }
        }
        uint32_t elapsed = now - _last;
        bool due = elapsed >= _period;
        if (due) {
            _last = now;
            if (_status == Status::RUNNING) advance(elapsed * 1e-6);
            else if (_status == Status::STOPPING) advanceStop(elapsed * 1e-6);
            if (_status == Status::FAULT) return false;
        }
        // Compute and validate ALL references before updating ANY outputs.
        // Then read/control all axes before committing the first motor.
        for (uint16_t i = 0; i < Axes; ++i) {
            axis::ExternalReference reference = axisReference(i, now, due);
            if (_axes[i].core->validateReference(reference) != Result::OK) {
                fail(Error::REFERENCE_LIMIT); return false;
            }
        }
        for (uint16_t i = 0; i < Axes; ++i) {
            axis::ExternalReference reference = axisReference(i, now, due);
            _axes[i].update(now, &reference);
            if (_axes[i].core->getFault() != axis::Fault::NONE) { fail(Error::AXIS_FAULT); return false; }
        }
        if (due && _status == Status::SETTLING) {
            bool settled = true;
            for (uint16_t i = 0; i < Axes; ++i) {
                const axis::Config& c = _axes[i].core->getConfig();
                if (fabs(_axes[i].core->getPosition() - _position[i]) > c.positionTolerance ||
                    fabs(_axes[i].core->getVelocity()) > c.velocityTolerance) settled = false;
            }
            _settle = settled ? saturate(_settle, elapsed, _settleLimit) : 0;
            if (settled && _settle >= _settleLimit) {
                for (uint16_t i = 0; i < Axes; ++i)
                    _axes[i].core->finishSynchronization(_stopped ? axis::Completion::STOPPED : axis::Completion::REACHED);
                _status = Status::IDLE; _ready = !_stopped;
            }
        }
        commit();
        return isBusy();
    }

    // Decelerate along the queued route, crossing a junction only if necessary
    // for braking. Discard the remainder when speed reaches zero.
    Result stop() {
        if (_error != Error::NONE) return Result::FAULT_ACTIVE;
        if (_status == Status::STOPPING) return Result::OK;
        _stopped = true; _ready = false;
        if (_status == Status::RUNNING && _velocity > 0.0) _status = Status::STOPPING;
        else {
            _head = _count = 0;
            if (isBusy()) _status = Status::SETTLING;
        }
        return Result::OK;
    }
    void brake() {
        for (uint16_t i = 0; i < Axes; ++i) if (_axes[i].valid()) _axes[i].core->brake();
        _head = _count = 0; _velocity = _pathAcceleration = 0.0;
        _status = _error == Error::NONE ? Status::IDLE : Status::FAULT; _ready = false; commit();
    }
    void disable() {
        for (uint16_t i = 0; i < Axes; ++i) if (_axes[i].valid()) _axes[i].core->disable();
        _head = _count = 0; _velocity = _pathAcceleration = 0.0;
        _status = _error == Error::NONE ? Status::IDLE : Status::FAULT; _ready = false; commit();
    }
    void emergencyStop() {
        for (uint16_t i = 0; i < Axes; ++i) if (_axes[i].valid()) _axes[i].core->emergencyStop();
        _error = Error::AXIS_FAULT; _status = Status::FAULT;
        _head = _count = 0; _velocity = _pathAcceleration = 0.0; _ready = false; commit();
    }
    Result clearFaultAt(uint32_t now) {
        if (isBusy()) return Result::BUSY;
        if (!complete()) return Result::NOT_INITIALIZED;
        for (uint16_t i = 0; i < Axes; ++i)
            if (_axes[i].core->isBusy()) return Result::BUSY;
        Result result = Result::OK;
        for (uint16_t i = 0; i < Axes; ++i) {
            uint32_t count;
            axis::Inputs inputs;
            if (!_axes[i].read(_axes[i].context, count, inputs)) {
                _axes[i].core->synchronizationFault();
                result = Result::FAULT_ACTIVE; continue;
            }
            _axes[i].core->tick(now, count, inputs);
            Result r = _axes[i].core->clearFault();
            if (r != Result::OK) result = r;
        }
        commit();
        if (result == Result::OK) {
            _error = Error::NONE; _status = Status::IDLE; _head = _count = 0;
            _ready = false; _velocity = _pathAcceleration = 0.0;
        }
        return result;
    }
    Result clearBuffer() {
        if (isBusy()) return Result::BUSY;
        _head = _count = 0; return Result::OK;
    }
    bool available() const { return complete() && _count < Buffer && _error == Error::NONE &&
                                   _status != Status::STOPPING && !(_status == Status::SETTLING && _stopped); }
    uint8_t queued() const { return _count; }
    uint8_t freeSlots() const { return Buffer - _count; }
    bool isBusy() const { return _status == Status::RUNNING || _status == Status::SETTLING || _status == Status::STOPPING; }
    bool ready() { bool value = _ready; _ready = false; return value; }
    Status getStatus() const { return _status; }
    Error getError() const { return _error; }
    double getPathSpeed() const { return _velocity; }
    uint32_t getPeriod() const { return _period; }
    axis::MotionState getReference(uint8_t index) const {
        return index < Axes ? axisReference(index, _last, true).state : axis::MotionState();
    }

 private:
    struct Segment {
        double target[Axes] = {}, direction[Axes] = {};
        double length = 0.0, speed = 0.0, acceleration = 0.0, deceleration = 0.0;
        double entry = 0.0, exit = 0.0;
        bool stop = false;
    };
    axis::AxisPort _axes[Axes];
    Segment _segments[Buffer];
    uint8_t _head = 0, _count = 0;
    Status _status = Status::IDLE;
    Error _error = Error::NONE;
    double _speed = 20.0, _acceleration = 100.0, _deceleration = 100.0, _dtA = 0.3;
    double _position[Axes] = {}, _origin[Axes] = {};
    double _time = 0.0, _distance = 0.0, _velocity = 0.0, _pathAcceleration = 0.0;
    double _peak = 0.0, _tAccel = 0.0, _tCruise = 0.0, _tDecel = 0.0;
    uint32_t _last = 0, _period = 0, _settleLimit = 0, _settle = 0;
    bool _ready = false, _stopped = false;

    Segment& segment(uint16_t offset) { return _segments[(static_cast<uint16_t>(_head) + offset) % Buffer]; }
    const Segment& segment(uint16_t offset) const { return _segments[(static_cast<uint16_t>(_head) + offset) % Buffer]; }
    bool complete() const {
        for (uint16_t i = 0; i < Axes; ++i) if (!_axes[i].valid() || !_axes[i].core->isInitialized()) return false;
        return true;
    }
    Result setting(double& destination, double value) {
        if (isBusy() || _count) return Result::BUSY;
        if (!isfinite(value) || value <= 0.0) return Result::INVALID_ARGUMENT;
        destination = value; return Result::OK;
    }
    static uint32_t saturate(uint32_t value, uint32_t elapsed, uint32_t limit) {
        return value >= limit || elapsed >= limit - value ? limit : value + elapsed;
    }
    Result geometry(Segment& s, const double* start) const {
        // Scaled norm avoids squaring large coordinates before validation.
        double scale = 0.0;
        for (uint16_t i = 0; i < Axes; ++i) {
            s.direction[i] = s.target[i] - start[i];
            if (!isfinite(s.direction[i])) return Result::INVALID_ARGUMENT;
            scale = fmax(scale, fabs(s.direction[i]));
        }
        double sum = 0.0;
        if (scale > 0.0) for (uint16_t i = 0; i < Axes; ++i) {
            double d = s.direction[i] / scale; sum += d * d;
        }
        s.length = scale * sqrt(sum);
        if (!isfinite(s.length)) return Result::INVALID_ARGUMENT;
        s.speed = _speed; s.acceleration = _acceleration; s.deceleration = _deceleration;
        for (uint16_t i = 0; i < Axes; ++i) {
            s.direction[i] = s.length > 0.0 ? s.direction[i] / s.length : 0.0;
            double projection = fabs(s.direction[i]);
            if (projection == 0.0) continue;
            const axis::Config& c = _axes[i].core->getConfig();
            s.speed = fmin(s.speed, c.maxSpeed / projection);
            s.acceleration = fmin(s.acceleration, c.acceleration / projection);
            s.deceleration = fmin(s.deceleration, c.deceleration / projection);
        }
        // Reject nonrepresentable profile arithmetic, including on AVR where
        // double has float precision. No wraparound-prone integer squares.
        if (!isfinite(s.speed * s.speed) || s.speed * s.speed == 0.0 ||
            !isfinite(2.0 * s.acceleration * s.length) ||
            !isfinite(2.0 * s.deceleration * s.length) ||
            !isfinite(s.acceleration + s.deceleration)) return Result::INVALID_ARGUMENT;
        return Result::OK;
    }
    double junction(const Segment& a, const Segment& b) const {
        if (a.stop || a.length == 0.0 || b.length == 0.0) return 0.0;
        double dot = 0.0;
        for (uint16_t i = 0; i < Axes; ++i) dot += a.direction[i] * b.direction[i];
        dot = fmax(-1.0, fmin(1.0, dot));
        // Collinear continuation preserves feed; reversal must stop.
        if (dot <= -0.999999) return 0.0;
        double limit = fmin(a.speed, b.speed);
        if (dot < 0.999999) {
            double acceleration = fmin(fmin(a.acceleration, a.deceleration),
                                       fmin(b.acceleration, b.deceleration));
            limit = fmin(limit, acceleration * _dtA / sqrt(2.0 * (1.0 - dot)));
        }
        return limit;
    }
    void replan(uint16_t first) {
        if (first >= _count) return;
        for (uint16_t n = first; n < _count; ++n)
            segment(n).exit = n + 1 < _count ? junction(segment(n), segment(n + 1)) : 0.0;
        // Reverse braking pass, then forward acceleration pass (GyverPlanner2).
        for (uint16_t n = _count; n-- > first;) {
            Segment& s = segment(n);
            s.entry = fmin(s.speed, sqrt(s.exit * s.exit + 2.0 * s.deceleration * s.length));
            if (n > first) segment(n - 1).exit = fmin(segment(n - 1).exit, s.entry);
        }
        double entry = first ? segment(first - 1).exit : 0.0;
        for (uint16_t n = first; n < _count; ++n) {
            Segment& s = segment(n); s.entry = entry;
            s.exit = fmin(s.exit, sqrt(entry * entry + 2.0 * s.acceleration * s.length));
            entry = s.exit;
        }
    }
    void beginSegment() {
        Segment& s = segment(0);
        for (uint16_t i = 0; i < Axes; ++i) _origin[i] = _position[i];
        _time = _distance = 0.0; _velocity = s.entry; _pathAcceleration = 0.0; _stopped = false;
        // Stable form of the triangular peak with unequal accel/decel.
        double weight = s.acceleration / (s.acceleration + s.deceleration);
        double peakSquared = (1.0 - weight) * s.entry * s.entry + weight * s.exit * s.exit +
                             2.0 * weight * s.deceleration * s.length;
        _peak = fmin(s.speed, sqrt(fmax(0.0, peakSquared)));
        _tAccel = fmax(0.0, (_peak - s.entry) / s.acceleration);
        _tDecel = fmax(0.0, (_peak - s.exit) / s.deceleration);
        double acceleratingDistance = (s.entry + _peak) * _tAccel * 0.5;
        double deceleratingDistance = (s.exit + _peak) * _tDecel * 0.5;
        _tCruise = _peak > 0.0 ? fmax(0.0, (s.length - acceleratingDistance - deceleratingDistance) / _peak) : 0.0;
        if (!isfinite(_tAccel + _tCruise + _tDecel)) fail(Error::NUMERIC);
    }
    void advance(double seconds) {
        // A delayed call may cross several short lines. Work is bounded by
        // Buffer and the emitted reference remains on the analytic trajectory.
        while (_count) {
            Segment& s = segment(0);
            double duration = _tAccel + _tCruise + _tDecel;
            double remaining = fmax(0.0, duration - _time);
            if (seconds >= remaining) {
                seconds -= remaining;
                for (uint16_t i = 0; i < Axes; ++i) _position[i] = s.target[i];
                bool hold = s.stop || _count == 1;
                _velocity = s.exit; _pathAcceleration = 0.0;
                _head = (static_cast<uint16_t>(_head) + 1) % Buffer; --_count;
                if (hold) { _velocity = 0.0; _status = Status::SETTLING; _settle = 0; return; }
                beginSegment();
                if (_status == Status::FAULT) return;
                continue;
            }
            _time += seconds;
            if (_time < _tAccel) {
                _pathAcceleration = s.acceleration;
                _velocity = s.entry + s.acceleration * _time;
                _distance = s.entry * _time + 0.5 * s.acceleration * _time * _time;
            } else {
                _distance = (s.entry + _peak) * _tAccel * 0.5;
                double t = _time - _tAccel;
                if (t < _tCruise) {
                    _pathAcceleration = 0.0; _velocity = _peak; _distance += _peak * t;
                } else {
                    t -= _tCruise; _pathAcceleration = -s.deceleration;
                    _velocity = fmax(s.exit, _peak - s.deceleration * t);
                    _distance += _peak * _tCruise + _peak * t - 0.5 * s.deceleration * t * t;
                }
            }
            _distance = fmin(s.length, fmax(0.0, _distance));
            for (uint16_t i = 0; i < Axes; ++i) _position[i] = _origin[i] + s.direction[i] * _distance;
            return;
        }
    }
    void advanceStop(double seconds) {
        while (_count) {
            Segment& s = segment(0);
            double remaining = fmax(0.0, s.length - _distance);
            double endSpeed = sqrt(fmax(0.0, _velocity * _velocity - 2.0 * s.deceleration * remaining));
            double stopTime = _velocity / s.deceleration;
            double boundaryTime = endSpeed > 0.0 ? 2.0 * remaining / (_velocity + endSpeed) : stopTime;
            double dt = fmin(seconds, boundaryTime);
            _distance += _velocity * dt - 0.5 * s.deceleration * dt * dt;
            _velocity = fmax(0.0, _velocity - s.deceleration * dt);
            _pathAcceleration = -s.deceleration;
            for (uint16_t i = 0; i < Axes; ++i) _position[i] = _origin[i] + s.direction[i] * _distance;
            if (seconds < boundaryTime) return;
            if (endSpeed == 0.0 || _count == 1) {
                _velocity = _pathAcceleration = 0.0; _head = _count = 0;
                _status = Status::SETTLING; _settle = 0; return;
            }
            seconds -= boundaryTime;
            for (uint16_t i = 0; i < Axes; ++i) _origin[i] = _position[i] = s.target[i];
            _head = (static_cast<uint16_t>(_head) + 1) % Buffer; --_count;
            _distance = 0.0; _velocity = endSpeed;
        }
    }
    axis::ExternalReference axisReference(uint8_t index, uint32_t now, bool due) const {
        axis::ExternalReference reference;
        reference.timestampUs = now; reference.updateControl = due;
        reference.state.position = _position[index];
        double direction = (_status == Status::RUNNING || _status == Status::STOPPING) && _count ?
                           segment(0).direction[index] : 0.0;
        reference.state.velocity = direction * _velocity;
        reference.state.acceleration = direction * _pathAcceleration;
        return reference;
    }
    void commit() { for (uint16_t i = 0; i < Axes; ++i) if (_axes[i].valid()) _axes[i].commit(); }
    void fail(Error error) {
        if (_error == Error::NONE) _error = error;
        _status = Status::FAULT; _head = _count = 0; _ready = false;
        _velocity = _pathAcceleration = 0.0;
        for (uint16_t i = 0; i < Axes; ++i) _axes[i].core->synchronizationFault();
        commit();
    }
};
}  // namespace planner
