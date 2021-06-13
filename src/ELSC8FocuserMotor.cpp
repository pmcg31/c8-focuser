#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>

#include "ELSC8FocuserMotor.hpp"

namespace ELS
{
    int C8FocuserMotor::g_fastSpeedRevTimeSecs = 5;
    int C8FocuserMotor::g_slowSpeedRevTimeSecs = 15;

    int C8FocuserMotor::g_c8FocuserMaxTurns = 40;

    uint32_t C8FocuserMotor::g_stepsPerRev[] = {
        6400,  /* MS_X8 */
        12800, /* MS_X16 */
        25600, /* MS_X32 */
        51200  /* MS_X64 */
    };

    bool C8FocuserMotor::g_microstepValues[][2] = {
        {false, false}, /* MS_X8 */
        {true, true},   /* MS_X16 */
        {true, false},  /* MS_X32 */
        {false, true}   /* MS_X64 */
    };

    C8FocuserMotor::C8FocuserMotor(int tmc2209EnablePin,
                                   int tmc2209StepPin,
                                   int tmc2209DirPin,
                                   int tmc2209Ms1Pin,
                                   int tmc2209Ms2Pin)
        : _enablePin(tmc2209EnablePin),
          _stepPin(tmc2209StepPin),
          _dirPin(tmc2209DirPin),
          _ms1Pin(tmc2209Ms1Pin),
          _ms2Pin(tmc2209Ms2Pin),
          _isEnabled(false),
          _isReversed(false),
          _microsteps(MS_X64),
          _direction(FD_FOCUS_INWARD),
          _speed(FS_NORMAL),
          _position(0),
          _maxPosition(0),
          _backlashEnabled(true),
          _backlashSteps(650),
          _stepsPerRev(0),
          _usPerStepSlow(0),
          _usPerStepFast(0),
          _moveInfo(0),
          _motorTest(0),
          _comms(0),
          _writer(0),
          _commsListener(0)
    {
        // Set up output pins (inputs to
        // tmc2209 driver)
        pinMode(_enablePin, OUTPUT);
        pinMode(_stepPin, OUTPUT);
        pinMode(_dirPin, OUTPUT);
        pinMode(_ms1Pin, OUTPUT);
        pinMode(_ms2Pin, OUTPUT);

        // Disable motor for now
        digitalWrite(_enablePin, HIGH);
        digitalWrite(_stepPin, LOW);

        esp_timer_create_args_t timerCreateArgs;
        timerCreateArgs.callback = &C8FocuserMotor::moveNextRedirect;
        timerCreateArgs.arg = this;
        esp_timer_create(&timerCreateArgs, &_moveTimer);

        if (!SPIFFS.begin(true))
        {
            Serial2.println("Failed to start SPIFFS");
            return;
        }

        _stepsPerRev = g_stepsPerRev[(int)_microsteps - 1];
        _usPerStepSlow = g_slowSpeedRevTimeSecs * 1000000 / _stepsPerRev;
        _usPerStepFast = g_fastSpeedRevTimeSecs * 1000000 / _stepsPerRev;
        _maxPosition = g_c8FocuserMaxTurns * _stepsPerRev;
        _position = _maxPosition / 2;

        if (!updateFieldsFromConfig())
        {
            updateConfigFromFields();
        }

        // Initialize tmc2209 pins
        syncEnabled();
        syncDir();
        syncMicrosteps();

        Serial2.printf("Motor is %sREVERSED\r\n", _isReversed ? "" : "NOT ");
        Serial2.print("Direction is ");
        switch (_direction)
        {
        case FD_FOCUS_INWARD:
            Serial2.println("INWARD");
            break;
        case FD_FOCUS_OUTWARD:
            Serial2.println("OUTWARD");
            break;
        }
        Serial2.print("Microsteps is ");
        switch (_microsteps)
        {
        case MS_X8:
            Serial2.println("X8");
            break;
        case MS_X16:
            Serial2.println("X16");
            break;
        case MS_X32:
            Serial2.println("X32");
            break;
        case MS_X64:
            Serial2.println("X64");
            break;
        }
        Serial2.printf("Max position is %u\r\n", _maxPosition);
        Serial2.printf("Position is %u\r\n", _position);
    }

    C8FocuserMotor::~C8FocuserMotor()
    {
        esp_timer_delete(_moveTimer);
    }

    bool C8FocuserMotor::isEnabled() const
    {
        return _isEnabled;
    }

    bool C8FocuserMotor::isReversed() const
    {
        return _isReversed;
    }

    Microsteps C8FocuserMotor::getMicrosteps() const
    {
        return _microsteps;
    }

    FocusDirection C8FocuserMotor::getDirection() const
    {
        return _direction;
    }

    FocusSpeed C8FocuserMotor::getSpeed() const
    {
        return _speed;
    }

    bool C8FocuserMotor::isBacklashEnabled() const
    {
        return _backlashEnabled;
    }

    uint32_t C8FocuserMotor::getBacklashSteps() const
    {
        return _backlashSteps;
    }

    void C8FocuserMotor::enableMotor(bool isEnabled)
    {
        if (isEnabled != _isEnabled)
        {
            _isEnabled = isEnabled;

            syncEnabled();
        }
    }

    void C8FocuserMotor::reverseMotor(bool isReversed)
    {
        if (isReversed != _isReversed)
        {
            _isReversed = isReversed;

            syncDir();

            updateConfigFromFields();
        }
    }

    void C8FocuserMotor::setMicrosteps(Microsteps microsteps)
    {
        if (_microsteps != microsteps)
        {
            _microsteps = microsteps;

            _stepsPerRev = g_stepsPerRev[(int)_microsteps - 1];
            _usPerStepSlow = g_slowSpeedRevTimeSecs * 1000000 / _stepsPerRev;
            _usPerStepFast = g_fastSpeedRevTimeSecs * 1000000 / _stepsPerRev;

            uint32_t oldMax = _maxPosition;
            uint32_t oldPos = _position;
            _maxPosition = g_c8FocuserMaxTurns * _stepsPerRev;
            if ((oldPos != 0) && (oldMax != 0))
            {
                _position = (uint32_t)(((double)oldPos / (double)oldMax) * (double)_maxPosition);
            }
            else
            {
                _position = _maxPosition / 2;
            }

            syncMicrosteps();

            updateConfigFromFields();
        }
    }

    void C8FocuserMotor::setDirection(FocusDirection direction)
    {
        if (_direction != direction)
        {
            _direction = direction;

            syncDir();

            updateConfigFromFields();
        }
    }

    void C8FocuserMotor::setSpeed(FocusSpeed speed)
    {
        if (_speed != speed)
        {
            _speed = speed;

            if (_comms != 0)
            {
                _comms->speed(_speed);
            }

            updateConfigFromFields();
        }
    }

    void C8FocuserMotor::enableBacklash(bool isEnabled)
    {
        if (_backlashEnabled != isEnabled)
        {
            _backlashEnabled = isEnabled;

            if (_comms != 0)
            {
                _comms->backlashEnabled(_backlashEnabled);
            }

            updateConfigFromFields();
        }
    }

    void C8FocuserMotor::setBacklashSteps(uint32_t steps)
    {
        if (_backlashSteps != steps)
        {
            _backlashSteps = steps;

            if (_comms != 0)
            {
                _comms->backlashSteps(_backlashSteps);
            }

            updateConfigFromFields();
        }
    }

    void C8FocuserMotor::testMotor(int numCycles)
    {
        if (_isEnabled)
        {
            if (_motorTest != 0)
            {
                if (_motorTest->isRunning())
                {
                    return;
                }

                delete _motorTest;
            }

            _motorTest = new MotorTest(this, numCycles);
            _motorTest->run();
        }
    }

    bool C8FocuserMotor::startComms()
    {
        _writer = new FCWriter();
        _commsListener = new CommsListener(this);
        _comms = new FocuserComms(_writer, _commsListener);

        return (xTaskCreate(readTaskRedirect,
                            "Comms Reader",
                            8192,
                            this,
                            1,
                            &_readTaskHandle) == pdPASS);
    }

    bool C8FocuserMotor::updateFieldsFromConfig()
    {
        File configFile = SPIFFS.open("/config.json", "r");
        DynamicJsonDocument doc(1024);
        if (!configFile)
        {
            Serial2.println("No config file (yet!)");
            return false;
        }
        else
        {
            DeserializationError error = deserializeJson(doc, configFile);
            configFile.close();
            if (error)
            {
                Serial2.println("Config file failed to parse");
                return false;
            }
            else
            {
                JsonVariant docPosition = doc["position"];
                JsonVariant docSpeed = doc["speed"];
                JsonVariant docDirection = doc["direction"];
                JsonVariant docMicrosteps = doc["microsteps"];
                JsonVariant docReversed = doc["reversed"];
                JsonVariant docBacklashEnabled = doc["backlashEnabled"];
                JsonVariant docBacklashSteps = doc["backlashSteps"];

                if ((docPosition.isUndefined() || docPosition.isNull()) ||
                    (docSpeed.isUndefined() || docSpeed.isNull()) ||
                    (docDirection.isUndefined() || docDirection.isNull()) ||
                    (docMicrosteps.isUndefined() || docMicrosteps.isNull()) ||
                    (docReversed.isUndefined() || docReversed.isNull()) ||
                    (docBacklashEnabled.isUndefined() || docBacklashEnabled.isNull()) ||
                    (docBacklashSteps.isUndefined() || docBacklashSteps.isNull()))
                {
                    Serial2.println("At least one field is missing or empty in config");
                    return false;
                }
                uint32_t tmpPosition = docPosition.as<unsigned int>();
                int tmpSpeedIdx = docSpeed.as<int>();
                int tmpDirectionIdx = docDirection.as<int>();
                int tmpMicrostepsIdx = docMicrosteps.as<int>();
                bool tmpReversed = docReversed.as<bool>();
                bool tmpBacklashEnabled = docBacklashEnabled.as<bool>();
                uint32_t tmpBacklashSteps = docBacklashSteps.as<unsigned int>();

                Serial2.printf("pos: %u speed: %d dir: %d micro: %d rev: %s backlash: %s backlashSteps: %u\r\n",
                               tmpPosition,
                               tmpSpeedIdx,
                               tmpDirectionIdx,
                               tmpMicrostepsIdx,
                               tmpReversed ? "true" : "false",
                               tmpBacklashEnabled ? "true" : "false",
                               tmpBacklashSteps);

                if (tmpMicrostepsIdx != (int)_microsteps)
                {
                    _microsteps = (Microsteps)tmpMicrostepsIdx;

                    syncMicrosteps();
                }

                if (tmpDirectionIdx != (int)_direction)
                {
                    _direction = (FocusDirection)tmpDirectionIdx;

                    syncDir();
                }

                if (tmpSpeedIdx != (int)_speed)
                {
                    _speed = (FocusSpeed)tmpSpeedIdx;
                }

                _position = tmpPosition;
                _isReversed = tmpReversed;

                _backlashEnabled = tmpBacklashEnabled;
                _backlashSteps = tmpBacklashSteps;
            }
        }

        return true;
    }

    bool C8FocuserMotor::updateConfigFromFields()
    {
        File configFile = SPIFFS.open("/config.json", "w");
        DynamicJsonDocument doc(1024);
        if (!configFile)
        {
            Serial2.println("Couldn't open config file for write");
            return false;
        }
        else
        {
            doc["position"] = _position;
            doc["speed"] = (int)_speed;
            doc["direction"] = (int)_direction;
            doc["microsteps"] = (int)_microsteps;
            doc["reversed"] = _isReversed;
            doc["backlashEnabled"] = _backlashEnabled;
            doc["backlashSteps"] = _backlashSteps;

            serializeJson(doc, configFile);
        }

        configFile.close();

        return true;
    }

    void C8FocuserMotor::syncEnabled()
    {
        digitalWrite(_enablePin, _isEnabled ? LOW : HIGH);

        if (_comms != 0)
        {
            _comms->motorEnabled(_isEnabled);
        }
    }

    void C8FocuserMotor::syncDir()
    {
        int level = (_direction == FD_FOCUS_INWARD) ? LOW : HIGH;
        if (_isReversed)
        {
            level = (level == HIGH) ? LOW : HIGH;
        }

        digitalWrite(_dirPin, level);
    }

    void C8FocuserMotor::syncMicrosteps()
    {
        bool *vals = g_microstepValues[(int)_microsteps - 1];

        digitalWrite(_ms1Pin, vals[0] ? HIGH : LOW);
        digitalWrite(_ms2Pin, vals[1] ? HIGH : LOW);

        if (_comms != 0)
        {
            _comms->microsteps(_microsteps);
            _comms->maxPos(_maxPosition);
            _comms->position(_position);
        }
    }

    void C8FocuserMotor::step()
    {
        digitalWrite(_stepPin, HIGH);
        delayMicroseconds(1);
        digitalWrite(_stepPin, LOW);
        delayMicroseconds(1);
    }

    void C8FocuserMotor::startMoving(uint32_t stepCount,
                                     int usPerStep,
                                     MoveInfo::MoveDoneCallback callback,
                                     void *data)
    {
        uint32_t backlashSteps = 0;
        if (_direction == FD_FOCUS_OUTWARD)
        {
            if (_backlashEnabled)
            {
                backlashSteps = _backlashSteps;
            }
        }

        _moveInfo = new MoveInfo(stepCount + backlashSteps,
                                 backlashSteps,
                                 _direction,
                                 usPerStep,
                                 callback,
                                 data);

        esp_timer_start_periodic(_moveTimer, usPerStep);

        if (_comms != 0)
        {
            _comms->movingRel(_direction, stepCount);
        }
    }

    void C8FocuserMotor::unwindBacklash(uint32_t backlashSteps,
                                        int usPerStep,
                                        MoveInfo::MoveDoneCallback callback,
                                        void *data)
    {
        setDirection(FD_FOCUS_INWARD);
        _moveInfo = new MoveInfo(backlashSteps,
                                 backlashSteps,
                                 FD_FOCUS_INWARD,
                                 usPerStep,
                                 callback,
                                 data);

        esp_timer_start_periodic(_moveTimer, usPerStep);
    }

    void C8FocuserMotor::abortMove()
    {
        if (_moveInfo != 0)
        {
            _moveInfo->isAbort = true;
        }
    }

    void C8FocuserMotor::moveNext()
    {
        if (_moveInfo == 0)
        {
            esp_timer_stop(_moveTimer);
            return;
        }

        if (_moveInfo->isAbort)
        {
            // Only abort if we're not unwinding backlash
            if (_moveInfo->backlashSteps != _moveInfo->stepCount)
            {
                esp_timer_stop(_moveTimer);
                MoveInfo::MoveDoneCallback callback = _moveInfo->callback;
                void *data = _moveInfo->data;
                uint32_t backlashSteps = _moveInfo->backlashSteps;
                int usPerStep = _moveInfo->usPerStep;
                delete _moveInfo;
                _moveInfo = 0;
                updateConfigFromFields();

                if (backlashSteps != 0)
                {
                    // Unwind backlash
                    unwindBacklash(backlashSteps, usPerStep, callback, data);
                }
                else
                {
                    if (_comms != 0)
                    {
                        _comms->stopped(_position);
                    }

                    (*callback)(data, true);
                }

                return;
            }
        }

        if (_direction == FD_FOCUS_INWARD)
        {
            if (_position > 0)
            {
                step();
                _moveInfo->stepsTaken++;

                if (_moveInfo->stepsTaken <= (_moveInfo->stepCount - _moveInfo->backlashSteps))
                {
                    _position--;
                }
            }
            else
            {
                _moveInfo->isAbort = true;
            }
        }
        else
        {
            if (_position < _maxPosition)
            {
                step();
                _moveInfo->stepsTaken++;

                if (_moveInfo->stepsTaken <= (_moveInfo->stepCount - _moveInfo->backlashSteps))
                {
                    _position++;
                }
            }
            else
            {
                _moveInfo->isAbort = true;
            }
        }

        if (_moveInfo->isAbort)
        {
            // Only abort if we're not unwinding backlash
            if (_moveInfo->backlashSteps != _moveInfo->stepCount)
            {
                esp_timer_stop(_moveTimer);
                MoveInfo::MoveDoneCallback callback = _moveInfo->callback;
                void *data = _moveInfo->data;
                uint32_t backlashSteps = _moveInfo->backlashSteps;
                int usPerStep = _moveInfo->usPerStep;
                delete _moveInfo;
                _moveInfo = 0;
                updateConfigFromFields();

                if (backlashSteps != 0)
                {
                    // Unwind backlash
                    unwindBacklash(backlashSteps, usPerStep, callback, data);
                }
                else
                {
                    if (_comms != 0)
                    {
                        _comms->stopped(_position);
                    }

                    (*callback)(data, true);
                }

                return;
            }
        }

        if (_moveInfo->stepsTaken == _moveInfo->stepCount)
        {
            esp_timer_stop(_moveTimer);
            MoveInfo::MoveDoneCallback callback = _moveInfo->callback;
            void *data = _moveInfo->data;
            int usPerStep = _moveInfo->usPerStep;
            uint32_t backlashSteps = _moveInfo->backlashSteps;

            // Pretend there were no backlash steps if we're
            // unwinding backlash
            if (_moveInfo->backlashSteps == _moveInfo->stepCount)
            {
                backlashSteps = 0;
            }

            delete _moveInfo;
            _moveInfo = 0;
            updateConfigFromFields();

            if (backlashSteps != 0)
            {
                // Unwind backlash
                unwindBacklash(backlashSteps, usPerStep, callback, data);
            }
            else
            {
                if (_comms != 0)
                {
                    _comms->stopped(_position);
                }

                (*callback)(data, false);
            }
        }
    }

    /* static */ void IRAM_ATTR C8FocuserMotor::moveNextRedirect(void *obj)
    {
        ((C8FocuserMotor *)(obj))->moveNext();
    }

    void C8FocuserMotor::readTask()
    {
        const size_t bufSize = 1023;

        char buffer[bufSize + 1];
        buffer[bufSize] = 0;
        size_t bufLen = 0;

        size_t bytesAvailable;

        // This is a FreeRTOS task and thus must not be
        // allowed to return
        while (true)
        {
            if ((bytesAvailable = Serial.available()) != 0)
            {
                int bytesToRead = bytesAvailable;
                int spaceAvailable = bufSize - bufLen;
                if (bytesToRead > spaceAvailable)
                {
                    bytesToRead = spaceAvailable;
                }

                bufLen += Serial.readBytes(buffer + bufLen, bytesToRead);
                buffer[bufLen] = 0;

                char *currentSegment = buffer;
                int bytesLeft = bufLen;
                while (bytesLeft > 0)
                {
                    // Search for LF or CR/LF pair
                    int crIdx = -1;
                    int lfIdx = -1;
                    for (int i = 0; i < bytesLeft; i++)
                    {
                        if (currentSegment[i] == '\r')
                        {
                            crIdx = i;
                        }
                        if (currentSegment[i] == '\n')
                        {
                            lfIdx = i;
                            break;
                        }
                    }

                    // Check if found
                    if (lfIdx != -1)
                    {
                        int cmdLen = lfIdx;
                        if ((crIdx != -1) && ((crIdx + 1) == lfIdx))
                        {
                            cmdLen -= 1;
                        }
                        currentSegment[cmdLen] = 0;

                        if (_comms != 0)
                        {
                            _comms->processLine(currentSegment);
                        }

                        currentSegment += lfIdx + 1;
                        bytesLeft -= lfIdx + 1;
                        bufLen -= lfIdx + 1;
                    }
                    else
                    {
                        // Were we searching from the beginning?
                        if (currentSegment == buffer)
                        {
                            // Is buffer full?
                            if (bufLen == bufSize)
                            {
                                bufLen = 0;
                            }
                        }
                        else
                        {
                            char *tmp = buffer;
                            int len = 0;
                            while (true)
                            {
                                *tmp = *currentSegment;

                                if (*tmp == 0)
                                {
                                    break;
                                }

                                tmp++;
                                currentSegment++;
                                len++;
                            }

                            bufLen = bytesLeft;
                        }

                        // Cause loop to exit
                        bytesLeft = 0;
                    }
                }
            }

            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
    }

    /* static */ void C8FocuserMotor::readTaskRedirect(void *obj)
    {
        ((C8FocuserMotor *)(obj))->readTask();
    }

    //
    // MoveInfo
    //

    C8FocuserMotor::MoveInfo::MoveInfo(uint32_t stepCount,
                                       uint32_t backlashSteps,
                                       FocusDirection direction,
                                       int usPerStep,
                                       MoveDoneCallback callback,
                                       void *data)
        : stepCount(stepCount),
          backlashSteps(backlashSteps),
          direction(direction),
          usPerStep(usPerStep),
          callback(callback),
          data(data),
          stepsTaken(0),
          isAbort(false)
    {
    }

    //
    // MotorTest
    //

    C8FocuserMotor::MotorTest::MotorTest(C8FocuserMotor *parent,
                                         int numCycles)
        : _parent(parent),
          _numCycles(numCycles),
          _stepCount(0),
          _usPerStep(0),
          _direction(FD_FOCUS_INWARD)
    {
        _stepCount = _parent->g_stepsPerRev[(int)_parent->_microsteps - 1];
        _usPerStep = _parent->g_slowSpeedRevTimeSecs * 1000000 / _stepCount;
    }

    bool C8FocuserMotor::MotorTest::isRunning() const
    {
        return _isRunning;
    }

    void C8FocuserMotor::MotorTest::run()
    {
        _isRunning = true;
        _parent->startMoving(_stepCount,
                             _usPerStep,
                             &moveDoneRedirect,
                             this);
    }

    void C8FocuserMotor::MotorTest::moveDone(bool isAbort)
    {
        if (isAbort)
        {
            _isRunning = false;
            return;
        }

        if (_direction == FD_FOCUS_INWARD)
        {
            _direction = FD_FOCUS_OUTWARD;
            _parent->setDirection(_direction);

            _parent->startMoving(_stepCount,
                                 _usPerStep,
                                 &moveDoneRedirect,
                                 this);
        }
        else
        {
            _direction = FD_FOCUS_INWARD;
            _parent->setDirection(_direction);

            _numCycles--;
            if (_numCycles != 0)
            {
                _parent->startMoving(_stepCount,
                                     _usPerStep,
                                     &moveDoneRedirect,
                                     this);
            }
            else
            {
                _isRunning = false;
            }
        }
    }

    /* static */ void C8FocuserMotor::MotorTest::moveDoneRedirect(void *obj,
                                                                  bool isAbort)
    {
        ((MotorTest *)(obj))->moveDone(isAbort);
    }

    //
    // CommsListener
    //

    C8FocuserMotor::CommsListener::CommsListener(C8FocuserMotor *parent)
        : _parent(parent)
    {
    }

    void C8FocuserMotor::CommsListener::focusRel(FocusDirection dir,
                                                 uint32_t steps)
    {
        startRelativeFocus(dir, steps);
    }

    void C8FocuserMotor::CommsListener::focusAbs(uint32_t position)
    {
        uint32_t steps = 0;
        FocusDirection dir = FD_FOCUS_OUTWARD;
        if (position < _parent->_position)
        {
            steps = _parent->_position - position;
            dir = FD_FOCUS_INWARD;
        }
        else
        {
            steps = position - _parent->_position;
        }

        startRelativeFocus(dir, steps);
    }

    void C8FocuserMotor::CommsListener::focusAbort()
    {
        if (_parent->_moveInfo != 0)
        {
            _parent->_moveInfo->isAbort = true;
        }
    }

    void C8FocuserMotor::CommsListener::enableMotor(bool isEnabled)
    {
        _parent->enableMotor(isEnabled);
    }

    void C8FocuserMotor::CommsListener::zero()
    {
        _parent->_position = 0;

        if (_parent->_comms != 0)
        {
            _parent->_comms->zeroed();
            _parent->_comms->position(_parent->_position);
        }
    }

    void C8FocuserMotor::CommsListener::getPos()
    {
        if (_parent->_comms != 0)
        {
            _parent->_comms->position(_parent->_position);
        }
    }

    void C8FocuserMotor::CommsListener::setMicrostep(Microsteps ms)
    {
        _parent->setMicrosteps(ms);
    }

    void C8FocuserMotor::CommsListener::getMicrostep()
    {
        if (_parent->_comms != 0)
        {
            _parent->_comms->microsteps(_parent->_microsteps);
        }
    }

    void C8FocuserMotor::CommsListener::getMotorEnabled()
    {
        if (_parent->_comms != 0)
        {
            _parent->_comms->motorEnabled(_parent->_isEnabled);
        }
    }

    void C8FocuserMotor::CommsListener::getMotorMoving()
    {
        if (_parent->_comms != 0)
        {
            if (_parent->_moveInfo != 0)
            {
                _parent->_comms->movingRel(_parent->_direction,
                                           _parent->_moveInfo->stepCount - _parent->_moveInfo->stepsTaken);
            }
            else
            {
                _parent->_comms->stopped(_parent->_position);
            }
        }
    }

    void C8FocuserMotor::CommsListener::getMaxPos()
    {
        if (_parent->_comms != 0)
        {
            _parent->_comms->maxPos(_parent->_maxPosition);
        }
    }

    void C8FocuserMotor::CommsListener::setSpeed(FocusSpeed speed)
    {
        _parent->setSpeed(speed);
    }

    void C8FocuserMotor::CommsListener::getSpeed()
    {
        if (_parent->_comms != 0)
        {
            _parent->_comms->speed(_parent->_speed);
        }
    }

    void C8FocuserMotor::CommsListener::enableBacklash(bool isEnabled)
    {
        _parent->enableBacklash(isEnabled);
    }

    void C8FocuserMotor::CommsListener::getBacklashEnabled()
    {
        if (_parent->_comms != 0)
        {
            _parent->_comms->backlashEnabled(_parent->isBacklashEnabled());
        }
    }

    void C8FocuserMotor::CommsListener::setBacklashSteps(uint32_t steps)
    {
        _parent->setBacklashSteps(steps);
    }

    void C8FocuserMotor::CommsListener::getBacklashSteps()
    {
        if (_parent->_comms != 0)
        {
            _parent->_comms->backlashSteps(_parent->getBacklashSteps());
        }
    }

    void C8FocuserMotor::CommsListener::startRelativeFocus(FocusDirection dir,
                                                           uint32_t steps)
    {
        _parent->setDirection(dir);

        int usPerStep = _parent->_usPerStepSlow;
        switch (_parent->_speed)
        {
        case FS_NORMAL:
            usPerStep = _parent->_usPerStepSlow;
            break;
        case FS_X3:
            usPerStep = _parent->_usPerStepFast;
            break;
        }

        _parent->startMoving(steps,
                             usPerStep,
                             &moveCompletedRedirect,
                             this);
    }

    void C8FocuserMotor::CommsListener::moveCompleted(bool isAbort)
    {
    }

    /* static */ void C8FocuserMotor::CommsListener::moveCompletedRedirect(void *obj,
                                                                           bool isAbort)
    {
        ((CommsListener *)(obj))->moveCompleted(isAbort);
    }

    //
    // FCWriter
    //

    C8FocuserMotor::FCWriter::FCWriter()
    {
        Serial.begin(115200);
    }

    bool C8FocuserMotor::FCWriter::writeLine(const char *line)
    {
        Serial.println(line);

        return true;
    }

    void C8FocuserMotor::FCWriter::close()
    {
        Serial.end();
    }
}