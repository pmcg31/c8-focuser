#pragma once

#include "FocuserComms.hpp"
#include "FocuserCommsListener.hpp"
#include "CommsWriter.hpp"

namespace ELS
{

    class C8FocuserMotor
    {
    public:
        C8FocuserMotor(int tmc2209EnablePin,
                       int tmc2209StepPin,
                       int tmc2209DirPin,
                       int tmc2209Ms1Pin,
                       int tmc2209Ms2Pin);

        ~C8FocuserMotor();

        bool isEnabled() const;
        bool isReversed() const;
        Microsteps getMicrosteps() const;
        FocusDirection getDirection() const;
        FocusSpeed getSpeed() const;

        void enableMotor(bool isEnabled);
        void reverseMotor(bool isReversed);
        void setMicrosteps(Microsteps microsteps);
        void setDirection(FocusDirection direction);
        void setSpeed(FocusSpeed speed);

        void testMotor(int numCycles);

        bool startComms();

    private:
        struct MoveInfo
        {
            typedef void (*MoveDoneCallback)(void *arg, bool isAbort);

            MoveInfo(uint32_t stepCount,
                     MoveDoneCallback callback,
                     void *data);

            uint32_t stepCount;
            MoveDoneCallback callback;
            void *data;
            uint32_t stepsTaken;
            bool isAbort;
        };

    private:
        class MotorTest
        {
        public:
            MotorTest(C8FocuserMotor *parent,
                      int numCycles);

            bool isRunning() const;

            void run();

        private:
            void moveDone(bool isAbort);

        private:
            static void moveDoneRedirect(void *obj, bool isAbort);

        private:
            C8FocuserMotor *_parent;
            int _numCycles;
            uint32_t _stepCount;
            int _usPerStep;
            FocusDirection _direction;
            bool _isRunning;
        };
        friend class MotorTest;

    private:
        bool updateFieldsFromConfig();
        bool updateConfigFromFields();

        void syncEnabled();
        void syncDir();
        void syncMicrosteps();
        void step();
        void startMoving(uint32_t stepCount,
                         int usPerStep,
                         MoveInfo::MoveDoneCallback callback,
                         void *data);
        void abortMove();
        void moveNext();

        void readTask();

    private:
        static void IRAM_ATTR moveNextRedirect(void *obj);

        static void readTaskRedirect(void *obj);

    private:
        // FocuserCommsListener
        class CommsListener : public FocuserCommsListener
        {
        public:
            CommsListener(C8FocuserMotor *parent);

        public:
            virtual void focusRel(FocusDirection dir, uint32_t steps) override;
            virtual void focusAbs(uint32_t position) override;
            virtual void focusAbort() override;
            virtual void enableMotor(bool isEnabled) override;
            virtual void zero() override;
            virtual void getPos() override;
            virtual void setMicrostep(Microsteps ms) override;
            virtual void getMicrostep() override;
            virtual void getMotorEnabled() override;
            virtual void getMotorMoving() override;
            virtual void getMaxPos() override;
            virtual void setSpeed(FocusSpeed speed) override;
            virtual void getSpeed() override;

        private:
            void startRelativeFocus(FocusDirection dir,
                                    uint32_t steps);
            void moveCompleted(bool isAbort);

        private:
            static void moveCompletedRedirect(void *obj,
                                              bool isAbort);

        private:
            C8FocuserMotor *_parent;
        };
        friend class CommsListener;

    private:
        class FCWriter : public CommsWriter
        {
        public:
            FCWriter();

            virtual bool writeLine(const char *line);
            virtual void close();
        };

    private:
        int _enablePin;
        int _stepPin;
        int _dirPin;
        int _ms1Pin;
        int _ms2Pin;

        bool _isEnabled;
        bool _isReversed;
        Microsteps _microsteps;
        FocusDirection _direction;
        FocusSpeed _speed;
        uint32_t _position;
        uint32_t _maxPosition;

        uint32_t _stepsPerRev;
        int _usPerStepSlow;
        int _usPerStepFast;

        MoveInfo *_moveInfo;
        esp_timer_handle_t _moveTimer;

        MotorTest *_motorTest;

        FocuserComms *_comms;
        FCWriter *_writer;
        CommsListener *_commsListener;
        TaskHandle_t _readTaskHandle;

    private:
        static int g_fastSpeedRevTimeSecs;
        static int g_slowSpeedRevTimeSecs;

        static int g_c8FocuserMaxTurns;

        static uint32_t g_stepsPerRev[];

        static bool g_microstepValues[][2];
    };

}