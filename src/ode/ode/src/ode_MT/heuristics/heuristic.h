#ifndef _ODE_MT_HEURISTIC_H_
#define _ODE_MT_HEURISTIC_H_

#include <list>
#include "ode_MT/util_MT.h"

class HeuristicAlertCallback;

template <typename T>
class dxHeuristic
{
public:
    enum AlertType
    {
        ALERTTYPE_FIXEDINTERVAL,
        ALERTTYPE_VALUECHANGE,
        ALERTTPYE_VALUEBELOWTHRESHHOLD,
        ALERTTYPE_VALUEABOVETHRESHHOLD,
        ALERTTPYE_SPEEDBELOWTHRESHHOLD,
        ALERTTYPE_SPEEDABOVETHRESHHOLD,
    };

    enum HeuristicState
    {
        STATE_INIT,
        STATE_RESET,
        STATE_RUNNING,
        STATE_PAUSED,
        STATE_STOPPED
    };

private:
    struct heuristicAlert
    {
        AlertType alertType;
        T threshhold;
        HeuristicAlertCallback* alertFunc;
    };

private:
    HeuristicState mState;

    T oldValue;
    T newValue;
    int frameCount;
    int frameCounter;

    int recordedSamples;
    T average;
    T speed;

    std::list<heuristicAlert> alertsList;

    void checkAlert(heuristicAlert& _alert, int _frameCount);
    void resetValues();

public:
    dxHeuristic(T _value);
    ~dxHeuristic();

    void simLoop();
    void update(int _frameCount, T _value);
    void pause();
    void reset();

    T getNewValue() { return newValue; }
    T getOldValue() { return oldValue; }
    void addAlert(AlertType _alertType, class HeuristicAlertCallback *_alertFunc, T _value);

};

#endif
