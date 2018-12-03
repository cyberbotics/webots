#include "heuristicsManager.h"
#include "ode_MT/util_MT.h"

#include "heuristic.h"
//#include "heuristicsManager.cpp"

template<typename T> dxHeuristic<T>::dxHeuristic(T _value)
{
    mState = STATE_INIT;
    newValue = _value;
    oldValue = _value;

    resetValues();
}

template<typename T> dxHeuristic<T>::~dxHeuristic()
{
    //threadCount = numThreads;

}

template<typename T> void dxHeuristic<T>::simLoop()
{

}

template<typename T> void dxHeuristic<T>::update(int _frameCount, T _value)
{
    if (mState > STATE_RUNNING)
        return;

    oldValue = newValue;
    newValue = _value;

    average = (average * recordedSamples + oldValue) / (recordedSamples + 1);
    ++recordedSamples;

    int countDelta = _frameCount - frameCount;
    if (countDelta > 0)
      speed = (newValue - oldValue) / countDelta;
    else
      speed = 0.0;

    typename std::list<heuristicAlert>::iterator itr;
    for (itr = alertsList.begin(); itr != alertsList.end(); ++itr)
    {
        //ODE_INFO("Checking Alert Type: %d\n", itr->alertType);
        checkAlert(*itr, _frameCount);
    }

    //value = _value;
    frameCount = _frameCount;
    frameCounter++;

}

template<typename T> void dxHeuristic<T>::resetValues() {
  average = 0.0;
  speed = 0.0;
  frameCount = 0;
  frameCounter = 0;
}

template<typename T> void dxHeuristic<T>::addAlert(AlertType _alertType, HeuristicAlertCallback* _alertFunc, T _value)
{
    heuristicAlert newAlert;
    newAlert.alertType = _alertType;
    newAlert.threshhold = _value;
    newAlert.alertFunc = _alertFunc;

    alertsList.push_back(newAlert);
}

template<typename T> void dxHeuristic<T>::pause()
{
    mState = STATE_PAUSED;
}

template<typename T> void dxHeuristic<T>::reset()
{
    mState = STATE_RESET;
    resetValues();
}

template<typename T> void dxHeuristic<T>::checkAlert(heuristicAlert &_alert, int _frameCount)
{
    switch (_alert.alertType)
    {
        case ALERTTYPE_FIXEDINTERVAL:
            if (frameCounter >= _alert.threshhold)
            {
                _alert.alertFunc->alertCallback(this);
                frameCounter = 0;
            }

            break;
        case ALERTTYPE_VALUECHANGE:
            if (newValue != oldValue)
            {
                ODE_INFO("Heuristic Alert! Value changed from %d to %d\n", oldValue, newValue);
                _alert.alertFunc->alertCallback(this);
            }
            break;
        default:
            break;
    }
}
