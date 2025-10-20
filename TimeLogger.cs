using UnityEngine;

namespace DefaultNamespace
{
    public class TimeLogger
    {
        public float _simTime;
        public float _colTime;
        public float _colEntry;
        public bool WasWaitingForThreadsToEnd;

        private float _simTimeStart;
        private float _simTimeEnd;

        private float _colTimeStart;
        private float _colTimeEnd;
        
        private float _colEntryTimeStart;
        private float _colEntryTimeEnd;

        public void StartSimClockwatch()
        {
            _simTimeStart = Time.realtimeSinceStartup;
        }

        public void StopSimClockwatch(bool waitingForThreadEnd)
        {
            WasWaitingForThreadsToEnd = waitingForThreadEnd;
            _simTimeEnd = Time.realtimeSinceStartup;
            _simTime = _simTimeEnd - _simTimeStart;
            _simTime *= 1000;
        }

        public void StartCollisionReadOutClockwatch()
        {
            _colTimeStart = Time.realtimeSinceStartup;
        }

        public void StopCollisionReadOutClockwatch(bool waitingForThreadEnd)
        {
            _colTimeEnd = Time.realtimeSinceStartup;
            _colTime = _colTimeEnd - _colTimeStart;
            _colTime *= 1000;

        }
        public void StartCollisionEntryClockwatch()
        {
            _colEntryTimeStart = Time.realtimeSinceStartup;
        }

        public void StopCollisionEntryClockwatch()
        {
            _colEntryTimeEnd = Time.realtimeSinceStartup;
            _colEntry = _colEntryTimeEnd - _colEntryTimeStart;
            _colEntry *= 1000;
        }
    }
}