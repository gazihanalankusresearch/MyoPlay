using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using GyroscopeDataEventArgs = Thalmic.Myo.GyroscopeDataEventArgs;
using AccelerometerDataEventArgs = Thalmic.Myo.AccelerometerDataEventArgs;
using TimeSpan = System.TimeSpan;
using DateTime = System.DateTime;
using Math = System.Math;

public class GyroscopeIntegrator : MonoBehaviour {

    public ThalmicMyo thalmicMyo;

    public Transform gyroscopeIntegrated;
    public Transform accelerometerLowpass;
    public Transform complementaryFilter;

    static float lowpassNewContribution = .99f;
    static float lowpassOldContribution = 1 - lowpassNewContribution;

    private Object _lock = new Object();

    List<GyroscopeDataEventArgs> gyroscopeEvents = new List<GyroscopeDataEventArgs>();
    List<AccelerometerDataEventArgs> accelerometerEvents = new List<AccelerometerDataEventArgs>();

    public void Start() {
        //thalmicMyo.internalMyo.GyroscopeData += gyroscopeReceived;
        thalmicMyo.internalMyo.AccelerometerData += accelerometerReceived;
    }

    bool firstGyroMeasurement = true;


    //TODO these should be initialized from previous frames. 
    GyroscopeDataEventArgs effectiveGyro;
    AccelerometerDataEventArgs effectiveAccel;

    GyroscopeDataEventArgs prevGyro;
    AccelerometerDataEventArgs prevAccel;

    public void Update() {
        lock (_lock) {
            // we sure can't do it online since we need to know the next gyroscope reading
            // doing it precisely can be a bit inefficient (have to slice the gyroscope values with accel times)
            // so, we'll have two modes:
            // 1. precise mode
            // 2. imprecise mode 

            // precise mode is confusing. may not even exist...
            bool preciseModeDesired = false;


            // make sure the values are ordered in time, so that we can merge them. 



            enforceOrder(gyroscopeEvents);
            enforceOrder(accelerometerEvents);


            var gyroEnum = gyroscopeEvents.GetEnumerator();
            var accelEnum = accelerometerEvents.GetEnumerator();

            bool gyroHasNext = gyroEnum.MoveNext();
            bool accelHasNext = accelEnum.MoveNext();

            while (gyroHasNext || accelHasNext) {
                bool useGyro = false;
                bool useAccel = false;

                if (gyroHasNext && accelHasNext) {
                    int gyroIsLater = DateTime.Compare(gyroEnum.Current.Timestamp, accelEnum.Current.Timestamp);
                    if (gyroIsLater == 0) {
                        // both are at the same time
                        useGyro = true;
                        useAccel = true;
                    } else if (gyroIsLater > 0) {
                        // accel is before
                        useAccel = true;
                    } else {
                        // gyro is before
                        useGyro = true;
                    }
                } else {
                    useGyro = gyroHasNext;
                    useAccel = accelHasNext;
                }

                bool moveWithGyro = false;
                bool moveWithAccel = false;

                // by now, I should 
                // have the effective value for both
                // have the necessary one(s) advance

                if (useGyro) {
                    prevGyro = effectiveGyro;
                    effectiveGyro = gyroEnum.Current;
                    gyroHasNext = gyroEnum.MoveNext();

                    if (prevGyro != null) {
                        moveWithGyro = true;
                    }
                }

                if (useAccel) {
                    prevAccel = effectiveAccel; 
                    effectiveAccel = accelEnum.Current;
                    accelHasNext = accelEnum.MoveNext();

                    moveWithAccel = true;
                }

                if (moveWithGyro && moveWithAccel) {
                    // TODO average the two and apply
                    Debug.LogError("Events are not ordered. TODO order them!");
                    throw new System.NotImplementedException();
                } else {
                    if (moveWithAccel) {
                        // TODO calculate and apply (time will go in here, will scale the contrib.)

                        Debug.Log("move with accel");

                        // currentup should always be Vector3.up
                        // localaccelup should be the vector in local coords. this is what we read. 
                        // accelup should be that vector in global space
                        // this does not require matrix inversion. good. 

                        TimeSpan dts = effectiveAccel.Timestamp.Subtract(prevAccel.Timestamp);
                        double dt = dts.TotalSeconds;

                        Vector3 currentUp = Vector3.up;
                        Vector3 localAccelUp = myoToUnity(effectiveAccel.Accelerometer).normalized;
                        Vector3 accelup = transform.InverseTransformDirection(localAccelUp);

                        float angle = Vector3.Angle(currentUp, accelup);
                        Vector3 axis = new Vector3(-accelup.z, 0, accelup.x); // TODO this axis may not be correct

                        //TODO this quaternion may be flipped. check it.
                        Quaternion correction = Quaternion.AngleAxis((float)(angle * lowpassNewContribution * dt), axis);

                        accelerometerLowpass.localRotation = correction * accelerometerLowpass.localRotation;
                        // TODO test this accelerometer thing first. then, merge it with gyro.
                            
                    } else if (moveWithGyro) {
                        // TODO calculate and apply
                        Debug.LogError("Events are not ordered. TODO order them!");
                        throw new System.NotImplementedException();
                    }
                }

            } //this leaves effective gyro and accel set. what I should do is to initialize them in the very first use. 

            gyroscopeEvents.Clear();
            accelerometerEvents.Clear();

            // eat up the data. 
            // I have a couple of gyro and accel readings 
            // now order them 
            //   for each time duration, 
            //     integrate gyro (avg with last value for area)
            //     use 


            Debug.Log(gyroscopeDataCount + " " + accelerometerDataCount);
            gyroscopeDataCount = accelerometerDataCount = 0;
            //if (gyroscopeIntegrated != null) {
            //    if (cumulativeHasQ) {
            //        gyroscopeIntegrated.localRotation = gyroscopeIntegrated.localRotation * cumulativeDq;
            //        cumulativeDq = Quaternion.identity;
            //        cumulativeHasQ = false;
            //    }
            //}
        }
    }

    private void enforceOrder<T>(List<T> someList) where T : Thalmic.Myo.MyoEventArgs {
        bool ordered = true;

        var i = someList.GetEnumerator();
        if (i.MoveNext()) {
            DateTime lastTime = i.Current.Timestamp;

            while (i.MoveNext()) {
                DateTime currentTime = i.Current.Timestamp;

                if (DateTime.Compare(currentTime, lastTime) < 0) {
                    ordered = false;
                    break;
                }

                lastTime = currentTime;
            }
        }

        if (!ordered) {
            Debug.LogError("Events are not ordered. TODO order them!");
            throw new System.NotImplementedException();
        }
    }

    DateTime lastTimeStamp;
    Thalmic.Myo.Vector3 lastGyroscope;

    bool cumulativeHasQ;
    Quaternion cumulativeDq = Quaternion.identity;

    int gyroscopeDataCount;

    // If you want to do this right, perhaps don't immediately merge. instead, record gyro and accel and merge them together in update.

    private Quaternion calculateasAppliedTogether(Quaternion q1, Quaternion q2) {
        bool ordered = false;
        bool firstOneFirst = true;

        if (ordered) {
            if (firstOneFirst) {
                return q1 * q2;
            } else {
                return q2 * q1;
            }
        } else {
            // apply slerp twice
            Quaternion slerp = Quaternion.Slerp(q1, q2, .5f);
            return slerp * slerp;
        }
    }

    static private Vector3 myoToUnity(Vector3 myo) {
        return new Vector3(-myo.y, myo.z, myo.x);
    }

    static private Vector3 myoToUnity(Thalmic.Myo.Vector3 myo) {
        return new Vector3(-myo.Y, myo.Z, myo.X);
    }

    private void gyroscopeReceived(object sender, GyroscopeDataEventArgs e) {
        lock (_lock) {
            gyroscopeEvents.Add(e);

            //if (firstGyroMeasurement) {
            //    firstGyroMeasurement = false;
            //} else {
            //    TimeSpan dt = e.Timestamp.Subtract(lastTimeStamp);
            //    Thalmic.Myo.Vector3 avg = (e.Gyroscope + lastGyroscope) * .5f;

            //    double dts = dt.TotalSeconds;

            //    double rx = avg.X * dts;
            //    double ry = avg.Y * dts;
            //    double rz = avg.Z * dts;

            //    double magnitude = Math.Sqrt(rx * rx + ry * ry + rz * rz);
            //    Vector3 myoAxis = new Vector3((float)(rx / magnitude), (float)(ry / magnitude), (float)(rz / magnitude));
            //    Vector3 axis = new Vector3(-myoAxis.y, myoAxis.z, myoAxis.x);
            //    float angle = (float)magnitude;

            //    Quaternion dq = Quaternion.AngleAxis(angle, axis);

            //    cumulativeDq = cumulativeDq * dq;
            //    cumulativeHasQ = true;
            //}

            //lastTimeStamp = e.Timestamp;
            //lastGyroscope = e.Gyroscope;
            ++gyroscopeDataCount;
        }
    }

    int accelerometerDataCount;

    //gyroscop and accelerometer arrive together. nice. 
    private void accelerometerReceived(object sender, AccelerometerDataEventArgs e) {
        lock (_lock) {
            accelerometerEvents.Add(e);

            ++accelerometerDataCount;
        }
    }

}
