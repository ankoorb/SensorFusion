//
// Created by Ankoor Bhagat on 1/22/20.
//

#ifndef KF_LIDAR_TRACKING_H
#define KF_LIDAR_TRACKING_H


#include <vector>
#include <string>
#include <fstream>
#include "KalmanFilter.h"
#include "MeasurementPackage.h"

class Tracking {
public:
    Tracking();
    virtual ~Tracking();
    void ProcessMeasurement(const MeasurementPackage &measurement_pack);
    KalmanFilter kf_;

private:
    bool is_initialized_;
    int64_t previous_timestamp_;

    //acceleration noise components
    float noise_ax;
    float noise_ay;

};



#endif //KF_LIDAR_TRACKING_H
