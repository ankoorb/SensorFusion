//
// Created by Ankoor Bhagat on 1/22/20.
//

#ifndef KF_LIDAR_MEASUREMENTPACKAGE_H
#define KF_LIDAR_MEASUREMENTPACKAGE_H

#include <eigen3/Eigen/Dense>

class MeasurementPackage {
public:

    enum SensorType {
        LASER, RADAR
    } sensor_type_;

    Eigen::VectorXd raw_measurements_;

    int64_t timestamp_;

};



#endif //KF_LIDAR_MEASUREMENTPACKAGE_H
