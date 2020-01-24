//
// Created by Ankoor Bhagat on 1/22/20.
//

#ifndef KF_LIDAR_KALMANFILTER_H
#define KF_LIDAR_KALMANFILTER_H


#include <eigen3/Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
public:

    /**
     * Constructor
     */
    KalmanFilter();

    /**
     * Destructor
     */
    virtual ~KalmanFilter();

    /**
     * Predict Predicts the state and the state covariance
     *   using the process model
     */
    void Predict();

    /**
     * Updates the state and
     * @param z The measurement at k+1
     */
    void Update(const VectorXd &z);

    // state vector
    VectorXd x_;

    // state covariance matrix
    MatrixXd P_;

    // state transistion matrix
    MatrixXd F_;

    // process covariance matrix
    MatrixXd Q_;

    // measurement matrix
    MatrixXd H_;

    // measurement covariance matrix
    MatrixXd R_;

};




#endif //KF_LIDAR_KALMANFILTER_H
