/**
 * @file Kalman.hpp
 * @author Ryan Liao (23RyanL@students.tas.tw)
 * @brief Kalman filter
 * @version 0.1
 * @date 2021-05-21
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once
#include "lib4253/Filter/AbstractFilter.hpp"
#include "Eigen/Core"
#include "Eigen/LU"
#include "Eigen/MatrixFunctions"

namespace lib4253{

class KalmanFilter : public AbstractFilter<Eigen::VectorXd>{
    public:
    /**
     * Create a blank estimator.
     */
    KalmanFilter() = default;

    /**
     * Create a Kalman filter with the specified matrices.
     *   A - System dynamics matrix
     *   C - Output matrix
     *   Q - Process noise covariance
     *   R - Measurement noise covariance
     *   P - Estimate error covariance
     */
    KalmanFilter(
        double dt,
        const Eigen::MatrixXd& A,
        const Eigen::MatrixXd& C,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R,
        const Eigen::MatrixXd& P
    );

    ~KalmanFilter() = default;

    /**
     * Initialize the filter with initial states as zero.
     */
    void initialize() override;

    /**
     * Initialize the filter with a guess for initial states.
     */
    void initialize(double t0, const Eigen::VectorXd& x0);

    void reset() override;

    void reset(double t0, const Eigen::VectorXd& x0);

    /**
     * Update the estimated state based on measured values. The
     * time step is assumed to remain constant.
     */
    Eigen::VectorXd filter(const Eigen::VectorXd& y) override;

    /**
     * Update the estimated state based on measured values,
     * using the given time step and dynamics matrix.
     */
    Eigen::VectorXd filter(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A);

    /**
     * Return the current state and time.
     */
    Eigen::VectorXd getOutput() const override;

    double getTime() const;

    private:

    // Matrices for computation
    Eigen::MatrixXd A, C, Q, R, P, K, P0;

    // System dimensions
    int m, n;

    // Initial and current time
    double t0, t;

    // Discrete time step
    double dt;

    // Is the filter initialized?
    bool initialized;

    // n-size identity
    Eigen::MatrixXd I;

    // Estimated states
    Eigen::VectorXd x_hat, x_hat_new;

    // Initial Guess
    Eigen::VectorXd x0;
};
}