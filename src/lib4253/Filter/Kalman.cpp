#include "lib4253/Filter/Kalman.hpp"
namespace lib4253{
KalmanFilter::KalmanFilter(
    double dt,
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P)
  : A(A), C(C), Q(Q), R(R), P0(P),
    m(C.rows()), n(A.rows()), dt(dt), initialized(false),
    I(n, n), x_hat(n), x_hat_new(n)
{
  I.setIdentity();
}

void KalmanFilter::initialize(double t0, const Eigen::VectorXd& x0) {
  x_hat = x0;
  P = P0;
  this->t0 = t0;
  t = t0;
  initialized = true;
}

void KalmanFilter::initialize() {
  x_hat.setZero();
  x0.setZero();
  P = P0;
  t0 = 0;
  t = t0;
  initialized = true;
}

void KalmanFilter::reset(){
  if(!initialized){
    t0 = 0;
    x0.setZero();
  }
  initialize(t0, x0);
}

void KalmanFilter::reset(double t0, const Eigen::VectorXd& x0){
  initialize(t0, x0);
}

Eigen::VectorXd KalmanFilter::filter(const Eigen::VectorXd& y) {
  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");

  x_hat_new = A * x_hat;
  P = A*P*A.transpose() + Q;
  K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
  x_hat_new += K * (y - C*x_hat_new);
  P = (I - K*C)*P;
  t += dt;
  return x_hat = x_hat_new;
}

Eigen::VectorXd KalmanFilter::filter(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A) {
  this->A = A;
  this->dt = dt;
  return filter(y);
}

Eigen::VectorXd KalmanFilter::getOutput() const{
  return x_hat;
}

double KalmanFilter::getTime() const{
  return t;
}

}