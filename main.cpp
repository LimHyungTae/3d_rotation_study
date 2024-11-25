#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <Eigen/Dense>
#include <iostream>


void getRPYFromEigenMatrix3d(const Eigen::Matrix3d& mat, double& roll, double& pitch, double& yaw, const std::string order="zyx") {
  if (order == "zyx") {
    Eigen::Vector3d rpy = mat.eulerAngles(2, 1, 0); 
    yaw = rpy[0];
    pitch = rpy[1];
    roll = rpy[2];
  } else if (order == "xyz") {
    Eigen::Vector3d ypr = mat.eulerAngles(0, 1, 2); 
    roll = ypr[0];
    yaw = ypr[2];
    pitch = ypr[1];
  } 
}

Eigen::Vector3d R2ypr(const Eigen::Matrix3d& R) {
  Eigen::Vector3d n = R.col(0);
  Eigen::Vector3d o = R.col(1);
  Eigen::Vector3d a = R.col(2);

  Eigen::Vector3d ypr(3);
  double y = atan2(n(1), n(0));
  double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
  double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
  ypr(0) = y;
  ypr(1) = p;
  ypr(2) = r;
  return ypr;
}

Eigen::Matrix3d generateRandomRotationMatrix() {
  // Random number generator for rotation angle and axis
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.0, 1.0);

  double x = dis(gen) * 2.0 - 1.0;
  double y = dis(gen) * 2.0 - 1.0;
  double z = dis(gen) * 2.0 - 1.0;

  Eigen::Vector3d axis(x, y, z);
  axis.normalize();

  double angle = dis(gen) * 2.0 * M_PI;

  Eigen::AngleAxisd angleAxis(angle, axis);

  Eigen::Matrix3d rotationMatrix = angleAxis.toRotationMatrix();

  return rotationMatrix;
}

int main() {
  for (size_t i = 0; i < 10; ++i) {
    Eigen::Matrix3d rotation = generateRandomRotationMatrix();
    gtsam::Rot3 R = gtsam::Rot3(rotation);

    double roll, pitch, yaw;

    std::cout << "----------------------------------------" << std::endl;
    getRPYFromEigenMatrix3d(rotation, roll, pitch, yaw);
    std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
    getRPYFromEigenMatrix3d(rotation, roll, pitch, yaw, "xyz");
    std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;

    auto vec = R.ypr();
    std::cout << "Roll: " << vec(2) << ", Pitch: " << vec(1) << ", Yaw: " << vec(0) << std::endl;
    auto vec2 = R.rpy();
    std::cout << "Roll: " << vec2(0) << ", Pitch: " << vec2(1) << ", Yaw: " << vec2(2) << std::endl;
    auto vec3 = R2ypr(rotation.matrix());
    std::cout << "Roll: " << vec3(2) << ", Pitch: " << vec3(1) << ", Yaw: " << vec3(0) << std::endl;
    std::cout << "----------------------------------------" << std::endl;
  }
  return 0;
}
