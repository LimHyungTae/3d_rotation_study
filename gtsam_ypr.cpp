#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <Eigen/Dense>
#include <iostream>

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

    // R.print();
    double roll, pitch, yaw;

    auto vec = R.ypr();
    std::cout << "[Before]  Roll: " << vec(2) << ", Pitch: " << vec(1) << ", Yaw: " << vec(0) << std::endl;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 0.1);
    double random_value = dis(gen);
    gtsam::Rot3 R_y = gtsam::Rot3::Ypr(random_value, 0, 0);
    gtsam::Rot3 Q = gtsam::Rot3::Ypr(0, vec(1), vec(2));
    
    gtsam::Rot3 R_prime = gtsam::Rot3::Ypr(vec(0) + random_value, vec(1), vec(2));

    auto R_updated = R * Q.inverse() * R_y * Q;

    vec = R_updated.ypr();
    std::cout << "random_value: " << random_value * 180 / 3.141592 << "\n";
    std::cout << "[GTSAM]  Roll: " << vec(2) << ", Pitch: " << vec(1) << ", Yaw: " << vec(0) << std::endl;
    vec = R_prime.ypr();
    std::cout << "[GTSAM]  Roll: " << vec(2) << ", Pitch: " << vec(1) << ", Yaw: " << vec(0) << std::endl;

  }
  return 0;
}
