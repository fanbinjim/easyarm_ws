#include "easyarm_dynamics/robot_model.hpp"

#include <fstream>
#include <stdexcept>
#include <string>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace easyarm_dynamics
{
namespace
{

pinocchio::Model buildModelFromUrdf(const std::string & urdf_path)
{
  if (urdf_path.empty()) {
    throw std::invalid_argument("URDF path must not be empty.");
  }

  std::ifstream urdf_file(urdf_path);
  if (!urdf_file.good()) {
    throw std::invalid_argument("URDF file cannot be opened: " + urdf_path);
  }

  pinocchio::Model model;
  try {
    pinocchio::urdf::buildModel(urdf_path, model);
  } catch (const std::exception & exception) {
    throw std::runtime_error(
      "Failed to build Pinocchio model from URDF '" + urdf_path + "': " + exception.what());
  }

  return model;
}

pinocchio::Model buildModelFromUrdfXml(const std::string & urdf_xml)
{
  if (urdf_xml.empty()) {
    throw std::invalid_argument("URDF XML must not be empty.");
  }

  pinocchio::Model model;
  try {
    pinocchio::urdf::buildModelFromXML(urdf_xml, model);
  } catch (const std::exception & exception) {
    throw std::runtime_error(
      std::string("Failed to build Pinocchio model from URDF XML: ") + exception.what());
  }

  return model;
}

std::string sizeErrorMessage(
  const char * name,
  Eigen::Index expected,
  Eigen::Index actual)
{
  return std::string(name) + " size mismatch. Expected " + std::to_string(expected) +
         ", got " + std::to_string(actual) + ".";
}

}  // namespace

RobotModel::RobotModel(const std::string & urdf_path)
: model_(buildModelFromUrdf(urdf_path)),
  data_(model_)
{
}

RobotModel RobotModel::fromUrdfXml(const std::string & urdf_xml)
{
  return RobotModel(buildModelFromUrdfXml(urdf_xml));
}

RobotModel::RobotModel(pinocchio::Model model)
: model_(std::move(model)),
  data_(model_)
{
}

Eigen::VectorXd RobotModel::gravity(const Eigen::VectorXd & q)
{
  validateConfiguration(q);
  return pinocchio::computeGeneralizedGravity(model_, data_, q);
}

Eigen::VectorXd RobotModel::nle(const Eigen::VectorXd & q, const Eigen::VectorXd & qd)
{
  validateConfiguration(q);
  validateTangentVector(qd, "qd");

  return pinocchio::nonLinearEffects(model_, data_, q, qd);
}

Eigen::MatrixXd RobotModel::massMatrix(const Eigen::VectorXd & q)
{
  validateConfiguration(q);

  std::cerr << "[RobotModel] massMatrix() is not used in current controller.\n";

  return Eigen::MatrixXd::Identity(nv(), nv());
}

Eigen::VectorXd RobotModel::inverseDynamics(
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & qd,
  const Eigen::VectorXd & qdd)
{
  validateConfiguration(q);
  validateTangentVector(qd, "qd");
  validateTangentVector(qdd, "qdd");

  return pinocchio::rnea(model_, data_, q, qd, qdd);
}

Eigen::Index RobotModel::nq() const noexcept
{
  return model_.nq;
}

Eigen::Index RobotModel::nv() const noexcept
{
  return model_.nv;
}

void RobotModel::validateConfiguration(const Eigen::VectorXd & q) const
{
  if (q.size() != nq()) {
    throw std::invalid_argument(sizeErrorMessage("q", nq(), q.size()));
  }
}

void RobotModel::validateTangentVector(const Eigen::VectorXd & vector, const char * name) const
{
  if (vector.size() != nv()) {
    throw std::invalid_argument(sizeErrorMessage(name, nv(), vector.size()));
  }
}

}  // namespace easyarm_dynamics
