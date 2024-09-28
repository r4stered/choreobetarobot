#pragma once

#include <random>

#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/plant/DCMotor.h>
#include <units/voltage.h>

static constexpr int numModules{4};

class SwerveDriveSim {
 public:
  SwerveDriveSim(const frc::SimpleMotorFeedforward<units::meters>& driveFF,
                 const frc::DCMotor& driveMotor, double driveGearing,
                 units::meter_t driveWheelRadius,
                 const frc::SimpleMotorFeedforward<units::radians>& steerFF,
                 const frc::DCMotor& steerMotor, double steerGearing,
                 const frc::SwerveDriveKinematics<numModules>& kinematics);
  SwerveDriveSim(const frc::LinearSystem<2, 1, 2>& drivePlant,
                 units::volt_t driveKs, const frc::DCMotor& driveMotor,
                 double driveGearing, units::meter_t driveWheelRadius,
                 const frc::LinearSystem<2, 1, 2>& steerPlant,
                 units::volt_t steerKs, const frc::DCMotor& steerMotor,
                 double steerGearing,
                 const frc::SwerveDriveKinematics<numModules>& kinematics);
  void SetDriveInputs(const std::array<units::volt_t, numModules>& inputs);
  void SetSteerInputs(const std::array<units::volt_t, numModules>& inputs);
  static Eigen::Matrix<double, 2, 1> CalculateX(
      const Eigen::Matrix<double, 2, 2>& discA,
      const Eigen::Matrix<double, 2, 1>& discB,
      const Eigen::Matrix<double, 2, 1>& x, units::volt_t input,
      units::volt_t kS);
  void Update(units::second_t dt);
  void Reset(const frc::Pose2d& pose, bool preserveMotion);
  void Reset(const frc::Pose2d& pose,
             const std::array<Eigen::Matrix<double, 2, 1>, numModules>&
                 moduleDriveStates,
             const std::array<Eigen::Matrix<double, 2, 1>, numModules>&
                 moduleSteerStates);
  frc::Pose2d GetPose() const;
  std::array<frc::SwerveModulePosition, numModules> GetModulePositions() const;
  std::array<frc::SwerveModulePosition, numModules> GetNoisyModulePositions(
      units::meter_t driveStdDev, units::radian_t steerStdDev);
  std::array<frc::SwerveModuleState, numModules> GetModuleStates();
  std::array<Eigen::Matrix<double, 2, 1>, numModules> GetDriveStates() const;
  std::array<Eigen::Matrix<double, 2, 1>, numModules> GetSteerStates() const;
  units::radians_per_second_t GetOmega() const;
  units::ampere_t GetCurrentDraw(const frc::DCMotor& motor,
                                 units::radians_per_second_t velocity,
                                 units::volt_t inputVolts,
                                 units::volt_t batteryVolts) const;
  std::array<units::ampere_t, numModules> GetDriveCurrentDraw() const;
  std::array<units::ampere_t, numModules> GetSteerCurrentDraw() const;
  units::ampere_t GetTotalCurrentDraw() const;

 private:
  std::random_device rd{};
  std::mt19937 generator{rd()};
  std::normal_distribution<double> randDist{0.0, 1.0};
  const frc::LinearSystem<2, 1, 2> drivePlant;
  const units::volt_t driveKs;
  const frc::DCMotor driveMotor;
  const double driveGearing;
  const units::meter_t driveWheelRadius;
  const frc::LinearSystem<2, 1, 2> steerPlant;
  const units::volt_t steerKs;
  const frc::DCMotor steerMotor;
  const double steerGearing;
  const frc::SwerveDriveKinematics<numModules> kinematics;
  std::array<units::volt_t, numModules> driveInputs{};
  std::array<Eigen::Matrix<double, 2, 1>, numModules> driveStates{};
  std::array<units::volt_t, numModules> steerInputs{};
  std::array<Eigen::Matrix<double, 2, 1>, numModules> steerStates{};
  frc::Pose2d pose{frc::Pose2d{}};
  units::radians_per_second_t omega{0};
};