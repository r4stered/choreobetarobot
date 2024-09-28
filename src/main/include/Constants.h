#pragma once

#include <numbers>

#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>
#include <units/length.h>

namespace constants {
namespace Swerve {

inline constexpr units::meter_t kTrackWidth{18.5_in};
inline constexpr units::meter_t kTrackLength{18.5_in};
inline constexpr units::meter_t kRobotWidth{25_in + 3.25_in * 2};
inline constexpr units::meter_t kRobotLength{25_in + 3.25_in * 2};
inline constexpr units::meters_per_second_t kMaxLinearSpeed{15.5_fps};
inline constexpr units::radians_per_second_t kMaxAngularSpeed{720_deg_per_s};
inline constexpr units::meter_t kWheelDiameter{4_in};
inline constexpr units::meter_t kWheelCircumference{kWheelDiameter *
                                                    std::numbers::pi};

inline constexpr double kDriveGearRatio = 6.75;
inline constexpr double kSteerGearRatio = 12.8;

inline constexpr units::meter_t kDriveDistPerPulse =
    kWheelCircumference / 1024.0 / kDriveGearRatio;
inline constexpr units::radian_t kSteerRadPerPulse =
    units::radian_t{2 * std::numbers::pi} / 1024.0;

inline constexpr double kDriveKP = 1.0;
inline constexpr double kDriveKI = 0.0;
inline constexpr double kDriveKD = 0.0;

inline constexpr double kSteerKP = 20.0;
inline constexpr double kSteerKI = 0.0;
inline constexpr double kSteerKD = 0.25;

inline const frc::SimpleMotorFeedforward<units::meters> kDriveFF{
    0.25_V, 2.5_V / 1_mps, 0.3_V / 1_mps_sq};

inline const frc::SimpleMotorFeedforward<units::radians> kSteerFF{
    0.5_V, 0.25_V / 1_rad_per_s, 0.01_V / 1_rad_per_s_sq};

struct ModuleConstants {
 public:
  const int moduleNum;
  const int driveMotorId;
  const int driveEncoderA;
  const int driveEncoderB;
  const int steerMotorId;
  const int steerEncoderA;
  const int steerEncoderB;
  const double angleOffset;
  const frc::Translation2d centerOffset;

  ModuleConstants(int moduleNum, int driveMotorId, int driveEncoderA,
                  int driveEncoderB, int steerMotorId, int steerEncoderA,
                  int steerEncoderB, double angleOffset, units::meter_t xOffset,
                  units::meter_t yOffset)
      : moduleNum(moduleNum),
        driveMotorId(driveMotorId),
        driveEncoderA(driveEncoderA),
        driveEncoderB(driveEncoderB),
        steerMotorId(steerMotorId),
        steerEncoderA(steerEncoderA),
        steerEncoderB(steerEncoderB),
        angleOffset(angleOffset),
        centerOffset(frc::Translation2d{xOffset, yOffset}) {}
};

inline const ModuleConstants FL_CONSTANTS{
    1, 0, 0, 1, 1, 2, 3, 0, kTrackLength / 2, kTrackWidth / 2};
inline const ModuleConstants FR_CONSTANTS{
    2, 2, 4, 5, 3, 6, 7, 0, kTrackLength / 2, -kTrackWidth / 2};
inline const ModuleConstants BL_CONSTANTS{
    3, 4, 8, 9, 5, 10, 11, 0, -kTrackLength / 2, kTrackWidth / 2};
inline const ModuleConstants BR_CONSTANTS{
    4, 6, 12, 13, 7, 14, 15, 0, -kTrackLength / 2, -kTrackWidth / 2};
}  // namespace Swerve
}  // namespace constants