// Copyright (c) Choreo contributors

#pragma once

#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "choreo/auto/AutoTrajectory.h"

namespace choreo {

/**
 * A loop that represents an autonomous routine.
 *
 * This loop is used to handle autonomous trigger logic and schedule commands.
 * This loop should
 * **not** be shared across multiple autonomous routines.
 *
 * @param SampleType the template type of trajectory sample to use
 * @param Year the field year. Used for proper mirroring of trajectories
 */
template <choreo::TrajectorySample SampleType, int Year>
class AutoLoop {
 public:
  /**
   * Creates a new loop with a specific name
   *
   * @param name The name of the loop
   * @see AutoFactory#newLoop Creating a loop from a AutoFactory
   */
  explicit AutoLoop(std::string_view name)
      : loop{frc::EventLoop{}}, name{name} {}

  /**
   * A constructor to be used when inhereting this class to instantiate a custom
   * inner loop
   *
   * @param name The name of the loop
   * @param loop The inner {@link EventLoop}
   */
  AutoLoop(std::string_view name, frc::EventLoop&& loop)
      : loop{std::move(loop)}, name{name} {}

  AutoLoop(const AutoLoop&) = delete;
  AutoLoop& operator=(const AutoLoop&) = delete;

  AutoLoop(AutoLoop&& other) noexcept = default;
  AutoLoop& operator=(AutoLoop&& other) noexcept = default;

  /**
   * Returns a frc2::Trigger that is true while this autonomous loop is being
   * polled.
   *
   * Using a frc2::Trigger.OnFalse() will do nothing as when this is false the
   * loop is not being polled anymore.
   *
   * @return A frc2::Trigger that is true while this autonomous loop is being
   * polled.
   */
  frc2::Trigger Enabled() {
    return frc2::Trigger{&loop, [this] {
                           return isActive &&
                                  frc::DriverStation::IsAutonomousEnabled();
                         }};
  }

  /// Polls the loop. Should be called in the autonomous periodic method.
  void Poll() {
    if (!frc::DriverStation::IsAutonomousEnabled() || isKilled) {
      isActive = false;
      return;
    }
    loop.Poll();
    isActive = true;
  }

  /**
   * Gets the event loop that this loop is using.
   *
   * @return The event loop that this loop is using.
   */
  frc::EventLoop* GetLoop() { return &loop; }

  /**
   * Resets the loop. This can either be called on auto init or auto end to
   * reset the loop incase you run it again. If this is called on a loop that
   * doesn't need to be reset it will do nothing.
   */
  void Reset() { isActive = false; }

  /** Kills the loop and prevents it from running again. */
  void Kill() {
    frc2::CommandScheduler::GetInstance().CancelAll();
    if (isKilled) {
      return;
    }
    Reset();
    FRC_ReportError(frc::warn::Warning, "Killed an Auto Loop");
    isKilled = true;
  }

  /**
   * Creates a frc2::CommandPtr that will poll this event loop and reset it when
   * it is cancelled.
   *
   * @return A command that will poll this event loop and reset it when it is
   * cancelled.
   * @see #Cmd(std::function<bool()>) A version of this method that takes a
   * condition to finish the loop.
   */
  frc2::CommandPtr Cmd() {
    return frc2::cmd::Run([this] { Poll(); })
        .FinallyDo([this] { Reset(); })
        .Until([this] { return !frc::DriverStation::IsAutonomousEnabled(); })
        .WithName("ChoreoAutoLoop");
  }

  /**
   * Creates a frc2::CommandPtr that will poll this event loop and reset it when
   * it is finished or canceled.
   *
   * @param finishCondition A condition that will finish the loop when it is
   * true.
   * @return A command that will poll this event loop and reset it when it is
   * finished or canceled.
   * @see #Cmd() A version of this method that doesn't take a condition and
   * never finishes.
   */
  frc2::CommandPtr Cmd(std::function<bool()> finishCondition) {
    return frc2::cmd::Run([this] { Poll(); })
        .FinallyDo([this] { Reset(); })
        .Until([this, finishCondition] {
          return !frc::DriverStation::IsAutonomousEnabled() ||
                 finishCondition();
        })
        .WithName("ChoreoAutoLoop");
  }

 private:
  /// The underlying EventLoop that triggers are bound to and polled
  frc::EventLoop loop;
  /// The name of the auto routine this loop is associated with
  std::string name;
  /// A boolean utalized in Enabled() to resolve trueness
  bool isActive = false;
  /// A boolean that is true when the loop is killed
  bool isKilled = false;
};

}  // namespace choreo
