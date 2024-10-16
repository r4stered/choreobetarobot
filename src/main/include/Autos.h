#pragma once

#include "choreo/auto/AutoFactory.h"
#include "choreo/auto/AutoLoop.h"
#include "choreo/auto/AutoTrajectory.h"
#include "choreo/trajectory/SwerveSample.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "subsystems/SwerveDrive.h"

class AutoRoutines {
 public:
  explicit AutoRoutines(SwerveDrive& swerveSub,
                        choreo::AutoFactory<choreo::SwerveSample>& factory)
      : swerveSub{swerveSub},
        factory{factory},
        loop{factory.NewLoop("Auto Routine Loops")} {}

  frc2::CommandPtr TestAuto() {
    factory.Bind("test", [] { return frc2::cmd::Print("Hello from marker"); });
    straightTraj = factory.Trajectory("Straight", 1, loop);

    loop.Enabled().OnTrue(frc2::cmd::RunOnce([this] {
                      swerveSub.ResetPose(
                          straightTraj.GetInitialPose().value(), true);
                    })
                    .AndThen(straightTraj.Cmd())
                    .WithName("Straight Traj Name"));
    return loop.Cmd().WithName("Test Auto Loop Cmd");
  }

 private:
  SwerveDrive& swerveSub;
  choreo::AutoTrajectory<choreo::SwerveSample> straightTraj;
  choreo::AutoFactory<choreo::SwerveSample>& factory;
  choreo::AutoLoop<choreo::SwerveSample> loop;
};