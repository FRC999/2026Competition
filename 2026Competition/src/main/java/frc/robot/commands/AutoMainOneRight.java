// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.lib.TrajectoryHelper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoMainOneRight extends SequentialCommandGroup {
  /** Creates a new AutoMainOneRight. */
  public AutoMainOneRight() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
         new DeferredCommand(
          () -> RobotContainer.runTrajectory2Poses(
              true,
              //new Pose2d(3.884, 6.966, new Rotation2d()),
              RobotContainer.driveSubsystem.getPose(),
              TrajectoryHelper.AutoDesiredPoses.BlueBumpRight2),
          Set.of(RobotContainer.driveSubsystem)),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueBumpRight2_BlueNeutralRightMiddle", false, false),
          new StartIntake(),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueNeutralRightMiddle_BlueNeutralLeftMiddle", false, false),
          new StopIntake(),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueNeutralLeftMiddle_BlueBumpLeft2", false, false),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueBumpLeft2_BlueAllianceLeft", false, false),
          new AutoShootUntilEmpty(),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueAllianceLeft_BlueHubSideLeft", false, false),
          new StartIntake(),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueHubSideLeft_BlueHubSideRight", false, false),
          new StopIntake(),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueHubSideRight_BlueAllianceRight", false, false),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueAllianceRight_BlueTower", false, false)
          .alongWith(new AutoShootUntilEmpty())
    );
  }
}
