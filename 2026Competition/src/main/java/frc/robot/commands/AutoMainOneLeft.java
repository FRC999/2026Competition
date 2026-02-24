// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.lib.TrajectoryHelper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoMainOneLeft extends SequentialCommandGroup {
  /** Creates a new AutoMainOne. */
  public AutoMainOneLeft() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
         new DeferredCommand(
          () -> RobotContainer.runTrajectory2Poses(
              true,
              //new Pose2d(3.884, 6.966, new Rotation2d()),
              TrajectoryHelper.AutoDesiredPoses.BlueTrenchLeft,
              //RobotContainer.driveSubsystem.getPose(),
              TrajectoryHelper.AutoDesiredPoses.BlueTrenchLeft2),
          Set.of(RobotContainer.driveSubsystem)),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueTrenchLeft2_BlueNeutralLeftMiddle", false, false),
          //new StartIntake(),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueNeutralLeftMiddle_BlueNeutralRightMiddle", false, false),
          //new StopIntake(),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueNeutralRight_BlueBumpRight2", false, false),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueBumpRight2_BlueAllianceRight", false, false),
          //new AutoShootUntilEmpty(),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueAllianceRight_BlueHubSideRight", false, false),
          //new StartIntake(),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueHubSideRight_BlueHubSideLeft", false, false),
          //new StopIntake(),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueHubSideLeft_BlueAllianceLeft", false, false),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueAllianceLeft_BlueTower", false, false)
          //.alongWith(new AutoShootUntilEmpty())
    );
  }
}
