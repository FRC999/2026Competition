// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.lib.TrajectoryHelper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoStrategySeven extends SequentialCommandGroup {
  /** Creates a new AutoStrategySeven. */
  public AutoStrategySeven() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DeferredCommand(
          () -> RobotContainer.runTrajectory2Poses(
              true,
              new Pose2d(3.538, 2.352, new Rotation2d(Math.toRadians(0))),
              //RobotContainer.driveSubsystem.getPose(),
              new Pose2d(TrajectoryHelper.AutoDesiredPoses.BlueBumpRight2.getTranslation(), new Rotation2d(Math.toRadians(0)))),
          Set.of(RobotContainer.driveSubsystem)),
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueBumpRight2_BlueNeutralMiddle", false, false),
      //new StartIntake(),
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueNeutralMiddle_BlueNeutralRight", false, false),
      //new StopIntake(),
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueNeutralRight_BlueBumpRight", false, false),
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueBumpRight_BlueOutpost", false, false),
         // .alongWith(new AutoShootUntilEmpty()),
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueOutpost_BlueTower", false, false)
         // .alongWith(new AutoShootUntilEmpty())
          //TODO: NEED TO ADD CLIMBING COMMANDS HERE
    );
  }
}
