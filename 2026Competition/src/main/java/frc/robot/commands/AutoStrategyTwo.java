// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.core.RotatedRect;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.lib.TrajectoryHelper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoStrategyTwo extends SequentialCommandGroup {
  /** Creates a new AutoStrategyTwo. */
  public AutoStrategyTwo() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoShootUntilEmpty(),
      RobotContainer.runTrajectory2Poses(
                RobotContainer.questNavSubsystem.getQuestRobotPose2d(),
                new Pose2d(TrajectoryHelper.AutoDesiredPoses.BlueNeurtralMiddle.getTranslation(), new Rotation2d(Math.toRadians(-90))),
                false),
      RobotContainer.runTrajectory2Poses(
                RobotContainer.questNavSubsystem.getQuestRobotPose2d(),
                new Pose2d(TrajectoryHelper.AutoDesiredPoses.BlueNeutralRight.getTranslation(), new Rotation2d(Math.toRadians(90))),
                false)
                .alongWith(new StartIntake()),
      new StopIntake(),
      RobotContainer.runTrajectory2Poses(
                RobotContainer.questNavSubsystem.getQuestRobotPose2d(),
                TrajectoryHelper.AutoDesiredPoses.BlueTower,
                false)
            .alongWith(new AutoShootUntilEmpty())
    );
  }
}
