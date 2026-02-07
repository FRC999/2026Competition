// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.lib.TrajectoryHelper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoStrategyOne extends SequentialCommandGroup {
  /** Creates a new AutoStrategyOne. */
  public AutoStrategyOne() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      RobotContainer.runTrajectory2Poses(
                RobotContainer.questNavSubsystem.getQuestRobotPose2d(),
                TrajectoryHelper.AutoDesiredPoses.BlueDepot,
                false)
            .alongWith(new AutoShootUntilEmpty()),
      new StartIntake()
      .raceWith(new WaitCommand(3)),
      new StopIntake(),
      RobotContainer.runTrajectory2Poses(
                RobotContainer.questNavSubsystem.getQuestRobotPose2d(),
                TrajectoryHelper.AutoDesiredPoses.BlueTower,
                false)
            .alongWith(new AutoShootUntilEmpty())
      
    );
  }
}
