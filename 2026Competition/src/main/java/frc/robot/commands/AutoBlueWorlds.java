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
public class AutoBlueWorlds extends SequentialCommandGroup {
  /** Creates a new AutoBlueWorlds. */
  public AutoBlueWorlds() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoShootUntilEmpty().raceWith(new WaitCommand(4)),
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueRightLine_PastCenter", false, false),
      new DeployIntakeSequence(),
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BluePastCenter_CenterBalls", false, false),
      new RetractIntakeSequence(),
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueCenterBalls_BlueRightBump", false, false),
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueRightBump_BlueDepot", false, false)
         .alongWith(new WaitCommand(2).andThen(new AutoShootUntilEmpty())),
      new DeployIntakeSequence(),
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueDepot_BlueLeftLine", false, false),
      new RetractIntakeSequence()
         // .alongWith(new AutoShootUntilEmpty())
          //TODO: NEED TO ADD CLIMBING COMMANDS HERE
    );
  }
}
