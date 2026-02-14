// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.lib.TrajectoryHelper.AutoDesiredPoses;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoStrategyEight extends SequentialCommandGroup {
  /** Creates a new AutoStrategyEight. */
  public AutoStrategyEight() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      RobotContainer.runTrajectory2Poses(
            true,
            RobotContainer.driveSubsystem.getPose(),
            AutoDesiredPoses.BlueBumpRight2),
        RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueBumpRight2_InterferenceRight",false, false),
        new StartIntake(),
        RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("InterferenceRight_InterferenceLeft",false, false),
        new StopIntake(),
        RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("InterferenceLeft_BlueBumpLeft",false, false),
        RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueBumpLeft2_BlueTower",false, false)
        // TODO: NEED TO ADD CLIMBING COMMANDS HERE
    );
  }
}
