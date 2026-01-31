// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoStrategyThree extends SequentialCommandGroup {
  /** Creates a new AutoStrategyThree. */
  public AutoStrategyThree() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DeferredCommand(
        () -> RobotContainer.runTrajectory2Poses(null, null, false), 
        Set.of()),
      new WaitCommand(4),
      new DeferredCommand(
        () -> RobotContainer.runTrajectory2Poses(null, null, false), 
        Set.of())
      );
  }
}
