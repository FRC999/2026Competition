// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AutoShootSupervisorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBlueTrenchToOutpostAndShoot extends SequentialCommandGroup {
  private static Command createDelayedIntakeCycleSequence(int cycleCount) {
    Command[] commands = new Command[cycleCount * 2 + 1];
    commands[0] = new WaitCommand(1);

    for (int i = 0; i < cycleCount; i++) {
      commands[i * 2 + 1] = new DeployIntakeSequence();
      commands[i * 2 + 2] = new RetractIntakeSequence();
    }

    return new SequentialCommandGroup(commands);
  }

  /** Creates a new AutoMoveOneMeterAndShootLastResort. */
  public AutoBlueTrenchToOutpostAndShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("BlueTrenchRight_BlueOutpost", false, false),
        // .alongWith(createDelayedIntakeCycleSequence(5)),
      new ShootWhileHeld(AutoShootSupervisorSubsystem.ShotMode.MOVING_AUTO, false)
        .raceWith(new WaitCommand(5)),
      //new RetractIntakeSequence(),
      new ShootWhileHeld(AutoShootSupervisorSubsystem.ShotMode.MOVING_AUTO, false)
        .raceWith(new WaitCommand(4)),
      //new RetractIntakeSequence(),
      new ShootWhileHeld(AutoShootSupervisorSubsystem.ShotMode.MOVING_AUTO, false)
        .raceWith(new WaitCommand(10))
    );
  }
}
