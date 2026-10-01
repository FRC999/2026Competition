package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.lib.ShotPlanner.Mode;

/** Finish the route, then shoot for the remaining autonomous budget. */
public class AutoBlueTrenchToOutpostAndShoot extends SequentialCommandGroup {
  public AutoBlueTrenchToOutpostAndShoot() {
    addCommands(
        RobotContainer.followCompetitionPath("BlueTrenchRight_BlueOutpost", false, PrecisionPathCommands.FieldFrame.ALLIANCE),
        new ShootWhileHeld(Mode.MOVING_AUTO, false));
  }
}
