package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.OperatorConstants.IntakeConstants;
import frc.robot.Constants.OperatorConstants.IntakeConstants.IntakePositions;
import frc.robot.RobotContainer;

public class InitialAutoDeployIntake extends SequentialCommandGroup {
  public InitialAutoDeployIntake() {
    addCommands(
        new InitialAutoDeployShove(),
        new RetractIntakeSequence());
  }

  private static final class InitialAutoDeployShove extends Command {
    private final Timer timeoutTimer = new Timer();

    private InitialAutoDeployShove() {
      addRequirements(RobotContainer.intakeSubsystem);
    }

    @Override
    public void initialize() {
      System.out.println("*****intake force command");
      timeoutTimer.restart();
      RobotContainer.intakeSubsystem.stopIntake();
      RobotContainer.intakeSubsystem.beginInitialAutoDeploy(
          IntakeConstants.INTAKE_PIVOT_INITIAL_AUTO_DEPLOY_DUTY);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
      timeoutTimer.stop();
      RobotContainer.intakeSubsystem.stopPivotAndHoldCurrentPosition();
    }

    @Override
    public boolean isFinished() {
      return RobotContainer.intakeSubsystem.isAtPosition(IntakePositions.IntakeInitialDeployDeg)
          || timeoutTimer.hasElapsed(IntakeConstants.INTAKE_PIVOT_INITIAL_AUTO_DEPLOY_TIMEOUT_SEC);
    }
  }
}
