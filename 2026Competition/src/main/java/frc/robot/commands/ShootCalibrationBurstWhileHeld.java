package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * Fixed-RPM calibration cycle for shooter + transfer + spindexer only.
 *
 * Behavior while held:
 * - Shooter spins to a fixed RPM once on initialize.
 * - Spindexer runs supply continuously.
 * - Transfer runs at a slow base speed until shooter is ready.
 * - Then transfer runs at FEED_RPS continuously.
 * - Balls are counted using throat-sensor falling edges (blocked -> clear).
 * - After BALLS_PER_BURST balls, transfer slows to STAGE_RPS until shooter is ready again.
 * - Repeats until the button is released.
 */
public class ShootCalibrationBurstWhileHeld extends Command {
  private enum CycleState {
    SPINUP,
    FEEDING,
    RECOVERING
  }

  private final double shooterRpm;

  private CycleState state = CycleState.SPINUP;
  private int burstBallCount = 0;
  private boolean previousBallAtThroat = false;
  private double lastCountTs = -1.0;

  public ShootCalibrationBurstWhileHeld(double shooterRpm) {
    this.shooterRpm = shooterRpm;

    addRequirements(
        RobotContainer.shooterSubsystem,
        RobotContainer.transferSubsystem,
        RobotContainer.spindexerSubsystem);
  }

  @Override
  public void initialize() {
    state = CycleState.SPINUP;
    burstBallCount = 0;
    previousBallAtThroat = RobotContainer.transferSubsystem.hasBallAtThroat();
    lastCountTs = -1.0;

    RobotContainer.autoShootSupervisorSubsystem.setShootRequested(false);
    RobotContainer.autoShootSupervisorSubsystem.setCalibrationActive(true);

    RobotContainer.shooterSubsystem.setTargetRpm(shooterRpm);
    RobotContainer.spindexerSubsystem.runSupply();
    RobotContainer.transferSubsystem.runVelocityRps(
        Constants.OperatorConstants.Transfer.THROAT_BLOCKED_STAGE_RPS);
  }

  @Override
  public void execute() {
    final double now = Timer.getFPGATimestamp();
    final boolean shooterReady = RobotContainer.shooterSubsystem.isReadyToShoot();
    final boolean ballAtThroat = RobotContainer.transferSubsystem.hasBallAtThroat();

    RobotContainer.spindexerSubsystem.runSupply();

    switch (state) {
      case SPINUP:
        RobotContainer.transferSubsystem.runVelocityRps(
            Constants.OperatorConstants.Transfer.THROAT_BLOCKED_STAGE_RPS);

        if (shooterReady) {
          state = CycleState.FEEDING;
        }
        break;

      case FEEDING:
        RobotContainer.transferSubsystem.runFeed();

        boolean throatCleared = previousBallAtThroat && !ballAtThroat;
        boolean debounceOk = (lastCountTs < 0.0)
            || ((now - lastCountTs) >= Constants.OperatorConstants.Transfer.THROAT_COUNT_DEBOUNCE_S);

        if (throatCleared && debounceOk) {
          burstBallCount++;
          lastCountTs = now;

          if (burstBallCount >= Constants.OperatorConstants.Transfer.BALLS_PER_BURST) {
            state = CycleState.RECOVERING;
            burstBallCount = 0;
            // RobotContainer.transferSubsystem.runVelocityRps(
            //     Constants.OperatorConstants.Transfer.STAGE_RPS);
            RobotContainer.spindexerSubsystem.runSlow();
          }
        }
        break;

      case RECOVERING:
        // RobotContainer.transferSubsystem.runVelocityRps(
        //     Constants.OperatorConstants.Transfer.STAGE_RPS);
            RobotContainer.spindexerSubsystem.runSlow();

        if (shooterReady) {
          state = CycleState.FEEDING;
        }
        break;

      default:
        break;
    }

    previousBallAtThroat = ballAtThroat;
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.transferSubsystem.stop();
    RobotContainer.spindexerSubsystem.stop();
    RobotContainer.shooterSubsystem.stop();
    RobotContainer.autoShootSupervisorSubsystem.setCalibrationActive(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}