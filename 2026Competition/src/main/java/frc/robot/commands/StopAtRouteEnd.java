package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PrecisionDrive;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** Brake once and qualify an ordinary route stop without chasing camera pose noise. */
public final class StopAtRouteEnd extends Command {
  // Provisional route acceptance, deliberately separate from centimeter-level alignment tests.
  public static final double POSITION_TOLERANCE_METERS = .20;
  public static final double HEADING_TOLERANCE_DEGREES = 5;
  public static final double MAX_MODULE_SPEED_MPS = .15;
  public static final double MAX_GYRO_RATE_RAD_PER_SEC = Math.toRadians(8);
  public static final double QUALIFY_SECONDS = .06;
  public static final double TIMEOUT_SECONDS = .50;
  private final PrecisionDrive drive;
  private final Pose2d target;
  private final BooleanSupplier localizationReady;
  private final Timer timer = new Timer();
  private double readySince;
  private boolean qualified, succeeded;

  public StopAtRouteEnd(PrecisionDrive drive, Pose2d target, BooleanSupplier localizationReady) {
    this.drive = drive; this.target = target; this.localizationReady = localizationReady;
    addRequirements(drive);
  }

  @Override public void initialize() {
    timer.restart(); readySince = Double.NaN; qualified = succeeded = false;
    drive.stop();
  }

  @Override public void execute() {
    drive.stop();
    Pose2d pose = drive.getPose();
    double positionError = pose.getTranslation().getDistance(target.getTranslation());
    double headingError = Math.abs(pose.getRotation().minus(target.getRotation()).getDegrees());
    boolean calm = Math.abs(drive.getGyroYawRateRadiansPerSecond()) <= MAX_GYRO_RATE_RAD_PER_SEC;
    var modules = drive.getModuleStates();
    calm &= modules.length > 0;
    for (var module : modules) calm &= Math.abs(module.speedMetersPerSecond) <= MAX_MODULE_SPEED_MPS;
    boolean ready = localizationReady.getAsBoolean() && positionError <= POSITION_TOLERANCE_METERS
        && headingError <= HEADING_TOLERANCE_DEGREES && calm;
    if (!ready) readySince = Double.NaN;
    else if (Double.isNaN(readySince)) readySince = timer.get();
    qualified = ready && timer.get() - readySince >= QUALIFY_SECONDS;
    Logger.recordOutput("Auto/RouteStop/PositionErrorMeters", positionError);
    Logger.recordOutput("Auto/RouteStop/HeadingErrorDegrees", headingError);
    Logger.recordOutput("Auto/RouteStop/Seconds", timer.get());
    Logger.recordOutput("Auto/RouteStop/Qualified", qualified);
  }

  @Override public boolean isFinished() { return qualified || timer.hasElapsed(TIMEOUT_SECONDS); }
  @Override public void end(boolean interrupted) {
    succeeded = !interrupted && qualified && localizationReady.getAsBoolean();
    timer.stop(); drive.stop();
    Logger.recordOutput("Auto/RouteStop/Result", interrupted ? "INTERRUPTED" : succeeded ? "SUCCEEDED" : "FAILED");
  }
  public boolean succeeded() { return succeeded; }
}
