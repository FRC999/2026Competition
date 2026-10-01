package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Zero drive velocity at the measured steering angles captured on entry to precision hold. */
final class PrecisionModuleAngleHoldRequest implements SwerveRequest {
  private final SwerveModule.ModuleRequest[] requests;

  PrecisionModuleAngleHoldRequest(SwerveModuleState[] measuredStates) {
    SwerveModuleState[] states = captureStoppedStates(measuredStates);
    requests = new SwerveModule.ModuleRequest[states.length];
    for (int i = 0; i < states.length; i++) {
      requests[i] = new SwerveModule.ModuleRequest()
          .withState(states[i])
          .withDriveRequest(SwerveModule.DriveRequestType.Velocity)
          .withSteerRequest(SwerveModule.SteerRequestType.Position);
    }
  }

  // Pure snapshot: do not retain the mutable drivetrain state objects or recapture every loop.
  static SwerveModuleState[] captureStoppedStates(SwerveModuleState[] measuredStates) {
    SwerveModuleState[] result = new SwerveModuleState[measuredStates.length];
    for (int i = 0; i < measuredStates.length; i++) {
      result[i] = new SwerveModuleState(0.0, measuredStates[i].angle);
    }
    return result;
  }

  @Override
  public StatusCode apply(
      SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
    // Constructed from this drivetrain's ModuleStates, in the same module order.
    // CTRE invokes custom requests on its odometry thread. No refreshes, logging or allocations here.
    for (int i = 0; i < modulesToApply.length; i++) {
      modulesToApply[i].apply(requests[i]);
    }
    return StatusCode.OK;
  }
}
