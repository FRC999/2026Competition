package frc.robot.subsystems;

										  
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorPhaseValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
// Simulation
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DebugTelemetrySubsystems;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.OperatorConstants.Turret;


/**
 * Turret using an absolute CAN Through-Bore (CANcoder) sensor (wraps every 360 deg) on a Talon FXS.
 *
 * <p>Key requirements (your rules):
 * <ul>
 *   <li>At boot, turret is within +/- 180 degrees of forward.</li>
 *   <li>Turret must never go beyond +/- 340 degrees of forward (umbilical safety).</li>
 *   <li>We track multi-turn angle in software (unwrap absolute) to enforce +/-340.</li>
 *   <li>We use hardware PID (PositionVoltage) but we dynamically toggle ContinuousWrap:
 *       <ul>
 *         <li>Wrap ON for "short way" moves (|delta| <= 180 deg)</li>
 *         <li>Wrap OFF when the short way would violate the +/-340 rule (forcing the long way)</li>
 *       </ul>
 *   </li>
 * </ul>
 */
public class TurretSubsystem extends SubsystemBase {

  // Turret motor controller on the specified CAN bus.
  private TalonFX turret;

  // Absolute encoder (CAN Through-Bore / CANcoder) on the same CAN bus.
  private CANcoder throughboreCANcoder = new CANcoder(Turret.CAN_ENCODER_ID, Turret.CANBUS_NAME);

  // Open-loop duty request (used for manual and SysId drive).
  private final DutyCycleOut dutyRequest = new DutyCycleOut(0);

  private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(false);
																							  
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(false);

  // Track what we last commanded so simulation can use true volts (not normalized output).

  // Absolute CAN Through-Bore (0..1 rotations). (Wraps every revolution.)
  private final StatusSignal<Angle> absPosSig = throughboreCANcoder.getAbsolutePosition();
																				 
  // Motor voltage is used for telemetry and SysId logging.
  private StatusSignal<Voltage> motorVoltageSig;

  // Integrated (relative) position from the TalonFX (multi-turn, does not wrap).
  private StatusSignal<Angle> motorPosSig;

  // Used to guard sim-only code paths.
  private final boolean isSim = RobotBase.isSimulation();

  // If the absolute sensor only increases when turret turns CW, set +1 for CW-positive convention.
  // If you ever re-install and it flips, change to -1.
  private static final double ANGLE_SIGN = -1.0;

  // ---------------- Software unwrap tracking ----------------

  /** last wrapped absolute angle (deg) in [0, 360) */
  private double lastAbsDegWrapped = 0.0;

  /** continuous turret angle (deg), 0=forward, CCW positive, clamped to +/-340 */
  private double continuousDeg = 0.0;

  // Unclamped multi-turn angle directly from motor position (deg).
  // This is what we use for visualization wrapping in Mechanism2d.
  private double continuousDegUnclamped = 0.0;


  /** derived velocity estimate */
  private double lastContinuousDeg = 0.0;
  private double lastUpdateTs = Timer.getFPGATimestamp();
  private double estVelDegPerSec = 0.0;

  /** continuous target angle (deg) in [-340, +340] */
  private double targetDeg = 0.0;

 /** turret ZERO reference in degrees in the absolute sensor frame */
private final double forwardDeg =
    (Constants.OperatorConstants.Turret.ABS_ZERO_TICKS
        / (double) Constants.OperatorConstants.Turret.ABS_TICKS_PER_REV) * 360.0;

  // ---------------- Continuous wrap toggling ----------------

  // Tracks the currently-applied wrap mode (so we don't spam configs).
  // With a ±180 turret and the requirement (-170 -> +170 goes through 0),
  // continuous wrap MUST remain disabled.
  private boolean continuousWrapEnabled = false;
  private final ClosedLoopGeneralConfigs clWrapOff = new ClosedLoopGeneralConfigs().withContinuousWrap(false);

  // ---------------- Simulation ----------------
  private Mechanism2d turretMech;
  private MechanismLigament2d turretArm;

  // Sim model used only in simulationPeriodic().
  private final DCMotorSim turretSim =
    new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1),
            Constants.OperatorConstants.Turret.SIM_TURRET_J_KGM2,
            Constants.OperatorConstants.Turret.SIM_GEAR_RATIO),
        DCMotor.getKrakenX60(1));

  // Integrated simulated position in rotations.
  private double simPosRot = 0.0;
  // Optional: simple supply drop model (ohms), same idea as KrakenMotorSubsystem.
  private static final double SIM_MOTOR_RESISTANCE_OHMS = 0.002;


  // ---------------- SysId Characterization ----------------

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.per(Seconds).of(Constants.OperatorConstants.SysId.TURRET_RAMP_RATE_V_PER_S),
              Volts.of(Constants.OperatorConstants.SysId.TURRET_STEP_V),
              Seconds.of(Constants.OperatorConstants.SysId.TURRET_TIMEOUT_S)),
          new SysIdRoutine.Mechanism(this::sysIdVoltageDrive, this::sysIdLog, this, "turret"));

  /**
   * Runtime gating for SysId. Requires BOTH compile-time enable and dashboard enable.
													
   */
  private boolean isSysIdEnabled() {
    // Both gates must be true to allow SysId to actually move hardware.
    return Constants.OperatorConstants.SysId.ENABLE_SYSID
        && SmartDashboard.getBoolean(Constants.OperatorConstants.SysId.SYSID_DASH_ENABLE_KEY, false);
  }

  public TurretSubsystem() {
    if(!EnabledSubsystems.turret){
      return;
    }
   turret = new TalonFX(Constants.OperatorConstants.Turret.MOTOR_ID,
        Constants.OperatorConstants.Turret.CANBUS_NAME);

    motorVoltageSig = turret.getMotorVoltage();

    motorPosSig = turret.getPosition();

    // Hardware config: motor output + current limits + gains.
    configureHardware();

    // CAN signal update rates (reduces bus load but keeps control inputs fresh).
    configureStatusSignals();

    // Seed continuous angle from absolute on boot.
    seedFromAbsoluteAtBoot();

    // Dashboard defaults.
    SmartDashboard.putBoolean(Constants.OperatorConstants.SysId.SYSID_DASH_ENABLE_KEY, false);
    SmartDashboard.putBoolean("Turret/ContinuousWrapEnabled", continuousWrapEnabled);

    if (Constants.DebugTelemetrySubsystems.turret) {
        turretMech = new Mechanism2d(2.0, 2.0);
        MechanismRoot2d root = turretMech.getRoot("TurretRoot", 1.0, 1.0);
        turretArm = new MechanismLigament2d("TurretArm", 0.8, 0.0);
        root.append(turretArm);

        SmartDashboard.putData("Turret/Mechanism", turretMech);
    }

												
  }

  private void configureStatusSignals() {			  
    // We want absolute angle to update quickly for unwrap math + control decisions.
    absPosSig.setUpdateFrequency(100.0);

    // Integrated motor position (used for continuous angle tracking once seeded).
    motorPosSig.setUpdateFrequency(100.0);

    // Motor voltage can be slower; mostly for telemetry and SysId.
    motorVoltageSig.setUpdateFrequency(50.0);

    // Let Phoenix reduce unnecessary CAN chatter.
    turret.optimizeBusUtilization();
  }

  private InvertedValue motorInvertedValue() {			
																			  
    // Convert the project constant into Phoenix's inversion enum.
    return Constants.OperatorConstants.Turret.MOTOR_INVERTED
        ? InvertedValue.CounterClockwise_Positive
        : InvertedValue.Clockwise_Positive;
  }

  private SensorPhaseValue sensorPhaseValue() {
    // Sensor phase aligns the remote sensor's positive direction with the motor/controller convention.
																									 
    boolean inverted = Constants.OperatorConstants.Turret.SENSOR_PHASE_INVERTED;
    return inverted ? SensorPhaseValue.Opposed : SensorPhaseValue.Aligned;
  }

  private void configureHardware() {
  // Motor output (brake + inversion)
  MotorOutputConfigs out = new MotorOutputConfigs()
      .withNeutralMode(NeutralModeValue.Brake)
      .withInverted(motorInvertedValue());

  // Current limits
  CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
  limits.SupplyCurrentLimitEnable = true;
  limits.SupplyCurrentLimit = Constants.OperatorConstants.Turret.SUPPLY_CURRENT_LIMIT_A;
  limits.SupplyCurrentLowerLimit = Constants.OperatorConstants.Turret.SUPPLY_CURRENT_LOWER_LIMIT_A;
  limits.SupplyCurrentLowerTime = Constants.OperatorConstants.Turret.SUPPLY_CURRENT_LOWER_TIME_S;
  limits.StatorCurrentLimitEnable = true;
  limits.StatorCurrentLimit = Constants.OperatorConstants.Turret.STATOR_CURRENT_LIMIT_A;

  // Slot0 gains for hardware position loop (voltage-based in Phoenix 6)
  Slot0Configs slot0 = new Slot0Configs()
      .withKP(Constants.OperatorConstants.Turret.kP)
      .withKI(Constants.OperatorConstants.Turret.kI)
      .withKD(Constants.OperatorConstants.Turret.kD)
      .withKS(Constants.OperatorConstants.Turret.kS)
      .withKV(Constants.OperatorConstants.Turret.kV)
      .withKA(Constants.OperatorConstants.Turret.kA);

  double cruiseMotorRps = (Constants.OperatorConstants.Turret.MM_CRUISE_DEG_PER_SEC / 360.0)
      * Constants.OperatorConstants.Turret.GEAR_RATIO_MOTOR_ROT_PER_TURRET_ROT;

  double accelMotorRps2 = (Constants.OperatorConstants.Turret.MM_ACCEL_DEG_PER_SEC2 / 360.0)
      * Constants.OperatorConstants.Turret.GEAR_RATIO_MOTOR_ROT_PER_TURRET_ROT;

  MotionMagicConfigs mm = new MotionMagicConfigs()
      .withMotionMagicCruiseVelocity(cruiseMotorRps)
      .withMotionMagicAcceleration(accelMotorRps2);

  // IMPORTANT: Turret closed-loop uses the TalonFX integrated (relative) sensor.
  // The CANcoder/ThroughBore is used ONLY to seed the integrated sensor at boot.
  TalonFXConfiguration cfg = new TalonFXConfiguration()
    .withMotorOutput(out)
    .withCurrentLimits(limits)
    .withSlot0(slot0)
    .withMotionMagic(mm)
    .withClosedLoopGeneral(clWrapOff);

  turret.getConfigurator().apply(cfg);
}			  

  public final double getRelativePosition() {
    // Integrated/relative position from CANcoder in rotations (does not wrap in the same way).
    return throughboreCANcoder.getPosition().getValueAsDouble();
  }

    /** Absolute throughbore position for telemetry (wraps every 1 rotation). */
  public final double getAbsolutePosition() {
    // Returns [0, 1) rotations, wraps at 1.0
    return throughboreCANcoder.getAbsolutePosition().getValueAsDouble();
  }

								
  // private void setContinuousWrap(boolean enable) {
  //   // If we're already in the desired wrap mode, do nothing (avoid CAN config spam).
  //   if (enable == continuousWrapEnabled) return;

  //   // Apply the wrap config to the motor controller.
  //   turret.getConfigurator().apply(enable ? clWrapOn : clWrapOff);

  //   // Record state and publish for debugging.
  //   continuousWrapEnabled = enable;
  //   SmartDashboard.putBoolean("Turret/ContinuousWrapEnabled", enable);
  // }

  /**
   * Read absolute angle (deg) in [0, 360).
   * If signal is stale, returns last known value.					   
   */
  private double getAbsDegWrapped() {
    // Refresh the absolute signal before using it.
    absPosSig.refresh();

    // Capture status so we can see CAN dropouts / signal errors on dashboard.
    StatusCode status = absPosSig.getStatus();
    SmartDashboard.putString("Turret/AbsStatus", status.toString());

    // If not OK, keep last known value (prevents large jumps in unwrap logic).
    if (status != StatusCode.OK) {
      return lastAbsDegWrapped;
    }

    // Read absolute rotations (nominally [0,1) but we defensively wrap it anyway).
    double rot = absPosSig.getValueAsDouble();
    rot = rot - Math.floor(rot); // ensure [0,1)

    // Convert rotations to degrees.
    double deg = rot * 360.0;

    // Force degrees into [0,360) for stable delta math.
    return wrapTo0To360(deg);
  }

  /**
   * Seed software continuous angle at boot, assuming within +/-180 of forward.
   */
  private void seedFromAbsoluteAtBoot() {
    // Small delay to let CAN signals become valid right after startup.
    Timer.delay(0.05);

    // Get current absolute angle (wrapped [0,360)).
    double absDeg = getAbsDegWrapped();

    // Initialize last wrapped state for future delta calculations.
    lastAbsDegWrapped = absDeg;	

    // Compute shortest signed angle difference from the defined forward reference.
    // wrapToPlusMinus180 handles wrap-around at 0/360.
    double deltaDeg = ANGLE_SIGN * wrapToPlusMinus180(absDeg - forwardDeg);
										 
    // Boot assumption: within +/-180 (or whatever BOOT_MAX_ABS_DEG is set to).
    deltaDeg = MathUtil.clamp(
        deltaDeg,
        -Constants.OperatorConstants.Turret.BOOT_MAX_ABS_DEG,
        Constants.OperatorConstants.Turret.BOOT_MAX_ABS_DEG);

    // Initialize continuous turret position in your forward-relative frame.
    continuousDeg = deltaDeg;

    // Seed the TalonFX integrated position so closed-loop uses relative sensor from
    // this point.
    // TalonFX position units are rotations; we keep continuousDeg in degrees.
    // Seed TalonFX integrated position in *motor* rotations, not turret rotations.
    double motorRot = motorRotFromTurretDeg(continuousDeg);
    turret.setPosition(ANGLE_SIGN * motorRot);

    // Initialize velocity bookkeeping.
    lastContinuousDeg = continuousDeg;
														 
    // Initialize dt tracking so velocity math starts clean.
    lastUpdateTs = Timer.getFPGATimestamp();

    // Start target at current so there is no immediate step command.
    targetDeg = continuousDeg;
							 
    // Publish seed diagnostics to dashboard.
    SmartDashboard.putNumber("Turret/SeedAbsDeg", absDeg);
    SmartDashboard.putNumber("Turret/SeedContinuousDeg", continuousDeg);	 
  }

  /**
   * Update continuousDeg by unwrapping absolute position, clamping to +/-340.
																	   
   */
  private void updateContinuousAngle() {
    // Capture time and dt for velocity estimation.
    double now = Timer.getFPGATimestamp();
    double dt = Math.max(1e-3, now - lastUpdateTs);

    // Force Phoenix to update the cached CAN/sim signals before we read them.
    // This is the missing step that makes motorPosSig change in simulation.
    BaseStatusSignal.refreshAll(motorPosSig, absPosSig);

    // Optional: if position is not OK, don't update the mechanism/angle this loop.
    if (motorPosSig.getStatus() != StatusCode.OK) {
      SmartDashboard.putString("Turret/MotorPosStatus", motorPosSig.getStatus().toString());
      lastUpdateTs = now;
      return;
    }

    double motorRotSensor = motorPosSig.getValueAsDouble();

    // Undo ANGLE_SIGN so motorRot is positive in your "CCW positive" turret
    // convention.
    double motorRot = motorRotSensor / ANGLE_SIGN;

    // Convert motor rotations -> turret degrees using gear ratio.
    double nextUnclamped = turretDegFromMotorRot(motorRot);

    // Safety-clamped degrees for control/safety logic
    double nextClamped = MathUtil.clamp(
        nextUnclamped,
        Constants.OperatorConstants.Turret.MIN_ANGLE_DEG,
        Constants.OperatorConstants.Turret.MAX_ANGLE_DEG);

    // Velocity estimate (deg/s) based on UNCLAMPED motion (smooth across limits)
    double prevUnclamped = continuousDegUnclamped;
    estVelDegPerSec = (nextUnclamped - prevUnclamped) / dt;

    // Commit state.
    lastContinuousDeg = continuousDeg;          // keep last clamped value for any debugging
    continuousDegUnclamped = nextUnclamped;     // used for Mechanism wrapping / display
    continuousDeg = nextClamped;                // used for safety + control
    lastUpdateTs = now;


    // Keep abs wrapped for telemetry/diagnostics
    lastAbsDegWrapped = getAbsDegWrapped();
    
  }


  // ---------------- Public API ----------------

  /** Continuous turret angle (deg), 0 = forward, CCW positive. */
  public double getAngleDeg() {
    // This is the software-unwrapped, safety-clamped turret angle.
    return continuousDeg;
  }

  public double getVelocityDegPerSec() {
    // Estimated velocity from continuous angle updates.
    return estVelDegPerSec;
  }

  public double getAppliedVolts() {
    // Refresh motor voltage signal (ensures telemetry reflects current output).
    motorVoltageSig.refresh();
    return motorVoltageSig.getValueAsDouble();
  }

  /** Absolute ticks (0-4095 equivalent) from wrapped absolute sensor. */
  public int getAbsoluteTicks() {
    // Convert last wrapped absolute degrees to a ticks-per-rev representation.
    double absDeg = lastAbsDegWrapped;

    // Scale degrees -> ticks and round to nearest int.
    int ticks = (int) Math.round((absDeg / 360.0) * Constants.OperatorConstants.Turret.ABS_TICKS_PER_REV);

    // Wrap into [0, ticksPerRev).
    ticks %= Constants.OperatorConstants.Turret.ABS_TICKS_PER_REV;
    if (ticks < 0) ticks += Constants.OperatorConstants.Turret.ABS_TICKS_PER_REV;

    return ticks;
							
  }

  /** Open-loop manual control with safety clamp. */
  public void setDutyCycle(double duty) {
    // Clamp duty to avoid commanding beyond your configured safe range.
    double maxDuty = isSim
      ? Constants.OperatorConstants.Turret.SIM_MAX_DUTY_CYCLE
      : Constants.OperatorConstants.Turret.MAX_DUTY_CYCLE;

    duty = MathUtil.clamp(duty, -maxDuty, maxDuty);


    // Send open-loop command to the motor controller.
    turret.setControl(dutyRequest.withOutput(duty));
  }

  public void setVoltageVolts(double volts) {
    // Clamp request to something physically plausible.
    // In sim we’ll assume 12V supply; on real robot, you can clamp to battery if you want.
    double v = MathUtil.clamp(volts, -12.0, 12.0);

    turret.setControl(voltageRequest.withOutput(v));
  }


  public void stop() {
    // Immediately stop output.
    turret.stopMotor();
  }

  /** Current continuous turret angle in degrees in this subsystem's reference frame (0 = "forward" per ABS_FORWARD_TICKS, CCW+). */
  public double getContinuousAngleDeg() {
    return continuousDeg;
  }

  /** Estimated turret angular velocity in deg/sec (sign matches getContinuousAngleDeg convention). */
  public double getEstimatedVelocityDegPerSec() {
    return estVelDegPerSec;
  }

  public void goToAngleDeg(double desiredDeg) {
    // Clamp to physical bounds. With ±180 hardware, we do not allow ±360
    // equivalents.
    double target = MathUtil.clamp(
        desiredDeg,
        Constants.OperatorConstants.Turret.MIN_ANGLE_DEG,
        Constants.OperatorConstants.Turret.MAX_ANGLE_DEG);

    targetDeg = target;

    // Enforce wrap disabled for required behavior (-170 -> +170 goes through 0).
    // setContinuousWrap(false);

    // Convert turret degrees -> motor rotations in Talon sensor frame.
    double motorRotTarget = ANGLE_SIGN * motorRotFromTurretDeg(target);

    turret.setControl(mmRequest.withPosition(motorRotTarget));

    SmartDashboard.putNumber("Turret/TargetDeg", targetDeg);
    System.out.println(targetDeg);
    SmartDashboard.putNumber("Turret/DeltaDegCmd", targetDeg - continuousDeg);
    SmartDashboard.putString("Turret/GoalStatus", "MM_WRAP_OFF_CLAMPED");
  }

  /** true if turret is within tolerance of desired angle (deg), using best safe equivalent. */
  public boolean atAngleDeg(double desiredDeg, double toleranceDeg) {
    // Compute the safe equivalent target we would command.
    double best = chooseBestEquivalentTargetDeg(desiredDeg);

    // If unreachable safely, then we can't be "at" it.
    if (Double.isNaN(best)) return false;

    // Compare current continuous position to the best safe target.
    return Math.abs(continuousDeg - best) <= toleranceDeg;
  }

  /**
   * Choose best equivalent target among {deg, deg+360, deg-360} that:
   *  - stays within [MIN_ANGLE_DEG, MAX_ANGLE_DEG]
   *  - minimizes travel from current continuousDeg
   */
  private double chooseBestEquivalentTargetDeg(double desiredDeg) {
    // Pull safety range from constants.
    double min = Constants.OperatorConstants.Turret.MIN_ANGLE_DEG;
    double max = Constants.OperatorConstants.Turret.MAX_ANGLE_DEG;

    // Clamp requested target into legal range first (keeps intent sane).
    desiredDeg = MathUtil.clamp(desiredDeg, min, max);

    // Consider equivalent angles one revolution away.
    double[] candidates = new double[] { desiredDeg, desiredDeg + 360.0, desiredDeg - 360.0 };

    // Track the best (closest) candidate within legal range.
    double best = Double.NaN;
    double bestDist = Double.POSITIVE_INFINITY;

    // Search candidates for the closest safe move.
    for (double c : candidates) {
      // Skip candidates that violate +/-340 hard limits.
      if (c < min || c > max) continue;

      // Distance is evaluated in continuous space (deg).
      double dist = Math.abs(c - continuousDeg);

      // Keep the closest.
      if (dist < bestDist) {
        bestDist = dist;
        best = c;
      }
    }

    return best;
  }

  /**
   * Convert a continuous target (deg relative to forward) into the wrapped absolute
   * rotation value [0,1).
   */
  private double wrappedRotFromContinuousDeg(double continuousDegTarget) {
    // Convert from forward-relative degrees back into encoder-frame absolute degrees.
    double absDeg = forwardDeg + (continuousDegTarget);

    // Wrap to [0,360) so we can convert to a wrapped rotation value.
    absDeg = wrapTo0To360(absDeg);

    // Convert degrees to rotations [0,1).
    return absDeg / 360.0;
  }

  private double motorRotFromTurretDeg(double turretDeg) {
    return turretDeg * Constants.OperatorConstants.Turret.MOTOR_ROT_PER_TURRET_DEG;
  }

  private double turretDegFromMotorRot(double motorRot) {
    return motorRot * Constants.OperatorConstants.Turret.TURRET_DEG_PER_MOTOR_ROT;
  }

  // ============================CALIBRATION METHODS===============================

  // Calibration sweep state
  private boolean isCalSweepEnabled = false;
  private double calSweepStartTimeSec = 0.0;

  // Live-tuned gains (calibration only)
  private double tunedKp = Constants.OperatorConstants.Turret.kP;
  private double tunedKd = Constants.OperatorConstants.Turret.kD;

  public void reseedIntegratedFromAbsoluteNow() {
    double absDeg = getAbsDegWrapped();
    lastAbsDegWrapped = absDeg;

    double deltaDeg = ANGLE_SIGN * wrapToPlusMinus180(absDeg - forwardDeg);
    deltaDeg = MathUtil.clamp(
        deltaDeg,
        -Constants.OperatorConstants.Turret.BOOT_MAX_ABS_DEG,
        Constants.OperatorConstants.Turret.BOOT_MAX_ABS_DEG);

    continuousDeg = deltaDeg;
    continuousDegUnclamped = deltaDeg;

    double motorRot = motorRotFromTurretDeg(continuousDeg);
    turret.setPosition(ANGLE_SIGN * motorRot);

    targetDeg = continuousDeg;

    SmartDashboard.putNumber("Turret/ReseedAbsDeg", absDeg);
    SmartDashboard.putNumber("Turret/ReseedContinuousDeg", continuousDeg);
  }

  public void captureAbsForwardTicksCandidate() {
    int ticks = getAbsoluteTicks();
    SmartDashboard.putNumber("Turret/AbsForwardTicksCandidate", ticks);
  }

  public void adjustKp(double delta) {
    tunedKp = Math.max(0.0, tunedKp + delta);
    applyTunedGains();
  }

  public void adjustKd(double delta) {
    tunedKd = Math.max(0.0, tunedKd + delta);
    applyTunedGains();
  }

  private void applyTunedGains() {
    Slot0Configs slot0 = new Slot0Configs()
        .withKP(tunedKp)
        .withKI(Constants.OperatorConstants.Turret.kI)
        .withKD(tunedKd)
        .withKS(Constants.OperatorConstants.Turret.kS)
        .withKV(Constants.OperatorConstants.Turret.kV)
        .withKA(Constants.OperatorConstants.Turret.kA);

    turret.getConfigurator().apply(slot0);

    SmartDashboard.putNumber("Turret/TunedKP", tunedKp);
    SmartDashboard.putNumber("Turret/TunedKD", tunedKd);
  }

  // =======================
  // Calibration API
  // =======================

  /**
   * Calibration: reseed TalonFX integrated position from absolute encoder NOW.
   */
  public void calibrationReseedIntegratedFromAbsoluteNow() {
    double absDeg = getAbsDegWrapped();
    lastAbsDegWrapped = absDeg;

    double deltaDeg = ANGLE_SIGN * wrapToPlusMinus180(absDeg - forwardDeg);
    deltaDeg = MathUtil.clamp(
        deltaDeg,
        -Constants.OperatorConstants.Turret.BOOT_MAX_ABS_DEG,
        Constants.OperatorConstants.Turret.BOOT_MAX_ABS_DEG);

    continuousDeg = deltaDeg;
    continuousDegUnclamped = deltaDeg;

    double motorRot = motorRotFromTurretDeg(continuousDeg);
    turret.setPosition(ANGLE_SIGN * motorRot);

    targetDeg = continuousDeg;

    SmartDashboard.putNumber("Turret/Cal/ReseedAbsDeg", absDeg);
    SmartDashboard.putNumber("Turret/Cal/ReseedContinuousDeg", continuousDeg);
  }

  /**
 * Calibration: capture absolute ticks at current turret pose to manually set
 * ABS_ZERO_TICKS.
 */
public void calibrationCaptureAbsZeroTicksCandidate() {
  int ticks = getAbsoluteTicks();
  SmartDashboard.putNumber("Turret/AbsTicks", ticks);
}

  /**
   * Calibration: go to a turret angle using Motion Magic (wrap disabled,
   * clamped).
   */
  public void calibrationGoToAngleDeg(double desiredDeg) {
    goToAngleDeg(desiredDeg); // uses your updated MotionMagicVoltage + clamp + wrap-off behavior
  }

  /** Calibration: start/stop a sweep test. */
  public void calibrationStartSweep() {
    // Using internal state so the command can just toggle enable/disable.
    isCalSweepEnabled = true;
    calSweepStartTimeSec = Timer.getFPGATimestamp();
    SmartDashboard.putBoolean("Turret/Cal/SweepEnabled", true);
  }

  public void calibrationStopSweep() {
    isCalSweepEnabled = false;
    SmartDashboard.putBoolean("Turret/Cal/SweepEnabled", false);
  }

  /** Calibration: adjust tuned Slot0 kP by delta (testing only). */
  public void calibrationAdjustKp(double delta) {
    tunedKp = Math.max(0.0, tunedKp + delta);
    applyTunedGains();
  }

  /** Calibration: adjust tuned Slot0 kD by delta (testing only). */
  public void calibrationAdjustKd(double delta) {
    tunedKd = Math.max(0.0, tunedKd + delta);
    applyTunedGains();
  }

  public void calibrationToggleSweep() {
  if (isCalSweepEnabled) {
    calibrationStopSweep();
  } else {
    calibrationStartSweep();
  }
}

  // ---------------- SysId commands ----------------

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    // If not enabled, return a no-op command so nothing moves.
    if (!isSysIdEnabled()) return new edu.wpi.first.wpilibj2.command.InstantCommand();

    // Otherwise run SysId's quasistatic ramp.
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    // If not enabled, return a no-op command so nothing moves.
    if (!isSysIdEnabled()) return new edu.wpi.first.wpilibj2.command.InstantCommand();

    // Otherwise run SysId's dynamic step.
    return sysIdRoutine.dynamic(direction);
  }

  // ---------------- SysId callbacks ----------------

  private void sysIdVoltageDrive(Voltage volts) {
    // Safety: if SysId isn't enabled, ensure turret is stopped.
    if (!isSysIdEnabled()) {
      stop();
      return;
    }

    // Apply the requested voltage directly.
    // Clamp to something physically plausible; Phoenix will also saturate to supply.
    double v = volts.in(Volts);
    double maxV = RobotController.getBatteryVoltage();
    v = MathUtil.clamp(v, -maxV, maxV);

    setVoltageVolts(v);
  }


  private void sysIdLog(SysIdRoutineLog log) {
    // Only log when SysId is enabled.
    if (!isSysIdEnabled()) return;

    log.motor("turret")
        .voltage(Volts.of(getAppliedVolts()))
        .angularPosition(Rotations.of(continuousDegUnclamped / 360.0))
        .angularVelocity(RotationsPerSecond.of(estVelDegPerSec / 360.0));
  }


  @Override
  public void periodic() {
    if (!EnabledSubsystems.turret) {
      return;
    }

    // Update continuous (multi-turn) angle state every loop.
    updateContinuousAngle();
    if(DebugTelemetrySubsystems.turret){
    // Telemetry block: expose key state for debugging and tuning.
      SmartDashboard.putNumber("Turret/AngleDeg", getAngleDeg());
      SmartDashboard.putNumber("Turret/VelDegPerSec", getVelocityDegPerSec());
      SmartDashboard.putNumber("Turret/AngleDeg_Unclamped", continuousDegUnclamped);
      SmartDashboard.putNumber("Turret/AngleDeg_Wrapped0to360", wrapTo0To360(continuousDegUnclamped));
      SmartDashboard.putNumber("Turret/AppliedVolts", getAppliedVolts());
      SmartDashboard.putNumber("Turret/AbsTicks", getAbsoluteTicks());
      SmartDashboard.putNumber("Turret/AbsDegWrapped", lastAbsDegWrapped);
      SmartDashboard.putNumber("Turret/TargetDeg", targetDeg);
      SmartDashboard.putBoolean("Turret/ContinuousWrapEnabled", continuousWrapEnabled);
      SmartDashboard.putNumber("Turret/ErrorDeg", targetDeg - continuousDeg);
      SmartDashboard.putBoolean("Turret/At1Deg", Math.abs(targetDeg - continuousDeg) <= 1.0);
      SmartDashboard.putBoolean("Turret/Cal/SweepEnabled", isCalSweepEnabled);
    } 

    // Calibration sweep: ping-pong between CAL_SWEEP_MIN_DEG and CAL_SWEEP_MAX_DEG.
    // if (isCalSweepEnabled) {
    //   double t = Timer.getFPGATimestamp() - calSweepStartTimeSec;

    //   double period = Constants.OperatorConstants.Turret.CAL_SWEEP_PERIOD_SEC;
    //   double minDeg = Constants.OperatorConstants.Turret.CAL_SWEEP_MIN_DEG;
    //   double maxDeg = Constants.OperatorConstants.Turret.CAL_SWEEP_MAX_DEG;

    //   // Triangle wave in [0, 1]
    //   double phase = (t % period) / period; // [0,1)
    //   double tri = phase < 0.5 ? (phase * 2.0) : (2.0 - phase * 2.0);

    //   double cmdDeg = minDeg + (maxDeg - minDeg) * tri;

    //   calibrationGoToAngleDeg(cmdDeg);
    //   SmartDashboard.putNumber("Turret/Cal/SweepCmdDeg", cmdDeg);
    // }

    if (Constants.DebugTelemetrySubsystems.turret && turretArm != null) {
      turretArm.setAngle(wrapTo0To360(continuousDegUnclamped));
    }

  }

  public double getSimCurrentDrawAmps() {
    if (!isSim || !EnabledSubsystems.turret) {
      return 0.0;
    }
    return turret.getSimState().getSupplyCurrent();
  }


  @Override
  public void simulationPeriodic() {
    // WPILib calls this automatically in simulation for each Subsystem.
    if (!EnabledSubsystems.turret) {
      return;
    }
    if (!isSim) {
      return;
    }

    final double dt = 0.02;

    // Phoenix simulated state
    var simState = turret.getSimState();

    // Give the controller a sane supply voltage first (same idea as KrakenMotorSubsystem)
    simState.setSupplyVoltage(12.0);

    // Feed CTRE’s motor voltage output into WPILib’s motor physics model
    turretSim.setInputVoltage(simState.getMotorVoltage());
    turretSim.update(dt);

    // Read physics model state
    final double posRot = turretSim.getAngularPositionRotations();
    final double velRps = turretSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

    // Push state back into Phoenix
    simState.setRawRotorPosition(posRot);
    simState.setRotorVelocity(velRps);

    // Keep your local copy if you still want it for any other debug/telemetry
    simPosRot = posRot;

    // Update CANcoder sim to match turret position
    var encoderSimState = throughboreCANcoder.getSimState();
    encoderSimState.setSupplyVoltage(12.0);
    double absRot = posRot % 1.0;
    if (absRot < 0) absRot += 1.0;
    encoderSimState.setRawPosition(absRot);

    encoderSimState.setVelocity(velRps);

    // Optional: approximate battery sag (identical concept to KrakenMotorSubsystem)
    simState.setSupplyVoltage(RoboRioSim.getVInVoltage());
  }



  // ---------------- Helpers ----------------

  private static double wrapTo0To360(double deg) {
    // Wrap any degrees into [0,360).
    double d = deg % 360.0;
    if (d < 0) d += 360.0;
    return d;
  }

  /** wrap to (-180, 180] */
  private static double wrapToPlusMinus180(double deg) {
    // Wrap degrees into (-180,180] to compute shortest signed difference.
    double d = ((deg + 180.0) % 360.0);
    if (d < 0) d += 360.0;
    return d - 180.0;
  }

}
