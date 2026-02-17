package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
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
import frc.robot.Constants.OperatorConstants.Hopper;

public class HopperSubsystem extends SubsystemBase {

  // =========================
  // 3-stage hopper definition
  // =========================
  public enum HopperStage {
    RETRACTED,
    MID,
    EXTENDED
  }

  // TODO: Replace these with real measured values once hardware exists.
  // These are in *motor rotations* (TalonFX integrated position units = rotations).
  private static final double STAGE_RETRACTED_POS_ROT = 0.0;
  private static final double STAGE_MID_POS_ROT = 15.0;
  private static final double STAGE_EXTENDED_POS_ROT = 30.0;

  // How close is "good enough" to stop the motor?
  private static final double STAGE_TOLERANCE_ROT = 0.25;

  // When we are close to target, slow down to reduce overshoot.
  private static final double SLOW_ZONE_ROT = 2.0;

  // Your requested discrete duty levels (skeleton).
  private static final double DUTY_FAST = 1.0;
  private static final double DUTY_SLOW = 0.5;

  // Mechanism2d: ligament length range (purely visual units, not meters).
  // Instant visual lengths for each stage (visual-only units)
  private static final double MECH_RETRACTED_LEN = 0.25;
  private static final double MECH_MID_LEN = 0.625;
  private static final double MECH_EXTENDED_LEN = 1.00;


  // =========================
  // Hardware
  // =========================
  private TalonFX hopperMotor;

  private final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);
  private final VoltageOut voltageOut = new VoltageOut(0.0);

  private StatusSignal<Angle> positionSig;
  private StatusSignal<AngularVelocity> velocitySig;
  private StatusSignal<Voltage> motorVoltageSig;

  // Cached telemetry values (rotations, rotations/sec)
  private double posRot = 0.0;
  private double velRps = 0.0;

  private enum ControlMode {
  IDLE,
  STAGE
  }

  private ControlMode controlMode = ControlMode.IDLE;

  // =========================
  // Stage/state
  // =========================
  private HopperStage desiredStage = HopperStage.RETRACTED;

  // =========================
  // Simulation
  // =========================
  private final boolean isSim = RobotBase.isSimulation();

  private final DCMotorSim hopperSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1),
              Constants.OperatorConstants.Hopper.SIM_J_KGM2,
              Constants.OperatorConstants.Hopper.SIM_GEAR_RATIO),
          DCMotor.getKrakenX60(1));

  // =========================
  // Mechanism2d visualization
  // =========================
  private Mechanism2d hopperMech;
  private MechanismLigament2d hopperLig;

  // =========================
  // SysId (voltage based)
  // =========================
  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              (Voltage volts) -> sysIdVoltageDrive(volts),
              (SysIdRoutineLog log) -> sysIdLog(log),
              this));

  public HopperSubsystem() {
    if (!EnabledSubsystems.hopper) {
      return;
    }

    hopperMotor = new TalonFX(Hopper.MOTOR_ID, Hopper.CANBUS_NAME);

    configureMotors();

    positionSig = hopperMotor.getPosition();
    velocitySig = hopperMotor.getVelocity();
    motorVoltageSig = hopperMotor.getMotorVoltage();

    positionSig.setUpdateFrequency(100.0);
    velocitySig.setUpdateFrequency(100.0);
    motorVoltageSig.setUpdateFrequency(50.0);

    hopperMotor.optimizeBusUtilization();

    // Mechanism2d (only if telemetry enabled, to avoid clutter/perf work)
    if (DebugTelemetrySubsystems.hopper) {
      hopperMech = new Mechanism2d(2.0, 2.0);
      MechanismRoot2d root = hopperMech.getRoot("HopperRoot", 1.0, 1.0);

      // Ligament points to the right; we change length to simulate extension.
      hopperLig = new MechanismLigament2d("HopperExtension", 0.25, 0.0);
      root.append(hopperLig);

      SmartDashboard.putData("Hopper/Mechanism", hopperMech);
    }
  }

  private void configureMotors() {
    MotorOutputConfigs out =
        new MotorOutputConfigs()
            .withInverted(Constants.OperatorConstants.Hopper.MOTOR_INVERTED)
            .withNeutralMode(Constants.OperatorConstants.Hopper.NEUTRAL_COAST);

    CurrentLimitsConfigs limits =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(Constants.OperatorConstants.Hopper.ENABLE_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.OperatorConstants.Hopper.SUPPLY_CURRENT_LIMIT_A)
            .withSupplyCurrentLowerTime(Constants.OperatorConstants.Hopper.SUPPLY_CURRENT_LOWER_TIME_S)
            .withSupplyCurrentLowerLimit(Constants.OperatorConstants.Hopper.SUPPLY_CURRENT_LOWER_LIMIT_A)
            .withStatorCurrentLimitEnable(Constants.OperatorConstants.Hopper.ENABLE_CURRENT_LIMIT)
            .withStatorCurrentLimit(Constants.OperatorConstants.Hopper.STATOR_CURRENT_LIMIT_A);

    Slot0Configs slot0 =
        new Slot0Configs()
            .withKP(Constants.OperatorConstants.Hopper.kP)
            .withKI(Constants.OperatorConstants.Hopper.kI)
            .withKD(Constants.OperatorConstants.Hopper.kD)
            .withKS(Constants.OperatorConstants.Hopper.kS)
            .withKV(Constants.OperatorConstants.Hopper.kV)
            .withKA(Constants.OperatorConstants.Hopper.kA);

    TalonFXConfiguration cfg =
        new TalonFXConfiguration().withMotorOutput(out).withCurrentLimits(limits).withSlot0(slot0);

    hopperMotor.getConfigurator().apply(cfg);
  }

  // =========================
  // Public API: stages
  // =========================
  public void requestStage(HopperStage stage) {
    desiredStage = stage;
  }

  public HopperStage getDesiredStage() {
    return desiredStage;
  }

  /**
   * Interpret a "stage duty" command as a stage request:
   *  -1.0  -> RETRACTED
   *  +0.5  -> MID
   *  +1.0  -> EXTENDED
   *
   * This does NOT directly drive the motor. It sets desiredStage,
   * and the periodic loop drives toward the target with setPositionDutyCycle().
  */
  /**
   * User-requested simple behavior:
   * duty =  1.0  -> EXTENDED
   * duty = -1.0  -> RETRACTED
   * duty =  0.5  -> MID
   *
   * In sim: Mechanism2d ligament jumps immediately.
   * In real: we request the stage and the periodic loop drives toward target encoder.
  */
  public void setStageDuty(double duty) {
    HopperStage stage;

    // Accept small float variation
    if (Math.abs(duty - 1.0) < 1e-3) {
      stage = HopperStage.EXTENDED;
    } else if (Math.abs(duty + 1.0) < 1e-3) {
      stage = HopperStage.RETRACTED;
    } else if (Math.abs(duty - 0.5) < 1e-3) {
      stage = HopperStage.MID;
    } else {
      // Anything else = stop
      stop();
      return;
    }

    desiredStage = stage;
    controlMode = ControlMode.STAGE;

    // Instant visual jump in simulation (and also fine on real dashboards)
    setLigamentForStage(stage);

    // Optional: if you want the encoder value to jump instantly in sim too:
    if (isSim) {
      double targetRot = stageTargetRot(stage);
      var simState = hopperMotor.getSimState();
      simState.setRawRotorPosition(targetRot);
      simState.setRotorVelocity(0.0);
    }
  }



  // Convenience wrappers
  public void requestRetracted() {
    requestStage(HopperStage.RETRACTED);
  }

  public void requestMid() {
    requestStage(HopperStage.MID);
  }

  public void requestExtended() {
    requestStage(HopperStage.EXTENDED);
  }

  // =========================
  // Motor control helpers
  // =========================
  public void stop() {
    if (!EnabledSubsystems.hopper) return;
    controlMode = ControlMode.IDLE;
    hopperMotor.setControl(dutyCycle.withOutput(0.0));
  }


  /** Open-loop duty cycle (clamped + hard stops). */
  public void setDutyCycle(double percent) {
    if (!EnabledSubsystems.hopper) return;

    double p =
        MathUtil.clamp(
            percent,
            -Constants.OperatorConstants.Hopper.MAX_DUTY_CYCLE,
            Constants.OperatorConstants.Hopper.MAX_DUTY_CYCLE);

    // Hard travel limits in motor-rotations (placeholders for now).
    // These are your "encoder tick" clamps.
    final double minRot = STAGE_RETRACTED_POS_ROT;
    final double maxRot = STAGE_EXTENDED_POS_ROT;

    // If we're at/over a limit, do not allow motion further into that limit.
    if (posRot <= minRot && p < 0.0) {
      p = 0.0;
    } else if (posRot >= maxRot && p > 0.0) {
      p = 0.0;
    }

    hopperMotor.setControl(dutyCycle.withOutput(p));
  }

  private void enforceHardStops() {
    final double minRot = STAGE_RETRACTED_POS_ROT;
    final double maxRot = STAGE_EXTENDED_POS_ROT;

    if (posRot <= minRot) {
      if (controlMode == ControlMode.STAGE && desiredStage == HopperStage.RETRACTED) {
        controlMode = ControlMode.IDLE;
        stop();
      }
    } else if (posRot >= maxRot) {
      if (controlMode == ControlMode.STAGE && desiredStage == HopperStage.EXTENDED) {
        controlMode = ControlMode.IDLE;
        stop();
      }
    }
  }




  /** Direct voltage control (used by SysId). */
  public void setVoltageVolts(double volts) {
    if (!EnabledSubsystems.hopper) return;
    double maxV = RobotController.getBatteryVoltage();
    double v = MathUtil.clamp(volts, -maxV, maxV);
    hopperMotor.setControl(voltageOut.withOutput(v));
  }

  /**
   * Stage-driving helper you requested:
   * Drive toward a target position using discrete duty cycle values.
   *
   * Behavior:
   * - far from target: +/- 1.0
   * - near target (slow zone): +/- 0.5
   * - within tolerance: 0.0 (stop)
   *
   * This works in real + sim because it uses the same encoder position signal.
   */
  public void setPositionDutyCycle(double targetPosRot) {
    double error = targetPosRot - posRot;

    if (Math.abs(error) <= STAGE_TOLERANCE_ROT) {
      setDutyCycle(0.0);
      return;
    }

    double dutyMag = (Math.abs(error) <= SLOW_ZONE_ROT) ? DUTY_SLOW : DUTY_FAST;
    double duty = Math.copySign(dutyMag, error);

    setDutyCycle(duty);
  }

  private double stageTargetRot(HopperStage stage) {
    return switch (stage) {
      case RETRACTED -> STAGE_RETRACTED_POS_ROT;
      case MID -> STAGE_MID_POS_ROT;
      case EXTENDED -> STAGE_EXTENDED_POS_ROT;
    };
  }

  // =========================
  // SysId
  // =========================
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  private boolean isSysIdEnabled() {
    // Keep your existing convention if you have a flag; default to true if enabled subsystem.
    return EnabledSubsystems.hopper;
  }

  private void sysIdVoltageDrive(Voltage volts) {
    if (!isSysIdEnabled()) {
      stop();
      return;
    }
    setVoltageVolts(volts.in(Volts));
  }

  private void sysIdLog(SysIdRoutineLog log) {
    if (!isSysIdEnabled()) return;

    log.motor("hopper")
        .voltage(Volts.of(motorVoltageSig.getValueAsDouble()))
        .angularPosition(Rotations.of(posRot))
        .angularVelocity(RotationsPerSecond.of(velRps));
  }

  // =========================
  // Simulation integration
  // =========================
  public double getSimCurrentDrawAmps() {
    if (!isSim || !EnabledSubsystems.hopper) {
      return 0.0;
    }
    return hopperSim.getCurrentDrawAmps();
  }

  @Override
  public void simulationPeriodic() {
    if (!isSim) return;
    if (!EnabledSubsystems.hopper) return;

    final double dt = 0.02;

    var simState = hopperMotor.getSimState();
    simState.setSupplyVoltage(RoboRioSim.getVInVoltage());

    // Kraken/Turret style: Phoenix computes motor volts; WPILib sim consumes it.
    hopperSim.setInputVoltage(simState.getMotorVoltage());
    hopperSim.update(dt);

    final double simPosRot = hopperSim.getAngularPositionRotations();
    final double simVelRps = hopperSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

    simState.setRawRotorPosition(simPosRot);
    simState.setRotorVelocity(simVelRps);

  }

  private void setLigamentForStage(HopperStage stage) {
    if (hopperLig == null) return;

    switch (stage) {
      case RETRACTED -> hopperLig.setLength(MECH_RETRACTED_LEN);
      case MID -> hopperLig.setLength(MECH_MID_LEN);
      case EXTENDED -> hopperLig.setLength(MECH_EXTENDED_LEN);
    }
  }


  // =========================
  // Main periodic
  // =========================
  @Override
  public void periodic() {
    if (!EnabledSubsystems.hopper) {
      return; 
    }

    BaseStatusSignal.refreshAll(positionSig, velocitySig, motorVoltageSig);

    posRot = positionSig.getValueAsDouble();
    velRps = velocitySig.getValueAsDouble();

    enforceHardStops();
    // Drive the staged behavior in both real + sim.
    // If you later want explicit commands to control motion instead, we can gate this behind a flag.
    if (controlMode == ControlMode.STAGE) {
      setPositionDutyCycle(stageTargetRot(desiredStage));
    } else {
      // IDLE: do nothing (motor stays at whatever last control set, or stopped)
    }

    // Update Mechanism2d length based on where we are between retracted and extended.
    

    if (DebugTelemetrySubsystems.hopper) {
      SmartDashboard.putString("Hopper/StageDesired", desiredStage.name());
      SmartDashboard.putNumber("Hopper/PosRot", posRot);
      SmartDashboard.putNumber("Hopper/VelRps", velRps);
      SmartDashboard.putNumber("Hopper/MotorVoltage", motorVoltageSig.getValueAsDouble());
    }
  }
}
