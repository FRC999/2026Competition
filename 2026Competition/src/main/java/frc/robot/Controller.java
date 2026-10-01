package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OperatorConstants.OIContants.ControllerDevice;
import frc.robot.Constants.OperatorConstants.OIContants.ControllerDeviceType;
import frc.robot.lib.DriverInput;

/** Hardware mapping plus a single deadband/response curve per axis. */
public class Controller extends Joystick {
  private final ControllerDevice config;
  public Controller(ControllerDevice config) { super(config.portNumber()); this.config = config; }
  private boolean xbox() { return config.controllerDeviceType() == ControllerDeviceType.XBOX; }
  public double getLeftStickX() {
    return DriverInput.shape(xbox() ? getRawAxis(0) : getX(), config.deadbandX(), config.cubeControllerLeftStick());
  }
  public double getLeftStickY() {
    return DriverInput.shape(xbox() ? getRawAxis(1) : getY(), config.deadbandY(), config.cubeControllerLeftStick());
  }
  public double getLeftStickOmega() {
    return DriverInput.shape(xbox() ? getRawAxis(3)-getRawAxis(2) : getTwist(),
        config.deadbandOmega(), config.cubeControllerRightStick());
  }
  public double getRightStickX() {
    return DriverInput.shape(xbox() ? getRawAxis(4) : getX(), config.deadbandOmega(), config.cubeControllerRightStick());
  }
  public double getRightStickY() {
    return DriverInput.shape(xbox() ? getRawAxis(5) : 0, config.deadbandY(), config.cubeControllerRightStick());
  }
}
