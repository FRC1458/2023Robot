package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program providing a real-time display of navX-MXP values.
 *
 * In the operatorControl() method, all data from the navX-MXP is retrieved
 * and output to the SmartDashboard.
 *
 * The output data values include:
 *
 * - Yaw, Pitch and Roll angles
 * - Compass Heading and 9-Axis Fused Heading (requires Magnetometer calibration)
 * - Linear Acceleration Data
 * - Motion Indicators
 * - Estimated Velocity and Displacement
 * - Quaternion Data
 * - Raw Gyro, Accelerometer and Magnetometer Data
 *
 * As well, Board Information is also retrieved; this can be useful for debugging
 * connectivity issues after initial installation of the navX-MXP sensor.
 *
 */
public class ArmNavX{
  AHRS ahrs;
  boolean error;

  public ArmNavX() {
    ahrs = new AHRS(I2C.Port.kOnboard);
  }

  /**
   * Runs during autonomous mode
   */
  public void autonomous() {
  }

  public double getPitch(){
    return ahrs.getPitch();
  }
  /**
   * Runs during test mode
   */
  public void test() {
  }
}