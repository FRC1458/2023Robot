package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NavX{
  AHRS ahrs;
  boolean error;

  public NavX() {
    ahrs = new AHRS(SPI.Port.kMXP);
  }

  public void autonomous() {
  }
  public void operatorControl() {

  }
  public void test() {
  }
}