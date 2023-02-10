package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
public class ArmNavX{
  AHRS ahrs;
  boolean error;

  public ArmNavX() {
    ahrs = new AHRS(I2C.Port.kOnboard);
  }
  public void autonomous() {
  }

  public double getPitch(){
    return ahrs.getPitch();
  }

  public void test() {
  }
}