package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;

import java.util.ArrayList;

// WARNING: UNTESTED CODE
public class ArmNavX{

  AHRS ahrs;
    int sampling = 4;
  ArrayList<Float> buffer;

  public ArmNavX(int sampling) {
    ahrs = new AHRS(I2C.Port.kOnboard);
    this.sampling = sampling;
  }

  public double getSmoothPitch(){
    double ret = 0;
    buffer.add(ahrs.getPitch());
    if (buffer.size() > sampling) {buffer.remove(0);}//sampling should be an int
    for (double i : buffer) {ret += i;}
    ret /= sampling;
    return ret;
  }
  public double getPitch() {
    return ahrs.getPitch() * 3.0;
  }

  //public void autonomous() {}
  //public void test() {}

}