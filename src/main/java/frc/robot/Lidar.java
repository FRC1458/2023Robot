package frc.robot;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DigitalInput;

import com.cuforge.libcu.Lasershark;

public class Lidar {
    DutyCycle lasershark;


    public Lidar() {
        lasershark = new DutyCycle(new DigitalInput(0));//change to robot constants

    }

    public double getDistanceCentimeters() {
        return lasershark.getOutput();
    }
}
