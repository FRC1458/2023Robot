package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

public class Lidar {
    DutyCycle lasershark;


    public Lidar() {

        lasershark = new DutyCycle(new DigitalInput(RobotConstants.lidarPort));
    }

    public double getDistanceCentimeters() {
        return lasershark.getOutput() * 400;
    }
}
