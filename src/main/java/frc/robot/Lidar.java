package frc.robot;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DigitalInput;

public class Lidar {
    DutyCycle lasershark;


    public Lidar() {
        lasershark = new DutyCycle(new DigitalInput(RobotConstants.lidarPort));
    }

    public double getDistanceCentimeters() {
        return lasershark.getOutput();
    }
}
