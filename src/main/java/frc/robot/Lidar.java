package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

public class Lidar {
    DutyCycle lasershark;


    public Lidar(int port) {

        lasershark = new DutyCycle(new DigitalInput(port));
    }

    public double getDistanceCentimeters() {
        return lasershark.getOutput() * 400;
    }

    public double getDistanceMeters() {return lasershark.getOutput() * 4;}
}
