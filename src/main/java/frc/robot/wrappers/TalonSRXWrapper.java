package frc.robot.wrappers;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.wrappers.Wrapper;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class TalonSRXWrapper extends Wrapper {
    WPI_TalonSRX talon;

    public TalonSRXWrapper (int port) {
        try{
            talon = new WPI_TalonSRX(port);
            isInitialized = true;
        }
        catch (RuntimeException ex ) {
            DriverStation.reportError("Error Initiating TalonSRX:  " + ex.getMessage(), true);
        }
    }
    public void set(double speed) {
        if (isInitialized) talon.set(ControlMode.PercentOutput, speed);
    }
}

