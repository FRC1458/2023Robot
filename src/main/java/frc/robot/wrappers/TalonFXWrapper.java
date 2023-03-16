package frc.robot.wrappers;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DriverStation;


public class TalonFXWrapper extends Wrapper {
    
    public TalonFX talon;

    public TalonFXWrapper(int port, boolean brakeMode) {
        try{
            talon = new TalonFX(port);
            isInitialized = true;
            if (brakeMode) {
                talon.setNeutralMode(NeutralMode.Brake);
            }
        }
        catch (RuntimeException ex ) {
            DriverStation.reportError("Error Initiating TalonFX:  " + ex.getMessage(), true);
        }
    }
    public void set(double speed) {
        if (isInitialized) talon.set(ControlMode.PercentOutput, speed);
    }
    public void setNeutralMode(NeutralMode mode) {
        if (isInitialized) talon.setNeutralMode(mode);
    }

    public double getEncoder() {
        return talon.getSelectedSensorPosition();
    }
}