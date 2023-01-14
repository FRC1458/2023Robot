package frc.robot.wrappers;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Ultrasonic;

public class UltrasoundWrapper extends Wrapper{

    Ultrasonic ultrasound;

    public UltrasoundWrapper(int output, int input) {
        try {
            ultrasound = new Ultrasonic(output, input);
            isInitialized = true;
        }
        catch (RuntimeException ex ) {
            DriverStation.reportError("Error Initiating Ultrasound:  " + ex.getMessage(), true);
        }
        if (isInitialized) Ultrasonic.setAutomaticMode(true);
    }

    public double getRangeInches() {
        if(isInitialized) return ultrasound.getRangeInches();
        return 0;
    }

    public double getRangeMM() {
        if(isInitialized) return ultrasound.getRangeMM();
        return 0;
    }
    
}
