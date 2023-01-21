package frc.robot.wrappers;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class CompressorWrapper extends Wrapper{
    Compressor compressor;

    public CompressorWrapper() {
        try{
            //compressor = new Compressor(1, PneumaticsModuleType.REVPH); // module id is 1, default for REV pneumatic hub
            //isInitialized = true;
        }
        catch (RuntimeException ex ) {
            DriverStation.reportError("Error Initiating Compressor:  " + ex.getMessage(), true);
        }
    }

    public void enableDigital() {
        if (isInitialized) compressor.enableDigital();
    }
}
