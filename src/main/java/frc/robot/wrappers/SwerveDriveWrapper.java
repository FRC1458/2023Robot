package frc.robot.wrappers;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.swervedrive.SwerveDrive;
import com.kauailabs.navx.frc.AHRS;



import frc.robot.*;

public class SwerveDriveWrapper extends Wrapper{
   
    public SwerveDrive swervedrive;

    public SwerveDriveWrapper (AHRS navx) {
        try{
            swervedrive = new SwerveDrive(navx) ;
            isInitialized = true;
        }
        catch (RuntimeException ex ){
            DriverStation.reportError("Error instantiating swervedrive:  " + ex.getMessage(), true);
        }

    }

    public void drive(double x, double y, double r, boolean fieldOriented) {
        if (isInitialized) swervedrive.drive(x, y, r, fieldOriented);
    }
    
    public void setEncoders() {
        if (isInitialized) swervedrive.setEncoders();
    }

}
