package frc.robot.wrappers;

import frc.robot.*;

import edu.wpi.first.wpilibj.DriverStation;

public class NavXWrapper extends Wrapper{
    private NavX navx;

    public NavXWrapper(){
        try{
            navx = new NavX();
        }
        catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
        }
    }
    public void operatorControl() {
        if (isInitialized) navx.operatorControl();
    }
}