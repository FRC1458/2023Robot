package frc.robot.wrappers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
//import jdk.internal.jline.console.completer.Completer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;

public class XboxControllerWrapper extends Wrapper{
    
    XboxController xbox;
    double deadZone;

    public XboxControllerWrapper(int port) {
        try{
            xbox = new XboxController(port);
            deadZone = 0.15;
            isInitialized = true;
        }
        catch (RuntimeException ex ) {
            DriverStation.reportError("Error Initiating Xbox Controller:  " + ex.getMessage(), true);
        }
    }

    public double getRightX() {
        if (isInitialized) {
            if (Math.abs(xbox.getRightX()) < deadZone) {
                return 0.0;
            }
            return xbox.getRightX();
        }
        return 0;
    }
    
    public double getRightY() {
        if (isInitialized) {
            if (Math.abs(xbox.getRightY()) < deadZone) {
                return 0.0;
            }
            return xbox.getRightY();
        }
        return 0;
    }
    
    public double getLeftX() {
        if (isInitialized) {
            if (Math.abs(xbox.getLeftX()) < deadZone) {
                return 0.0;
            }
            return xbox.getLeftX();
        }
        return 0;
    }
    
    public double getLeftY() {
        if (isInitialized) {
            if (Math.abs(xbox.getLeftY()) < deadZone) {
                return 0.0;
            }
            return xbox.getLeftY();
        }
        return 0;
    }

    public boolean getAButton() {
        if (isInitialized) return xbox.getAButton();
        return false;
    }
    
    public boolean getXButton() {
        if (isInitialized) return xbox.getXButton();
        return false;
    }
    
    public boolean getBButton() {
        if (isInitialized) return xbox.getBButton();
        return false;
    }
    
    public boolean getYButton() {
        if (isInitialized) return xbox.getYButton();
        return false;
    }

    public double getRightTriggerAxis() {
        if (isInitialized) return xbox.getRightTriggerAxis();
        return 0;
    }
    
    public double getLeftTriggerAxis() {
        if (isInitialized) return xbox.getLeftTriggerAxis();
        return 0;
    }
    public boolean getRightBumper() {
        if (isInitialized) return xbox.getRightBumper();
        return false;
    }
    public boolean getStartButton() {
        if (isInitialized) return xbox.getStartButton();
        return false;
    }
    public boolean getLeftBumper() {
        if (isInitialized) return xbox.getLeftBumper();
        return false;
    }
    public int getPOV() {
        if (isInitialized) return xbox.getPOV();
        return -1;
    }
    public boolean getRawButton(int button) {
        if (isInitialized) return xbox.getRawButton(button);
        return false;
    }

    public boolean getBackButton() {
        if (isInitialized) return xbox.getBackButton();
        return false;
    }
    
    public boolean getBackButtonPressed() {
        if (isInitialized) return xbox.getBackButtonPressed();
        return false;
    }

    public boolean getBackButtonReleased() {
        if (isInitialized) return xbox.getBackButtonReleased();
        return false;
    }

    public void setRumble(double val) {
        setRumble(val);
    }
}
