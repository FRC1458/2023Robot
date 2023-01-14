package frc.robot.wrappers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.swing.text.html.HTMLDocument.HTMLReader.IsindexAction;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class LimelightWrapper extends Wrapper {

    NetworkTable limelight;
    
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;

    public LimelightWrapper() {
        try{
            limelight = NetworkTableInstance.getDefault().getTable("limelight");
            tx = limelight.getEntry("tx");
            ty = limelight.getEntry("ty");
            ta = limelight.getEntry("ta");

            isInitialized = true;
        }
        catch (RuntimeException ex) {
            DriverStation.reportError("Error Initiating Limelight:  " + ex.getMessage(), true);
        }
    }

    public double getx() {
        if (isInitialized) return tx.getDouble(0.0);
        return 0;
    }
    
    public double gety() {
        if (isInitialized) return ty.getDouble(0.0);
        return 0;
    }

    public double getArea() {
        if (isInitialized) return ta.getDouble(0.0);
        return 0;
    }
    

}

