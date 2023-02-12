package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class BaseLimelight {
    
    int pipeline;
    NetworkTable table;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry tledMode;

    BaseLimelight(int pipeline) {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        tledMode = table.getEntry("ledMode");
    }

    void set_pipeline(int new_pipeline) {pipeline = new_pipeline;}
    float get_rotation() {return table.getEntry("tx").getFloat(0.000f);}

    void display() {
        float x = tx.getFloat(0.000f);
        float y = ty.getFloat(0.000f);
        float ledMode = ty.getFloat(0.000f);
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightLedMode", ledMode);
    }
    
}