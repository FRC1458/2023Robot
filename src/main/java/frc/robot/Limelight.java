package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//import com.kauailabs.navx.frc.AHRS;



public class Limelight {
    int pipeline;
    NetworkTable table;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry led;
    float x;
    float y;
    float area;

    public Limelight(){
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        led = table.getEntry("ledMode");
        led.setNumber(3.000);
        x = tx.getFloat(69);
        y = ty.getFloat(69);
        area = ta.getFloat(69);
    }

    public void readPeriodic() {
        //read values periodically
        x = tx.getFloat(69);
        y = ty.getFloat(69);
        area = ta.getFloat(69);
        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
    }

    public double getXOffset() {
        return (x);
    }
    public double getYOffset() {
        return (y);
    }
}