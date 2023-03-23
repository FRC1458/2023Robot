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
    NetworkTableEntry botPos;
    double x;
    double y;
    double area;
    final double[] defaultVals = new double[6];
    double[] positionValues;

    public Limelight(int pipeline) {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        led = table.getEntry("ledMode");
        botPos = table.getEntry("botpose_targetspace");

        led.setNumber(3.000);
        x = tx.getDouble(0);
        y = ty.getDouble(0);
        area = ta.getDouble(0);
        this.pipeline = pipeline;
    }

    public void readPeriodic() {
        //read values periodically
        x = tx.getDouble(0);
        y = ty.getDouble(0);
        area = ta.getDouble(0);
        //post to smart dashboard periodically
        //SmartDashboard.putNumber("LimelightX", x);
        //SmartDashboard.putNumber("LimelightY", y);
        //SmartDashboard.putNumber("LimelightArea", area);
        positionValues = botPos.getDoubleArray(defaultVals);
        
    }

    public double getXOffset() {
        return positionValues[0];
    }

    public double getYOffset() {
       return positionValues[1];
    }

    public double getRotation() {
        return positionValues[4];
    }
    public void setPipeline(int newPipeline) {
        pipeline = newPipeline;
    }
}