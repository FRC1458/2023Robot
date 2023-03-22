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

    public Limelight(int pipeline) {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        led = table.getEntry("ledMode");
        botPos = table.getEntry("botpose");

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
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

    }

    public double getXOffset() {
        return botPos.getDoubleArray(defaultVals)[0];
    }

    public double getYOffset() {
       return botPos.getDoubleArray(defaultVals)[1];
    }

    public double getYaw() {
        return botPos.getDoubleArray(defaultVals)[5];
    }
    public void setPipeline(int newPipeline) {
        pipeline = newPipeline;
    }

    public double getRotation() {
        return table.getEntry("tx").getDouble(0);
    }

    public void display() {
        double x = tx.getDouble(0);
        double y = ty.getDouble(0);
        double ledMode = ty.getDouble(0);
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightLedMode", ledMode);
    }

    public double get_rotation() {
        return table.getEntry("tx").getDouble(0);
    }
}