package frc.robot.wrappers;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DriverStation;

public class CANSparkMaxWrapper extends Wrapper{
    public CANSparkMax spark;

    public CANSparkMaxWrapper (int num) {
        try{
            spark = new CANSparkMax(num, MotorType.kBrushless);
            isInitialized = true;
        }
        catch (RuntimeException ex ) {
            DriverStation.reportError("Error Initiating CAN Spark Max:  " + ex.getMessage(), true);
        }
    }

    public void getAppliedOutput() {
        if (isInitialized) spark.getAppliedOutput();
    }

    public void getOutputCurrent() {
        if (isInitialized) spark.getOutputCurrent();
    }

    public void getPIDController() {
        if (isInitialized) spark.getPIDController();
    }

    public void getEncoder() {
        if (isInitialized) spark.getEncoder();
    }

    public void set(double speed) {
        if (isInitialized) spark.set(speed);
    }
}