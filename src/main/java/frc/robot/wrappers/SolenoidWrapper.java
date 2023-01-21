package frc.robot.wrappers;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;


public class SolenoidWrapper extends Wrapper {
    DoubleSolenoid solenoid;

    public SolenoidWrapper(int forwardID, int reverseID) {
        try{
            //solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, forwardID, reverseID);
            //isInitialized = true;
        }
        catch (RuntimeException ex ) {
            DriverStation.reportError("Error Initiating Solenoid:  " + ex.getMessage(), true);
        }
    }

    public void set(boolean mode) {
        if (isInitialized) {
            if (mode) solenoid.set(Value.kForward);

            else solenoid.set(Value.kReverse);
        }
    }

    public boolean get() {
        if (isInitialized) return solenoid.get() == Value.kForward;
        return false;
    }
}