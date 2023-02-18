package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;

public class Solenoid {
    DoubleSolenoid solenoid;
    int forwardChannel;
    int reverseChannel;
    PneumaticsModuleType type;
    int id;

    public Solenoid(int id) {
        solenoid = new DoubleSolenoid(type, forwardChannel, reverseChannel);
        this.id = id;
    }

    public void forward() {
        solenoid.set(Value.kForward);
    }

    public void backward() {
        solenoid.set(Value.kReverse);
    }

    public void off() {
        solenoid.set(Value.kOff);
    }

    public int getForwardChannel() {
        return forwardChannel;
    }

    public int getReverseChannel() {
        return reverseChannel;
    }
}