package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Solenoid {
    DoubleSolenoid solenoid;
    int forwardChannel;
    int reverseChannel;
    PneumaticsModuleType type;
    int id;

    public Solenoid(int id, int forwardChannel, int reverseChannel) {
        this.forwardChannel = forwardChannel;
        this.reverseChannel = reverseChannel;
        type = PneumaticsModuleType.REVPH;
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