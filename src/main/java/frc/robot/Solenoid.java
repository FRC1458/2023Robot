package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Solenoid {
    DoubleSolenoid solenoid;
    PneumaticsModuleType type;

    public Solenoid(int module, int forwardChannel, int reverseChannel) {
        type = PneumaticsModuleType.REVPH;
        solenoid = new DoubleSolenoid(module, type, forwardChannel, reverseChannel);
    }

    public void forward() {
        solenoid.set(Value.kForward);
    }

    public void reverse() {
        solenoid.set(Value.kReverse);
    }

    public void off() {
        solenoid.set(Value.kOff);
    }
}