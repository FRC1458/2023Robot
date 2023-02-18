package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.wrappers.TalonFXWrapper;

public class Arm {
    private Solenoid armSolenoid;
    private Solenoid clawSolenoid;
    private TalonFXWrapper armMotor;
    private ArmNavX armnavx;
    private int direction = 1;

    private enum armStates {
        TOP,
        MIDDLE,
        BOTTOM,
        START
    }

    public Arm(Solenoid armSolenoid, Solenoid clawSolenoid, ArmNavX armnavx) {
        this.armSolenoid = armSolenoid;
        this.clawSolenoid = clawSolenoid;
        this.armnavx = armnavx;
        armMotor = new TalonFXWrapper(43);
    }

    public void runArm() {

    }

    public void bricked() {
        if (armnavx.getPitch() > 62) {
            direction = -1;
        }
        else if (armnavx.getPitch() < 58) {
            direction = 1;
        } 
        else {
            direction = 0;
            armSolenoid.forward();
        }


    }

    public void erect() {
        if (armnavx.getPitch() > 30) {
            direction = -1;
        } else {
            direction = 1;
        }

        armMotor.set(0);
        armSolenoid.forward();
    }

    public void flaccid() {
        if (armnavx.getPitch() > -20) {
            direction = -1;
        } else {
            direction = 1;
        }

        armMotor.set(0);
        armSolenoid.forward();
    }
}

// Starting: -60
// Bottom: -20
// Middle: 20
// Top: 60