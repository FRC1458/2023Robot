package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.wrappers.TalonFXWrapper;

public class Arm {
    private Solenoid armSolenoid;
    private Solenoid clawSolenoid;
    private final TalonFXWrapper armMotor;
    private ArmNavX armnavx;
    private int direction = 1;
    private boolean clawOpened;

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
        clawOpened = false;

    }

    public void runArm() {

    }

    public void bricked() {
        if (armMotor.getEncoder() > 62) {
            direction = -1;
        }
        else if (armMotor.getEncoder() < 58) {
            direction = 1;
        } 
        else {
            direction = 0;
            armSolenoid.forward();
        }


    }

    public void erect() {
        if (armMotor.getEncoder() > 30) {
            direction = -1;
        } else {
            direction = 1;
        }

        armMotor.set(0);
        armSolenoid.forward();
    }

    public void flaccid() {
        if (armMotor.getEncoder() > -20) {
            direction = -1;
        } else {
            direction = 1;
        }

        armMotor.set(0);
        armSolenoid.forward();
    }

    public void setArm(int armState) {
        if (armState == 1) {
            flaccid();
        }
        else if (armState == 2) {
            erect();
        }
        else if (armState == 3) {
            bricked();
        }
        armState = 0;
    }

    public void openClaw() {
        clawSolenoid.forward();
    }

    public void closeClaw() {
        clawSolenoid.reverse();
    }
}

// Starting: -60
// Bottom: -20
// Middle: 20
// Top: 60