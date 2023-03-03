package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.wrappers.TalonFXWrapper;

public class Arm {
    private Solenoid armSolenoid;
    private Solenoid clawSolenoid;
    private TalonFXWrapper armMotor;
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
        this.armMotor = new TalonFXWrapper(43);
        clawOpened = false;

    }

    public void runArm() {

    }

    public int height() {
        double pitch = armnavx.getPitch();
        SmartDashboard.putNumber("Arm NavX Pitch", pitch);
        if (pitch > 5) {
            //pitch = armnavx.getPitch();
            armMotor.set(0.25);
        }
        else if (pitch < -5) {
            //pitch = armnavx.getPitch();
            armMotor.set(-0.25);
        }
        else {
            direction = 0;
            armMotor.set(0.075);
            return 1;
        }
        return 3;
    }

    public void middle() {
        if (armMotor.getEncoder() > 30) {
            direction = -1;
        } else {
            direction = 1;
        }

        armMotor.set(0);
        armSolenoid.forward();
    }

    public void down() {
        if (armMotor.getEncoder() > -20) {
            direction = -1;
        } else {
            direction = 1;
        }

        armMotor.set(0);
        armSolenoid.forward();
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

