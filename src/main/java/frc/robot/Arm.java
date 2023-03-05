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
    private boolean armExtended;

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
        armMotor = new TalonFXWrapper(43, true);
        clawOpened = true;
        armExtended = false;
    }
    public void idle() {
        armMotor.set(0);
    }
    public boolean up() {
        SmartDashboard.putNumber("Arm NavX Pitch", getPitch());
        if (getPitch() > 2) {
            armMotor.set(RobotConstants.armSpeed);
        }
        else if (getPitch() < -2) {
            armMotor.set(-1 * RobotConstants.armSpeed);
        }
        else {
            return true;
        }
        return false;
    }

    public boolean middle() {
        if (getPitch() > 32) {
            armMotor.set(RobotConstants.armSpeed);
        }
        else if (getPitch() < 28) {
            armMotor.set(-1 * RobotConstants.armSpeed);
        }
        else {
            return true;
        }
        return false;
    }

    public boolean down() {
        if (getPitch() > 62) {
            armMotor.set(RobotConstants.armSpeed);
        }
        else if (getPitch() < 58) {
            armMotor.set(-1 * RobotConstants.armSpeed);
        }
        else {
            return true;
        }
        return false;
    }

    public void moveUp() {
        armMotor.set(RobotConstants.armSpeed);
    }
    public void moveDown() {
        armMotor.set(-1 * RobotConstants.armSpeed);
    }

    public void extendArm() {
        armSolenoid.forward();
    }
    public void retractArm() {
        armSolenoid.reverse();
    }
    public void openClaw() {
        clawSolenoid.reverse();
        clawOpened = true;
    }

    public void closeClaw() {
        if (clawOpened) {
            clawSolenoid.forward();
        }
    }

    private double getPitch() {
        return armnavx.getPitch();
    }
    public double getEncoder() {
        return armMotor.getEncoder();
    }
}

// Starting: -60
// Bottom: -20
// Middle: 20
// Top: 60

