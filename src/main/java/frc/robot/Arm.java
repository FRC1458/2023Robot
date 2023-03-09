package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.wrappers.TalonFXWrapper;

public class Arm {
    private Solenoid armSolenoid;
    private Solenoid clawSolenoid;
    private TalonFXWrapper armMotor;
    private ArmNavX armnavx;
    private armStates state = armStates.IDLE;
    private boolean armExtended = false;

    private enum armStates {
        TOP,
        MIDDLE,
        BOTTOM,
        IDLE
    }

    public Arm(Solenoid armSolenoid, Solenoid clawSolenoid, ArmNavX armnavx) {
        this.armSolenoid = armSolenoid;
        this.clawSolenoid = clawSolenoid;
        this.armnavx = armnavx;
        armMotor = new TalonFXWrapper(43, true);
    }

    public void idle() {
        armMotor.set(0);
    }

    public void runArm(boolean goDown, boolean goUp) {
        if (goDown || goUp) {
            state = armStates.IDLE;
        }

        switch (state) {
            case TOP:
                up();
                break;
            case MIDDLE:
                middle();
                break;
            case BOTTOM:
                down();
                break;
            case IDLE:
                runManual(goDown, goUp);
                break;
        }

        if (armnavx.getPitch() > 70) {
            closeClaw();
        }
    }

    public void setUp() {
        state = armStates.TOP;
    }

    public void setMiddle() {
        state = armStates.MIDDLE;
    }
    
    public void setBottom() {
        state = armStates.BOTTOM;
    }

    public void runManual(boolean goDown, boolean goUp) {
        if (goDown && (armnavx.getPitch() < 57 || !armExtended)) {
            moveDown();
        } else if (goUp) {
            moveUp();
        }
    }


    public void up() {
        moveToPreset(0);
    }

    public void middle() {
        moveToPreset(30);
    }

    public void down() {
        moveToPreset(55);
    }

    public void moveToPreset(double presetAngle) {
        if (armnavx.getPitch() > presetAngle + 0.5) {
            moveUp();
        }
        else if (armnavx.getPitch() < presetAngle - 0.5) {
            moveDown();
        }
        else {
            state = armStates.IDLE;
        }      
    }

    public void moveUp() {
        armMotor.set(RobotConstants.armSpeed);
    }

    public void moveDown() {
        armMotor.set(-1 * RobotConstants.armSpeed);
    }

    public void extendArm() {
        if (armnavx.getPitch() < 57) {
            armExtended = true;
            armSolenoid.forward();
        }
    }

    public void retractArm() {
        armExtended = false;
        armSolenoid.reverse();
    }

    public void openClaw() {
        if (armnavx.getPitch() < 67) {
            clawSolenoid.reverse();
        }
    }

    public void closeClaw() {
        clawSolenoid.forward();
    }
}