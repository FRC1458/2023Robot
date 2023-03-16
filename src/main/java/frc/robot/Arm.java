package frc.robot;

import frc.robot.wrappers.TalonFXWrapper;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;

public class Arm {
    private final Solenoid armSolenoid;
    private final Solenoid clawSolenoid;
    private final TalonFXWrapper armMotor;
    private final ArmNavX armnavx;
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
    
    public double encoderPosition() {
        TalonFXSensorCollection sensors = armMotor.talon.getSensorCollection();
        double absPos = sensors.getIntegratedSensorPosition();
        System.out.println("ArmNavX: " + (90-armnavx.getPitch()));
        System.out.print("Encoder: " + absPos*RobotConstants.armEncoderRatio);
        return absPos * RobotConstants.armEncoderRatio;
    }

    public void reset() {
        armnavx.reset();
        TalonFXSensorCollection sensors = armMotor.talon.getSensorCollection();
        double new_position = 90 - armnavx.getPitch() / RobotConstants.armEncoderRatio;
        sensors.setIntegratedSensorPosition(new_position, 0);
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
        } else {
            armMotor.set(0);
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
        if (!armExtended) {
            extendArm();
        }
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
        if (armnavx.getPitch() > -10) {
            armMotor.set(RobotConstants.armSpeed);
        }
        else {
            armMotor.set(0);
        }
    }

    public void moveDown() {
        if (armnavx.getPitch() < 90) {
            armMotor.set(-1 * RobotConstants.armSpeed);
        }
        else {
            armMotor.set(0);
        }
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