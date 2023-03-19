package frc.robot;


import frc.robot.wrappers.XboxControllerWrapper;

public class XboxController implements Controller {

    boolean balancePressed = false;

    XboxControllerWrapper xbox;
    public XboxController () {
        xbox = new XboxControllerWrapper(0);
    }

    @Override
    public boolean lockWheels() {
        return xbox.getXButton();
    }

    @Override
    public boolean stateManual() {
        return xbox.getPOV() == 270;
    }

    @Override
    public boolean stateAlign() {
        return false;
    }

    @Override
    public boolean stateBalance() {
        if (xbox.getPOV() == 90 && !balancePressed) {
            balancePressed = true;
            return true;
        } else if (xbox.getPOV() != 90) {
            balancePressed = false;
        }
        return false;
    }

    @Override
    public boolean resetNavX() {
        return xbox.getStartButton();
    }

    @Override
    public boolean resetArm() {
        return xbox.getBackButtonPressed();
    }

    @Override
    public boolean armTop() {
        return xbox.getYButton();
    }

    @Override
    public boolean armMiddle() {
        return xbox.getBButton();
    }

    @Override
    public boolean armBottom() {
        return xbox.getAButton();
    }

    @Override
    public boolean fieldOriented() {
        return true;
    }

    @Override
    public boolean clawOpen() {
        return xbox.getLeftTriggerAxis() > .7;
    }

    @Override
    public boolean clawClose() {
        return xbox.getRightTriggerAxis() > .7;
    }

    @Override
    public boolean armExtend() {
        return xbox.getRightBumper();
    }

    @Override
    public boolean armRetract() {
        return xbox.getLeftBumper();
    }

    @Override
    public boolean armUp() {
        return xbox.getPOV() == 0;
    }

    @Override
    public boolean armDown() {
        return xbox.getPOV() == 180;
    }

    @Override
    public double getSwerveR() {
        return xbox.getRightX();
    }

    @Override
    public double getSwerveX() {
        return xbox.getLeftX();
    }

    @Override
    public double getSwerveY() {
        return xbox.getLeftY();
    }
}
