package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.swervedrive.SwerveDrive;

public class Autonomous {

    boolean hasRun = false;
    SwerveDrive swerve;
    Arm arm;
    Timer timer = new Timer();
    public Autonomous(SwerveDrive swerve, Arm arm) {
        this.swerve = swerve;
        this.arm = arm;
    }
    public void resetAuto() {
        hasRun = false;
    }
    public void runAuto(boolean isCenter) {
        if (hasRun) {
            return;
        }
        hasRun = true;
        swerve.resetNavX();
        swerve.drive(0, -0.0000000000001, 0, true);
        arm.reset();
        arm.setUp();
        timer.reset();
        timer.start();
        while(arm.getState() == Arm.armStates.TOP) {
            if (timer.hasElapsed(5)) return;
            arm.runArm(false, false);
        }
        swerve.drive(0, 0,0, true);
        arm.runArm(false, false);
        arm.extendArm();
        timer.reset();
        timer.start();
        while (!timer.hasElapsed(2)) {
            swerve.drive(0, -0.1, 0, true);
        }
        swerve.drive(0, 0, 0, true);
        arm.openClaw();
        timer.reset();
        while (!timer.hasElapsed(1.5)){}
        driveBackwards(0.5);
        arm.retractArm();
        arm.closeClaw();
        timer.reset();
        while (!timer.hasElapsed(0.5)) {}
        arm.setInside();
        timer.reset();
        while (arm.getState() == Arm.armStates.INSIDE) {
            if (timer.hasElapsed(5)) return;
            arm.runArm(false, false);
        }
        arm.runArm(false, false);
        /*double direction = swerve.turnToAngle(180);
        while (direction != 0) {
            direction = swerve.turnToAngle(180);
            swerve.drive(0, 0, direction, true);
        }
        swerve.drive(0, 0, 0, true);
        swerve.resetNavX();*/
        driveBackwards(3);
        swerve.drive(0, 0.0000001, 0, true);
    }
    public void simpleAuto() {
        if (hasRun) {
            return;
        }
        hasRun = true;
        driveBackwards(3);
    }
    public void driveBackwards(double time) {
        timer.start();
        timer.reset();
        swerve.drive(0, 0.3, 0, false);
        while (!timer.hasElapsed(time)) {}
        swerve.drive(0, 0, 0, true);
    }
}
