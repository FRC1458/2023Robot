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
        while(arm.getState() == Arm.armStates.TOP) {
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
        driveBackwards();
    }
    public void simpleAuto() {
        if (hasRun) {
            return;
        }
        hasRun = true;
        driveBackwards();
    }
    public void driveBackwards() {
        timer.start();
        timer.reset();
        swerve.drive(0, 0.3, 0, false);
        while (!timer.hasElapsed(3)) {}
        swerve.drive(0, 0, 0, true);
    }
}
