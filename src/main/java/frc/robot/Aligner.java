package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.swervedrive.SwerveDrive;

public class Aligner {

    private enum States{
        START,
        MOVE,
        SLOW,
        STOP
    }

    private final SwerveDrive swerve;
    private final Timer timer;
    private final Limelight limelight;
    private final Lidar lidar;
    private final Arm arm;

    private final double metersToSwerve =  RobotConstants.metersToSwerve;//multiply by meters to give a value to input to swerve
    private double testOffset = 0.01; //Multiplies to slow bot for testing for safety
    private double xOffset;
    private double yDistance;
    private double xDistance;
    private final double yFinalDistance = 0.61;//in m, distance from robot to base of scoring area, add to RobotConstants
    private States state = States.START;

    private int armState;

    public Aligner(SwerveDrive swerve, Limelight limelight, Lidar lidar, Arm arm) {
        timer = new Timer();
        this.swerve = swerve;
        this.limelight = limelight;
        this.lidar = lidar;
        this.arm = arm;
    }

    public boolean align(int armState) {
        switch (state) {
            case START:
                start();
                break;
            case MOVE:
                move(armState);
                break;
            case SLOW:
                slow();
                break;
            case STOP:
                stop();
                break;
        }
        return false;
    }
    private void start() {
        xOffset = limelight.getXOffset();
        yDistance = lidar.getDistanceMeters();
        xOffset = (Math.PI/2)-(Math.toRadians(xOffset));
        xDistance = (yDistance/(Math.tan(xOffset)));
        yDistance = yDistance - yFinalDistance;
        state = States.MOVE;
    }
//-.1 for metersToSwerve * 10, -.1*xDistance for xDistance*metersToSwerve * 10
//Speeds divided by 100 for testing
    private void move(int armState) {
        timer.start();
        if (xDistance < 10) {//Just to be safe, remove if testing outdoors
            swerve.drive(-0.1 * testOffset, -0.1 * xDistance * testOffset, 0, true); //change to higher speed, multiply by sign of xDistance
        }
        if (timer.hasElapsed(xDistance * metersToSwerve * 10)) {//some value * xDistance, currently assumes swerve input is .1m/s
            state = States.SLOW;
        }
        setArm(armState);//needs to be done asynchronously
    }

    private void slow() {
        timer.reset();
        swerve.drive(0.01, 0.01, 0, true);
        if (timer.hasElapsed(yDistance)) {
            state = States.STOP;
        }
    }

    private void stop() {
        timer.stop();
        //need to return to teleopPeriodic
    }

    public void setArm(int state) {
        if (state == 1) {
            arm.down();
        } else if (state == 2) {
            arm.middle();
        } else {
            arm.height();
        }
    }

    public void reset() {
        state = States.START;
        timer.reset();
    }
}
