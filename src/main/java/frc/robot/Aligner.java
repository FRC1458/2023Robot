package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.swervedrive.SwerveDrive;

public class Aligner {

    private enum States{
        START,
        ROTATE,
        MOVE,
        SLOW,
        STOP
    }

    private final SwerveDrive swerve;
    private Timer timer;
    private final Limelight limelight;


    private final double metersToSwerve =  RobotConstants.metersToSwerve;//multiply by meters to give a value to input to swerve
    private double testOffset = 0.01; //Multiplies to slow bot for testing for safety
    private double xOffset;
    private double yOffset;
    private double rTarget;
    private  double yGoal = 20;
    private final double yFinalDistance = 0.61;//in m, distance from robot to base of scoring area, add to RobotConstants
    private States state = States.START;


    public Aligner(SwerveDrive swerve, Limelight limelight ) {
        timer = new Timer();
        this.swerve = swerve;
        this.limelight = limelight;
    }

    public void align() {
        limelight.readPeriodic();
        xOffset = limelight.getXOffset();
        yOffset = limelight.getYOffset();
        rTarget = limelight.getRotation();
        String alignment = "Straight";
        if (xOffset < -.1) {
            alignment = "Left";
        } else if (xOffset > .1) {
            alignment = "Right";
        }
        SmartDashboard.putString("LimelightAlignment", alignment);
        SmartDashboard.putNumber("xOffset", xOffset);
        SmartDashboard.putNumber("yOffset", yOffset);
        SmartDashboard.putNumber("rTarget", rTarget);
        SmartDashboard.putString("Align State", state.toString());



        switch (state) {
            case START:
                //start();
                state = States.ROTATE;
            case ROTATE:
                rotate();
                break;
            case MOVE:
                //move();
                break;
            case STOP:
                stop();
                break;
        }
    }

    private void rotate() {
        double direction = swerve.turnToAngle(0, rTarget);
        if (Math.abs(rTarget) < 3) {
            state = States.MOVE;
        }
        swerve.drive(0, 0, -rTarget/200, true);
    }
    private void move() {
        /*yDistance -= 20;//align 20cm in front of april tag
        double time = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
        timer.reset();
        swerve.drive(0.1, 0.1, 0, true);
        if (timer.hasElapsed(Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2)))) {
            state = States.STOP;
        }*/
    }
    public void stop() {
        SmartDashboard.putNumber("xError (degrees)", limelight.getXOffset());
        SmartDashboard.putNumber("yError (degrees)", limelight.getYOffset());
    }

    public void reset() {
        state = States.START;
    }

}
