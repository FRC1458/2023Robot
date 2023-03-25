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
    private final Timer timer;
    private final Limelight limelight;

    private final double metersToSwerve =  RobotConstants.metersToSwerve;//multiply by meters to give a value to input to swerve
    private double testOffset = 0.01; //Multiplies to slow bot for testing for safety
    private double xOffset;
    private double yOffset;
    private double yDistance;
    private double xDistance;
    private final double yFinalDistance = 0.61;//in m, distance from robot to base of scoring area, add to RobotConstants
    private States state = States.START;


    public Aligner(SwerveDrive swerve, Limelight limelight ) {
        timer = new Timer();
        this.swerve = swerve;
        this.limelight = limelight;
    }

    public void align() {

        xOffset = limelight.getXOffset();
        yOffset = limelight.getYOffset();
        yDistance = (0.123825/Math.tan(Math.toRadians(yOffset))) * 100;
        xDistance = ((yDistance/Math.tan(Math.toRadians(90 - xOffset))) - (0.135*100));
        String alignment = "Straight";
        if (xDistance < -.1) {
            alignment = "Left";
        } else if (xDistance > .1) {
            alignment = "Right";
        }
        //SmartDashboard.putString("LimelightAlignment", alignment);
        //SmartDashboard.putNumber("xDistance", xDistance);
        //SmartDashboard.putNumber("yDistance", yDistance);
        //SmartDashboard.putString("Align State", state.toString());

        switch (state) {
            case START:
                //start();
                state = States.ROTATE;
                break;
            case ROTATE:
                rotate();
                break;
            case MOVE:
                move();
                break;
            case STOP:
                break;
        }
    }

    private void rotate() {
        double direction = swerve.turnToAngle(0);
        if (direction == 0) {
            state = States.MOVE;
        }
        swerve.drive(0, 0, direction, true);
    }
    private void move() {
    }

    public void reset() {
        state = States.START;
    }

}
