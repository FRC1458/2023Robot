package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private final double metersToSwerve =  RobotConstants.metersToSwerve;//multiply by meters to give a value to input to swerve
    private double testOffset = 0.01; //Multiplies to slow bot for testing for safety
    private double xOffset;
    private double yDistance;
    private double xDistance;
    private final double yFinalDistance = 0.61;//in m, distance from robot to base of scoring area, add to RobotConstants
    private States state = States.START;


    public Aligner(SwerveDrive swerve, Limelight limelight ) {
        timer = new Timer();
        this.swerve = swerve;
        this.limelight = limelight;
    }

    public boolean align() {

        xOffset = limelight.getXOffset();
        String alignment = "Straight";
        if (xOffset < -2) {
            alignment = "Left";
        } else if (xOffset > 2) {
            alignment = "Right";
        }
        SmartDashboard.putString("LimelightAlignment", alignment);

        switch (state) {
            case MOVE:
                move();
                break;
            case STOP:
                break;
        }
        return false;
    }

    private void move() {
    }

}
