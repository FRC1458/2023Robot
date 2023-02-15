package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.swervedrive.SwerveDrive;

public class Aligner {

    private enum States{
        START,
        FAST,
        SLOW,
        STOP
    }

    private final SwerveDrive swerve;
    private final Timer timer;
    private final Limelight limelight;
    private final Lidar lidar;

    private final double metersToSwerve = 0.0000235;//multiply by meters to give a value to input to swerve
    private double xOffset;
    private double yDistance;
    private double xDistance;
    private final double yFinalDistance = 0.61;//in m, add to RobotConstants
    private States state = States.START;

    public Aligner(SwerveDrive swerve, Limelight limelight, Lidar lidar) {
        timer = new Timer();
        this.swerve = swerve;
        this.limelight = limelight;
        this.lidar = lidar;
    }

    public boolean align() {
        switch (state) {
            case START:
                start();
                break;
            case FAST:
                fast();
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
        state = States.FAST;
    }
//-.1 for metersToSwerve * 10, -.1*xDistance for xDistance*metersToSwerve * 10
//y component and timer divided by 10 from calculated values for safety for test, change back if everything works
    private void fast() {
        timer.start();
        swerve.drive(-0.1, -.01 * xDistance, 0, true); //change to higher speed, multiply by sign of xDistance
        if (timer.hasElapsed(xDistance * metersToSwerve)) {//some value * xDistance, currently assumes swerve input is .1m/s
            state = States.SLOW;
        }
    }

    private void slow() {//same as fast but with slower swerve.drive() for fine-tuning
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

    public void reset() {
        state = States.START;
        timer.reset();
    }
}
