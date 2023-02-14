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

    private double xOffset;
    private double yDistance;
    private double xDistance;
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

    private double convertToRad(double degrees) {
        return ((degrees*Math.PI)/(180));
    }
    private void start() {
        xOffset = limelight.getXOffset();
        yDistance = lidar.getDistanceCentimeters();
        xDistance = Math.tan(convertToRad(xOffset));
        state = States.FAST;
    }

    private void fast() {
        timer.start();
        swerve.drive((0.02), 0, 0, true); //change to higher speed, multiply by sign of xDistance
        if (timer.hasElapsed(xDistance)) {//some value * xDistance
            state = States.SLOW;
        }
    }

    private void slow() {//same as fast but with slower swerve.drive() for fine-tuning
        timer.reset();
        swerve.drive(0.01, 0, 0, true);
        if (timer.hasElapsed(yDistance)) {
            state = States.STOP;
        }
    }

    private void stop() {
        timer.stop();
        //need to return to teleopPeriodic
    }

}
