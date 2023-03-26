package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.swervedrive.SwerveDrive;

public class Balancer {

    private enum States {
        START,
        ORIENT,
        FORWARD,
        CLIMB,
        TUNINGFORWARD,
        TUNINGBACKWARD,
        LOCK,
        STOP
    }
    private final SwerveDrive swerve;
    private final Timer timer;
    private final AHRS navx;

    private States state = States.START;
    public Balancer(SwerveDrive swerve, AHRS navx) {
        this.swerve = swerve;
        timer = new Timer();
        this.navx = navx;
    }


    public boolean balance() {
        switch (state) {
            case START:
                start();
                break;
            case ORIENT:
                orient();
                break;
            case FORWARD:
                forward();
                break;
            case CLIMB:
                climb();
                break;
            case TUNINGFORWARD:
                tuningForward();
                break;
            case TUNINGBACKWARD:
                tuningBackward();
                break;
            case LOCK:
                lock();
                break;
            case STOP:
                stop();
                break;
        }
        SmartDashboard.putString("Balance State", state.toString());
        return false;
    }

    private void start() {
        timer.reset();
        timer.start();              
        state = States.ORIENT;
    }
    
    private void orient() {
        swerve.drive(0, -0.001, 0, true);
        switchState(1, States.FORWARD);
    }
    
    private void forward() {
        swerve.drive(0, -0.3, 0, true); //0,-0.2,0
        if (Math.abs(getPitch()) > RobotConstants.balancePitchStart) {
            nextState(States.CLIMB);
            return;
        }
        switchState(5, States.STOP);
    }

    private void climb() {
        swerve.drive(0, -0.2, 0, true); //0,-0.2,0
        switchState(2, States.TUNINGFORWARD);
    }
    
    private void tuningForward() {
        if (getPitch() < -1 * RobotConstants.balancePitchHeavy) {
            swerve.drive(0, (-1 * RobotConstants.balanceSpeedHeavy), 0, true);
        } else if (getPitch() < RobotConstants.balancePitchSmall) {
            switchState(0.5, States.LOCK);
        } else {
            nextState(States.TUNINGBACKWARD);
        }
    }

    private void tuningBackward() {
        if (getPitch() > RobotConstants.balancePitchHeavy) {
            swerve.drive(0, RobotConstants.balanceSpeedHeavy, 0, true);
        } else if (getPitch() > -1 * RobotConstants.balancePitchSmall) {
            switchState(0.5, States.LOCK);
        } else {
            nextState(States.TUNINGFORWARD);
        }
    }
    
    private void lock() {
        swerve.drive(0.01, 0, 0, true);
        switchState(0.5, States.STOP);
    }
    
    private void stop() {
        swerve.drive(0, 0, 0, true);
    }
    
    private void switchState(double time, States next) {
        if (timer.hasElapsed(time)) {
            nextState(next);
        }
    }

    private void nextState(States next) {
        state = next;
        timer.reset();
        timer.start();
    }

    public void reset() {
        state = States.START;
    }

    public double getPitch() {
        return navx.getPitch() * -1;
    }

}