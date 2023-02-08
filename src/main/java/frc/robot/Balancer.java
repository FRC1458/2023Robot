package frc.robot;

import javax.net.ssl.CertPathTrustManagerParameters;
import javax.xml.transform.SourceLocator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive; 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotConstants;
import frc.robot.swervedrive.SwerveDrive;
import frc.robot.swervedrive.Wheel;
import frc.robot.wrappers.*;

import com.kauailabs.navx.frc.AHRS;

import com.ctre.phoenix.motorcontrol.ControlMode;

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
    private double maxPitch = 0;

    public Balancer(SwerveDrive swerve, AHRS navx) {
        this.swerve = swerve;
        timer = new Timer();
        this.navx = navx;
    }


    public boolean balance() {
        double tempPitch = getPitch();
        SmartDashboard.putString("Balancer state: ", state.toString());
        SmartDashboard.putNumber("Balancer Pitch", tempPitch);
        if (Math.abs(tempPitch) > maxPitch) {
            maxPitch = Math.abs(tempPitch);
        }
        SmartDashboard.putNumber("Max pitch: ", maxPitch);

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
        return false;
    }

    
    private void start() {
        timer.start();
        state = States.ORIENT;
    }
    
    private void orient() {
        swerve.drive(0, -0.01, 0, true);
        switchState(0.5, States.FORWARD);
    }
    
    private void forward() {
        swerve.drive(0, -0.2, 0, true);
        if (Math.abs(getPitch()) > 10) {
            nextState(States.CLIMB);
            return;
        }
        switchState(10, States.TUNINGFORWARD);
    }

    private void climb() {
        switchState(1.5, States.TUNINGFORWARD);
    }
    
    private void tuningForward() {
        if (getPitch() > 5) {
            swerve.drive(0, -0.05, 0, true); 
        }
        else if (getPitch() > 4) {
            swerve.drive(0, -0.025, 0, true); 
        }
        else {
            nextState(States.TUNINGBACKWARD);
        }
    }

    private void tuningBackward() {
        if (getPitch() < -5) {
            swerve.drive(0, 0.05, 0, true);
        }
        else if (getPitch() < -4) {
            swerve.drive(0, 0.025, 0, true);
        }
        else if (getPitch() < 4) {
            switchState(0.5, States.LOCK);
        }
        else {
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
        timer.reset();
        maxPitch = 0;
    }

    private double getPitch() {
        return navx.getPitch() * 3.0;
    }

}