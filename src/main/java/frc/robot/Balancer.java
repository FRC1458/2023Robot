package frc.robot;

import javax.net.ssl.CertPathTrustManagerParameters;
import javax.xml.transform.SourceLocator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//Documentation: first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/(last_part).html
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive; //might not be needed if we have SwerveDrive working
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
        if (navx.getPitch() < -10) {
            switchState(3.5, States.LOCK);
            return;
        }
        switchState(10, States.STOP);
    }

    private void lock() {
        swerve.drive(0.01, 0, 0, true);
        switchState(0.5, States.STOP);
    }

    private void stop() {
        swerve.drive(0, 0, 0, true);
    }

    private void switchState(double time, States nextState) {
        if (timer.hasElapsed(time)) {
            state = nextState;
            timer.reset();
            timer.start();
        }
    }
}