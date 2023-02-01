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
        TUNING,
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
            case TUNING:
                tuning();
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

    //start is for timer.start()
    private void start() {
        timer.start();
        state = States.ORIENT;
    }
    //orients wheels to face front of robot
    private void orient() {
        swerve.drive(0, -0.01, 0, true);
        switchState(0.5, States.FORWARD);
    }
    private boolean b = true;
    //moves forward onto charge station until flat on top
    private void forward() {
        swerve.drive(0, -0.2, 0, true);// TRY SPEED INCREASE (SLOWLY)
        if (navx.getPitch() < -10) {// can be changed as needed
            //to make sure timer only resets once
            if (b) {
                timer.reset();
                timer.start();
                b = false;
            }
            switchState(2, States.TUNING);//can be changed as needed
            return;
        }
        switchState(10, States.TUNING);//failsafe, will stop robot if doesnt go on station
    }

    public boolean c = true;
    //adjusts on charging station
    private void tuning() {
        //for all if statements speed/pitch can be changed, faster speed/change in pitch = may not balance
        if (navx.getPitch() < -5) {
            swerve.drive(0, -0.05, 0, true); //
        }
        else if (navx.getPitch() < -4) {
            swerve.drive(0, -0.025, 0, true); //-0.025
        }
        else if (navx.getPitch() > 5) {
            swerve.drive(0, 0.05, 0, true); //0.05
        }
        else if (navx.getPitch() > 4) {
            swerve.drive(0, 0.025, 0, true); //0.025
        }
        else {
            //resets timer, only once
            if (c) {
                timer.reset();
                timer.start();
                c = false;
            }
            swerve.drive(0, 0.005, 0, true); //compensates for overshoot
            switchState(0.1, States.LOCK);
        }
    }
    //turns wheels to lock robot in place
    private void lock() {
        swerve.drive(0.01, 0, 0, true);
        switchState(0.5, States.STOP);
    }
    //stop makes sure speed = 0
    private void stop() {
        swerve.drive(0, 0, 0, true);
    }
    //switches state using timer
    private void switchState(double time, States nextState) {
        if (timer.hasElapsed(time)) {
            state = nextState;
            timer.reset();
            timer.start();
        }
    }
}