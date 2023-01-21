package frc.robot.swervedrive;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
//import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;

import frc.robot.RobotConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import frc.robot.wrappers.Wrapper;
import edu.wpi.first.math.geometry.Rotation2d;


public class Wheel{
    private CANSparkMax angleMotor;
    private CANSparkMax speedMotor;
    private SparkMaxPIDController pidController;
    public final AbsoluteEncoder absoluteEncoder;
    
    //private ControlType controltype;


    private double speed;
    private double goalAngle;

    public final String wheelName;
    private double offset;

    private boolean diagnostic;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

    public Wheel (int angleMotorID, int speedMotorID, String wheelName, double offset) {
        this.angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        this.speedMotor = new CANSparkMax(speedMotorID, MotorType.kBrushless);
        angleMotor.getEncoder();
        speedMotor.getEncoder();
        this.absoluteEncoder = this.angleMotor.getAbsoluteEncoder(Type.kDutyCycle);
        angleMotor.getEncoder();
        this.offset = offset;

        this.wheelName = wheelName;
        
        speedMotor.restoreFactoryDefaults();
        angleMotor.restoreFactoryDefaults();

        pidController = angleMotor.getPIDController();
        //pidController.setFeedbackDevice(absoluteEncoder);

        SmartDashboard.putNumber("Rotations", 0);

        angleMotor.restoreFactoryDefaults();
        speedMotor.restoreFactoryDefaults();

        absoluteEncoder.setPositionConversionFactor(2*Math.PI);
        absoluteEncoder.setVelocityConversionFactor(2*Math.PI/60);

        kP = RobotConstants.kP; 
        kI = RobotConstants.kI;
        kD = RobotConstants.kD; 
        kIz = 0; 
        kFF = 0.000156; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;

        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);

        int smartMotionSlot = 0;
        pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);

        angleMotor.burnFlash();
        speedMotor.burnFlash();
    }

    public void drive (double speed, double angle) {
        this.speed = speed;
        
        goalAngle = angle;
        
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
    
        // if PID coefficients on SmartDashboard have changed, write new values to controller

        if((p != kP)) { pidController.setP(p); kP = p; }
        if((i != kI)) { pidController.setI(i); kI = i; }
        if((d != kD)) { pidController.setD(d); kD = d; }

        
        double processVariable = absoluteEncoder.getPosition();
        SmartDashboard.putNumber("ProccessVariable" + wheelName, processVariable);

        // SmartDashboard.putNumber("Output" + wheelName, angleMotor.getAppliedOutput());

        SmartDashboard.putNumber("Angle Motor Current (Amps)" + wheelName, angleMotor.getOutputCurrent());
        // SmartDashboard.putNumber("Speed Motor Current (Amps)", speedMotor.getOutputCurrent());

        
        double currentAngle = (processVariable * (360 / RobotConstants.swerveDriveGearRatio));


        double diff = (currentAngle - goalAngle) % 360;

        if (Math.abs(diff) > 180) {
          diff = diff - 360*Math.signum(diff); // add or subtract 360 so the difference is always smaller than 180
        }

        double realGoalRotations = (currentAngle - diff) * RobotConstants.swerveDriveGearRatio/360;

        SmartDashboard.putNumber("Current Angle " + wheelName, currentAngle);

        SmartDashboard.putNumber("Goal Angle " + wheelName, goalAngle);

        SmartDashboard.putNumber("difference " + wheelName, diff);

        SmartDashboard.putNumber("SPEED " + wheelName + wheelName, speed);

        SmartDashboard.putNumber("Absolute Encoder Angle " + wheelName, (offset + getAbsoluteEncoderValue()) * (360/RobotConstants.swerveDriveGearRatio));

        // rotations = SmartDashboard.getNumber("Rotations", 0);

        if (speed != 0 || diagnostic){
            //pidController.setReference(realGoalRotations, CANSparkMax.ControlType.kPosition);
        }
        //speedMotor.set(speed);
    }

    public void printTalon() {
        SmartDashboard.putNumber(wheelName + " Talon", getAbsoluteEncoderValue());
    }

    public double getAbsoluteEncoderValue() {
        return ((absoluteEncoder.getPosition() % 4096)*RobotConstants.swerveDriveGearRatio/4096.0);
    }

    public double angleMotorDiagnostic() {
        diagnostic = true;
        this.drive(0, (absoluteEncoder.getPosition() * (360 / RobotConstants.swerveDriveGearRatio)) + 720);
        return (offset + getAbsoluteEncoderValue()) * (360/RobotConstants.swerveDriveGearRatio) - (absoluteEncoder.getPosition() * (360 / RobotConstants.swerveDriveGearRatio));
    }
}