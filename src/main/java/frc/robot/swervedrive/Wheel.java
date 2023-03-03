package frc.robot.swervedrive;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;


public class Wheel {
    private CANSparkMax angleMotor;
    private CANSparkMax speedMotor;
    private SparkMaxPIDController pidController;
    public final RelativeEncoder encoder;
    private TalonSRX absoluteEncoder;

    private double speed;
    private double goalAngle;

    public final String wheelName;
    private double offset;

    private boolean diagnostic;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

    public Wheel(int angleMotorID, int speedMotorID, int absoluteEncoderID, String wheelName, double offset) {
        this.angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        this.speedMotor = new CANSparkMax(speedMotorID, MotorType.kBrushless);
        this.absoluteEncoder = new TalonSRX(absoluteEncoderID);
        this.offset = offset;

        this.absoluteEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

        this.wheelName = wheelName;

        pidController = angleMotor.getPIDController();
        encoder = angleMotor.getEncoder();
        //SmartDashboard.putNumber("Rotations", 0);

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

        //SmartDashboard.putNumber("P Gain", kP);
        //SmartDashboard.putNumber("I Gain", kI);
        //SmartDashboard.putNumber("D Gain", kD);
    }

    public void drive(double speed, double angle) {
        this.speed = speed;

        goalAngle = angle;

        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);

        if ((p != kP)) {
            pidController.setP(p);
            kP = p;
        }
        if ((i != kI)) {
            pidController.setI(i);
            kI = i;
        }
        if ((d != kD)) {
            pidController.setD(d);
            kD = d;
        }

        double processVariable = encoder.getPosition();
        double currentAngle = (processVariable * (360 / RobotConstants.swerveDriveGearRatio));
        double diff = (currentAngle - goalAngle) % 360;

        if (Math.abs(diff) > 180) {
            diff = diff - 360 * Math.signum(diff);
        }

        double realGoalRotations = (currentAngle - diff) * RobotConstants.swerveDriveGearRatio / 360;

        //SmartDashboard.putNumber("Current Angle " + wheelName, currentAngle);
        //SmartDashboard.putNumber("Goal Angle " + wheelName, goalAngle);
        //SmartDashboard.putNumber("difference " + wheelName, diff);
        //SmartDashboard.putNumber("SPEED " + wheelName + wheelName, speed);
        //SmartDashboard.putNumber("Absolute Encoder Angle " + wheelName, (offset + getAbsoluteEncoderValue()) * (360 / RobotConstants.swerveDriveGearRatio));

        if (speed != 0 || diagnostic) {
            pidController.setReference(realGoalRotations, CANSparkMax.ControlType.kPosition);
        }
        speedMotor.set(speed);
    }

    public void printTalon() {
        SmartDashboard.putNumber(wheelName + " Talon", getAbsoluteEncoderValue());
    }

    public void setEncoders(double offset) {
        encoder.setPosition(offset + (absoluteEncoder.getSelectedSensorPosition(0) % 4096) * RobotConstants.swerveDriveGearRatio / 4096.0);
        SwerveModuleState state = new SwerveModuleState(0.000000000001, Rotation2d.fromDegrees(0));
        this.drive(state.speedMetersPerSecond, state.angle.getDegrees());
    }

    public double getAbsoluteEncoderValue() {
        return ((absoluteEncoder.getSelectedSensorPosition(0) % 4096) * RobotConstants.swerveDriveGearRatio / 4096.0);
    }

    public double angleMotorDiagnostic() {
        diagnostic = true;
        this.drive(0, (encoder.getPosition() * (360 / RobotConstants.swerveDriveGearRatio)) + 720);
        return (offset + getAbsoluteEncoderValue()) * (360 / RobotConstants.swerveDriveGearRatio) - (encoder.getPosition() * (360 / RobotConstants.swerveDriveGearRatio));
    }
}