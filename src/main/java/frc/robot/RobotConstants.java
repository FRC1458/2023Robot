package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class RobotConstants {
    public enum ControllerType {XBOX, JOYSTICK}
    public static final ControllerType controller = ControllerType.XBOX;

    public final static int frontLeftAngleID = 19;
    public final static int frontRightAngleID = 15;
    public final static int backLeftAngleID = 13;
    public final static int backRightAngleID = 4;
    
    public final static int frontLeftSpeedID = 9;
    public final static int frontRightSpeedID = 5;
    public final static int backLeftSpeedID = 3;
    public final static int backRightSpeedID = 7;
    
    public final static double frontLeftAngleOffset = -5;
    public final static double frontRightAngleOffset = 4.4;
    public final static double backLeftAngleOffset = 0.4;
    public final static double backRightAngleOffset = 0;

    public final static double kP = 0.1;
    public final static double kI = 0.000001;
    public final static double kD = 0.000001;

    public final static double frontLeftXMeters = 0.273;
    public final static double frontLeftYMeters = 0.273;
    public final static double frontRightXMeters = 0.273;
    public final static double frontRightYMeters = -0.273;
    public final static double backLeftXMeters = -0.273;
    public final static double backLeftYMeters = 0.273;
    public final static double backRightXMeters = -0.273;
    public final static double backRightYMeters = -0.273;

    public final static double swerveDriveGearRatio = 12.8;//THING WE ARE TESTING!!!

    // public final static int intakeMotorID = 29;
    // public final static int rightElevatorMotorID = 36;
    // public final static int leftElevatorMotorID = 37;
    // public final static int leftDepositorMotorID = 27;
    // public final static int rightDepositorMotorID = 23;

    public final static int intakeSolenoidForwardID = 10;
    public final static int intakeSolenoidReverseID = 11;
    // public final static int rightIntakeSolenoidForwardID = 0;
    // public final static int rightIntakeSolenoidReverseID = 0;
    public final static int elevatorSolenoidForwardID = 8;
    public final static int elevatorSolenoidReverseID = 9;
    // public final static int rightElevatorSolenoidForwardID = 0;
    // public final static int rightElevatorSolenoidReverseID = 0;

    public final static int bottomLimitSwitchID = 8;
    public final static int middleLimitSwitchID = 6;
    public final static int topLimitSwitchID = 7;

    public final static double elevSpeedUp = .35;
    public final static double elevSpeedDown = .2;
    public final static double elevSpeedManual = .15;
    public final static double regularSpeed = .75;
    public final static double boostedSpeed = .75;

    public final static boolean fieldOriented = true;

    public final static int frontLeftAbsoluteEncoderID = 28;
    public final static int frontRightAbsoluteEncoderID = 18;
    public final static int backLeftAbsoluteEncoderID = 17;
    public final static int backRightAbsoluteEncoderID = 27;

    // CHANGE ALL THESE VALUES TO REAL NUMBERS
    public final static double frontLeftDistance = 0;
    public final static double frontRightDistance = 0;
    public final static double backLeftDistance = 0;
    public final static double backRightDistance = 0;
    
    public final static Rotation2d frontLeftAngle = new Rotation2d(0);
    public final static Rotation2d frontRightAngle = new Rotation2d(0);
    public final static Rotation2d backLeftAngle = new Rotation2d(0);
    public final static Rotation2d backRightAngle = new Rotation2d(0);

    public final static int lidarPort = 1;
    public final static int armLidarPort = 4;
}
