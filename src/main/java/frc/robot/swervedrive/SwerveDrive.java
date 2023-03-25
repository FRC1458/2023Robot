package frc.robot.swervedrive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;

public class SwerveDrive {

    ChassisSpeeds speeds;
    public final Wheel frontLeft;
    public final Wheel frontRight;
    public final Wheel backLeft;
    public final Wheel backRight;
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;
    private final AHRS navX;

    public SwerveDrive(AHRS navX) {
        this.navX = navX;
        frontLeft = new Wheel(RobotConstants.frontLeftAngleID, RobotConstants.frontLeftSpeedID, RobotConstants.frontLeftAbsoluteEncoderID, "Front Left (1)", RobotConstants.frontLeftAngleOffset);
        frontRight = new Wheel(RobotConstants.frontRightAngleID, RobotConstants.frontRightSpeedID, RobotConstants.frontRightAbsoluteEncoderID, "Front Right (2)", RobotConstants.frontRightAngleOffset);
        backLeft = new Wheel(RobotConstants.backLeftAngleID, RobotConstants.backLeftSpeedID, RobotConstants.backLeftAbsoluteEncoderID, "Back Left (3)", RobotConstants.backLeftAngleOffset);
        backRight = new Wheel(RobotConstants.backRightAngleID, RobotConstants.backRightSpeedID, RobotConstants.backRightAbsoluteEncoderID, "Back Right (4)", RobotConstants.backRightAngleOffset);

        Translation2d frontLeftLocation = new Translation2d(RobotConstants.frontLeftXMeters, RobotConstants.frontLeftYMeters);
        Translation2d frontRightLocation = new Translation2d(RobotConstants.frontRightXMeters, RobotConstants.frontRightYMeters);
        Translation2d backLeftLocation = new Translation2d(RobotConstants.backLeftXMeters, RobotConstants.backLeftYMeters);
        Translation2d backRightLocation = new Translation2d(RobotConstants.backRightXMeters, RobotConstants.backRightYMeters);

        SwerveModulePosition frontLeftPosition = new SwerveModulePosition(RobotConstants.frontLeftDistance, RobotConstants.frontLeftAngle);
        SwerveModulePosition frontRightPosition = new SwerveModulePosition(RobotConstants.frontRightDistance, RobotConstants.frontRightAngle);
        SwerveModulePosition backLeftPosition = new SwerveModulePosition(RobotConstants.backLeftDistance, RobotConstants.backLeftAngle);
        SwerveModulePosition backRightPosition = new SwerveModulePosition(RobotConstants.backRightDistance, RobotConstants.backRightAngle);
        SwerveModulePosition[] positions = {frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition};

        kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), positions, new Pose2d(5.0, 13.5, new Rotation2d()));

        speeds = new ChassisSpeeds();


    }

    public void drive(double x, double y, double r, boolean fieldOriented) {
        speeds.vxMetersPerSecond = x;
        speeds.vyMetersPerSecond = y;
        speeds.omegaRadiansPerSecond = r;

        if (fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, r, Rotation2d.fromDegrees(-(navX.getYaw())));
        }

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);


        frontLeft.drive(states[2].speedMetersPerSecond, states[2].angle.getDegrees());
        frontRight.drive(states[0].speedMetersPerSecond, states[0].angle.getDegrees());
        backLeft.drive(states[3].speedMetersPerSecond, states[3].angle.getDegrees());
        backRight.drive(states[1].speedMetersPerSecond, states[1].angle.getDegrees());

    }

    public double turnToAngle(double goalAngle) {
        double error = 0.25;
        double currentAngle = navX.getYaw();

        double diff = (currentAngle - goalAngle) % 360;

        if (Math.abs(diff) > 180) {
            diff = diff - 360 * Math.signum(diff);
        }

        double realGoalAngle = (currentAngle - diff);

        if (Math.abs(currentAngle - realGoalAngle) > error) {
            if (currentAngle > realGoalAngle) {
                return -.2;
            } else {
                return .2;
            }
        }
        return 0;

    }

    public void setEncoders() {
        frontLeft.setEncoders(RobotConstants.frontLeftAngleOffset);
        frontRight.setEncoders(RobotConstants.frontRightAngleOffset);
        backLeft.setEncoders(RobotConstants.backLeftAngleOffset);
        backRight.setEncoders(RobotConstants.backRightAngleOffset);
    }

    public void resetNavX() {
        navX.reset();
    }
}