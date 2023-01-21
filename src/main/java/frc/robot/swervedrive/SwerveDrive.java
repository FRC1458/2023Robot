package frc.robot.swervedrive;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.RobotConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.wrappers.*;

public class SwerveDrive {

    ChassisSpeeds speeds;
    public final Wheel frontLeft;
    public final Wheel frontRight;
    public final Wheel backLeft;
    public final Wheel backRight;
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;
    Pose2d pose;
    AHRS ahrs;
    //ahrs = new AHRS(NavX, kProcessedData, 50);

    public SwerveDrive() {
        frontLeft = new Wheel(RobotConstants.frontLeftAngleID, RobotConstants.frontLeftSpeedID, RobotConstants.frontLeftAbsoluteEncoderID, "Front Left (1)", RobotConstants.frontLeftAngleOffset);
        frontRight = new Wheel(RobotConstants.frontRightAngleID, RobotConstants.frontRightSpeedID, RobotConstants.frontRightAbsoluteEncoderID, "Front Right (2)", RobotConstants.frontRightAngleOffset);
        backLeft = new Wheel(RobotConstants.backLeftAngleID, RobotConstants.backLeftSpeedID, RobotConstants.backLeftAbsoluteEncoderID, "Back Left (3)", RobotConstants.backLeftAngleOffset);
        backRight = new Wheel(RobotConstants.backRightAngleID, RobotConstants.backRightSpeedID, RobotConstants.backRightAbsoluteEncoderID, "Back Right (4)", RobotConstants.backRightAngleOffset);

        // Locations for the swerve drive modules relative to the robot center.
        Translation2d frontLeftLocation = new Translation2d(RobotConstants.frontLeftXMeters, RobotConstants.frontLeftYMeters);
        Translation2d frontRightLocation = new Translation2d(RobotConstants.frontRightXMeters, RobotConstants.frontRightYMeters);
        Translation2d backLeftLocation = new Translation2d(RobotConstants.backLeftXMeters, RobotConstants.backLeftYMeters);
        Translation2d backRightLocation = new Translation2d(RobotConstants.backRightXMeters, RobotConstants.backRightYMeters);

        // Positions for each wheel
        SwerveModulePosition frontLeftPosition = new SwerveModulePosition(RobotConstants.frontLeftDistance, RobotConstants.frontLeftAngle);
        SwerveModulePosition frontRightPosition = new SwerveModulePosition(RobotConstants.frontRightDistance, RobotConstants.frontRightAngle);
        SwerveModulePosition backLeftPosition = new SwerveModulePosition(RobotConstants.backLeftDistance, RobotConstants.backLeftAngle);
        SwerveModulePosition backRightPosition = new SwerveModulePosition(RobotConstants.backRightDistance, RobotConstants.backRightAngle);
        SwerveModulePosition[] positions = {frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition}; 
        //Need to add these constants to RobotConstants.java!!!!

        // Creating my kinematics object using the module locations
        kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
        
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), positions, new Pose2d(5.0, 13.5, new Rotation2d()));

        speeds = new ChassisSpeeds();

        //ahrs = new AHRS(SPI.Port.kMXP); //REMEM
    }

    public void drive(double x, double y, double r, boolean fieldOriented) {
        speeds.vxMetersPerSecond = x;
        speeds.vyMetersPerSecond = y;
        speeds.omegaRadiansPerSecond = r;

        if (fieldOriented) {
            //speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, r, Rotation2d.fromDegrees(-(ahrs.getYaw())));
        }
        //SmartDashboard.putNumber("angle from navx", ahrs.getYaw());

        SmartDashboard.putNumber("X", x);
        SmartDashboard.putNumber("Y", y);
        SmartDashboard.putNumber("R", r);
        //SmartDashboard.putNumber("Robot Angle", ahrs.getYaw());

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        //Set to angle that we get from the NavX
        //double angle = 0;

        //Rotation2d gyroAngle = Rotation2d.fromDegrees(angle);

        // Update the pose
        //pose = odometry.update(gyroAngle, moduleStates[0], moduleStates[1], moduleStates[2], moduleStates[3]);

        frontLeft.drive(states[2].speedMetersPerSecond, states[2].angle.getDegrees());
        frontRight.drive(states[0].speedMetersPerSecond, states[0].angle.getDegrees());
        backLeft.drive(states[3].speedMetersPerSecond, states[3].angle.getDegrees());
        backRight.drive(states[1].speedMetersPerSecond, states[1].angle.getDegrees());

        // SmartDashboard.putNumber("Back Left goal angle", states[3].angle.getDegrees());
        // SmartDashboard.putNumber("Back Left actual angle", backLeft.absoluteEncoder.getPosition() * (360 / RobotConstants.swerveDriveGearRatio) % 360);
        // SmartDashboard.putNumber("Difference", (states[3].angle.getDegrees() - (backLeft.absoluteEncoder.getPosition() * (360 / RobotConstants.swerveDriveGearRatio) % 360) ) % 360);
    }

    public double turnToAngle(double goalAngle) {
        double error = 1;
        double currentAngle = 0;//ahrs.getYaw();

        double diff = (currentAngle - goalAngle) % 360;

        if (Math.abs(diff) > 180) {
          diff = diff - 360*Math.signum(diff); // add or subtract 360 so the difference is always smaller than 180
        }

        double realGoalAngle = (currentAngle - diff);

        if (Math.abs(currentAngle - realGoalAngle) > error) {
            if (currentAngle > realGoalAngle) {
                return -.5;
            }
            else {
                return .5;
            }
        }
        return 0;

    }


    public void readAbsoluteEncoder() {
        SmartDashboard.putNumber("front left absolute", frontLeft.getAbsoluteEncoderValue());
        SmartDashboard.putNumber("front right absolute", frontRight.getAbsoluteEncoderValue());
        SmartDashboard.putNumber("back left absolute", backLeft.getAbsoluteEncoderValue());
        SmartDashboard.putNumber("back right absolute", frontRight.getAbsoluteEncoderValue());

    }
    public void resetNavX () {
        //ahrs.reset();
    }
}