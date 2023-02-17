package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.swervedrive.SwerveDrive;
import frc.robot.wrappers.JoystickWrapper;
import frc.robot.wrappers.TalonFXWrapper;
import frc.robot.wrappers.XboxControllerWrapper;

public class Robot extends TimedRobot {
  private TalonFXWrapper arm;
  private Lidar lidar1;
  private Lidar lidar2;
  private AHRS navx;
  private ArmNavX armNavx;
  private Limelight limelight;
  public Robot() {
    super(0.03);
    if (RobotConstants.TalonFX) {
      arm = new TalonFXWrapper(43);//change to motor id
    }
    if (RobotConstants.Lidar1) {
      lidar1 = new Lidar(RobotConstants.lidarPort);
    }
    if (RobotConstants.Lidar2) {
      lidar2 = new Lidar(RobotConstants.armLidarPort);
    }
    if (RobotConstants.navx) {
      navx = new AHRS(SPI.Port.kMXP);
    }
    if (RobotConstants.armNavx) {
      armNavx = new ArmNavX(4.0);
    }
    if (RobotConstants.limelight) {
      limelight = new Limelight();
    }
  }
  @Override
  public void robotInit() {
  }
  @Override
  public void teleopInit() {
  }
  @Override
  public void teleopPeriodic() {
    if (RobotConstants.TalonFX){
      arm.set(-0.2);
    }
    if (RobotConstants.Lidar1){
      SmartDashboard.putNumber("Lidar1 (cm)", lidar1.getDistanceCentimeters());
    }
    if (RobotConstants.Lidar2){
      SmartDashboard.putNumber("Lidar2 (cm)", lidar2.getDistanceCentimeters());
    }
    if (RobotConstants.navx){
    }
    if (RobotConstants.armNavx){
    }
    if (RobotConstants.limelight){

    }
  }
  @Override
  public void autonomousInit() {
  }
  @Override
  public void autonomousPeriodic() {
  }
  @Override public void testInit() {
  }
  @Override public void testPeriodic() {
  }
}