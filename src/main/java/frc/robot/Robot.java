package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.swervedrive.SwerveDrive;
import frc.robot.wrappers.JoystickWrapper;
import frc.robot.wrappers.XboxControllerWrapper;

public class Robot extends TimedRobot {
    enum States {
    AUTONOMOUS,
    MANUAL,
    DETECT_BALL,
    MOVE_TO_BALL,
    PICK_UP_BALL,
    GO_TO_HUB,
    DROP_BALL,
    AIM,
    SHOOT,
    GO_TO_HUMAN; 


  }

  States state;
  private JoystickWrapper leftStick;
  private JoystickWrapper rightStick;
  private XboxControllerWrapper xboxController;

  private boolean lockWheels;
  private boolean resetNavX;
  private boolean fieldOriented;

  private double regularSpeed;
  private double boostedSpeed; 

  SwerveDrive swerveDrive;
  Lidar lidar;
  
  private final Balancer balancer;

  private final AHRS navX;
  private final ArmNavX armNavX;


  public Robot() {
    super(0.03);
        leftStick = new JoystickWrapper(0);
    rightStick = new JoystickWrapper(1);
    xboxController = new XboxControllerWrapper(0);
                state = States.MANUAL;

    navX = new AHRS(SPI.Port.kMXP);
    armNavX = new ArmNavX();
    swerveDrive = new SwerveDrive(navX);
    balancer = new Balancer(swerveDrive, navX);
    lidar = new Lidar(RobotConstants.lidarPort);
    armlidar = new Lidar(RobotConstants.armLidarPort);

    regularSpeed = RobotConstants.regularSpeed;
    boostedSpeed = RobotConstants.boostedSpeed;

    fieldOriented = RobotConstants.fieldOriented;

  }
  @Override
  public void robotInit() {
  }

  @Override
  public void teleopInit() {
        swerveDrive.resetNavX();
    swerveDrive.setEncoders();
  }
  @Override
  public void teleopPeriodic() {
    double xAxis;
    double yAxis;
    double rAxis;

    SmartDashboard.putNumber("Lidar data", lidar.getDistanceCentimeters());
    SmartDashboard.putNumber("Arm NavX angle", armNavX.getPitch());

        if (RobotConstants.controller == RobotConstants.ControllerType.XBOX) {
      xAxis = xboxController.getLeftX();
      yAxis = xboxController.getLeftY();
      rAxis = xboxController.getRightX();
      lockWheels = xboxController.getAButton();
      resetNavX = xboxController.getStartButton();
      
    }
    else if (RobotConstants.controller == RobotConstants.ControllerType.JOYSTICK) {
      xAxis = leftStick.getRawAxis(0);
      yAxis = leftStick.getRawAxis(1);
      rAxis = leftStick.getRawAxis(3);
      resetNavX = rightStick.getRawButton(4);
    }
    else {
      xAxis = 0;
      yAxis = 0;
      rAxis = 0;
    }

    if (resetNavX) {
      swerveDrive.resetNavX();
      swerveDrive.setEncoders();
    }

    double x,y,r,speedIncrease;
    speedIncrease = regularSpeed;

                x = -(Math.abs(xAxis)*xAxis) * speedIncrease;
    y= Math.abs(yAxis)*yAxis * speedIncrease;
    r= Math.abs(rAxis)*rAxis * speedIncrease;

    if (xboxController.getBackButtonPressed()) {
      fieldOriented = !fieldOriented;
    }

    if (lockWheels) {
      swerveDrive.drive(0.01, 0, 0, true);
    }

    swerveDrive.drive(x, y, r, true);
  }

  @Override
  public void autonomousInit() {
    swerveDrive.resetNavX();
    swerveDrive.setEncoders();
    balancer.reset();
  }

  @Override
  public void autonomousPeriodic() {
    balancer.balance();
  }
}