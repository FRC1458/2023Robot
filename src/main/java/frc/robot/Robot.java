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
      MANUAL,
      ALIGN
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
  Lidar armLidar;

  Limelight limelight;
  private final Balancer balancer;

  private final AHRS navX;
  private final ArmNavX armNavX;


  public Robot() {
    super(0.03);
    leftStick = new JoystickWrapper(0);
    rightStick = new JoystickWrapper(1);
    xboxController = new XboxControllerWrapper(0);

    navX = new AHRS(SPI.Port.kMXP);
    armNavX = new ArmNavX();
    swerveDrive = new SwerveDrive(navX);
    balancer = new Balancer(swerveDrive, navX);
    lidar = new Lidar(RobotConstants.lidarPort);
    armLidar = new Lidar(RobotConstants.armLidarPort);
    limelight = new Limelight();

    regularSpeed = RobotConstants.regularSpeed;
    boostedSpeed = RobotConstants.boostedSpeed;

    fieldOriented = RobotConstants.fieldOriented;

  }
  @Override
  public void robotInit() {
  }

  @Override
  public void teleopInit() {
    state = States.MANUAL;
    swerveDrive.resetNavX();
    swerveDrive.setEncoders();
  }
  @Override
  public void teleopPeriodic() {
    double xAxis;
    double yAxis;
    double rAxis;
    double x,y,r,speedIncrease;
    speedIncrease = regularSpeed;
    int armState = 1; //1 is bottom, 2 is middle, 3 is lifted all the way
    boolean clawState = true; //true is open, false is closed

    SmartDashboard.putNumber("Lidar data", lidar.getDistanceCentimeters());
    SmartDashboard.putNumber("Arm Lidar data", armLidar.getDistanceCentimeters());
    SmartDashboard.putNumber("Arm NavX angle", armNavX.getPitch());

    limelight.readPeriodic();

    if (RobotConstants.controller == RobotConstants.ControllerType.XBOX) {
      xAxis = xboxController.getLeftX();
      yAxis = xboxController.getLeftY();
      rAxis = xboxController.getRightX();
      lockWheels = xboxController.getXButton();
      resetNavX = xboxController.getStartButton();
      //why not have it swap arm states using a single button?
      //I used else-if statements instead of just if statements like originally
      if (xboxController.getYButton()) {
        armState = 3;
      }
      else if (xboxController.getBButton()) {
        armState = 2;
      }
      else if (xboxController.getAButton()) {
        armState = 1;
      }
      //why not just have clawState = true when a button/trigger is pressed, and false otherwise?
      if (xboxController.getRightTriggerAxis() > 0.7) {
        clawState = false;
      }
      else if (xboxController.getLeftTriggerAxis() > 0.7) {
        clawState = true;
      }
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

    if (xboxController.getRightBumper()) {
      state = States.ALIGN;
    }
    if (xboxController.getLeftBumper()) {
      state = States.ALIGN;
    }
    }

    x = -(Math.abs(xAxis)*xAxis) * speedIncrease;
    y= Math.abs(yAxis)*yAxis * speedIncrease;
    r= Math.abs(rAxis)*rAxis * speedIncrease;

    switch(state) {
      case MANUAL:
        manual(x, y, r);
        break;
      case ALIGN:
        align();
        break;
    }

    if (xboxController.getBackButtonPressed()) {
      fieldOriented = !fieldOriented;
    }

    if (lockWheels) {
      swerveDrive.drive(0.01, 0, 0, true);
    }
  }

  private void manual(double x, double y, double r) {
    swerveDrive.drive(x, y, r, true);
  }

  private void align() {
    swerveDrive.drive(0, (limelight.getYOffset()/1000000), 0, true);
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