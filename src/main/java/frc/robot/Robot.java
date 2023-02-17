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

  enum States {
      MANUAL,
      ALIGN,
      BALANCE
  }

  States state;
  private JoystickWrapper leftStick;
  private JoystickWrapper rightStick;
  private XboxControllerWrapper xboxController;

  private boolean lockWheels;
  private boolean stateManual;
  private boolean stateAlign;
  private boolean stateBalance;
  private boolean resetNavX;
  private boolean arm3;
  private boolean arm2;
  private boolean arm1;
  private boolean clawFalse;
  private boolean clawTrue;
  private boolean fieldOriented;

  private double regularSpeed;
  private double boostedSpeed; 

  SwerveDrive swerveDrive;
  Lidar lidar;
  Lidar armLidar;

  Limelight limelight;
  private final Balancer balancer;
  private final Aligner aligner;

  private TalonFXWrapper arm;

  private final AHRS navX;
  private final ArmNavX armNavX;


  public Robot() {
    super(0.03);
    //leftStick = new JoystickWrapper(0);//uncomment if using them
    //rightStick = new JoystickWrapper(1);
    xboxController = new XboxControllerWrapper(0);

    navX = new AHRS(SPI.Port.kMXP);
    armNavX = new ArmNavX(4.0);
    swerveDrive = new SwerveDrive(navX);
    lidar = new Lidar(RobotConstants.lidarPort);
    armLidar = new Lidar(RobotConstants.armLidarPort);
    limelight = new Limelight();

    balancer = new Balancer(swerveDrive, navX);
    aligner = new Aligner(swerveDrive, limelight, lidar);

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
    aligner.reset();
    balancer.reset();
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
    SmartDashboard.putString("State", state.toString());

    limelight.readPeriodic();

    if (RobotConstants.controller == RobotConstants.ControllerType.XBOX) {
      xAxis = xboxController.getLeftX();
      yAxis = xboxController.getLeftY();
      rAxis = xboxController.getRightX();

      lockWheels = xboxController.getXButton();
      resetNavX = xboxController.getStartButton();
      stateManual = xboxController.getAButton();
      stateAlign = xboxController.getRightBumper();
      stateBalance = xboxController.getLeftBumper();

      arm3 = xboxController.getYButton();
      arm2 = xboxController.getBButton();
      arm1 = xboxController.getAButton();//also used for stateManual
      clawFalse = xboxController.getRightTriggerAxis() > 0.7;
      clawTrue = xboxController.getLeftTriggerAxis() > 0.7;
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

    if (arm3) {
      armState = 3;
    }
    else if (arm2) {
      armState = 2;
    }
    else if (arm1) {
      armState = 1;
    }

    if (clawFalse) {
      clawState = false;
    }
    else if (clawTrue) {
      clawState = true;
    }

    if (resetNavX) {
      swerveDrive.resetNavX();
      swerveDrive.setEncoders();
    }

    if (xboxController.getBackButtonPressed()) {
      fieldOriented = !fieldOriented;
    }

    if (lockWheels) {
      swerveDrive.drive(0.01, 0, 0, true);
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
      case BALANCE:
        balance();
        break;
    }

    if (stateManual) {
      state = States.MANUAL;
    }
    else if (stateAlign) {
      state = States.ALIGN;
    }
    else if (stateBalance) {
      state = States.BALANCE;
    }
  }

  private void manual(double x, double y, double r) {
    swerveDrive.drive(x, y, r, true);
  }
  private void align() {
    aligner.align();
  }
  private void balance() {
    balancer.balance();
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

  BaseLimelight reflective_tape;
  boolean pos = false;
  double iterate = 500;
  double thresh = 3;
  @Override public void testInit() {
    reflective_tape = new BaseLimelight(0);
  }
  @Override public void testPeriodic() {
    double x = 0.0;
    reflective_tape.display();
    double r = reflective_tape.get_rotation();
    SmartDashboard.putNumber("RequiredRotation", r);
    if (r > thresh) {x = 0.05;}
    if (r < -thresh) {x = -0.05;}
    swerveDrive.drive(-x, 0, 0, false);
  }
}