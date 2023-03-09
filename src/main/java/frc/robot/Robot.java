package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
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

   enum ArmStates {
    IDLE,
    DOWN,
    MIDDLE,
    UP,
    MANUAL
  }

  States state;

  ArmStates armState;
  private JoystickWrapper leftStick;
  private JoystickWrapper rightStick;
  private XboxControllerWrapper xboxController;

  private boolean lockWheels;
  private boolean stateManual;
  private boolean stateAlign;
  private boolean stateBalance;
  private boolean resetNavX;
  private boolean armTop;
  private boolean armMiddle;
  private boolean armBottom;
  private boolean fieldOriented;
  private boolean clawOpen;
  private boolean clawClose;
  private boolean armOpen = false;
  private boolean armClose = true;
  private boolean armUp;
  private boolean armDown;

  private double regularSpeed;
  private double boostedSpeed; 

  SwerveDrive swerveDrive;
  Lidar lidar;
  Lidar armLidar;

  Limelight limelight;
  private Balancer balancer;
  private Aligner aligner;
  private Autonomous autonomous;

  private final AHRS navX;
  private final ArmNavX armNavX;


  Solenoid armSolenoid = new Solenoid(4, 3, 2); // change to correct values
  Solenoid clawSolenoid = new Solenoid(4, 1, 0);
  Arm arm;

  public Robot() {
    super(0.03);
    //leftStick = new JoystickWrapper(0);//uncomment if using them
    //rightStick = new JoystickWrapper(1);
    xboxController = new XboxControllerWrapper(0);

    navX = new AHRS(SPI.Port.kMXP);
    armNavX = new ArmNavX(4);
    swerveDrive = new SwerveDrive(navX);
    //lidar = new Lidar(RobotConstants.lidarPort);
    armLidar = new Lidar(RobotConstants.armLidarPort);
    limelight = new Limelight(0);

    balancer = new Balancer(swerveDrive, navX);
    aligner = new Aligner(swerveDrive, limelight, lidar, arm);
    autonomous = new Autonomous(balancer);

    regularSpeed = RobotConstants.regularSpeed;
    boostedSpeed = RobotConstants.boostedSpeed;

    fieldOriented = RobotConstants.fieldOriented;

    arm = new Arm(armSolenoid, clawSolenoid, armNavX);
  }

  @Override
  public void robotInit() {
  }

  @Override
  public void teleopInit() {
    state = States.MANUAL;
    armState = ArmStates.IDLE;

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

    //SmartDashboard.putNumber("Lidar data", lidar.getDistanceCentimeters());
    SmartDashboard.putNumber("Arm Lidar data", armLidar.getDistanceCentimeters());
    SmartDashboard.putNumber("Arm NavX angle", armNavX.getPitch());
    SmartDashboard.putString("State", state.toString());
    //SmartDashboard.putNumber("Arm Encoder", arm.getEncoder());

    limelight.readPeriodic();


    if (RobotConstants.controller == RobotConstants.ControllerType.XBOX) {
      xAxis = xboxController.getLeftX();
      yAxis = xboxController.getLeftY();
      rAxis = xboxController.getRightX();

      lockWheels = xboxController.getXButton();
      resetNavX = xboxController.getStartButton();
      stateManual = xboxController.getAButton();
      //stateAlign = xboxController.getRightBumper();
      //stateBalance = xboxController.getLeftBumper();

      armTop = xboxController.getYButton();
      armMiddle = xboxController.getBButton();
      armTop = xboxController.getAButton(); // also used for stateManual
      clawOpen = xboxController.getLeftTriggerAxis() > 0.7;
      clawClose = xboxController.getRightTriggerAxis() > 0.7;

      if (xboxController.getPOV() == 0) {
        armUp = true;
        armDown = false;
      } else if (xboxController.getPOV() == 180) {
        armDown = true;
        armUp = false;
      } else {
        armUp = false;
        armDown = false;
      }

      armOpen = xboxController.getRightBumper();
      armClose = xboxController.getLeftBumper();
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
    runArm();
  }

  public void runArm() {
    if (armTop) {
      arm.setUp();
    } else if (armMiddle) {
      arm.setMiddle();
    } else if (armBottom) {
      arm.setBottom();
    }

    if (armUp) {
      
    } else {

    }
  }

  private void manual(double x, double y, double r) {
    swerveDrive.drive(x, y, r, true);
  }

  private void align() {
    //aligner.align(armState);
  }

  private void balance() {
    //balancer.balance();
  }

  @Override
  public void autonomousInit() {
    swerveDrive.resetNavX();
    swerveDrive.setEncoders();
    balancer.reset();
  }

  @Override
  public void autonomousPeriodic() {
    //autonomous.autonomous();
    balancer.balance();
  }

  Limelight reflective_tape;
  boolean pos = false;
  double iterate = 500;
  double thresh = 3;

  @Override 
  public void testInit() {
    swerveDrive.setEncoders();
  }

  @Override
  public void testPeriodic() {
  }
}