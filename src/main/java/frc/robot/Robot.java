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
  private boolean arm3;
  private boolean arm2;
  private boolean arm1;
  private boolean fieldOriented;
  private boolean clawOpen;
  private boolean clawClose;
  private boolean armExtend = false;
  private int dpadValue;

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

    boolean clawState = true; //true is open, false is closed


    //SmartDashboard.putNumber("Lidar data", lidar.getDistanceCentimeters());
    SmartDashboard.putNumber("Arm Lidar data", armLidar.getDistanceCentimeters());
    SmartDashboard.putNumber("Arm NavX angle", armNavX.getPitch());
    SmartDashboard.putString("State", state.toString());
    SmartDashboard.putNumber("Arm Encoder", arm.getEncoder());

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

      arm3 = xboxController.getYButton();
      arm2 = xboxController.getBButton();
      arm1 = xboxController.getAButton();//also used for stateManual
      clawOpen = xboxController.getLeftTriggerAxis() > 0.7;
      clawClose = xboxController.getRightTriggerAxis() > 0.7;
      dpadValue = xboxController.getPOV();
      if (xboxController.getRightBumper() && (armNavX.getPitch() < 55)) {
        armExtend = true;
      }
      else if (xboxController.getLeftBumper()) {
        armExtend = false;
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

    if (dpadValue == 0 || dpadValue == 180) {
      armState = ArmStates.MANUAL;
    }
    else if (arm3) {
      armState = ArmStates.UP;

    }
    else if (arm2) {
      armState = ArmStates.MIDDLE;
    }
    else if (arm1) {
        armState = ArmStates.DOWN;
        //xboxController.setRumble(0.2);
        //Timer.delay(0.5);
        //xboxController.setRumble(0);
    }

    if (clawClose || armNavX.getPitch() > 70) {
      arm.closeClaw();
    }
    else if (clawOpen && armNavX.getPitch() < 70) {
      arm.openClaw();
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

    if (armExtend) {
      arm.extendArm();
    }
    else {
      arm.retractArm();
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

    switch(armState) {
      case IDLE:
        idle();
        break;
      case UP:
        up();
        break;
      case MIDDLE:
        middle();
        break;
      case DOWN:
        down();
        break;
      case MANUAL:
        manual();
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
    //aligner.align(armState);
  }
  private void balance() {
    //balancer.balance();
  }

  private void idle() {
    arm.idle();
  }

  private void up() {
    if(arm.up()){
      armState = ArmStates.IDLE;
    }
  }

  private void middle() {
    if(arm.middle()) {
      armState = ArmStates.IDLE;
    }
  }

  private void down() {
    if(arm.down()) {
      armState = ArmStates.IDLE;
    }
  }

  private void manual() {
    if (dpadValue == 0) {
      arm.moveUp();
    }
    else if (dpadValue == 180 && ((armNavX.getPitch() < 55) || !armExtend)) {//check max for extended arm
      arm.moveDown();
    }
    else {
      arm.idle();
    }
  }
  @Override
  public void autonomousInit() {
    swerveDrive.resetNavX();
    swerveDrive.setEncoders();
    balancer.reset();
  }

  @Override
  public void autonomousPeriodic() {
    autonomous.autonomous();
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