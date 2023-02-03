// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

<<<<<<< Updated upstream
import frc.robot.RobotConstants;
import frc.robot.swervedrive.SwerveDrive;
import frc.robot.swervedrive.Wheel;
import frc.robot.wrappers.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI; //needed to initialize navx
import edu.wpi.first.wpilibj.I2C; //needed to initialize armNavX

import com.ctre.phoenix.motorcontrol.ControlMode;

//import com.cuforge.libcu.Lasershark;
=======
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
>>>>>>> Stashed changes

class Lasershark implements Sendable {

  private DutyCycle _pwmInput;


  public Lasershark(int input) {
      this._pwmInput = new DutyCycle(new DigitalInput(input));
      SendableRegistry.addLW(this, "Lasershark", _pwmInput.getFPGAIndex() + 1);
  }

  public Lasershark(DigitalSource source) {
      this._pwmInput = new DutyCycle(source);
      SendableRegistry.addLW(this, "Lasershark", _pwmInput.getFPGAIndex() + 1);
  }

  public double getDistanceFeet() {
      return this._pwmInput.getOutput() * 4000 / 25.4 / 12;
  }

  public double getDistanceInches() {
      return this._pwmInput.getOutput() * 4000 / 25.4;
  }

  public double getDistanceCentimeters() {
      return this._pwmInput.getOutput() * 4000 / 10.0;
  }
  
<<<<<<< Updated upstream

  States state;
  ElevatorStates elevatorState;
  // private TalonSRXWrapper leftMotor;
  // private TalonSRXWrapper leftMotor2;
  // private TalonSRXWrapper rightMotor;
  // private TalonSRXWrapper rightMotor2;
  private JoystickWrapper leftStick;
  private JoystickWrapper rightStick;
  private XboxControllerWrapper xboxController;
  private double leftAxis;
  private double rightAxis;

  private double elevatorSpeed;

  //private CameraWrapper camera;

  private boolean lockWheels;
  private boolean depositButton;
  private boolean IntakePulleyButton;
  private boolean elevatorUpButton;
  private boolean elevatorDownButton;
  private boolean climbButton;
  private boolean dropBall;
  private boolean resetNavX;
  private boolean speedIncreaseButton;
  private boolean pulleyButton;
  private boolean stopElevatorButton;
  private boolean fieldOriented;
  private boolean intakeOn = false;

  private boolean PulleyInUse;
  private boolean depositor;
  private boolean climb;
  private boolean depositorBool;
  private boolean climbBool;
  private boolean climbCounterBool;

  private boolean intakeButton;
  private boolean intakeReverseButton;
  private boolean intakeForwardButton;

  private boolean oldIntakeButton;

  private boolean intakeSolenoidButton;
  private boolean elevSolenoidEngageButton;
  private boolean elevSolenoidDisengageButton;

  private boolean elevatorManualUp;
  private boolean elevatorManualDown;

  private boolean elevTop;
  private boolean elevMiddle;
  private boolean elevBottom;

  private double elevSpeedUp;
  private double elevSpeedDown;
  private double elevSpeedManual;

  private int counter;
  private int climbCounter;

  private boolean engagedElevator = false;

  private double regularSpeed;
  private double boostedSpeed; 

  SwerveDrive swerveDrive;

  int controllerType; 

  private DigitalInput bottomLimitSwitch;
  private DigitalInput middleLimitSwitch;
  private DigitalInput topLimitSwitch;

  private boolean middleBool = false;

  private boolean AUTO_taxi = false;
  private boolean AUTO_depositorDone = false;

  private final Balancer balancer;

  private final AHRS navX;
  private final AHRS armNavX;

  boolean turnTo = false;

  public Robot() {
    super(0.03);
    //create variables
    leftStick = new JoystickWrapper(0);
    rightStick = new JoystickWrapper(1);
    xboxController = new XboxControllerWrapper(0);
    //topCam = new Camera();
    //bottomCam = new Camera();
    //Ball = new Ball();
    state = States.MANUAL;

    navX = new AHRS(SPI.Port.kMXP);
    armNavX = new AHRS(I2C.Port.kMXP);
    swerveDrive = new SwerveDrive(navX);
    balancer = new Balancer(swerveDrive, navX);
    // ballCamera = new CameraWrapper(true);

    //navx = new NavX();

    //shark = new Lasershark(0);

    // ultrasound = new UltrasoundWrapper(0, 1);

    compressor = new CompressorWrapper();

    intakeSolenoid = new SolenoidWrapper(RobotConstants.intakeSolenoidForwardID, RobotConstants.intakeSolenoidReverseID);
    elevatorSolenoid = new SolenoidWrapper(RobotConstants.elevatorSolenoidForwardID, RobotConstants.elevatorSolenoidReverseID);
   
    elevSpeedUp = RobotConstants.elevSpeedUp;
    elevSpeedDown = RobotConstants.elevSpeedDown;
    elevSpeedManual = RobotConstants.elevSpeedManual;
    regularSpeed = RobotConstants.regularSpeed;
    boostedSpeed = RobotConstants.boostedSpeed;

    fieldOriented = RobotConstants.fieldOriented;


=======
  public double getDistanceMeters() {
      return this._pwmInput.getOutput() * 4000 / 1000.0;
>>>>>>> Stashed changes
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("Lasershark");
      builder.addDoubleProperty("Distance (ft)", this::getDistanceFeet, null);
  }
}

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
<<<<<<< Updated upstream
  public void teleopPeriodic() {
    double xAxis;
    double yAxis;
    double rAxis;


    //SET CONTROLLER TYPE HERE
    //SET TO 0 FOR XBOX CONTROLLER
    //SET TO 1 FOR EVERYTHING ELSE

    controllerType = 0;

    //Controllers222222222%
    if (controllerType == 0) {
      xAxis = xboxController.getLeftX();
      yAxis = xboxController.getLeftY();
      rAxis = xboxController.getRightX();
      lockWheels = xboxController.getAButton();
      //depositButton = xboxController.getAButton();
      //pulleyButton = xboxController.getRightBumper();
      //elevatorDepositButton = xboxController.getBButton();
      //climbButton = xboxController.getXButton();
      //elevatorClimbButton = xboxController.getYButton();
      intakeButton = xboxController.getRightBumper() && !xboxController.getLeftBumper() && !elevTop;
      intakeReverseButton = (xboxController.getRightTriggerAxis() > 0.7);
      intakeForwardButton = (xboxController.getLeftTriggerAxis() > 0.7) && !xboxController.getRightBumper();
      
      depositButton = xboxController.getLeftBumper() && !xboxController.getRightBumper() && !elevTop;

      // dropBall = xboxController.getRightBumper();
      resetNavX = xboxController.getStartButton();
      //speedIncreaseButton = xboxController.getRightTriggerAxis() > 0.7;

      elevTop = (xboxController.getPOV() == 0) && xboxController.getStartButton() && xboxController.getLeftBumper();
      elevMiddle = (xboxController.getPOV() == 0);
      elevBottom = (xboxController.getPOV() == 180);
      

      elevSolenoidEngageButton = xboxController.getXButton();
      elevSolenoidDisengageButton = xboxController.getYButton();

      intakeSolenoidButton = xboxController.getBButton();

      stopElevatorButton = xboxController.getAButton();

      elevatorManualUp = xboxController.getPOV() == 90;
      elevatorManualDown = xboxController.getPOV() == 270;
    }
    else if (controllerType == 1) { // Joysticks
      xAxis = leftStick.getRawAxis(0);
      yAxis = leftStick.getRawAxis(1);
      rAxis = leftStick.getRawAxis(3);
      depositButton = leftStick.getRawButton(0);
      elevatorUpButton = leftStick.getRawButton(1);
      climbButton = leftStick.getRawButton(2);
      elevatorDownButton = leftStick.getRawButton(3);
      speedIncreaseButton = rightStick.getRawButton(7);
      resetNavX = rightStick.getRawButton(4); //Navx Reset coded already -Taven
    }
    else {
      xAxis = 0;
      yAxis = 0;
      rAxis = 0;
    }
    // Change for current elevator (no middle state)
    if (elevTop) elevatorState = ElevatorStates.TOP;
    else if (elevMiddle) elevatorState = ElevatorStates.MIDDLE;
    else if (elevBottom) elevatorState = ElevatorStates.BOTTOM;

    if (resetNavX) {
      swerveDrive.resetNavX();
      swerveDrive.setEncoders();
    }

    double x,y,r,speedIncrease;
    speedIncrease = 0.25; // regularSpeed (variable)
=======
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  Lasershark shark = new Lasershark(0);
  @Override
  public void teleopPeriodic() {
    double dist = shark.getDistanceInches();
    SmartDashboard.putNumber("Distance", dist);
  }
>>>>>>> Stashed changes

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

<<<<<<< Updated upstream
    if (lockWheels) {
      swerveDrive.drive(0.01, 0, 0, true);
    }

    swerveDrive.drive(x, y, r, true);
  }
=======
  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}
>>>>>>> Stashed changes

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
