// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Robot.java
package frc.robot;

import javax.net.ssl.CertPathTrustManagerParameters;
import javax.xml.transform.SourceLocator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//Documentation: first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/(last_part).html
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive; //might not be needed if we have SwerveDrive working
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotConstants;
import frc.robot.swervedrive.SwerveDrive;
import frc.robot.swervedrive.Wheel;
import frc.robot.wrappers.*;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import com.ctre.phoenix.motorcontrol.ControlMode;

//import com.cuforge.libcu.Lasershark;

//Xbox support
import edu.wpi.first.wpilibj.XboxController;

//import edu.wpi.first.wpilibj.Ultrasonic;

import edu.wpi.first.wpilibj.Solenoid;

//class Camera;
/**sssP
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //define variables
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

  enum ElevatorStates {
    TOP,
    MIDDLE,
    BOTTOM,
    STOP,
    MANUAL;
  }

  private CompressorWrapper compressor;
  private final SolenoidWrapper intakeSolenoid;
  private final SolenoidWrapper elevatorSolenoid;
  

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

  boolean turnTo = false;

  public Robot() {
    super(0.03);
    navX = new AHRS(SPI.Port.kMXP);
    //create variables
    leftStick = new JoystickWrapper(0);
    rightStick = new JoystickWrapper(1);
    xboxController = new XboxControllerWrapper(0);
    //topCam = new Camera();
    //bottomCam = new Camera();
    //Ball = new Ball();
    state = States.MANUAL;

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


  }


  @Override
  public void robotInit() {
    //set to defaults
    //compressor.enableDigital();
    
    // intakeSolenoid.set(false);

    // elevatorState = ElevatorStates.STOP;
    // leftElevatorMotor.setNeutralMode(NeutralMode.Brake);
    // rightElevatorMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void teleopInit() {
    // ballCamera = new CameraWrapper(true);
    swerveDrive.resetNavX();
    swerveDrive.setEncoders();
  }

  

  @Override
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
    else if (controllerType == 1) {
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

    if (elevTop) elevatorState = ElevatorStates.TOP;
    else if (elevMiddle) elevatorState = ElevatorStates.MIDDLE;
    else if (elevBottom) elevatorState = ElevatorStates.BOTTOM;

    if (resetNavX) {
      swerveDrive.resetNavX();
      swerveDrive.setEncoders();
    }

    double x,y,r,speedIncrease;
    speedIncrease = regularSpeed;

    // if(speedIncreaseButton){
    //   speedIncrease = boostedSpeed;
    // }
    x = -(Math.abs(xAxis)*xAxis) * speedIncrease;
    y= Math.abs(yAxis)*yAxis * speedIncrease;
    r= Math.abs(rAxis)*rAxis * speedIncrease;

    if (xboxController.getBackButtonPressed()) {
      fieldOriented = !fieldOriented;
    }

    swerveDrive.drive(x, y, r, true);
  }

  

  @Override
  public void autonomousInit() {
    swerveDrive.resetNavX();
    swerveDrive.setEncoders();
  }

  @Override
  public void autonomousPeriodic() {
    balancer.balance();
  }
}