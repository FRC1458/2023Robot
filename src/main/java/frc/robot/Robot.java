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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotConstants;
import frc.robot.swervedrive.SwerveDrive;
import frc.robot.swervedrive.Wheel;
import frc.robot.wrappers.*;


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
  // private boolean elevatorDepositButton;
  // private boolean elevatorClimbButton;
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

  //sensors
  //private NavX navx;
  // CameraWrapper ballCamera;
  //Lasershark shark;
  // UltrasoundWrapper ultrasound;

  SwerveDrive swerveDrive;

  //Set Controller Type
  int controllerType;

  private TalonSRXWrapper intakeMotor;
  private TalonSRXWrapper leftDepositorMotor;
  private TalonSRXWrapper rightDepositorMotor;
  // private TalonSRXWrapper intakePulleyMotor;
  private TalonFXWrapper leftElevatorMotor; //to go up go clockwise
  private TalonFXWrapper rightElevatorMotor; //to go up go counter-clockwise
 

  private DigitalInput bottomLimitSwitch;
  private DigitalInput middleLimitSwitch;
  private DigitalInput topLimitSwitch;

  private boolean middleBool = false;

  private Timer middleTimer;

  private Timer AUTO_totalTimer;
  private Timer AUTO_depositTimer;
  private Timer AUTO_taxiTimer;

  private boolean AUTO_taxi = false;
  private boolean AUTO_depositorDone = false;




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
    swerveDrive = new SwerveDrive();
    // ballCamera = new CameraWrapper(true);

    //navx = new NavX();

    //shark = new Lasershark(0);

    // ultrasound = new UltrasoundWrapper(0, 1);

    compressor = new CompressorWrapper();

    intakeSolenoid = new SolenoidWrapper(RobotConstants.intakeSolenoidForwardID, RobotConstants.intakeSolenoidReverseID);
    elevatorSolenoid = new SolenoidWrapper(RobotConstants.elevatorSolenoidForwardID, RobotConstants.elevatorSolenoidReverseID);
    
    intakeMotor = new TalonSRXWrapper(RobotConstants.intakeMotorID);
    leftDepositorMotor = new TalonSRXWrapper(RobotConstants.leftDepositorMotorID);
    rightDepositorMotor = new TalonSRXWrapper(RobotConstants.rightDepositorMotorID);
    leftElevatorMotor = new TalonFXWrapper(RobotConstants.leftElevatorMotorID);
    rightElevatorMotor = new TalonFXWrapper(RobotConstants.rightElevatorMotorID);
    bottomLimitSwitch = new DigitalInput(RobotConstants.bottomLimitSwitchID);
    middleLimitSwitch = new DigitalInput(RobotConstants.middleLimitSwitchID);
    topLimitSwitch = new DigitalInput(RobotConstants.topLimitSwitchID);

    elevSpeedUp = RobotConstants.elevSpeedUp;
    elevSpeedDown = RobotConstants.elevSpeedDown;
    elevSpeedManual = RobotConstants.elevSpeedManual;
    regularSpeed = RobotConstants.regularSpeed;
    boostedSpeed = RobotConstants.boostedSpeed;

    fieldOriented = RobotConstants.fieldOriented;

    middleTimer = new Timer();
    AUTO_taxiTimer = new Timer();
    AUTO_totalTimer = new Timer();
    AUTO_depositTimer = new Timer();
  }


  @Override
  public void robotInit() {
    //set to defaults
    compressor.enableDigital();
    
    intakeSolenoid.set(false);

    elevatorState = ElevatorStates.STOP;
    leftElevatorMotor.setNeutralMode(NeutralMode.Brake);
    rightElevatorMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void teleopInit() {
    // ballCamera = new CameraWrapper(true);
    swerveDrive.setEncoders();
    swerveDrive.resetNavX();
  }


  @Override
  public void teleopPeriodic() {
    double xAxis;
    double yAxis;
    double rAxis;

    // Elevator stuff
  // if (elevatorDepositButton && !depositorBool) {
  //   moveElevator(1);
  //   depositor = true;
  //   depositorBool = true;
  // } else if (elevatorDepositButton && depositorBool) {
  //   moveElevator(-1);
  //   depositor = true;
  //   depositorBool = false;
  // }
  // if (elevatorClimbButton && !climbBool) {
  //   moveElevator(1);
  //   climb = true;
  //   climbBool = true;
  // } else if (elevatorClimbButton && climbBool) {
  //   moveElevator(-1);
  //   climb = true;
  //   climbBool = false;
  // }
  // if (depositor) {
  //   bottomLimitToMiddleLimit(elevatorSpeed);
  // } else if (climb) {
  //   middleLimitToTopLimit(elevatorSpeed);
  // }

  // if (climbCounterBool) {
  //   counter++;
  // }
  // if (counter == 500) {
  //   moveElevator(0);
  // }

    // SmartDashboard.putNumber("BallX", ballCamera.getBallX());

    //SET CONTROLLER TYPE HERE
    //SET TO 0 FOR XBOX CONTROLLER
    //SET TO 1 FOR EVERYTHING ELSE

    controllerType = 0;

    //Controllers
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
      rAxis = rightStick.getRawAxis(0);
      depositButton = leftStick.getRawButton(0);
      elevatorUpButton = leftStick.getRawButton(1);
      climbButton = leftStick.getRawButton(2);
      elevatorDownButton = leftStick.getRawButton(3);
      speedIncreaseButton = rightStick.getRawButton(7);
      resetNavX = rightStick.getRawButton(4);
    }
    else {
      xAxis = 0;
      yAxis = 0;
      rAxis = 0;
    }

    if (elevTop) elevatorState = ElevatorStates.TOP;
    else if (elevMiddle) elevatorState = ElevatorStates.MIDDLE;
    else if (elevBottom) elevatorState = ElevatorStates.BOTTOM;

    //setting elvator position
    // if (elevTop && topLimitSwitch.get()) {
    //   rightElevatorMotor.set(.2);
    //   leftElevatorMotor.set(-.2);
    // }
    // else if (elevBottom && bottomLimitSwitch.get()) {
    //   rightElevatorMotor.set(-.2);
    //   leftElevatorMotor.set(.2);
    // }


    // // Setting speed of depositor motors
    // if (depositButton) {
    //   leftDepositorMotor.set(0.2);
    //   rightDepositorMotor.set(-0.2);
    //   intakeMotor.set(0.5);
    // }
    // else if (dropBall) {
    //   leftDepositorMotor.set(-0.2);
    //   rightDepositorMotor.set(0.2);
    //   intakeMotor.set(0);
    // }
    // else {
    //   leftDepositorMotor.set(0);
    //   rightDepositorMotor.set(0);
    //   intakeMotor.set(0);
    // }
    // if (xboxController.getRightTriggerAxis() > 0.7) {
    //   leftElevatorMotor.set(1);
    //   rightElevatorMotor.set(-1);
    // } else if (xboxController.getLeftTriggerAxis() > 0.7) {
    //   leftElevatorMotor.set(-1);
    //   rightElevatorMotor.set(1);
    // } else {
    //   leftElevatorMotor.set(0);
    //   rightElevatorMotor.set(0);
    // }
    if (resetNavX) {
      swerveDrive.resetNavX();
      swerveDrive.setEncoders();
    }

    intakeSolenoid.set(intakeSolenoidButton);

    if (elevSolenoidEngageButton) {
      elevatorSolenoid.set(true);
      engagedElevator = true;
    }
    else if(elevSolenoidDisengageButton) {
      elevatorSolenoid.set(false);
      engagedElevator = false;
      elevatorState = ElevatorStates.STOP;
    }

    if (engagedElevator) {
      
      // ethan comment this out when limit switches are working
      if (elevatorManualUp) {
        leftElevatorMotor.set(-elevSpeedManual);
        rightElevatorMotor.set(elevSpeedManual);
        elevatorState = ElevatorStates.MANUAL;
      }
      else if (elevatorManualDown) {
        leftElevatorMotor.set(elevSpeedManual);
        rightElevatorMotor.set(-elevSpeedManual);
        elevatorState = ElevatorStates.MANUAL;
      }
      else if (elevatorState == ElevatorStates.MANUAL) {
        elevatorState = ElevatorStates.STOP;
      }

      switch(elevatorState) {
        case TOP:
          moveToTop();
          break;
        case MIDDLE:
          moveToMiddle();
          break;
        case BOTTOM:
          moveToBottom();
          break;
        case STOP:
          stopElevator();
        case MANUAL:
          break;
      }

      if (stopElevatorButton) {
        elevatorState = ElevatorStates.STOP;
      }
    }
    else {
      leftElevatorMotor.set(0);
      rightElevatorMotor.set(0);
    }

    

    if (intakeButton != oldIntakeButton && intakeButton == true) {
      intakeOn = !intakeOn;
    }

    oldIntakeButton = intakeButton;

    if (intakeForwardButton) {
      intakeMotor.set(.5);
    }

    else if (intakeOn) {
      rightDepositorMotor.set(.2);
      leftDepositorMotor.set(.2);
      intakeMotor.set(.5);
    }
    else if (depositButton) {
      rightDepositorMotor.set(-1);
      leftDepositorMotor.set(-1);
    }
    else if (intakeReverseButton) {
      intakeMotor.set(-.5);
    }
    else {
      intakeMotor.set(0);
      rightDepositorMotor.set(0);
      leftDepositorMotor.set(0);
    }
    
    // if (pulleyButton){
    //   intakePulleyMotor.set(0.5);
    //   PulleyInUse = true;
    // }

    // if (PulleyInUse = true) {
    //   counter = counter + 1;
    // }

    // if (counter == 10) {
    //   intakePulleyMotor.set(0);
    //   PulleyInUse = false;
    // }
    //double distanceToBall = shark.getDistanceCentimeters();
    //SmartDashboard.putNumber("distanceToBall", distanceToBall);

    //navx.operatorControl();

    /*
    double pov = leftStick.getPOV();

    turnTo = (pov != -1) && (rAxis == 0);

    if (turnTo) {
      rAxis = swerveDrive.turnToAngle(pov);
      if (rAxis == 0) {
        turnTo = false;
      }
    }
    */
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

    swerveDrive.drive(x, y, r, fieldOriented);
  }

  // public void bottomLimitToMiddleLimit (double speed) {
  //   if (speed > 0) {
  //     if (!middleLimitSwitch.get()) {
  //       moveElevator(0);
  //     }
  //   }
  //   else {
  //     if (!bottomLimitSwitch.get()) {
  //       moveElevator(0);
  //     }
  //   }
  // }

  // public void middleLimitToTopLimit (double speed) {
  //   if (speed > 0) {
  //     if (!middleLimitSwitch.get()) {
  //       moveElevator(0);
  //     }
  //   }
  //   else {
  //     if (!topLimitSwitch.get()) {
  //       moveElevator(0);
  //     }
  //   }
  // }
  

  @Override
  public void autonomousInit() {
    elevatorSolenoid.set(true);
    engagedElevator = true;
    elevatorState = ElevatorStates.STOP;
    swerveDrive.setEncoders();
    swerveDrive.resetNavX();
  }

  @Override
  public void autonomousPeriodic() {
    AUTO_totalTimer.start();

    switch(elevatorState) {
      case TOP:
        moveToTop();
        break;
      case MIDDLE:
        moveToMiddle();
        break;
      case BOTTOM:
        moveToBottom();
        break;
      case STOP:
        stopElevator();
      case MANUAL:
        break;
    }

    if (AUTO_depositTimer.get() < 1) {
      AUTO_depositTimer.start();
      rightDepositorMotor.set(-1);
      leftDepositorMotor.set(-1);
    }

    if (AUTO_depositTimer.get() >= 1 && AUTO_depositorDone == false) {
      rightDepositorMotor.set(0);
      leftDepositorMotor.set(0);
      elevatorState = ElevatorStates.BOTTOM;
      AUTO_depositTimer.stop();
      AUTO_taxi = true;
      AUTO_taxiTimer.start();
      AUTO_depositorDone = true;
      
    }

    if (AUTO_taxi && AUTO_taxiTimer.get() < 2) {
      swerveDrive.drive(0, 0.3, 0, false);
    }

    if (AUTO_taxiTimer.get() >= 2) {
      swerveDrive.drive(0, 0, 0, false);
      AUTO_taxi = false;
      AUTO_taxiTimer.stop();
    }
  }
  

  // @Override
  // public void teleopExit() {
  //   ballCamera.endCamera();
  // }
  
  // @Override
  // public void autonomousExit() {
  //   ballCamera.endCamera();
  // }

  public void moveElevator(int input) {
    if (input == 1) {
      leftElevatorMotor.set(.2);
      rightElevatorMotor.set(-.2);
      elevatorSpeed = 1;
    } else if (input == -1) {
      leftElevatorMotor.set(-.2);
      rightElevatorMotor.set(.2);
      elevatorSpeed = -1;
    } else if (input == 0) {
      leftElevatorMotor.set(0);
      rightElevatorMotor.set(0);
      elevatorSpeed = 0;
    }
  }

  public void moveToTop() {
    if (topLimitSwitch.get()) {
      leftElevatorMotor.set(-elevSpeedUp);
      rightElevatorMotor.set(elevSpeedUp);
    }
    else {
      elevatorState = ElevatorStates.STOP;
    }
  }

  public void moveToBottom() {
    if (bottomLimitSwitch.get()) {
      leftElevatorMotor.set(elevSpeedDown);
      rightElevatorMotor.set(-elevSpeedDown);
    }
    else {
      elevatorState = ElevatorStates.STOP;
    }
  }

  public void moveToMiddle() {
    if (middleLimitSwitch.get() && !middleBool && topLimitSwitch.get()) {
      leftElevatorMotor.set(-elevSpeedUp);
      rightElevatorMotor.set(elevSpeedUp);
      middleTimer.reset();
      middleTimer.stop();
    }
    else if (!middleLimitSwitch.get() && !middleBool) {
      middleBool = true;
      leftElevatorMotor.set(-elevSpeedUp);
      rightElevatorMotor.set(elevSpeedUp);
      middleTimer.start();
    }
    else if (middleBool && middleTimer.get() < 0.15) {
      leftElevatorMotor.set(-elevSpeedUp);
      rightElevatorMotor.set(elevSpeedUp);
    }
    else {
      elevatorState = ElevatorStates.STOP;
      middleTimer.stop();
      middleTimer.reset();
      middleBool = false;
    }
  }

  public void stopElevator() {
    leftElevatorMotor.set(0);
    rightElevatorMotor.set(0);
  }
}