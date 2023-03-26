package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.swervedrive.SwerveDrive;
import frc.robot.wrappers.JoystickWrapper;

public class Robot extends TimedRobot {

  enum States {
      MANUAL,
      ALIGN,
      BALANCE
  }

  States state;
  private JoystickWrapper leftStick;
  private JoystickWrapper rightStick;
  private Controller controller;

  private double regularSpeed;
  private double boostedSpeed;

  SwerveDrive swerveDrive;
  Lidar armLidar;

  Limelight limelight;
  private Balancer balancer;
  private Aligner aligner;

  private final AHRS navX;
  private final ArmNavX armNavX;

  Timer timer = new Timer();
  int counter = 0;
  Solenoid armSolenoid = new Solenoid(4, 3, 2); // change to correct values
  Solenoid clawSolenoid = new Solenoid(4, 1, 0);
  Arm arm;
  Autonomous autonomous;

  public Robot() {
    super(0.03);
    //leftStick = new JoystickWrapper(0);//uncomment if using them
    //rightStick = new JoystickWrapper(1);
    controller = new XboxController();

    navX = new AHRS(SPI.Port.kMXP);
    armNavX = new ArmNavX(4);
    swerveDrive = new SwerveDrive(navX);
    //lidar = new Lidar(RobotConstants.lidarPort);
    armLidar = new Lidar(RobotConstants.armLidarPort);
    limelight = new Limelight(0);

    balancer = new Balancer(swerveDrive, navX);
    aligner = new Aligner(swerveDrive, limelight);

    regularSpeed = RobotConstants.regularSpeed;
    boostedSpeed = RobotConstants.boostedSpeed;

    arm = new Arm(armSolenoid, clawSolenoid);
    autonomous = new Autonomous(swerveDrive, arm);
  }

  @Override
  public void robotInit() {
  }

  @Override
  public void teleopInit() {
    state = States.MANUAL;
    superiorReset();
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
    SmartDashboard.putString("State", state.toString());
    //SmartDashboard.putNumber("Arm Encoder", arm.getEncoder());

    limelight.readPeriodic();

    xAxis = controller.getSwerveX();
    yAxis = controller.getSwerveY();
    rAxis = controller.getSwerveR();

    if (controller.resetNavX()) {
      superiorReset();
      swerveDrive.resetNavX();
    }

    if (controller.resetArm()) {
      arm.reset();
    }

    if (controller.lockWheels()) {
      swerveDrive.drive(0, 0, 0.01, true);
    }

    x = -(Math.abs(xAxis)*xAxis) * speedIncrease;
    y= Math.abs(yAxis)*yAxis * speedIncrease;
    r= Math.abs(rAxis)*rAxis * speedIncrease;

    switch(state) {
      case MANUAL:
        manual(x, y, r);
        break;
      case ALIGN:
        aligner.align();
        break;
      case BALANCE:
        balancer.balance();
        break;
    }


    if (controller.stateManual()) {
      state = States.MANUAL;
    }
    else if (controller.stateAlign()) {
      aligner.reset();
      state = States.ALIGN;
    }
    else if (controller.stateBalance()) {
      state = States.BALANCE;
      balancer.reset();
    }
    runArm();

  }

  public void runArm() {
    if (controller.armTop()) {
      arm.setUp();
    } else if (controller.armMiddle()) {
      arm.setMiddle();
    } else if (controller.armBottom()) {
      arm.setBottom();
    }
    arm.runArm(controller.armDown(), controller.armUp());

    if (controller.armExtend()) {
      arm.extendArm();
    } else if (controller.armRetract()) {
      arm.retractArm();
    }

    if (controller.clawOpen()) {
      arm.openClaw();
    } else if (controller.clawClose()) {
      arm.closeClaw();
    }

    SmartDashboard.putNumber("Arm Position", arm.encoderPosition());
    SmartDashboard.putNumber("Robot Pitch", balancer.getPitch());
  }

  private void manual(double x, double y, double r) {
    swerveDrive.drive(x, y, r, true);
  }

  @Override
  public void autonomousInit() {
    superiorReset();
    autonomous.resetAuto();
  }

  @Override
  public void autonomousPeriodic() {
    autonomous.runAuto(true);
    //autonomous.simpleAuto();
  }

  @Override 
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  public void superiorReset() {
    //swerveDrive.resetNavX();
    aligner.reset();
    swerveDrive.setEncoders();
    balancer.reset();
  }
}