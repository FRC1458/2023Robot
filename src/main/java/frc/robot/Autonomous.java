package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.swervedrive.SwerveDrive;


public class Autonomous {
    private Balancer balancer;
    private Arm arm;
    private boolean finishedScoring;
    private boolean armRaised; 
    private SwerveDrive swerve;
    private Lidar lidar;


    public Autonomous(Balancer balancer) {
        this.balancer = balancer;
    }

    public void autonomous() {
        if(RobotConstants.position == RobotConstants.Position.LEFT) {
            left();
        }
        else if (RobotConstants.position == RobotConstants.Position.CENTER) {
            center();
        }
        else if (RobotConstants.position == RobotConstants.Position.RIGHT) {
            right();
        }
        else {
            SmartDashboard.putString("Wrong position", "It's an enum, how did you mess it up");
        }
    }
    //do initial scoring and possible second scoring later
    private void left() {
        scoreStart();
    }
    private void center() {
        scoreStart();
        if (RobotConstants.willBalance && finishedScoring) {
            balancer.balance();
        }
    }
    private void right() {
        scoreStart();
    }

    private void scoreStart(){
        if (finishedScoring = false){  
             arm.moveToPreset(105);

            if (arm.encoderPosition()>103.5) {
                arm.extendArm();
                armRaised = true;
            }
            else if (armRaised && lidar.getDistanceCentimeters()>10){ //change lidar distamce
                swerve.drive(0, -0.1, 0, true);
            }
            else if (lidar.getDistanceCentimeters()<10){ //change lidar distance
                //arm.openClaw();
                finishedScoring = true;
            }
    }
    }
}

