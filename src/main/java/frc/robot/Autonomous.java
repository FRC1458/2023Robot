package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous {
    private Balancer balancer;

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
        if (RobotConstants.willBalance) {
            balancer.balance();
        }
    }
    private void center() {
        if (RobotConstants.willBalance) {
            balancer.balance();
        }
    }
    private void right() {
        if (RobotConstants.willBalance) {
            balancer.balance();
        }
    }
}

