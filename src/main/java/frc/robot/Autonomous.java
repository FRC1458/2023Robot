package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous {

    private enum Positions {
        RIGHT,
        CENTER,
        LEFT
    }
    private Positions position;
    private boolean willBalance;
    private Balancer balancer;

    public Autonomous(boolean willBalance, Balancer balancer) {
        this.willBalance = willBalance;
        this.balancer = balancer;
        if (RobotConstants.position.equalsIgnoreCase("right")) {
            position = Positions.RIGHT;
        }
        else if (RobotConstants.position.equalsIgnoreCase("center")) {
            position = Positions.CENTER;
        }
        else if (RobotConstants.position.equalsIgnoreCase("left")) {
            position = Positions.LEFT;
        }
        else {
            SmartDashboard.putString("Autonomous Failure", "Invalid value for RobotConstants.position");
        }
    }

    public void autonomous() {
        switch (position) {
            case LEFT:
                left();
                break;
            case CENTER:
                center();
                break;
            case RIGHT:
                right();
                break;
        }
    }
    //do initial scoring and possible second scoring later
    private void left() {
        if (willBalance) {
            balancer.balance();
        }
    }
    private void center() {
        if (willBalance) {
            balancer.balance();
        }
    }
    private void right() {
        if (willBalance) {
            balancer.balance();
        }
    }
}

