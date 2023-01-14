package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.swervedrive.SwerveDrive;

public class Diagnostic {
    public void angleMotorDiagnostic(SwerveDrive sd) {
        SmartDashboard.putNumber(sd.backLeft.wheelName + " encoder and abs encoder difference", sd.backLeft.angleMotorDiagnostic());
    }
}
