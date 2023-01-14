package frc.robot.wrappers;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Vision.Camera;

public class CameraWrapper extends Wrapper{
    
    Camera camera;

    public CameraWrapper(boolean onBlue) {
        try{
            camera = new Camera(onBlue);
            isInitialized = true;
        }
        catch (RuntimeException ex ) {
            DriverStation.reportError("Error Initiating Camera:  " + ex.getMessage(), true);
        }
    }

    public double getBallX() {
        if(isInitialized) return camera.getBallX();
        return 1000;
    }

    public double getBallSize() {
        if (isInitialized) return camera.getBallSize();
        return 0;
    }

    public void endCamera() {
        if (isInitialized) camera.endCamera();
        isInitialized = false;
        camera = null;
    }
}
