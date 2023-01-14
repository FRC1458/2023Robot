package frc.robot.Vision;

import frc.robot.pipelines.*;

import org.opencv.core.KeyPoint;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camera {
    private static final int IMG_WIDTH = 320;
    private static final int IMG_HEIGHT = 240;
    //private VisionThread visionThread;
    private final Object imgLock = new Object();
    private VisionThread visionThread;
    double centerX, size;
    private UsbCamera camera;

    public Camera (boolean onBlue) {

        try {
            camera = CameraServer.startAutomaticCapture();
            camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
        }
        catch (Exception e) {
            DriverStation.reportError("camera failed to initialize", e.getStackTrace());
            return;
        }
        if (onBlue) {
            visionThread = new VisionThread(camera, new BBDPipeline(), pipeline -> {
                if (!pipeline.findBlobsOutput().empty()) {
                    KeyPoint ball = pipeline.findBlobsOutput().toList().get(0);
                    synchronized (imgLock) {
                        centerX = ball.pt.x;
                        size = ball.size;
                        // SmartDashboard.putString("ball", ball.toString());
                    }
                }
                else{
                    size = 0;
                }
            });
        }
        else {
            visionThread = new VisionThread(camera, new RBDPipeline(), pipeline -> {
                if (!pipeline.findBlobsOutput().empty()) {
                    KeyPoint ball = pipeline.findBlobsOutput().toList().get(0);
                    synchronized (imgLock) {
                        centerX = ball.pt.x;
                        size = ball.size;
                        // SmartDashboard.putString("ball", ball.toString());
                    }
                }
                else{
                    size = 0;
                }
            });
        }
        visionThread.start();
    }

    public double getBallX() {
        synchronized (imgLock) {
            if (size > 0) {
                return (2*centerX / IMG_WIDTH) - 1;
            }
        }

        return 1000.0;
    }

    public double getBallSize() {
        synchronized (imgLock) {
            return size;
        }
    }

    public void endCamera() {
        visionThread.stopVision();
        camera.close();
    }

}
