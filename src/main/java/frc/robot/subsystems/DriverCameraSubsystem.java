package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("unused")

public class DriverCameraSubsystem extends SubsystemBase {
    private UsbCamera camera;
    private boolean cameraInitialized = false;
  
    public DriverCameraSubsystem() {
      // Initialize camera once with proper configuration
      initializeCamera();
    }
  
    private void initializeCamera() {
      try {
        // Use camera index 0 (the one that's working)
        camera = CameraServer.startAutomaticCapture(0);
        
        // Configure camera with more conservative settings
        camera.setResolution(320, 240);
        camera.setFPS(45); // Default was 15 fps
               
        // IMPORTANT: Set to only maintain the connection without trying to reconnect
        // This prevents the constant reconnection attempts
        camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        // camera.getProperty("rotation").set(180); // added to flip camera image

        
        

        cameraInitialized = true;
        System.out.println("Camera initialized successfully");
      } catch (Exception e) {
        System.err.println("Camera initialization failed: " + e.getMessage());
      }
    }
  
    @Override
    public void periodic() {
      // Leave this empty or add minimal code
      // Don't try to reconnect or change settings here
    }
  }