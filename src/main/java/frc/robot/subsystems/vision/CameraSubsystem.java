package frc.robot.subsystems.vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {
    
    private UsbCamera camera;
    private ShuffleboardTab cameraTab;
    
    public CameraSubsystem() {
        // Initialize the USB camera
        camera = CameraServer.startAutomaticCapture();
        
        // Configure camera settings
        camera.setResolution(320, 240);      // Adjust resolution as needed
        camera.setFPS(15);                   // Adjust frame rate as needed
        camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        
        // Create a dedicated tab for the camera feed
        cameraTab = Shuffleboard.getTab("Camera");
        
        // Add the camera to the ShuffleBoard tab
        cameraTab.add("USB Camera", camera)
                .withPosition(0, 0)
                .withSize(4, 3);
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // You can add any periodic camera checks or updates here if needed
    }
}