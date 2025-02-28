package frc.robot.subsystems.vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {
    
    private UsbCamera camera;
    
    public CameraSubsystem() {
        // Just start the camera - that's all you need for it to show up in the dashboard
        camera = CameraServer.startAutomaticCapture();
        
        // Configure camera settings
        camera.setResolution(320, 240);
        camera.setFPS(15);
        camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        
        // The camera will now be available in Shuffleboard from the sources list
        // No need to explicitly add it to a tab
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}