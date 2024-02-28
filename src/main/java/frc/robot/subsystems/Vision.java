package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.util.PixelFormat;
// import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Vision extends SubsystemBase {
  private UsbCamera visionCam;

  private boolean isConnected = false;

  public Vision() {

    visionCam = new UsbCamera("cam0", 1);
    visionCam = CameraServer.startAutomaticCapture();
    visionCam.setVideoMode(PixelFormat.kYUYV, 320, 240, 30);

  }
}