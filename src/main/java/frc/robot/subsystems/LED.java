// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;
  private int m_rainbowFirstPixelHue = 0;
  

  public LED() {
    m_led = new AddressableLED(9);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(25);
    m_led.setLength(m_ledBuffer.getLength());

    // for (int i = 0; i < m_ledBuffer.getLength(); i++) {
    //   // Sets the specified LED to the RGB values for red
    //   m_ledBuffer.setRGB(i, 0, 182, 174);
    // }

    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 0, 100, 0);
    }

    m_led.setData(m_ledBuffer);
    m_led.start();
    // m_led.setSyncTime(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateRainbow();
    // setColor();
    m_led.setData(m_ledBuffer);
    
  }

  public void setColor(){
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 200, 0, 0);
   }
   m_led.setData(m_ledBuffer);
   m_led.start();
   System.out.println("ERICK TRAN");
  }

  private void updateRainbow() {
    // Increment the hue
    m_rainbowFirstPixelHue += 3; // You might need to adjust this value
    // If we've gone all the way around the hue circle, start back at 0
    if (m_rainbowFirstPixelHue >= 180) {
        m_rainbowFirstPixelHue = 0;
    }

    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Calculate the hue, offset by the first pixel's hue
        final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
        // Set the LED at the current index to that hue
        m_ledBuffer.setHSV(i, hue, 255, 128);
    }
}
}



