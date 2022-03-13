package frc.robot;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.devices.LidarV4;

public class Robot extends TimedRobot {
    private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kMXP);
    private final LidarV4 lidar = new LidarV4(0x62);
    // private I2C i2cDevice = new I2C(I2C.Port.kOnboard, 0x52);
    // private AddressableLED m_led =  new AddressableLED(9);
    // private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(16);
    private final int THREAD_COUNT = 1;
    private final double THREAD_PERIOD = 0.0002;
    private int mult = 0;
    private volatile boolean keepRunning = false;
    private final Runnable colorSensorReader = new Runnable() {
      private int timesRun = 0;
      private long startTime = -1;
      byte[] raw = new byte[3];


    
      @Override
      public void run () {
        startTime = System.currentTimeMillis();
        while(keepRunning) {body();}
      }
      private void body() {
        timesRun++;
        long currentTime = System.currentTimeMillis();
        if (currentTime - startTime > 1000) {
          System.out.println("Runs: " + timesRun + " Color Sensor Measurement:"
            + colorSensor.getProximity() + " Lidar Measurement: "
            + lidar.getDistance());
          timesRun = 0;
          startTime = currentTime;
        }
    
        // i2cDevice.read(0x10, 3, raw);
        colorSensor.getProximity();
        lidar.getDistance();
      }
    };
    private final Runnable wakerUpper = new Runnable() {
      @Override
      public void run() {
      }
    };
    private final Thread[] threads = new Thread[THREAD_COUNT];
    private final Notifier[] wakerUppers = new Notifier[THREAD_COUNT];

    @Override
    public void robotInit() {
     


      // Reuse buffer
      // Default to a length of 60, start empty output
      // Length is expensive to set, so only set it once, then just update data
      // m_ledBuffer = new AddressableLEDBuffer(16);
      // m_led.setLength(m_ledBuffer.getLength());

      // Set the data
      // m_led.setData(m_ledBuffer);
      // m_led.start();
      for (var i = 0; i < THREAD_COUNT; i++) {
        wakerUppers[i] = new Notifier(wakerUpper);
        }
      }

    @Override
    public void teleopInit() {
      // for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        // m_ledBuffer.setRGB(i, 0, 64, 0);
      // }
      // m_led.setData(m_ledBuffer);
      // m_led.start();
      for (int i = 0; i < THREAD_COUNT; i++) {
        threads[i] = new Thread(colorSensorReader);

      }
      keepRunning = true;
      for (int i = 0; i < THREAD_COUNT; i++) {
        threads[i].start();
        wakerUppers[i].startPeriodic(THREAD_PERIOD);
      }
    }
    @Override
    public void teleopPeriodic() {
      // for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        // m_ledBuffer.setRGB(i, 0, mult*64, (1-mult)*64);
      // }
      mult = (mult+1) % 2;
      // m_led.setData(m_ledBuffer);
    }

    @Override
    public void disabledInit() {
      // for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        // m_ledBuffer.setRGB(i, 64, 0, 0);
      // }
      // m_led.setData(m_ledBuffer);
      // m_led.start();
      for (int i = 0; i < THREAD_COUNT; i++) {
        keepRunning = false;
      }
    }
    @Override
    public void disabledPeriodic() {

      // for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        // m_ledBuffer.setRGB(i, 64*mult, 0, 64*(1-mult));
      // }
      mult = (mult+1) % 2;
      // m_led.setData(m_ledBuffer);
    }
}
