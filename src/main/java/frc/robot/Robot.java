/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import com.ctre.phoenix.motorcontrol.*;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.sensors.PigeonIMU;
//import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.Joystick;

//import static org.junit.Assume.assumeNoException;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Preferences;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;

  TalonSRX talon16 = new TalonSRX(16);
  PigeonIMU pigeon = new PigeonIMU(2);
  Joystick joy = new Joystick(0);
  Faults _faults = new Faults(); /* temp to fill with latest faults */
  long  lastms;
  double lastcount;
  int AState = 1;
  double targetVelocity_UnitsPer100ms = 0;    // For set velocity mode
  double targetPos = 0;                       // For motion magic mode
  int waitcount = -1;
  double baseY;       // null (initial) position of joystick
  double realY;       // Adjusted current position of joystick

	Preferences prefs;
		
	double KP;
  double KI;
  double KD;
  double KF;

  int testmode = 0;     // Initial test mode will be 0

//hi
		
	

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
      // Set talon parameters to default values
      talon16.configFactoryDefault();

      GetPrefs();  // Get and set PID parameters for talon16

    talon16.setSensorPhase(true);  // correct encoder to motor direction
     
    // Tell the talon that he has a quad encoder
    talon16.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
   
    // Set minimum output (closed loop)  to 0 for now
    talon16.configNominalOutputForward(0, 30);
    talon16.configNominalOutputReverse(0, 30);
    
    // Set maximum forward and backward to full speed
    talon16.configPeakOutputForward(1, 30);
    talon16.configPeakOutputReverse(-1, 30);

    // Motion magic cruise (max speed) is 100 counts per 100 ms
		talon16.configMotionCruiseVelocity(500, 30);

    // Motion magic acceleration is 50 counts
		talon16.configMotionAcceleration(100, 30);

		// Zero the sensor once on robot boot up 
		talon16.setSelectedSensorPosition(0, 0, 30);
     
    // Zero the joystick
    baseY = joy.getY();
    System.out.print("baseY: " );
    System.out.println(baseY);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() 
  
  {
  
    talon16.config_kF(0, 0, 30);
    talon16.config_kP(0, 0, 30);
    talon16.config_kI(0, 0, 30);
    talon16.config_kD(0, 0, 30);
    SmartDashboard.putNumber("KP", 0);
    SmartDashboard.putNumber("KI", 0);
    SmartDashboard.putNumber("KD", 0);
    SmartDashboard.putNumber("KF", 0);

    AState = 1;     // Set state to 1
    System.out.println("Autonomous Init");
   
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    long time = System.currentTimeMillis();
    double rawcounts;
    double countsper100;
    double duration;
    double suggestKF = 0;
    double velocity;
    double error;

    UpdateDashboard();

    if (waitcount > 0)
    {
      waitcount--;
      return;
    }
    switch (AState)
    {
      case 0:

        talon16.set(ControlMode.PercentOutput, 0 );   // Stop
        break;

      case 1:    
        // Determine kF - find nominal speed
        talon16.set(ControlMode.PercentOutput, .75 );   // Set speed to .75 forward
        SmartDashboard.putNumber("Joystick", .75);
        waitcount = 50;
        AState = 2;
        System.out.println("State 1");
        break;

      case 2:
        waitcount = -1;
        velocity = talon16.getSelectedSensorVelocity();

        // If sensor polarity is backward, notify
        //    There is nothing else we can do
        if (velocity < 0)
        {
          System.out.println("Sensor polarity is reversed");
          System.out.println("Reverse parameter in talon16.setSensorPhase(); statement");
          AState = 0;   // Now quit
          break;
        }

        // Sensor polarity is correct - calculate suggested KF

        SmartDashboard.putNumber("Sensor Vel:", velocity);
        suggestKF = .75 * 1023 / talon16.getSelectedSensorVelocity();
        System.out.print("Suggested kF: " );
        System.out.println(suggestKF);
        talon16.set(ControlMode.PercentOutput, 0 );   // Set speed to .75 forward
        talon16.config_kF(0, suggestKF, 30);
        SmartDashboard.putNumber("KF", 0);
        waitcount = 50;

        //  Now, go into closed loop mode with the new KF and try for 75% speed again.
        talon16.set(ControlMode.Velocity, velocity);

        AState = 3;
        break;

        // We have now gone about 1 second trying for 75% speed closed loop
        //   Catch the error and suggest a new KP

      case 3:
        error = talon16.getClosedLoopError(0);
        System.out.print("Closed loop error (KP = 0): " );
        System.out.println(error);
        AState = 4;
        break;

      case 4:
        
        

      default:
        System.out.print("Unsupported case: ");
        System.out.println(AState);
        AState = 0;

        break;


    }
  
};

@Override
  public void teleopInit() 
    {
      talon16.setSelectedSensorPosition(0, 0, 30);
      GetPrefs();   // Get and set PID parameters
      pigeon.setFusedHeading(0.0, 30);
    }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  
    realY = joy.getY() - baseY;

    // If trigger button pushed, bump the mode
    if (joy.getRawButton(1)) 
    {
      BumpMode();
    }


    switch(testmode)
    {
      case 0: 
          talon16.set(ControlMode.PercentOutput, .75 );
          break;

      case 1:
          talon16.set(ControlMode.PercentOutput, realY);
          break;

      case 2:
          targetVelocity_UnitsPer100ms = realY * 1000.0 * 200.0 / 600.0;
          talon16.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
          break;

      case 3:
          targetPos = joy.getY() * 40000;  // Max range = +/- 1024 counts per rev times 4 revs
          talon16.set(ControlMode.MotionMagic, targetPos);
          break;
      
      default:
          break;
  
    }

 
    SmartDashboard.putNumber("Error: ", talon16.getClosedLoopError(0) );
    SmartDashboard.putNumber("Error-", -1 * talon16.getClosedLoopError(0));
    SmartDashboard.putNumber("Joystick", realY);
    SmartDashboard.putNumber("Sensor Vel:", talon16.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Velocity Target", targetVelocity_UnitsPer100ms);
    SmartDashboard.putNumber("Sensor Pos:", talon16.getSelectedSensorPosition());
    SmartDashboard.putNumber("MM Target", targetPos);
    SmartDashboard.putBoolean("Forward Limit", talon16.getSensorCollection().isFwdLimitSwitchClosed());
    SmartDashboard.putBoolean("Backward Limit", talon16.getSensorCollection().isRevLimitSwitchClosed());
    DoPigeon();
  }


  @Override
  public void testInit() {
    GetPrefs();       // Get and set PID parameters
    talon16.setSelectedSensorPosition(0, 0, 30);

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    prefs = Preferences.getInstance();
    testmode = prefs.getInt("TestMode", 1);
    SmartDashboard.putNumber("TestMode", testmode);
    switch(testmode)
    {
      case 1:
        
    }
			/* 4096 ticks/rev * 10 Rotations in either direction */

		double targetPos = joy.getY() * 4096;

		talon16.set(ControlMode.MotionMagic, targetPos);
    SmartDashboard.putNumber("Sensor Pos:", talon16.getSelectedSensorPosition());

	
  }

  public void GetPrefs()
  {
    prefs = Preferences.getInstance();
		KP = prefs.getDouble("KP", 1.0);
		KI = prefs.getDouble("KI", 0.1);
    KD = prefs.getDouble("KD", 2.0);
    KF = prefs.getDouble("KF", 3.7);
    
    SmartDashboard.putNumber("KP", KP);
    SmartDashboard.putNumber("KI", KI);
    SmartDashboard.putNumber("KD", KD);
    SmartDashboard.putNumber("KF", KF);

    talon16.config_kF(0, 3.70, 30);
    talon16.config_kP(0, KP, 30);
    talon16.config_kI(0, KI, 30);
    talon16.config_kD(0, KD, 30);
 
  }

  public void BumpMode()
  {
    //  Weak design but wait for button to be released
    do{}while (joy.getRawButton(1));
    
    testmode = (testmode + 1) % 4;
    switch (testmode)
    {
      case 0:
        SmartDashboard.putString("Mode", " 0 - Constant 75%");
        SmartDashboard.putString("Help", "Note sensor speed to calculate kF");
        break;
 
      case 1:
        SmartDashboard.putString("Mode", " 1 - Open loop voltage");
        SmartDashboard.putString("Help", "Joystick sets power");
        break;
 
      case 2:
        SmartDashboard.putString("Mode", " 2 - Closed loop speed");
        SmartDashboard.putString("Help", "Joystick sets target speed");
        break;
 
      case 3:
        SmartDashboard.putString("Mode", " 3 - MotionMagic (position)");
        SmartDashboard.putString("Help", "Joystick sets target position");
        break;
 

    }

  }

    
  public void UpdateDashboard()
  {
    SmartDashboard.putNumber("Error: ", talon16.getClosedLoopError(0) );
    SmartDashboard.putNumber("Error-", -1 * talon16.getClosedLoopError(0));
    SmartDashboard.putNumber("Joystick", joy.getY());
    SmartDashboard.putNumber("Sensor Vel:", talon16.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Velocity Target", targetVelocity_UnitsPer100ms);
    SmartDashboard.putNumber("Sensor Pos:", talon16.getSelectedSensorPosition());
    SmartDashboard.putNumber("MM Target", targetPos);
  }
  
  public void DoPigeon(){
    PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();

		PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();

		double [] xyz_dps = new double [3];

		/* grab some input data from Pigeon and gamepad*/

		pigeon.getGeneralStatus(genStatus);

		pigeon.getRawGyro(xyz_dps);

		pigeon.getFusedHeading(fusionStatus);

    double currentAngle = fusionStatus.heading;

		boolean angleIsGood = (pigeon.getState() == PigeonIMU.PigeonState.Ready) ? true : false;

    double currentAngularRate = xyz_dps[2];
    
    SmartDashboard.putBoolean("AngleGood", angleIsGood);
    SmartDashboard.putNumber("Angle", currentAngle);
    SmartDashboard.putNumber("Rate", currentAngularRate);
    



  }

}
