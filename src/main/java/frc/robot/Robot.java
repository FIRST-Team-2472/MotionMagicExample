/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;


import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Robot extends TimedRobot {

  TalonFX talon16 = new TalonFX(50);
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
  
  
  NetworkTableEntry tKP, tKF, tKI, tKD;
  ShuffleboardTab main;	
	double KP;
  double KI;
  double KD;
  double KF;

  int testmode = 0;     // Initial test mode will be 0


		
  @Override
  public void robotInit() {

    main = Shuffleboard.getTab("vaules");

    tKF = main.add("KF", 0.0).getEntry();
    tKP = main.add("KP", 0.0).getEntry();
    tKI = main.add("KI", 0.0).getEntry();
    tKD = main.add("KD", 0.0).getEntry();

      // Set talon parameters to default values
      talon16.configFactoryDefault();

      GetPrefs();  // Get and set PID parameters for talon16

    talon16.setSensorPhase(true);  // correct encoder to motor direction
        
    // Set minimum output (closed loop)  to 0 for now
    talon16.configNominalOutputForward(0, 30);
    talon16.configNominalOutputReverse(0, 30);
    
    // Set maximum forward and backward to full speed
    talon16.configPeakOutputForward(1, 30);
    talon16.configPeakOutputReverse(-1, 30);

    // Motion magic cruise (max speed) is 100 counts per 100 ms
		talon16.configMotionCruiseVelocity(10000, 30);

    // Motion magic acceleration is 50 counts
		talon16.configMotionAcceleration(4000, 30);

		// Zero the sensor once on robot boot up 
		talon16.setSelectedSensorPosition(0, 0, 30);
     
    // Zero the joystick
    baseY = joy.getY();
    System.out.print("baseY: " );
    System.out.println(baseY);

  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    AState = 1;     // Set state to 1
    System.out.println("Autonomous Init");
    GetPrefs();
   
  }

  @Override
  public void autonomousPeriodic() {
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
      GetPrefs();  
    }

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
          targetPos = joy.getY() * 120000;  // Max range = +/- 1024 counts per rev times 4 revs
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
    //prefs = Preferences.getInstance();
    //testmode = prefs.getInt("TestMode", 1);
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
		KP = tKP.getDouble(0.0);
		KI = tKI.getDouble(0.0);
    KD = tKD.getDouble(0.0);
    KF = tKF.getDouble(0.0);

    talon16.config_kF(0, KF, 30);
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
  

}
