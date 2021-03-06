/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates.swatbot;


import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends IterativeRobot {
    Gyro driveGyro = new Gyro(1);
    Victor rightDrive = new Victor(2);
    Victor leftDrive = new Victor(1);
    RobotDrive driveTrain = new RobotDrive(leftDrive, rightDrive);
    Encoder rightEncoder = new Encoder(1, 2, true, CounterBase.EncodingType.k4X);
    Encoder leftEncoder = new Encoder(3, 5, true, CounterBase.EncodingType.k4X);
    Joystick driveStick = new Joystick(1);
    RobotDrive robotTrain = new RobotDrive(leftDrive, rightDrive);
    SWATDrive driveSystem = new SWATDrive(robotTrain, driveGyro, rightEncoder);

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {        
        leftDrive.setSafetyEnabled(false);
        rightDrive.setSafetyEnabled(false);
        
        rightEncoder.setDistancePerPulse(Math.PI*6.0/250.0);
        leftEncoder.setDistancePerPulse(Math.PI*6.0/250.0);
        driveSystem.distanceEncoder.setDistancePerPulse(Math.PI*6.0/250.0);
        driveSystem.driveTrain.setSafetyEnabled(false);
    }

    Timer autoTimer = new Timer();
    
    public void autonomousInit()
    {
        driveSystem.driveTrain.setSafetyEnabled(false);
        driveSystem.resetControllers();

        nextStep = false;
        stepNumber = 0;
        autoTimer.reset();
        autoTimer.start();
    }
    
    boolean nextStep = false;
    int stepNumber = 0;
    public void autonomousPeriodic() {
        driveSystem.driveTrain.setSafetyEnabled(false);
        
        switch (stepNumber) {
            case 0:
            {
                nextStep = driveSystem.gyroDistanceDrive(15.0);
                break;
            }
            
            case 1:
            {
                nextStep = driveSystem.gyroTurn(90.0);
                autoTimer.reset();
                break;
            }
                
            case 2:
            {
                //Give the robot a second to stop completely.
                if(autoTimer.get() > 1.0)
                {
                    nextStep = true;
                }
                driveSystem.stopDrive();
                break;
            }
                
            case 3:
            {

                nextStep = driveSystem.gyroDistanceDrive(-10.0, 0.6);
                break;
            }
                
            case 4:
            {
                nextStep = driveSystem.gyroTurn(-45.0);
                break;
            }
                
            default:
            {
                driveSystem.stopDrive();
                break;
            }              
        }
        
        if(nextStep == true)
        {
            stepNumber++;
            driveSystem.resetControllers();
            nextStep = false;
        }

        
        SmartDashboard.putNumber("Angle ", driveSystem.driveGyro.getAngle());
        SmartDashboard.putNumber("Time ", autoTimer.get());
    }

    public void teleopInit() {
        driveSystem.distanceEncoder.reset();
        leftEncoder.reset();
        
        driveSystem.distanceEncoder.start();
        leftEncoder.start();
    }
    
    public void teleopPeriodic() {
        
        rightDrive.setSafetyEnabled(false);
        leftDrive.setSafetyEnabled(false);
        driveTrain.setSafetyEnabled(false);

        driveSystem.driveTrain.setSafetyEnabled(false);
        driveSystem.controlDrive(-driveStick.getY(), driveStick.getX());
        SmartDashboard.putNumber("Turn ", driveStick.getX());
        SmartDashboard.putNumber("Drive ", -driveStick.getY());
        SmartDashboard.putNumber("Angle ", driveSystem.driveGyro.getAngle());
        SmartDashboard.putNumber("Right Encoder ", driveSystem.distanceEncoder.getDistance());
        SmartDashboard.putNumber("Left Encoder ", leftEncoder.getDistance());

    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
