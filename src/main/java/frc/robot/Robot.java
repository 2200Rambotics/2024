package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.File;

import frc.robot.subsystems.ClimberSubsystem;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    BufferedImage img, newImg;
    File file;
    ByteArrayOutputStream outStreamObj;
    ByteArrayInputStream inStreambj;
    // PID intake velocity values:
    // P: 0.0002 | I: 0.000001 | D: 0.0 | F: 0.0
    Timer excitingTimer = new Timer();

    Timer startTimer;
    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();

        startTimer = new Timer();
        startTimer.start();
    }

    @Override
    public void robotPeriodic() {
        if (!this.isTest()) {
            CommandScheduler.getInstance().run();
            // m_robotContainer.logger.putPose();
        }
        if(excitingTimer.get() < 2){
           // m_robotContainer.led.exciteMode = true;
        } else{
           // m_robotContainer.led.exciteMode = false;
        }

        m_robotContainer.led.exciteMode = excitingTimer.get() < 2;

        if(startTimer != null && startTimer.get() > 5){
            m_robotContainer.drivetrain.seedFieldRelative();
            startTimer = null;
            System.out.println("Gyro Initialized!");
            System.out.println(m_robotContainer.logger.getAngle());
            System.out.println(m_robotContainer.pigeon.Y);

        }

        // try{
            // img = ImageIO.read(new File("src/main/deploy/autoImages/A145.png"));
            // file = new File("src/main/deploy/autoImages/A145.png");
            //  img = ImageIO.read(file);
            // outStreamObj = new ByteArrayOutputStream();
            // ImageIO.write(img, "png", outStreamObj);
            // byte [] byteArray = outStreamObj.toByteArray();
            // // SmartDashboard.putRaw("Test", byteArray);
        //     System.out.println("Yes");
        // } catch (IOException e)
        // {
        //     System.out.println("No");
        //     e.printStackTrace();
            // img = null;
        // }

        

        // SmartDashboard.putNumber("Current memory (MB)",
        // Runtime.getRuntime().totalMemory() / (1024.0 * 1024.0));
        // SmartDashboard.putNumber("Maximum memory (MB)",
        // Runtime.getRuntime().maxMemory() / (1024.0 * 1024.0));
    }

    // XboxController testController = new
    // XboxController(Constants.DRIVER_CONTROLLER_PORT);
    // XboxController cotestController = new
    // XboxController(Constants.CODRIVER_CONTROLLER_PORT);
    @Override
    public void disabledInit() {
        excitingTimer.restart();
    }
    
    @Override
    public void disabledPeriodic() {
        m_robotContainer.savedAllianceRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    PIDMotor[] motorArray;

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    int currIndex = 0;

    double stickY;

    @Override
    public void testPeriodic() {

        if (m_robotContainer.driverController.getHID().getPOV() == 0) {
            m_robotContainer.climber.leftMotor.setPercentOutput(0.3);
        } else if (m_robotContainer.driverController.getHID().getPOV() == 180) {
            m_robotContainer.climber.leftMotor.setPercentOutput(-0.9);
        } else {
            m_robotContainer.climber.leftMotor.setPercentOutput(0);
        }

        if (m_robotContainer.driverController.getHID().getPOV() == 270) {
            m_robotContainer.climber.leftRatchetServo.setPosition(ClimberSubsystem.LEFT_SERVO_ENGAGE_POS);
        }
        if (m_robotContainer.driverController.getHID().getPOV() == 90) {
            m_robotContainer.climber.leftRatchetServo.setPosition(ClimberSubsystem.LEFT_SERVO_RELEASE_POS);
        }

        if (m_robotContainer.driverController.getHID().getYButton()) {
            m_robotContainer.climber.rightMotor.setPercentOutput(-0.3);
        } else if (m_robotContainer.driverController.getHID().getAButton()) {
            m_robotContainer.climber.rightMotor.setPercentOutput(0.9);

        } else {
            m_robotContainer.climber.rightMotor.setPercentOutput(0);
        }

        if (m_robotContainer.driverController.getHID().getXButton()) {
            m_robotContainer.climber.rightRatchetServo.setPosition(ClimberSubsystem.RIGHT_SERVO_ENGAGE_POS);
        }
        if (m_robotContainer.driverController.getHID().getBButton()) {
            m_robotContainer.climber.rightRatchetServo.setPosition(ClimberSubsystem.RIGHT_SERVO_RELEASE_POS);
        }

        // if(testController.getRightBumperPressed() && currIndex <
        // motorArray.length-1){
        // motorArray[currIndex].set(0);
        // ++currIndex;
        // } else if (testController.getLeftBumperPressed() && currIndex > 0){
        // motorArray[currIndex].set(0);
        // --currIndex;
        // }
        // stickY = testController.getLeftY();
        // if(stickY > -0.1 && stickY < 0.1){
        // stickY = 0;
        // }
        // //motorArray[currIndex].set(stickY);
        // SmartDashboard.putNumber("Motor Index", currIndex);
        // SmartDashboard.putString("Motor Name", motorArray[currIndex].name);

        // double analogLeft = testController.getLeftTriggerAxis();
        // if(analogLeft < 0.1){
        // analogLeft = 0;
        // }
        // double analogRight = testController.getRightTriggerAxis();
        // if(analogRight < 0.1){
        // analogRight = 0;
        // }

        // if(testController.getLeftBumper()){
        // analogLeft = -1*analogLeft;
        // }

        // if(testController.getAButton()){
        // motorArray[3].set(1);
        // } else {
        // motorArray[3].set(0);
        // }

        // motorArray[7].set(-1*analogLeft);
        // motorArray[8].set(analogLeft);
        // motorArray[9].set(-1*analogRight);
        // motorArray[10].set(analogRight);

        // SmartDashboard.putNumber("Bottom Shooter Velocity",
        // motorArray[9].getVelocity());
        // SmartDashboard.putNumber("Top Shooter Velocity",
        // motorArray[10].getVelocity());

        // double stickRightY = testController.getRightY();

        // if(stickRightY > -0.1 && stickRightY < 0.1){
        // stickRightY = 0;
        // }
        // motorArray[4].set(stickRightY);
        // motorArray[5].set(-1*stickRightY);

    }

    @Override
    public void testExit() {
    }
}
