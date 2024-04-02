package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;

public class ShooterSubsystem extends SubsystemBase {
  public PIDMotor shooterTop, shooterBottom, intakeTop, intakeBottom;

  Timer shooterTimer;
  public IntakeState intakeState;
  public ShooterState shooterState;
  public boolean okToShoot = true;

  public boolean slowDownShooters = false;
  double currentVelocity = 0;

  // TODO: Calculate velocity
  public double shooterV;

  public ShooterSubsystem() {
    shooterTop = PIDMotor.makeMotor(Constants.SHOOTER_TOP_ID, "Shooter Top", 0.00009, 0, 0, 0.00009, ControlType.kVelocity);
    shooterBottom = PIDMotor.makeMotor(Constants.SHOOTER_BOTTOM_ID, "Shooter Bottom", 0.00009, 0, 0, 0.00009, ControlType.kVelocity);
    intakeTop = PIDMotor.makeMotor(Constants.INTAKE_TOP_ID, "Intake Top", 0, 0, 0, 0, ControlType.kPosition);
    intakeBottom = PIDMotor.makeMotor(Constants.INTAKE_BOTTOM_ID, "Intake Bottom", 0, 0, 0, 0, ControlType.kPosition);

    shooterBottom.setCurrentLimit(40);
    shooterTop.setCurrentLimit(40);
    intakeBottom.setCurrentLimit(30);
    intakeTop.setCurrentLimit(30);

    shooterBottom.setIdleCoastMode();
    shooterTop.setIdleCoastMode();
    shooterBottom.setInverted(true);

    shooterTimer = new Timer();
    intakeState = IntakeState.Idle;
    shooterState = ShooterState.Idle;
    shooterV = 0;
  }

  @Override
  public void periodic() {
    // TODO: calculate shooterV when in limelight shooter state
    manageShooterRollers();
    if(slowDownShooters){
      shooterBottom.setTarget(currentVelocity);
      shooterTop.setTarget(currentVelocity);
      if (DriverStation.isAutonomous()){
        currentVelocity -= 800;
      } else {
        currentVelocity -= 250;
      }
      if(currentVelocity <= 0 ){
        shooterBottom.setTarget(0);
        shooterTop.setTarget(0);
      }
      if(currentVelocity <= -2500){
        slowDownShooters = false;
      }
    }
  }

  public enum IntakeState {
    Idle,
    ShootNow,
    ShootAuto,
    Intake,
    ReverseIntake,
    Preload,
    ResetTimer,
    Spit
  }

  public enum ShooterState {
    Idle,
    SpinFixed,
    SpinLimelight,
    SpinAmp
  }

  public void spinDownShooters(){
    currentVelocity = shooterV;
    slowDownShooters = true;
  }

  public void manageShooterRollers() {

    switch (shooterState) {
      case Idle:
        if(!slowDownShooters){
          shooterBottom.setPercentOutput(0);
          shooterTop.setPercentOutput(0);
        }
        break;
      case SpinFixed:
      case SpinLimelight:
        shooterBottom.setTarget(shooterV);
        shooterTop.setTarget(shooterV);
        break;
      case SpinAmp:
        shooterTop.setTarget(shooterV);
        shooterBottom.setPercentOutput(0);
      default:
        break;
    }

    switch (intakeState) {
      // Stops all the motors.
      case Idle: {
        // disableBrakeMode();
        intakeTop.setPercentOutput(0);
        intakeBottom.setPercentOutput(0);
      }
        break;
      case ShootAuto:
      // TODO
        break;
      case ShootNow:
        // disableBrakeMode();
        intakeBottom.setPercentOutput(-1);
        intakeTop.setPercentOutput(1);
        break;
      case Intake: {
        // enableBrakeMode();
        intakeBottom.setPercentOutput(-0.4);
        intakeTop.setPercentOutput(0.4);
        shooterTimer.restart();
        intakeTop.resetEncoder();
        intakeBottom.resetEncoder();
      }
        break;
      case Preload: {
        // disableBrakeMode();
        if (shooterTimer.get() < 0.15) {
          intakeBottom.setPercentOutput(0);
          intakeTop.setPercentOutput(0);
          intakeTop.resetEncoder();
          intakeBottom.resetEncoder();
        } else if (shooterTimer.get() > 0.35) {
          intakeBottom.setPercentOutput(0.15);
          intakeTop.setPercentOutput(-0.15);
          if (Math.abs(intakeBottom.getPosition()) >= 3.75 || Math.abs(intakeTop.getPosition()) >= 3.75) {
            intakeBottom.setPercentOutput(0);
            intakeTop.setPercentOutput(0);
            intakeState = IntakeState.Idle;
          }
        }
      }
        break;
      case ReverseIntake: {
        // disableBrakeMode();
        intakeBottom.setPercentOutput(0.5);
        intakeTop.setPercentOutput(-0.5);
      }
        break;
      case Spit: {
        // disableBrakeMode();
        intakeBottom.setPercentOutput(0.15);
        intakeTop.setPercentOutput(-0.15);
      }
        break;
      case ResetTimer:
        shooterTimer.restart();
        intakeTop.resetEncoder();
        intakeBottom.resetEncoder();
        break;
    }
  }

  public boolean isShooterAtVelocity(){
    return shooterTop.atVelocity(500) && shooterBottom.atVelocity(500);
  }


  public void disableBrakeMode() {
    //intakeBottom.setIdleCoastMode();
    //intakeTop.setIdleCoastMode();
    // shooterTop.setIdleCoastMode();
    // shooterBottom.setIdleCoastMode();
  }

  public void enableBrakeMode() {
    //intakeBottom.setIdleBrakeMode();
    //intakeTop.setIdleBrakeMode();
    // shooterTop.setIdleBrakeMode();
    // shooterBottom.setIdleBrakeMode();
  }

  public void zeroEncoders() {
    intakeBottom.resetEncoder();
    intakeTop.resetEncoder();
    shooterBottom.resetEncoder();
    shooterTop.resetEncoder();
  }
}
