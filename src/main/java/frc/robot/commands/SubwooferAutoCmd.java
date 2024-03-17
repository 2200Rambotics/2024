package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.ShooterSubsystem.IntakeState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class SubwooferAutoCmd extends Command {
  private final ShooterSubsystem shooter;
  private final ArmSubsystem arm;
  int state;
  Timer shootTimer;
  boolean isDone;

  public SubwooferAutoCmd(ArmSubsystem arm, ShooterSubsystem shooter) {
    this.arm = arm;
    this.shooter = shooter;
    // addRequirements(shooter);
    shootTimer = new Timer();
  }

  @Override
  public void initialize() {
    state = 0;
    isDone = false;
    shootTimer.restart();
    arm.unsafeSetPosition(ArmPosition.SubWoofer);
  }
  
  @Override
  public void execute() {
    switch(state){
      case 0:
        shooter.shooterState = ShooterState.SpinFixed;
        shooter.shooterV = Constants.SUBWOOFER_SHOOT_SPEED;
        if(arm.leftShoulderMotor.atPosition() && arm.wristMotor.atPosition() && shootTimer.get() > 0.5){
          shootTimer.restart();
          state = 1;
        }
        break;
      case 1:
        shooter.intakeState = IntakeState.ShootNow;
        if(shootTimer.get() > 0.3){
          isDone = true;
        }
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    arm.unsafeSetPosition(ArmPosition.Stowed);
    shooter.intakeState = IntakeState.Idle;
    shooter.shooterState = ShooterState.Idle;
    shooter.spinDownShooters();
  }

  @Override
  public boolean isFinished() {
    return isDone;
  }
}
