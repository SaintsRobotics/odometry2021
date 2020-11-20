/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class MoveToPosition extends CommandBase {
  /**
   * Creates a new MoveToPosition.
   */
  private SwerveDrivetrain m_drivetrain;
  private PIDController m_xPID;
  private PIDController m_yPID;
  private PIDController m_anglePID;
  private Pose2d m_currentPose;
  private Pose2d m_targetPose;

  private double m_xSpeed;
  private double m_ySpeed;
  private double m_angleSpeed;

  public MoveToPosition(SwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);

    m_xPID.reset();
    m_yPID.reset();
    m_anglePID.reset();
    //pids
    m_xPID = new PIDController(.3, 0, 0);
    m_yPID = new PIDController(.3, 0, 0);
    m_anglePID = new PIDController(.3, 0, 0);
    //pose
    m_currentPose = m_drivetrain.getLocation();
    m_targetPose = new Pose2d(10, 10, new Rotation2d(2));

    //input
    m_xPID.enableContinuousInput(0, Math.PI);
    m_yPID.enableContinuousInput(0, Math.PI);
    m_anglePID.enableContinuousInput(0, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //m_drivetrain.move(x, y, a, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_xSpeed = m_xPID.calculate(m_currentPose.getTranslation().getX(), m_targetPose.getTranslation().getX());
    m_ySpeed = m_xPID.calculate(m_currentPose.getTranslation().getY(), m_targetPose.getTranslation().getY());
    m_angleSpeed = m_anglePID.calculate(m_currentPose.getRotation().getRadians(), m_targetPose.getRotation().getRadians());

    m_drivetrain.move(m_xSpeed, m_ySpeed, m_angleSpeed, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.move(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_currentPose.getTranslation().getX() == m_targetPose.getTranslation().getX() && m_currentPose.getTranslation().getY() == m_targetPose.getTranslation().getY() && m_currentPose.getRotation().getRadians() == m_targetPose.getRotation().getRadians()){
      return true;
    }

    return false;
  }
}
