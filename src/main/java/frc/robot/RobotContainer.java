// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCMD;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;


public class RobotContainer {
  private final XboxController xbox = new XboxController(3);
  
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final InstantCommand resetGyro = new InstantCommand(swerveSubsystem::zeroHeading, swerveSubsystem);
  private final SwerveJoystickCMD swerveCMD = new SwerveJoystickCMD(swerveSubsystem,
                () -> -xbox.getRawAxis(OIConstants.kDriverYAxis),
                () -> xbox.getRawAxis(OIConstants.kDriverXAxis),
                () -> xbox.getRawAxis(OIConstants.kDriverRotAxis),
                () -> true/*
                () -> !xbox.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx */);

  // Autonomous Commands
  String onePieceJSON = "output/OnePiece.wpilib.json";
  String onePieceReturnJSON = "output/OnePieceReturn.wpilib.json";
  Trajectory selectedPath = new Trajectory();

 


  public RobotContainer() {
    CommandScheduler.getInstance().setDefaultCommand(swerveSubsystem, swerveCMD);
    configureBindings();
  }

 
  private void configureBindings() {
    Constants.OperatorConstants.button2.onTrue(resetGyro);

  }

  
  public Command getAutonomousCommand() {
      PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
      PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
      ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    
      try {
        Path onePiecePath = Filesystem.getDeployDirectory().toPath().resolve(onePieceJSON);
        Trajectory onePieceTraj = TrajectoryUtil.fromPathweaverJson(onePiecePath);
        selectedPath = onePieceTraj;
    
     } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + onePieceJSON, ex.getStackTrace());

        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);
          // generate trajectory
        selectedPath = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), 
        List.of(
          new Translation2d(1, 0),
          new Translation2d(1, -1)), 
          new Pose2d(2, -1, Rotation2d.fromDegrees(180)), 
          trajectoryConfig);
     }


    // create command to follow trajectory
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      selectedPath, 
        swerveSubsystem::getPose, 
        DriveConstants.kDriveKinematics, 
        xController, 
        yController,
        thetaController,
        swerveSubsystem::setModuleStates,
        swerveSubsystem);


      // add some init and wrap up, and return everything
    return new SequentialCommandGroup(
      new InstantCommand(() -> swerveSubsystem.resetOdometry(selectedPath.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> swerveSubsystem.stopModules())
    );
  }
}
