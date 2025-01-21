// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.libzodiac.hardware.group.TalonFXSwerveModule;
import frc.libzodiac.subsystem.Zwerve;
import frc.libzodiac.util.Rotation2dSupplier;
import frc.libzodiac.util.Translation2dSupplier;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The driver's controller
    final CommandXboxController driver = new CommandXboxController(0);
    // The robot's subsystems
    private final Zwerve swerve;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        Zwerve.SwerveConfig swerveConfig = new Zwerve.SwerveConfig();
        swerveConfig.ROBOT_WIDTH = 0.7;
        swerveConfig.ROBOT_LENGTH = 0.7;
        swerveConfig.MAX_SPEED = 3;
        swerveConfig.MAX_ANGULAR_SPEED = 2 * Math.PI;
        swerveConfig.frontLeft = new TalonFXSwerveModule(1, 5, 9, 2215, false, true);
        swerveConfig.rearLeft = new TalonFXSwerveModule(2, 6, 10, -1373, false, true);
        swerveConfig.frontRight = new TalonFXSwerveModule(4, 8, 12, 1856, false, true);
        swerveConfig.rearRight = new TalonFXSwerveModule(3, 7, 11, 3349, false, true);
        swerveConfig.gyroId = 0;
        swerveConfig.headingController = new PIDController(0.4, 0.01, 0.01);
        swerveConfig.headingController.setIZone(Math.PI / 4);
        this.swerve = new Zwerve(swerveConfig);

        // Configure the button bindings
        this.configureButtonBindings();

        this.swerve.setFieldCentric(true);
        // Configure default commands
        this.setDriveCommand(true);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        driver.a().onTrue(Commands.runOnce(swerve::toggleFieldCentric));
        driver.b().onTrue(Commands.none());
        driver.x().onTrue(Commands.none());
        driver.y().onTrue(Commands.runOnce(swerve::zeroHeading));
        driver.leftBumper().onTrue(Commands.none());
        driver.rightBumper().onTrue(Commands.runOnce(() -> setDriveCommand(false)).repeatedly())
                .onFalse(Commands.runOnce(() -> setDriveCommand(true)).repeatedly());
        driver.start().onTrue(Commands.runOnce(swerve::resetEncoders));
        driver.back().whileTrue(Commands.runOnce(swerve::centerModules));
    }

    public void setDriveCommand(boolean driveDirectAngle) {
        var translation = new Translation2dSupplier(() -> -driver.getLeftY(),
                () -> -driver.getLeftX());
        /*
          Converts driver input into a ChassisSpeeds that is controlled by angular velocity.
         */
        var angularVelocityInput = new Zwerve.SwerveInputStream(swerve, translation).withRotation(driver::getRightX)
                .deadband(0.05);

        var heading = new Rotation2dSupplier(driver::getRightX, driver::getRightY);

        /*
          Clone's the angular velocity input stream and converts it to a direct angle input stream.
         */
        var directAngleInput = new Zwerve.SwerveInputStream(swerve, translation).withRotation(driver::getRightX)
                .deadband(0.05).withHeading(heading);

        /*
          Direct angle input can only be used in field centric mode.
         */
        this.swerve.setDefaultCommand(this.swerve.driveCommand(
                (driveDirectAngle && this.swerve.getFieldCentric()) ? directAngleInput : angularVelocityInput,
                this.swerve.getFieldCentric()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
        //        public static final class AutoConstants {
        //            public static final double kMaxSpeedMetersPerSecond = 3;
        //            public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        //            public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        //            public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
        //
        //            public static final double kPXController = 1;
        //            public static final double kPYController = 1;
        //            public static final double kPThetaController = 1;
        //
        //            // Constraint for the motion profiled robot angle controller
        //            public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        //                    kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
        //        }
        //        // Create config for trajectory
        //        TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
        //                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //                // Add kinematics to ensure max speed is actually obeyed
        //                .setKinematics(DriveConstants.kinematics);
        //
        //        // An example trajectory to follow. All units in meters.
        //        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        //                // Start at the origin facing the +X direction
        //                Pose2d.kZero,
        //                // Pass through these two interior waypoints, making an 's' curve path
        //                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        //                // End 3 meters straight ahead of where we started, facing forward
        //                new Pose2d(3, 0, Rotation2d.kZero), config);
        //
        //        var thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
        //                AutoConstants.kThetaControllerConstraints);
        //        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        //
        //        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(exampleTrajectory,
        //                chassis::getPose, // Functional interface to feed supplier
        //                DriveConstants.kinematics,
        //
        //                // Position controllers
        //                new PIDController(AutoConstants.kPXController, 0, 0),
        //                new PIDController(AutoConstants.kPYController, 0, 0), thetaController, chassis::setModuleStates,
        //                chassis);
        //
        //        // Reset odometry to the initial pose of the trajectory, run path following
        //        // command, then stop at the end.
        //        return Commands.sequence(new InstantCommand(() -> chassis.resetOdometry(exampleTrajectory.getInitialPose())),
        //                swerveControllerCommand, new InstantCommand(() -> chassis.drive(0, 0, 0, false)));
    }

    public void setMotorBrake(boolean brake) {
        swerve.setMotorBrake(brake);
    }
}
