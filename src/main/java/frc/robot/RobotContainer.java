// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.libzodiac.drivetrain.Zwerve;
import frc.libzodiac.hardware.TalonFXMotor;
import frc.libzodiac.hardware.group.TalonFXSwerveModule;
import frc.libzodiac.util.CommandUtil;
import frc.libzodiac.util.Rotation2dSupplier;
import frc.libzodiac.util.Translation2dSupplier;

import java.util.Collection;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The driver's controller
    private final CommandXboxController driver = new CommandXboxController(0);
    // The robot's subsystems
    private final Zwerve drivetrain;
    private final PowerDistribution powerDistribution = new PowerDistribution();

    private final TalonFXMotor.MusicPlayer musicPlayer = new TalonFXMotor.MusicPlayer();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        Zwerve.Config swerveConfig = new Zwerve.Config();
        swerveConfig.ROBOT_WIDTH = 0.7;
        swerveConfig.ROBOT_LENGTH = 0.7;
        swerveConfig.MAX_SPEED = 3;
        swerveConfig.MAX_ANGULAR_SPEED = 2 * Math.PI;

        swerveConfig.frontLeft = new TalonFXSwerveModule.Config(1, 5, 9, 2215, true, true);
        swerveConfig.rearLeft = new TalonFXSwerveModule.Config(2, 6, 10, 1917, true, true);
        swerveConfig.frontRight = new TalonFXSwerveModule.Config(4, 8, 12, 1914, true, true);
        swerveConfig.rearRight = new TalonFXSwerveModule.Config(3, 7, 11, 3328, true, true);

        swerveConfig.gyro = 0;

        swerveConfig.headingController = new PIDController(0.4, 0.01, 0.01);
        swerveConfig.headingController.setIZone(Math.PI / 4);

        swerveConfig.ANGLE_GEAR_RATIO = 150.0 / 7.0;
        swerveConfig.DRIVE_GEAR_RATIO = 6.75;
        swerveConfig.WHEEL_RADIUS = 0.05;

        swerveConfig.drivePid = new PIDController(0.2, 7.5, 0.0005);
        swerveConfig.anglePid = new PIDController(0.5, 1, 0.0005);

        this.drivetrain = new Zwerve(swerveConfig);

        // Configure the button bindings
        this.configureButtonBindings();

        this.drivetrain.setFieldCentric(true);
        this.setDirectAngle(true);
        this.setDriveCommand();

        Collection<ParentDevice> motors = this.drivetrain.getMotors();
        this.musicPlayer.addInstrument(motors);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        driver.a().onTrue(Commands.runOnce(this::toggleFieldCentric));
        driver.b().onTrue(Commands.none());
        driver.x().onTrue(Commands.none());
        driver.y().onTrue(Commands.runOnce(this::zeroHeading));
        driver.leftBumper().onTrue(Commands.none());
        driver.rightBumper().onChange(Commands.runOnce(this::toggleDirectAngle));
        driver.start().onTrue(Commands.none());
        driver.back().whileTrue(Commands.runOnce(drivetrain::centerModules).repeatedly());
    }

    public void setDirectAngle(boolean directAngle) {
        this.drivetrain.setDirectAngle(directAngle);
        this.setDriveCommand();
    }

    private void setDriveCommand() {
        var translation2dSupplier = new Translation2dSupplier(() -> -driver.getLeftY(), () -> -driver.getLeftX());

        /*
          Converts driver input into a ChassisSpeeds that is controlled by angular velocity.
         */
        var angularVelocityInput = new Zwerve.SwerveInputStream(drivetrain, translation2dSupplier).withRotation(
                driver::getRightX).deadband(0.05);

        /*
          Clone's the angular velocity input stream and converts it to a direct angle input stream.
         */
        var directAngleInput = new Zwerve.SwerveInputStream(drivetrain, translation2dSupplier).withHeading(
                new Rotation2dSupplier(() -> -driver.getRightX(), () -> -driver.getRightY())).deadband(0.05);

        /*
          Direct angle input can only be used in field centric mode.
         */
        this.drivetrain.setDefaultCommand(
                this.drivetrain.driveCommand(directAngleInput, angularVelocityInput, this.drivetrain.getDirectAngle(),
                        this.drivetrain.getFieldCentric()));
    }

    public void toggleFieldCentric() {
        this.drivetrain.toggleFieldCentric();
        this.setDriveCommand();
        CommandUtil.rumbleController(driver.getHID(), 0.5, 0.5);
    }

    private void zeroHeading() {
        this.drivetrain.zeroHeading();
        CommandUtil.rumbleController(driver.getHID(), 0.5, 0.5);
    }

    public void toggleDirectAngle() {
        this.drivetrain.toggleDirectAngle();
        this.setDriveCommand();
        CommandUtil.rumbleController(driver.getHID(), 0.3, 0.2);
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
        drivetrain.setMotorBrake(brake);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("Voltage", powerDistribution.getVoltage());
        SmartDashboard.putData("Drivetrain", drivetrain);
        SmartDashboard.putData("Music Player", musicPlayer);
    }

    public TalonFXMotor.MusicPlayer musicPlayer() {
        return musicPlayer;
    }

    public CommandXboxController getDriverController() {
        return driver;
    }
}
