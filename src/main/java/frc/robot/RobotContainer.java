// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.libzodiac.drivetrain.PathPlanner;
import frc.libzodiac.drivetrain.Swerve;
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
    private final Swerve drivetrain;
    private final PowerDistribution powerDistribution = new PowerDistribution();

    private final SendableChooser<Command> autoChooser;

    private final TalonFXMotor.MusicPlayer musicPlayer = new TalonFXMotor.MusicPlayer();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        Swerve.Config swerveConfig = new Swerve.Config();
        swerveConfig.ROBOT_WIDTH = 0.7;
        swerveConfig.ROBOT_LENGTH = 0.7;
        swerveConfig.MAX_SPEED = 3;
        swerveConfig.MAX_ANGULAR_VELOCITY = Math.PI;

        // TODO: tune PID arguments for each swerve module
        swerveConfig.drivePID = new PIDController(0.2, 7.5, 0.0005);
        swerveConfig.anglePID = new PIDController(10, 10, 0.01);

        swerveConfig.frontLeft = new TalonFXSwerveModule.Config(1, 5, 9, 2215)
                .withAngleReversion(true)
                .withDriveReversion(true);
        swerveConfig.rearLeft = new TalonFXSwerveModule.Config(2, 6, 10, -789)
                .withAngleReversion(true)
                .withDriveReversion(true);
        swerveConfig.frontRight = new TalonFXSwerveModule.Config(4, 8, 12, 1914)
                .withAngleReversion(true)
                .withDriveReversion(true);
        swerveConfig.rearRight = new TalonFXSwerveModule.Config(3, 7, 11, 3328)
                .withAngleReversion(true)
                .withDriveReversion(true);

        swerveConfig.gyro = 0;

        swerveConfig.headingController = new PIDController(1.75, 0.025, 0.2);
        swerveConfig.headingController.setIZone(Math.PI / 8);

        swerveConfig.ANGLE_GEAR_RATIO = 150.0 / 7.0;
        swerveConfig.DRIVE_GEAR_RATIO = 6.75;
        swerveConfig.WHEEL_RADIUS = 0.05;

        this.drivetrain = new Swerve(swerveConfig, new Pose2d()); //TODO: Set initial pose

        PathPlanner.initInstance(this.drivetrain);

        // Configure the button bindings
        this.configureButtonBindings();

        this.drivetrain.setFieldCentric(true);
        this.drivetrain.setDirectAngle(true);
        this.setDriveCommand();

        // Build an auto chooser
        autoChooser = PathPlanner.getInstance().buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        try (var camera = CameraServer.startAutomaticCapture()) {
            camera.setExposureAuto();
            camera.setWhiteBalanceAuto();
        }

        Collection<TalonFXMotor> motors = this.drivetrain.getTalonFXMotors();
        this.musicPlayer.addInstrument(motors);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        this.driver.a().onTrue(Commands.runOnce(this::toggleFieldCentric));
        this.driver.b().onTrue(Commands.none());
        this.driver.x().onTrue(Commands.none());
        this.driver.y().onTrue(Commands.runOnce(this::zeroHeading));
        this.driver.leftBumper().onTrue(Commands.none());
        this.driver.rightBumper().onChange(Commands.runOnce(this::toggleDirectAngle));
        this.driver.start().onTrue(Commands.none());
        this.driver.back().whileTrue(Commands.runOnce(this.drivetrain::centerModules).repeatedly());
    }

    private void setDriveCommand() {
        var translation2dSupplier = new Translation2dSupplier(() -> -this.driver.getLeftY(), () -> -this.driver.getLeftX());

        /*
          Converts driver input into a ChassisSpeeds that is controlled by angular velocity.
         */
        var angularVelocityInput = new Swerve.InputStream(this.drivetrain, translation2dSupplier).rotation(() -> -this.driver.getRightX()).deadband(0.05);

        /*
          Clone's the angular velocity input stream and converts it to a direct angle input stream.
         */
        var directAngleInput = new Swerve.InputStream(this.drivetrain, translation2dSupplier).heading(new Rotation2dSupplier(() -> -this.driver.getRightY(), () -> -this.driver.getRightX())).deadband(0.05);

        /*
          Direct angle input can only be used in field centric mode.
         */
        this.drivetrain.setDefaultCommand(this.drivetrain.getDriveCommand(directAngleInput, angularVelocityInput, this.drivetrain::getDirectAngle, this.drivetrain::getFieldCentric));
    }

    public void toggleFieldCentric() {
        this.drivetrain.toggleFieldCentric();
        CommandUtil.rumbleController(this.driver.getHID(), 0.5, 0.5);
    }

    private void zeroHeading() {
        this.drivetrain.zeroHeading();
        CommandUtil.rumbleController(this.driver.getHID(), 0.5, 0.5);
    }

    public void toggleDirectAngle() {
        this.drivetrain.toggleDirectAngle();
        CommandUtil.rumbleController(this.driver.getHID(), 0.3, 0.2);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return this.autoChooser.getSelected();
    }

    public void setMotorBrake(boolean brake) {
        this.drivetrain.setMotorBrake(brake);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("Voltage", this.powerDistribution.getVoltage());
        SmartDashboard.putData("Drivetrain", this.drivetrain);
        SmartDashboard.putData("Field", this.drivetrain.getField());
    }

    public TalonFXMotor.MusicPlayer getMusicPlayer() {
        return this.musicPlayer;
    }

    public CommandXboxController getDriverController() {
        return this.driver;
    }
}
