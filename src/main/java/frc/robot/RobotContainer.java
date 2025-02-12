// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
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

    private final SendableChooser<Command> pathPlannerAutoChooser;

    private final TalonFXMotor.MusicPlayer musicPlayer = new TalonFXMotor.MusicPlayer();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        Zwerve.Config swerveConfig = new Zwerve.Config();
        swerveConfig.ROBOT_WIDTH = 0.7;
        swerveConfig.ROBOT_LENGTH = 0.7;
        swerveConfig.MAX_SPEED = 3;
        swerveConfig.MAX_ANGULAR_VELOCITY = 2 * Math.PI;

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

        this.drivetrain = new Zwerve(swerveConfig, new Pose2d()); //TODO: Set initial pose

        PathPlanner.initInstance(this.drivetrain);

        // Configure the button bindings
        this.configureButtonBindings();

        this.drivetrain.setFieldCentric(true);
        this.drivetrain.setDirectAngle(true);
        this.setDriveCommand();

        // Build an auto chooser
        pathPlannerAutoChooser = PathPlanner.getInstance().buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", pathPlannerAutoChooser);

        try (var camera = CameraServer.startAutomaticCapture()) {
            camera.setExposureAuto();
            camera.setWhiteBalanceAuto();
        }

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
        var translation2dSupplier = new Translation2dSupplier(() -> -this.driver.getLeftY(),
                () -> -this.driver.getLeftX());

        /*
          Converts driver input into a ChassisSpeeds that is controlled by angular velocity.
         */
        var angularVelocityInput = new Zwerve.InputStream(this.drivetrain, translation2dSupplier).rotation(
                this.driver::getRightX).deadband(0.05);

        /*
          Clone's the angular velocity input stream and converts it to a direct angle input stream.
         */
        var directAngleInput = new Zwerve.InputStream(this.drivetrain, translation2dSupplier).heading(
                new Rotation2dSupplier(() -> -this.driver.getRightX(), () -> -this.driver.getRightY())).deadband(0.05);

        /*
          Direct angle input can only be used in field centric mode.
         */
        this.drivetrain.setDefaultCommand(
                this.drivetrain.getDriveCommand(directAngleInput, angularVelocityInput, this.drivetrain::getDirectAngle,
                        this.drivetrain::getFieldCentric));
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
        return this.pathPlannerAutoChooser.getSelected();
    }

    public void setMotorBrake(boolean brake) {
        this.drivetrain.setMotorBrake(brake);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("Voltage", this.powerDistribution.getVoltage());
        SmartDashboard.putData("Drivetrain", this.drivetrain);
        SmartDashboard.putData("Field", this.drivetrain.getField());
        SmartDashboard.putData("Music Player", this.musicPlayer);
    }

    public TalonFXMotor.MusicPlayer getMusicPlayer() {
        return this.musicPlayer;
    }

    public CommandXboxController getDriverController() {
        return this.driver;
    }
}
