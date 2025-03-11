// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
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
import frc.libzodiac.hardware.CANCoder;
import frc.libzodiac.hardware.Limelight;
import frc.libzodiac.hardware.Pigeon;
import frc.libzodiac.hardware.TalonFXMotor;
import frc.libzodiac.hardware.group.TalonFXSwerveModule;
import frc.libzodiac.util.CommandUtil;
import frc.libzodiac.util.Rotation2dSupplier;
import frc.libzodiac.util.Translation2dSupplier;
import frc.robot.subsystem.ArmStage2;
import frc.robot.subsystem.Climber;
import frc.robot.subsystem.Intake;

import java.util.Arrays;
import java.util.Collection;
import java.util.stream.IntStream;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements Sendable {
    // The driver's controller
    private final CommandXboxController driver = new CommandXboxController(0);
    //    private final CommandXboxController operator = new CommandXboxController(1);
    private final Swerve drivetrain;
    private final Intake intake = new Intake();
    private final PowerDistribution powerDistribution = new PowerDistribution(63, PowerDistribution.ModuleType.kRev);
    private final ArmStage2 armStage2 = new ArmStage2();
    private final Limelight limelight;
    private final SendableChooser<Command> autoChooser;
    private final TalonFXMotor.MusicPlayer musicPlayer = new TalonFXMotor.MusicPlayer();
    private final Climber climber = new Climber();

    private boolean autoHeading = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // TODO: tune PID arguments for each swerve module
        final var frontLeft = new TalonFXSwerveModule.Config().withAngle(1)
                                                              .withDrive(5)
                                                              .withEncoder(new CANCoder(9, 0.294678))
                                                              .withAngleInverted(true)
                                                              .withDriveInverted(true);
        final var rearLeft = new TalonFXSwerveModule.Config().withAngle(2)
                                                             .withDrive(6)
                                                             .withEncoder(new CANCoder(10, -0.251709))
                                                             .withAngleInverted(true)
                                                             .withDriveInverted(true);
        final var frontRight = new TalonFXSwerveModule.Config().withAngle(4)
                                                               .withDrive(8)
                                                               .withEncoder(new CANCoder(12, 0.035645))
                                                               .withAngleInverted(true)
                                                               .withDriveInverted(true);
        final var rearRight = new TalonFXSwerveModule.Config().withAngle(3)
                                                              .withDrive(7)
                                                              .withEncoder(new CANCoder(11, 0.566162))
                                                              .withAngleInverted(true)
                                                              .withDriveInverted(true);

        final var heading = new PIDController(1.75, 0.025, 0.2);
        heading.setIZone(Math.PI / 8);

        var constraints = new PathConstraints(Units.MetersPerSecond.of(5), Units.MetersPerSecondPerSecond.of(1.5),
                                              Units.DegreesPerSecond.of(360), Units.DegreesPerSecondPerSecond.of(720));

        this.drivetrain = new Swerve.Config().withRobotWidth(Units.Meters.of(0.7))
                                             .withRobotLength(Units.Meters.of(0.7))
                                             .withConstraints(constraints)
                                             .withFrontLeft(frontLeft)
                                             .withFrontRight(frontRight)
                                             .withRearLeft(rearLeft)
                                             .withRearRight(rearRight)
                                             .withDrivePID(new PIDController(0.2, 7.5, 0.0005))
                                             .withAnglePID(new PIDController(10, 10, 0.01))
                                             .withGyro(new Pigeon(13))
                                             .withHeadingPID(heading)
                                             .withDriveGearRatio(6.75)
                                             .withAngleGearRatio(150.0 / 7.0)
                                             .withWheelRadius(Units.Millimeter.of(50))
                                             // Initial pose can be blank because we use PathPlanner to set the initial pose
                                             .withInitialPose(new Pose2d())
                                             .build();

        // TODO: add path constraints
        PathPlanner.initInstance(this.drivetrain, constraints);

        // Configure the button bindings
        this.configureButtonBindings();

        this.drivetrain.setFieldCentric(true);
        this.drivetrain.setDirectAngle(true);
        this.setDriveCommand();

        this.limelight = new Limelight(this.drivetrain);
        this.limelight.setValidIDs(IntStream.rangeClosed(1, 22).toArray());
        this.limelight.setPipeline(0);

        // Build an auto chooser
        autoChooser = PathPlanner.getInstance().buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        //noinspection resource
        CameraServer.startAutomaticCapture();

        Collection<TalonFXMotor> motors = this.drivetrain.getTalonFXMotors();
        this.musicPlayer.addInstrument(motors);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
     * {@link XboxController}), and then calling passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        // Drivetrain
        this.driver.a().onTrue(Commands.runOnce(this::toggleAutoHeading));
        this.driver.x().onTrue(Commands.runOnce(this::toggleSlowMode));
        this.driver.back().onTrue(Commands.runOnce(this::zeroHeading));
        this.driver.start().onTrue(Commands.runOnce(this::toggleFieldCentric));
        this.driver.rightBumper().onChange(Commands.runOnce(this::toggleDirectAngle));
        this.driver.povLeft()
                   .onTrue(Commands.runOnce(
                           () -> this.drivetrain.setTargetHeading(new Rotation2d(Units.Degrees.of(-54)))));
        this.driver.povRight()
                   .onTrue(Commands.runOnce(
                           () -> this.drivetrain.setTargetHeading(new Rotation2d(Units.Degrees.of(54)))));

        // Arm
        this.driver.povLeft().onTrue(this.armStage2.getIntakeCommand());
        this.driver.povRight().onTrue(this.armStage2.getIntakeCommand());
        this.driver.povUp().onTrue(this.armStage2.getL4Command());
        this.driver.povDown().onTrue(this.armStage2.getIdleCommand());
        this.driver.b().onTrue(this.armStage2.getDropCommand()).onFalse(this.armStage2.getRestoreCommand());

        // Intake
        this.driver.leftTrigger().onTrue(this.intake.getOuttakeCommand()).onFalse(this.intake.getStopCommand());
        this.driver.rightTrigger().onTrue(this.intake.getIntakeCommand()).onFalse(this.intake.getStopCommand());

        //Climber
        this.driver.y().onTrue(this.climber.getSwitchClimberStateCommand());
        this.driver.leftBumper().onTrue(this.climber.getClimbCommand());
    }

    private void setDriveCommand() {
        var translation2dSupplier = new Translation2dSupplier(() -> -this.driver.getLeftY(),
                                                              () -> -this.driver.getLeftX());

        Rotation2dSupplier headingSupplier = new Rotation2dSupplier(() -> {
            var heading = new Rotation2d(-this.driver.getRightY(), -this.driver.getRightX());
            if (this.autoHeading) {
                var headings = new Rotation2d[]{
                        new Rotation2d(0), new Rotation2d(Math.PI / 3), new Rotation2d(Math.PI * 2 / 3),
                        new Rotation2d(Math.PI), new Rotation2d(-Math.PI * 2 / 3), new Rotation2d(-Math.PI / 3)
                };
                return Arrays.stream(headings).min((a, b) -> {
                    var a0 = a.minus(heading).getMeasure().abs(Units.Radians);
                    var b0 = b.minus(heading).getMeasure().abs(Units.Radians);
                    return Double.compare(a0, b0);
                }).get();
            }
            return heading;
        });

        /*
          Converts driver input into a ChassisSpeeds that is controlled by angular velocity.
         */
        var angularVelocityInput = new Swerve.InputStream(this.drivetrain, translation2dSupplier).rotation(
                () -> -this.driver.getRightX()).deadband(0.05);

        /*
          Clone's the angular velocity input stream and converts it to a direct angle input stream.
         */
        var directAngleInput = new Swerve.InputStream(this.drivetrain, translation2dSupplier).heading(headingSupplier)
                                                                                             .deadband(0.05);

        /*
          Direct angle input can only be used in field centric mode.
         */
        this.drivetrain.setDefaultCommand(
                this.drivetrain.getDriveCommand(directAngleInput, angularVelocityInput, this.drivetrain::getDirectAngle,
                                                this.drivetrain::getFieldCentric));
    }

    private void toggleAutoHeading() {
        this.autoHeading = !this.autoHeading;
    }

    private void toggleSlowMode() {
        this.drivetrain.toggleSlowMode();
        CommandUtil.rumbleController(this.driver.getHID(), 0.3, 0.5);
    }

    private void zeroHeading() {
        this.drivetrain.zeroHeading();
        CommandUtil.rumbleController(this.driver.getHID(), 0.5, 0.5);
    }

    public void toggleFieldCentric() {
        this.drivetrain.toggleFieldCentric();
        CommandUtil.rumbleController(this.driver.getHID(), 0.5, 0.5);
    }

    public void toggleDirectAngle() {
        this.drivetrain.toggleDirectAngle();
        CommandUtil.rumbleController(this.driver.getHID(), 0.3, 0.5);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return this.autoChooser.getSelected();
    }

    public void brake() {
        this.drivetrain.brake();
    }

    public void shutdown() {
        this.drivetrain.shutdown();
    }

    public TalonFXMotor.MusicPlayer getMusicPlayer() {
        return this.musicPlayer;
    }

    public CommandXboxController getDriverController() {
        return this.driver;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Robot");
        builder.setActuator(true);
        builder.addBooleanProperty("Auto Heading", () -> this.autoHeading,
                                   (autoHeading) -> this.autoHeading = autoHeading);

        builder.addDoubleProperty("Match Time", DriverStation::getMatchTime, null);
        builder.addDoubleProperty("Voltage", this.powerDistribution::getVoltage, null);
        SmartDashboard.putData("Drivetrain", this.drivetrain);
        SmartDashboard.putData("Field", this.drivetrain.getField());
        SmartDashboard.putData("Arm", this.armStage2);

        PathPlanner.getInstance().createPath(, , , );
    }
}
