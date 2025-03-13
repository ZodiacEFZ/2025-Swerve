package frc.robot.subsystem;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.hardware.MagEncoder;
import frc.libzodiac.hardware.SparkMaxMotor;
import frc.libzodiac.hardware.TalonFXMotor;
import frc.libzodiac.util.Maths;

import java.util.Collection;
import java.util.HashSet;

public class ArmStage2 extends SubsystemBase {
    private static final Angle POSITION_LIMIT = Units.Degrees.of(-80);
    private static final Angle REVERSE_POSITION_LIMIT = Units.Degrees.of(-45);
    private static final Angle FORWARD_POSITION_LIMIT = Units.Degrees.of(115);
    private static final Angle INTAKE_POSITION = Units.Degrees.of(0);
    private static final Angle L1_POSITION = Units.Degrees.of(-25);
    private static final Angle L2_POSITION = Units.Degrees.of(-12);
    private static final Angle L3_POSITION = Units.Degrees.of(13);
    private static final Angle L4_POSITION = Units.Degrees.of(80);
    private static final Angle ROTATION_HORIZONTAL = Units.Rotations.of(0);
    private static final Angle ROTATION_HORIZONTAL_REVERSED = Units.Rotations.of(
            42.0 * 60 / 18 / 2);
    private static final Angle ROTATION_VERTICAL = ROTATION_HORIZONTAL_REVERSED.div(2);
    private final MagEncoder encoder = new MagEncoder(27, -3700);
    private final TalonFXMotor motor = new TalonFXMotor(28);
    private final SparkMaxMotor wrist = new SparkMaxMotor(24);
    private Angle position = null;

    public ArmStage2() {
        this.encoder.setInverted(true);
        this.encoder.setContinuous(false);

        this.motor.factoryDefault();
        this.motor.setInverted(true);
        var slot0Config = new Slot0Configs();
        slot0Config.kP = 50;
        slot0Config.kI = 0;
        slot0Config.kD = 0;
        slot0Config.kS = 0;
        slot0Config.kV = 0;
        slot0Config.kA = 0;
        slot0Config.kG = 0.35;
        slot0Config.GravityType = GravityTypeValue.Arm_Cosine;
        this.motor.applyConfiguration(slot0Config);
        var motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = 1;
        motionMagicConfigs.MotionMagicAcceleration = 5;
        motionMagicConfigs.MotionMagicJerk = 10;
        this.motor.applyConfiguration(motionMagicConfigs);
        this.motor.setBrakeWhenNeutral(true);
        this.motor.setSensorToMechanismRatio(58.0 / 50 * 100);
        var angle = Maths.limitAngle(this.encoder.get());
        this.motor.setRelativeEncoderPosition(Maths.limitAngle(angle, POSITION_LIMIT));
        this.motor.setSoftwareLimitSwitch(REVERSE_POSITION_LIMIT, FORWARD_POSITION_LIMIT);

        this.wrist.factoryDefault();
        this.wrist.setPID(0.5, 0, 0.5);
        var config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.inverted(true);
        config.closedLoop.pid(0.5, 0, 0.5);
        config.closedLoop.maxMotion.allowedClosedLoopError(0.1);
        config.closedLoop.maxMotion.maxAcceleration(6000).maxVelocity(4096);
        config.softLimit.forwardSoftLimitEnabled(true)
                        .forwardSoftLimit(ROTATION_HORIZONTAL_REVERSED.in(Units.Rotations))
                        .reverseSoftLimitEnabled(true)
                        .reverseSoftLimit(ROTATION_HORIZONTAL.in(Units.Rotations));
        this.wrist.applyConfiguration(config);
    }

    public Command getMoveToCommand(Angle position) {
        return runOnce(() -> this.moveTo(position));
    }

    public void moveTo(Angle position) {
        this.motor.MotionMagicPosition(position);
    }

    public Command getRestoreCommand() {
        return runOnce(this::restore);
    }

    public void restore() {
        if (this.position == null) {
            return;
        }
        this.moveTo(this.position);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        SmartDashboard.putData("L4", this.getL4Command());
        SmartDashboard.putData("L3", this.getL3Command());
        SmartDashboard.putData("L2", this.getL2Command());
        SmartDashboard.putData("L1", this.getL1Command());
        SmartDashboard.putData("Intake", this.getIntakeCommand());
        SmartDashboard.putData("Drop", this.getDropCommand());
        SmartDashboard.putData("Idle", this.getIdleCommand());

        SmartDashboard.putData("Rotate Horizontal", this.getRotateCommand(ROTATION_HORIZONTAL));
        SmartDashboard.putData("Rotate Vertical", this.getRotateCommand(ROTATION_VERTICAL));
        SmartDashboard.putData("Rotate Horizontal Reversed",
                               this.getRotateCommand(ROTATION_HORIZONTAL_REVERSED));
    }

    public Command getL4Command() {
        return runOnce(this::L4);
    }

    public Command getL3Command() {
        return runOnce(this::L3);
    }

    public Command getL2Command() {
        return runOnce(this::L2);
    }

    public Command getL1Command() {
        return runOnce(this::L1);
    }

    public Command getIntakeCommand() {
        return runOnce(this::intake);
    }

    public Command getDropCommand() {
        return runOnce(this::drop);
    }

    public Command getIdleCommand() {
        return runOnce(this::idle);
    }

    public Command getRotateCommand(Angle position) {
        return runOnce(() -> this.rotate(position));
    }

    public void L4() {
        this.moveTo(L4_POSITION);
        this.rotate(ROTATION_HORIZONTAL_REVERSED);
        this.position = L4_POSITION;
    }

    public void L3() {
        this.moveTo(L3_POSITION);
        this.rotate(ROTATION_HORIZONTAL);
        this.position = L3_POSITION;
    }

    public void L2() {
        this.moveTo(L2_POSITION);
        this.rotate(ROTATION_HORIZONTAL);
        this.position = L2_POSITION;
    }

    public void L1() {
        this.moveTo(L1_POSITION);
        this.rotate(ROTATION_HORIZONTAL);
        this.position = L1_POSITION;
    }

    public void intake() {
        this.moveTo(INTAKE_POSITION);
        this.rotate(ROTATION_VERTICAL);
        this.position = INTAKE_POSITION;
    }

    public void drop() {
        if (this.position == null || this.position.equals(INTAKE_POSITION)) {
            return;
        }
        if (this.position.equals(L4_POSITION)) {
            this.moveTo(L4_POSITION.plus(Units.Degrees.of(10)));
        } else {
            this.moveTo(this.position.minus(Units.Degrees.of(10)));
        }
    }

    public void idle() {
        this.moveTo(REVERSE_POSITION_LIMIT);
        this.rotate(ROTATION_HORIZONTAL);
        this.position = null;
    }

    public void rotate(Angle position) {
        this.wrist.MAXMotionPosition(position);
    }

    public Collection<TalonFX> getTalonFXMotors() {
        Collection<TalonFX> motors = new HashSet<>();
        motors.add(this.motor.getMotor());
        return motors;
    }
}
