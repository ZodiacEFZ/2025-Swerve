package frc.robot.subsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.hardware.MagEncoder;
import frc.libzodiac.hardware.TalonFXMotor;

public class Climber extends SubsystemBase {
    private final MagEncoder encoder = new MagEncoder(30, 0);
    private final TalonFXMotor motor = new TalonFXMotor(31);
    private Position position = Position.DOWN;

    public Climber() {
        this.encoder.setInverted(true);
        this.encoder.setContinuous(false);
        this.motor.factoryDefault();
        var climberPID = new PIDController(75, 1, 0);
        this.motor.setPID(climberPID);
        this.motor.setBrakeWhenNeutral(true);
        this.motor.setSensorToMechanismRatio(200);
        var encoderAngle = this.encoder.get().in(Units.Radians);
        var angle = new Rotation2d(Math.cos(encoderAngle), Math.sin(encoderAngle)).getMeasure();
        this.motor.setRelativeEncoderPosition(
                angle.in(Units.Radians) < 0 ? angle.plus(Units.Radians.of(Math.PI * 2)) : angle);
        this.motor.setSoftwareLimitSwitch(Position.DOWN.position, Position.CLIMB.position);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climber", this.getPosition().in(Units.Radians));
    }

    private Angle getPosition() {
        return this.motor.getPosition();
    }

    public Command getDownCommand() {
        return runOnce(this::down);
    }

    public void down() {
        this.motor.setPosition(Position.DOWN.position);
        this.position = Position.DOWN;
    }

    public Command getUpCommand() {
        return runOnce(this::up);
    }

    public void up() {
        this.motor.setPosition(Position.UP.position);
        this.position = Position.UP;
    }

    public Command getClimbCommand() {
        return runOnce(this::climb);
    }

    public void climb() {
        this.motor.setPosition(Position.CLIMB.position);
        this.position = Position.CLIMB;
    }

    public Command getSwitchClimberStateCommand() {
        return runOnce(this::switchClimberState);
    }

    private void switchClimberState() {
        if (this.position == Position.UP) {
            this.down();
        } else {
            this.up();
        }
    }

    enum Position {
        DOWN(0),
        UP(1.4),
        CLIMB(4);

        final Angle position;

        Position(double rad) {
            this.position = Units.Radians.of(rad);
        }
    }
}

