package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.hardware.TalonFXMotor;

import java.util.Collection;
import java.util.HashSet;

public class Intake extends SubsystemBase {
    private final TalonFXMotor intakeMotor = new TalonFXMotor(20);

    public Intake() {
        this.intakeMotor.factoryDefault();
        this.intakeMotor.setInverted(true);
        this.intakeMotor.setPID(0.1, 0, 0);
        this.intakeMotor.setBrakeWhenNeutral(true);
    }

    public Command getIntakeCommand() {
        return runOnce(this::intake);
    }

    public void intake() {
        this.intakeMotor.setVoltage(Units.Volts.of(4));
    }

    public Command getOuttakeCommand() {
        return runOnce(this::outtake);
    }

    public void outtake() {
        this.intakeMotor.setVoltage(Units.Volts.of(-4));
    }

    public Command getStopCommand() {
        return runOnce(this::stop);
    }

    public void stop() {
        this.intakeMotor.brake();
    }

    public Collection<TalonFX> getTalonFXMotors() {
        Collection<TalonFX> motors = new HashSet<>();
        motors.add(this.intakeMotor.getMotor());
        return motors;
    }
}

