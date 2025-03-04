package frc.robot.subsystem;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libzodiac.hardware.SparkMaxMotor;
import frc.libzodiac.hardware.TalonSRXMotor;
import frc.libzodiac.util.Rectangle;

/**
 * The arm on the robot.
 */
public class Arm extends SubsystemBase {
    private final TalonSRXMotor shoulderLeader = new TalonSRXMotor(21);
    private final TalonSRXMotor shoulderFollower = new TalonSRXMotor(22);
    private final TalonSRXMotor elbow = new TalonSRXMotor(23);
    private final SparkMaxMotor wrist = new SparkMaxMotor(24);

    private final Distance posteriorArmLength = Units.Meters.of(0.86);
    private final Distance forearmLength = Units.Meters.of(0.77);
    private final Distance intakeLength = Units.Meters.of(0.25);

    /**
     * The position of the arm installed on the robot. The x component is the distance from the front of the robot to
     * the axis of the first stage of the arm, viewing from the left side of the robot. The y component is the height of
     * the first stage of the arm.
     */
    private final Translation2d armPosition = new Translation2d(0.2, 0.24);
    /**
     * The position limit of the arm's components. The x component is the distance from the front of the robot to the
     * axis of the first stage of the arm, viewing from the left side of the robot. The y component is the height of the
     * first stage of the arm.
     */
    // There is no top limit for the arm so the y component is set to 10 meters.
    private final Rectangle positionLimit = new Rectangle(new Translation2d(-0.3, 0), new Translation2d(1, 10));
    private Pose2d targetPose;

    /**
     * Constructs an Arm.
     */
    public Arm() {
        this.shoulderLeader.factoryDefault();
        this.shoulderFollower.factoryDefault();
        this.elbow.factoryDefault();
        this.wrist.factoryDefault();

        this.wrist.setPID(0.1, 0, 0);
        var config = new SparkMaxConfig();
        config.closedLoop.maxMotion.maxAcceleration(0.1).maxVelocity(0.2);
        this.wrist.applyConfiguration(config);
        this.wrist.setSensorToMechanismRatio(100); // 30 * 60 / 18

        this.targetPose = new Pose2d(this.getPosition(), this.getIntakeRotation());
    }

    private Translation2d getPosition() {
        return this.armPosition.plus(this.getStage2Position());
    }

    private Rotation2d getIntakeRotation() {
        return new Rotation2d(this.wrist.getPosition());
    }

    private Translation2d getStage2Position() {
        return this.getStage1Position()
                       .plus(new Translation2d(
                               this.forearmLength.times(Math.cos(this.getStage2Angle().in(Units.Radians))),
                               this.forearmLength.times(Math.sin(this.getStage2Angle().in(Units.Radians)))));
    }

    private Translation2d getStage1Position() {
        return new Translation2d(this.posteriorArmLength.times(Math.cos(this.getStage1Angle().in(Units.Radians))),
                this.posteriorArmLength.times(Math.sin(this.getStage1Angle().in(Units.Radians))));
    }

    private Angle getStage2Angle() {
        return this.getStage1Angle().plus(this.getStage2AngleRaw());
    }

    private Angle getStage1Angle() {
        return this.shoulderLeader.getPosition();
    }

    private Angle getStage2AngleRaw() {
        return this.elbow.getPosition();
    }

    private Translation2d calculatePosition(Angle stage1Angle, Angle stage2Angle) {
        double l1 = this.posteriorArmLength.in(Units.Meters);
        double l2 = this.forearmLength.in(Units.Meters);
        double theta1 = stage1Angle.in(Units.Radians);
        double theta2 = stage2Angle.in(Units.Radians);
        double x = l1 * Math.cos(theta1) + l2 * Math.cos(theta1 + theta2);
        double y = l1 * Math.sin(theta1) + l2 * Math.sin(theta1 + theta2);
        return new Translation2d(x, y);
    }

    /**
     * Calculates the angles of the stages of the arm with the given position. The first element of the pair is the
     * angle of the first stage. The second element of the pair is the angle of the second stage.
     *
     * @param position  The position of the end of the arm.
     * @param rightHand Whether the arm is in the right hand system.
     *
     * @return A Pair<Angle, Angle>, representing the angles of the stages of the arm.
     */
    private Pair<Angle, Angle> calculateStageAngles(Translation2d position, boolean rightHand) {
        var stage2Position = position.minus(this.armPosition);
        double l0 = stage2Position.getNorm();
        double l1 = this.posteriorArmLength.in(Units.Meters);
        double l2 = this.forearmLength.in(Units.Meters);

//        double theta0 = Math.atan2(stage2Position.getY(), stage2Position.getX());
//        double delta1 = Math.acos((l0 * l0 + l1 * l1 - l2 * l2) / (2 * l0 * l1));
//        double delta2 = Math.acos((l0 * l0 - l1 * l1 + l2 * l2) / (2 * l0 * l2));
//        if (upper) {
//            double theta1 = theta0 + delta1;
//            double theta2 = theta0 - theta1 - delta2;
//            if (theta1 < Math.PI) {
//                return new Pair<>(Units.Radians.of(theta1), Units.Radians.of(theta2));
//            }
//        }
//        double theta1 = theta0 - delta1;
//        double theta2 = theta0 - theta1 + delta2;

        double cosTheta2 = (l0 * l0 - l1 * l1 - l2 * l2) / (2 * l1 * l2);
        double sinTheta2 = Math.sqrt(1 - cosTheta2 * cosTheta2);
        double theta1, theta2;
        theta2 = rightHand ? Math.atan2(sinTheta2, cosTheta2) : Math.atan2(-sinTheta2, cosTheta2);
        theta1 = Math.atan2(stage2Position.getY(), stage2Position.getX()) -
                         Math.atan2(l2 * sinTheta2, l1 + l2 * cosTheta2);
        return new Pair<>(Units.Radians.of(theta1), Units.Radians.of(theta2));
    }

    public void moveTo(Translation2d position) {
        if (!this.isWithinPositionLimit(position)) {
            return;
        }
        this.targetPose = new Pose2d(position, this.targetPose.getRotation());
    }

    private boolean isWithinPositionLimit(Translation2d position) {
        return this.positionLimit.contains(position);
    }

    @Override
    public void periodic() {
        // TODO: Implement the control logic to move the arm to the target position.
        Translation2d deltaPosition = this.targetPose.getTranslation().minus(this.getPosition());
        this.wrist.MAXMotionPosition(this.targetPose.getRotation().getMeasure());
    }
}
