// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.libzodiac.hardware.LEDController;
import frc.libzodiac.ui.Elastic;
import frc.libzodiac.util.GameUtil;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    private final RobotContainer bot;
    private final Timer disabledTimer;
    private Command autonomousCommand;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {
        this.bot = new RobotContainer();
        this.disabledTimer = new Timer();
        LEDController.initInstance(0, 180);
        LEDController.getInstance().add("LED0", 0, 119);
        LEDController.getInstance().add("LED1", 120, 179);
    }

    @Override
    public void driverStationConnected() {
        this.bot.getMusicPlayer().loadMusic("music/SuperMarioFlag-3.chrp", 3);
        this.bot.getMusicPlayer().play();

        this.bot.getDriverController().setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        Timer.delay(0.5);
        this.bot.getDriverController().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        this.bot.brake();
        this.disabledTimer.reset();
        this.disabledTimer.start();
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        this.bot.getMusicPlayer().stop();

        Elastic.selectTab("Autonomous");
        this.bot.brake();

        this.autonomousCommand = this.bot.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (this.autonomousCommand != null) {
            this.autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (this.autonomousCommand != null) {
            this.autonomousCommand.cancel();
        }

        this.bot.getMusicPlayer().stop();
        Elastic.selectTab("Teleoperated");
    }

    @Override
    public void testInit() {
        this.bot.getMusicPlayer().stop();
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        SmartDashboard.putData("Robot", this.bot);
    }

    @Override
    public void disabledPeriodic() {
        if (this.disabledTimer.hasElapsed(1)) {
            this.bot.shutdown();
            this.disabledTimer.stop();
        }

        var pattern = LEDPattern.solid(
                                        GameUtil.isRedAlliance() ? Color.kRed : Color.kBlue)
                                .breathe(Units.Seconds.of(2));
        LEDController.getInstance().applyAll(pattern);
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
