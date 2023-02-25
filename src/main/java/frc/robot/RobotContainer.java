// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.BeltControl;
import frc.robot.commands.ChangeLED;
import frc.robot.commands.ShooterControl;
import frc.robot.commands.Drivetrain.AutoAlign;
import frc.robot.commands.Drivetrain.AutoBalance;
import frc.robot.commands.Drivetrain.ChangeSpeed;
import frc.robot.commands.Drivetrain.GetOnRamp;
import frc.robot.commands.Drivetrain.TankDrive;
import frc.robot.commands.Drivetrain.ToggleDirection;
import frc.robot.commands.IntakeCmds.CustomSolControl;
import frc.robot.commands.IntakeCmds.IntakeMotorControl;
import frc.robot.commands.IntakeCmds.SolControl;
import frc.robot.commands.Utils.Time;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.RaspPiCam;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.AirSys;
import frc.robot.subsystems.Intake.MotorSys;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public final static DriveTrain m_drivetrain = new DriveTrain();
  public final static Shooter m_shooter = new Shooter(2);
  public final static Belt m_belt = new Belt(0);
  public final static Elevator m_elevator = new Elevator();
  public final static AirSys m_airsys = new AirSys();
  public final static MotorSys m_motorsys = new MotorSys();
  public final static LEDs m_leds = new LEDs();
  public final static RaspPiCam m_rasppicam = new RaspPiCam();

  // Controller
  public static Joystick XboxC = new Joystick(0);
  public static Joystick buttonBox = new Joystick(1); // ButtonBox that controls speed and direction

  // Controller Buttons
  public static JoystickButton YB = new JoystickButton(XboxC, 4);
  public static JoystickButton AB = new JoystickButton(XboxC, 1);
  public static JoystickButton XB = new JoystickButton(XboxC, 3);
  public static JoystickButton BB = new JoystickButton(XboxC, 2);

  // Upper controller buttons
  public static JoystickButton LBB = new JoystickButton(XboxC, 5);
  public static JoystickButton RBB = new JoystickButton(XboxC, 6);

  // D-pad
  public static POVButton upButton = new POVButton(XboxC, 0);
  public static POVButton downButton = new POVButton(XboxC, 180);
  public static POVButton rightButton = new POVButton(XboxC, 90);
  public static POVButton leftButton = new POVButton(XboxC, 270);

  // Buttons from the buttonbox
  public static JoystickButton B1 = new JoystickButton(buttonBox, 11);
  public static JoystickButton B2 = new JoystickButton(buttonBox, 10);
  public static JoystickButton B3 = new JoystickButton(buttonBox, 9);
  public static JoystickButton B4 = new JoystickButton(buttonBox, 8);
  public static JoystickButton B5 = new JoystickButton(buttonBox, 7);
  public static JoystickButton B6 = new JoystickButton(buttonBox, 6);
  public static JoystickButton B7 = new JoystickButton(buttonBox, 12);

  // Other
  public static HashMap<Command, String> autoMap = new HashMap<>();

  public Command ramAutoBuilder(String pathName, HashMap<String, Command> eventMap) {
    // PathPlanner AutoBuilder, builds a full autonomous command
    RamseteAutoBuilder pathBuilder = new RamseteAutoBuilder(
        m_drivetrain::getPose,
        m_drivetrain::resetOdometery,
        new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
        Constants.DriveConstants.kDriveKinematics,
        new SimpleMotorFeedforward(
            Constants.AutoConstants.ksVolts,
            Constants.AutoConstants.kvVoltSecondsPerMeter,
            Constants.AutoConstants.kaVoltSecondsSquaredPerMeter),
        m_drivetrain::getWheelSpeeds,
        new PIDConstants(Constants.AutoConstants.kPDriveVel, 0, 0),
        m_drivetrain::tankDriveVolts,
        eventMap,
        true,
        m_drivetrain);
    List<PathPlannerTrajectory> pathToFollow = PathPlanner.loadPathGroup(pathName,
        PathPlanner.getConstraintsFromPath(pathName));
    final Command auto = pathBuilder.fullAuto(pathToFollow);
    autoMap.put(auto, pathName);
    return auto;
  }

  public static SendableChooser<Command> m_chooser = new SendableChooser<>();
  public static HashMap<Command, String> autoRoutines = new HashMap<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Fill in the HashMap values
    RobotContainer.m_drivetrain.zeroHeading();
    RobotContainer.m_drivetrain.resetEncoders();

    setRed3GPMap();
    setMidMap();
    setRed3GPABMap();
    setRed1GPEventMap();
    setRed1GPABEventMap();

    m_drivetrain.setDefaultCommand(new TankDrive());

    configureButtonBindings(); // Configure the button bindings
    Shuffleboard.getTab("Auto").add(m_chooser).withSize(3, 1);
    Command instantCmd = new InstantCommand();
    m_chooser.setDefaultOption("Nothing", instantCmd);
    autoMap.put(instantCmd, "nothing");
    m_chooser.addOption("Pos 1 - 2 Game Pieces", ramAutoBuilder("Red 1 - 2 GP", Constants.AutoConstants.red1GPEventMap));
    m_chooser.addOption("Pos 1 - 2 Game Pieces & Auto Balance", ramAutoBuilder("Red 1 - 2 GP AB", Constants.AutoConstants.red1GPABEventMap));
    m_chooser.addOption("Pos 2 - Preload & Auto Balance", ramAutoBuilder("Mid", Constants.AutoConstants.mid));
    m_chooser.addOption("Pos 3 - 2 Game Pieces", ramAutoBuilder("Red 3 - 2 GP", Constants.AutoConstants.red3GPEventMap));
    m_chooser.addOption("Pos 3 - Preload & Auto Balance", ramAutoBuilder("Red 3 - 1 GP AB", Constants.AutoConstants.red3GPABEventMap));
    
    m_chooser.addOption("Balance Test", new SequentialCommandGroup(
      new GetOnRamp(),
      new AutoBalance()
    ));
  }

  public static double getDeadZone(int axis) {
    double rawAxisValue = Double.valueOf(XboxC.getRawAxis(axis));
    if (rawAxisValue >= 0.08 || rawAxisValue <= -0.08) {
      Math.round(rawAxisValue * 100 / 100);
    } else {
      rawAxisValue = 0.0;
    }

    return -rawAxisValue;
  }

  public void setRed3GPMap() {
    Constants.AutoConstants.red3GPEventMap.put("Start", new SequentialCommandGroup(
        new SequentialCommandGroup(new ShooterControl(true), new Time(100), new BeltControl(true, false)),
        new Time(1000),
        new ParallelCommandGroup(new ShooterControl(false), new BeltControl(false, false))));
    Constants.AutoConstants.red3GPEventMap.put("deployIntake", new SequentialCommandGroup(
        new CustomSolControl(true),
        new ParallelCommandGroup(new BeltControl(true, false), new IntakeMotorControl(true, false)),
        new Time(1100),
        new ParallelCommandGroup(new BeltControl(false, false), new IntakeMotorControl(false, false)),
        new CustomSolControl(false)));
    Constants.AutoConstants.red3GPEventMap.put("Stop", new SequentialCommandGroup(
        new ShooterControl(1),
        new ParallelCommandGroup(new ShooterControl(true), new BeltControl(true, false)),
        new Time(1000),
        new ParallelCommandGroup(new ShooterControl(false), new BeltControl(false, false))));
  }

  public void setMidMap() {
    Constants.AutoConstants.mid.put("Start", new SequentialCommandGroup(
        new SequentialCommandGroup(new ShooterControl(true), new Time(100), new BeltControl(true, false)),
        new Time(1000),
        new ParallelCommandGroup(new ShooterControl(false), new BeltControl(false, false))));
    Constants.AutoConstants.mid.put("Stop", new SequentialCommandGroup(
        new GetOnRamp(),
        new AutoBalance()));
  }

  public void setRed3GPABMap() {
    Constants.AutoConstants.red3GPABEventMap.put("Start", new SequentialCommandGroup(
        new SequentialCommandGroup(new ShooterControl(true), new Time(100), new BeltControl(true, false)),
        new Time(1000),
        new ParallelCommandGroup(new ShooterControl(false), new BeltControl(false, false))));
    Constants.AutoConstants.red3GPABEventMap.put("Stop", new SequentialCommandGroup(
        new GetOnRamp(),
        new AutoBalance()));
  }

  public void setRed1GPEventMap() {
    Constants.AutoConstants.red1GPEventMap.put("Start", new SequentialCommandGroup(
        new SequentialCommandGroup(new ShooterControl(true), new Time(100), new BeltControl(true, false)),
        new Time(800),
        new ParallelCommandGroup(new ShooterControl(false), new BeltControl(false, false)),
        new CustomSolControl(true),
        new ParallelCommandGroup(new BeltControl(true, false), new IntakeMotorControl(true, false))
        ));
    Constants.AutoConstants.red1GPEventMap.put("deployIntake", new SequentialCommandGroup(
        new Time(600),
        new ParallelCommandGroup(new BeltControl(false, false), new IntakeMotorControl(false, false)),
        new CustomSolControl(false)));
    Constants.AutoConstants.red1GPEventMap.put("Stop", new SequentialCommandGroup(
        new ShooterControl(1),
        new ParallelCommandGroup(new ShooterControl(true), new BeltControl(true, false)),
        new Time(1000),
        new ParallelCommandGroup(new ShooterControl(false), new BeltControl(false, false))));
  }

  public void setRed1GPABEventMap() {
    Constants.AutoConstants.red1GPABEventMap.put("Start", new SequentialCommandGroup(
        new SequentialCommandGroup(new ShooterControl(true), new Time(100), new BeltControl(true, false)),
        new Time(800),
        new ParallelCommandGroup(new ShooterControl(false), new BeltControl(false, false)),
        new CustomSolControl(true),
        new ParallelCommandGroup(new BeltControl(true, false), new IntakeMotorControl(true, false))
        ));
    Constants.AutoConstants.red1GPABEventMap.put("deployIntake", new SequentialCommandGroup(
        new Time(400),
        new ParallelCommandGroup(new BeltControl(false, false), new IntakeMotorControl(false, false)),
        new CustomSolControl(false)));
    Constants.AutoConstants.red1GPABEventMap.put("score2nd", new SequentialCommandGroup(
        new ShooterControl(1),
        new ParallelCommandGroup(new ShooterControl(true), new BeltControl(true, false)),
        new Time(1000),
        new ParallelCommandGroup(new ShooterControl(false), new BeltControl(false, false))));

    Constants.AutoConstants.red1GPABEventMap.put("Stop", new SequentialCommandGroup(
        new GetOnRamp(),
        new AutoBalance()));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    YB.onTrue(new ChangeSpeed(0.8));
    XB.onTrue(new ChangeSpeed(0.6));
    AB.onTrue(new ChangeSpeed(0.45));

    BB.onTrue(new ToggleDirection());

    B5.onTrue(new ParallelCommandGroup(new ShooterControl(true), new BeltControl(true, false)));
    B5.onFalse(new ParallelCommandGroup(new ShooterControl(false), new BeltControl(false, false)));

    RBB.onTrue(new ParallelCommandGroup(new IntakeMotorControl(true, true), new BeltControl(true, true)));
    RBB.onFalse(new ParallelCommandGroup(new IntakeMotorControl(false, true), new BeltControl(false, true)));

    LBB.onTrue(new SolControl());

    B4.onTrue(new ShooterControl(0));
    B4.onTrue(new BeltControl(0));
    B3.onTrue(new ShooterControl(1));
    B2.onTrue(new ShooterControl(2));
  
    // rightButton.onTrue(new AutoAlign());

    // downbutton.onTrue(new SolControl(false));
    B7.onTrue(new ChangeLED());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}