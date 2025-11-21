/*
 * Copyright (c) 2025 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.subsystems;

import java.util.Arrays;

import ftclib.driverio.FtcDashboard;
import ftclib.motor.FtcMotorActuator.MotorType;
import ftclib.motor.FtcServoActuator;
import ftclib.subsystem.FtcShooter;
import ftclib.vision.FtcLimelightVision;
import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.vision.Vision;
import trclib.motor.TrcMotor;
import trclib.motor.TrcServo;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.subsystem.TrcShootParamTable;
import trclib.subsystem.TrcShooter;
import trclib.subsystem.TrcSubsystem;
import trclib.vision.TrcVisionTargetInfo;

/**
 * This class implements a Shooter Subsystem. This implementation consists of one or two shooter motors. For
 * two-motor shooter, the two motors can be arranged to spin in the same direction (2-stage shooting) or in opposite
 * directions. For opposite spinning motor arrangement, one can spin the motors at different speed to create back spin
 * when shooting the object. In the two-motor configuration, because the two motors may not be identical (even if they
 * are the same model), the subsystem allows you to tune different PID coefficients for each motor. The shooter
 * subsystem also supports optionally mounting on a pan and tilt platform. This allows for aiming the shooter at
 * the shooting target.
 */
public class Shooter extends TrcSubsystem
{
    public static final String FAR_ZONE_SHOOT_POINT             = "Target_9.44ft_2";
    public static final String GOAL_ZONE_SHOOT_POINT            = "Target_2.93ft_2";

    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "Shooter";
        public static final boolean NEED_ZERO_CAL               = true;

        public static final boolean HAS_TWO_SHOOTER_MOTORS      = false;
        public static final boolean HAS_PAN_MOTOR               = true;
        public static final boolean HAS_TILT_MOTOR              = true;
        public static final boolean HAS_LAUNCHER                = true;

        // Shooter Motor1
        public static final String SHOOTER_MOTOR1_NAME          = SUBSYSTEM_NAME + ".ShootMotor1";
        public static final MotorType SHOOTER_MOTOR1_TYPE       = MotorType.DcMotor;
        public static final boolean SHOOTER_MOTOR1_INVERTED     = false;

        // Shooter Motor2
        public static final String SHOOTER_MOTOR2_NAME          = SUBSYSTEM_NAME + ".ShootMotor2";
        public static final MotorType SHOOTER_MOTOR2_TYPE       = MotorType.DcMotor;
        public static final boolean SHOOTER_MOTOR2_INVERTED     = true;

        // Assume shooter motor1 and motor2 are the same type and have same gear ratio but they could have different
        // PID coefficients due to different motor strengths and frictions.
        public static final double GOBILDA5000_CPR              = 28.0;
        public static final double SHOOT_MOTOR_GEAR_RATIO       = 20.0/28.0;
        public static final double SHOOT_MOTOR_REV_PER_COUNT    = 1.0/(GOBILDA5000_CPR * SHOOT_MOTOR_GEAR_RATIO);
        public static final double SHOOT_MOTOR_MAX_VEL          = 6000.0;

        public static final double SHOOT_MOTOR1_PID_KP          = 1.0;
        public static final double SHOOT_MOTOR1_PID_KI          = 0.01;
        public static final double SHOOT_MOTOR1_PID_IZONE       = 160/60.0; // in RPS
        public static final double SHOOT_MOTOR1_PID_KD          = 0.0;
        public static final double SHOOT_MOTOR1_PID_KF          = 0.0125;

        public static final double SHOOT_MOTOR2_PID_KP          = 1.0;
        public static final double SHOOT_MOTOR2_PID_KI          = 0.0;
        public static final double SHOOT_MOTOR2_PID_KD          = 0.0;
        public static final double SHOOT_MOTOR2_PID_KF          = 0.0125;

        public static final double SHOOT_PID_TOLERANCE_RPM      = 75.0;
        public static final boolean SHOOT_SOFTWARE_PID_ENABLED  = true;
        public static final double SHOOT_MOTOR_OFF_DELAY        = 0.5;      // in sec
        public static final double SHOOT_VEL_TRIGGER_THRESHOLD  = 350.0;    // in RPM

        // Pan Motor
        public static final String PAN_MOTOR_NAME               = SUBSYSTEM_NAME + ".PanMotor";
        public static final MotorType PAN_MOTOR_TYPE            = MotorType.DcMotor;
        public static final boolean PAN_MOTOR_INVERTED          = true;
        public static final String PAN_ENCODER_NAME             = null;
        public static final boolean PAN_ENCODER_INVERTED        = false;

        public static final double PAN_MOTOR_PID_KP             = 0.03;
        public static final double PAN_MOTOR_PID_KI             = 0.01;
        public static final double PAN_MOTOR_PID_KD             = 0.0;
        public static final double PAN_MOTOR_PID_KF             = 0.0;
        public static final double PAN_MOTOR_PID_IZONE          = 5.0;
        public static final double PAN_PID_TOLERANCE            = 1.0;
        public static final boolean PAN_SOFTWARE_PID_ENABLED    = true;

        public static final double PAN_GEAR_RATIO               = 75.0/26.0;
        public static final double PAN_DEG_PER_COUNT            =
            360.0/(RobotParams.MotorSpec.GOBILDA_223_ENC_PPR*PAN_GEAR_RATIO);
        public static final double PAN_POS_OFFSET               = 95.0;
        public static final double PAN_ENCODER_ZERO_OFFSET      = 0.0;
        public static final double PAN_POWER_LIMIT              = 1.0;
        public static final double PAN_MIN_POS                  = -345.0;
        public static final double PAN_MAX_POS                  = PAN_POS_OFFSET;
        public static final double PAN_POS_PRESET_TOLERANCE     = 5.0;
        public static final double[] PAN_POS_PRESETS            =
            {
                PAN_MIN_POS, -330.0, -300.0, -270.0, -240.0, -210.0, -180.0, -150.0, -120.0, -90.0, -60.0, -30.0,
                0.0, 30.0, 60.0, PAN_MAX_POS
            };

        public static final double PAN_ZERO_CAL_POWER           = 0.3;
        public static final double PAN_STALL_MIN_POWER          = Math.abs(PAN_ZERO_CAL_POWER);
        public static final double PAN_STALL_TOLERANCE          = 0.1;
        public static final double PAN_STALL_TIMEOUT            = 0.1;
        public static final double PAN_STALL_RESET_TIMEOUT      = 0.0;

        // Tilt Motor
        public static final String TILT_MOTOR_NAME              = SUBSYSTEM_NAME + ".TiltMotor";
        public static final MotorType TILT_MOTOR_TYPE           = MotorType.CRServo;
        public static final boolean TILT_MOTOR_INVERTED         = false;
        public static final String TILT_ENCODER_NAME            = SUBSYSTEM_NAME + ".TiltEncoder";
        public static final boolean TILT_ENCODER_INVERTED       = false;

        public static final double TILT_MOTOR_PID_KP            = 0.06;
        public static final double TILT_MOTOR_PID_KI            = 0.005;
        public static final double TILT_MOTOR_PID_KD            = 0.0025;
        public static final double TILT_MOTOR_PID_KF            = 0.0;
        public static final double TILT_MOTOR_PID_IZONE         = 3.0;
        public static final double TILT_PID_TOLERANCE           = 1.0;
        public static final boolean TILT_SOFTWARE_PID_ENABLED   = true;

//        public static final double TILT_GEAR_RATIO              = 543.0/56.0;   // Not accurate, why???
//        public static final double TILT_DEG_PER_COUNT           = 360.0/TILT_GEAR_RATIO;
        public static final double TILT_DEG_PER_COUNT           = 37.471013190648257044337576357835;
        public static final double TILT_POS_OFFSET              = 25.0;
        public static final double TILT_ENCODER_ZERO_OFFSET     = 0.124848;
        public static final double TILT_POWER_LIMIT             = 1.0;
        public static final double TILT_MIN_POS                 = TILT_POS_OFFSET;
        public static final double TILT_MAX_POS                 = 45.0;
        public static final double TILT_POS_PRESET_TOLERANCE    = 2.0;
        public static final double[] TILT_POS_PRESETS           =
            {TILT_MIN_POS, 30.0, 35.0, 40.0, TILT_MAX_POS};

        public static final TrcShootParamTable shootParamTable = new TrcShootParamTable()
            //   entry_name,            dist,           shoot1_vel, shoot2_vel, tilt_angle
            .add("Target_2.14ft",       25.7,           3500.0,     0.0,        26.0)
            .add("Target_2.49ft",       29.9,           3600.0,     0.0,        26.0)
            .add("Target_2.93_ft_1",    35.25,          3650.0,     0.0,        26.0)
            .add("Target_2.93ft_2",     35.25000001,    3600.0,     0.0,        30.0)
            .add("Target_3.67ft",       44.0,           3650.0,     0.0,        30.0)
            .add("Target_3.67ft",       44.000001,      3700.0,     0.0,        33.0)
            .add("Target_4.42ft",       53.0,           3950.0,     0.0,        33.0)
            .add("Target_5.43ft_1",     65.2,           4125.0,     0.0,        33.0)
            .add("Target_5.43ft_2",     65.2000001,     4025.0,     0.0,        38.0)
            .add("Target_8.29ft",       85.4,           4300.0,     0.0,        38.0)
            .add("Target_8.29ft",       99.5,           4600.0,     0.0,        38.0)
            .add("Target_9.44ft_1",     111.3,          4760,       0.0,        38.0)
            .add(FAR_ZONE_SHOOT_POINT,  111.3000001,    4750.0,     0.0,        42.0)
            .add("Target_10.65ft",      127.8,          5000.0,     0.0,        42.0);

        // Launcher
        public static final String LAUNCHER_SERVO_NAME          = SUBSYSTEM_NAME + ".Launcher";
        public static final boolean LAUNCHER_SERVO_INVERTED     = false;
        public static double LAUNCHER_REST_POS                  = 0.47;
        public static double LAUNCHER_LAUNCH_POS                = 1.0;
        public static double LAUNCHER_LAUNCH_DURATION           = 0.75;     // in seconds
        public static double LAUNCHER_RETRACT_TIME              = 0.25;     // in seconds

        public static double TURRET_X_OFFSET                    = 0.0;      // inches from robot center
        public static double TURRET_Y_OFFSET                    = -3.246;   // inches from robot center
        public static TrcPose2D CAMERA_POSE_ON_TURRET           = new TrcPose2D(0.0, -2.9837, 0.0);
    }   //class Params

    public static final TrcMotor.PidParams shootMotor1PidParams = new TrcMotor.PidParams()
        .setPidCoefficients(
            Params.SHOOT_MOTOR1_PID_KP, Params.SHOOT_MOTOR1_PID_KI, Params.SHOOT_MOTOR1_PID_KD,
            Params.SHOOT_MOTOR1_PID_KF, Params.SHOOT_MOTOR1_PID_IZONE)
        .setPidControlParams(Params.SHOOT_PID_TOLERANCE_RPM/60.0, Params.SHOOT_SOFTWARE_PID_ENABLED);
    public static final TrcMotor.PidParams shootMotor2PidParams = new TrcMotor.PidParams()
        .setPidCoefficients(
            Params.SHOOT_MOTOR2_PID_KP, Params.SHOOT_MOTOR2_PID_KI, Params.SHOOT_MOTOR2_PID_KD,
            Params.SHOOT_MOTOR2_PID_KF)
        .setPidControlParams(Params.SHOOT_PID_TOLERANCE_RPM/60.0, Params.SHOOT_SOFTWARE_PID_ENABLED);
    public static final TrcMotor.PidParams panMotorPidParams = new TrcMotor.PidParams()
        .setPidCoefficients(
            Params.PAN_MOTOR_PID_KP, Params.PAN_MOTOR_PID_KI, Params.PAN_MOTOR_PID_KD, Params.PAN_MOTOR_PID_KF,
            Params.PAN_MOTOR_PID_IZONE)
        .setPidControlParams(Params.PAN_PID_TOLERANCE, Params.PAN_SOFTWARE_PID_ENABLED);
    public static final TrcMotor.PidParams tiltMotorPidParams = new TrcMotor.PidParams()
        .setPidCoefficients(
            Params.TILT_MOTOR_PID_KP, Params.TILT_MOTOR_PID_KI, Params.TILT_MOTOR_PID_KD, Params.TILT_MOTOR_PID_KF,
            Params.TILT_MOTOR_PID_IZONE)
        .setPidControlParams(Params.TILT_PID_TOLERANCE, Params.TILT_SOFTWARE_PID_ENABLED);
    public static final FtcServoActuator.TuneParams launcherTuneParams = new FtcServoActuator.TuneParams(
        Params.LAUNCHER_REST_POS, Params.LAUNCHER_LAUNCH_POS, Params.LAUNCHER_LAUNCH_DURATION,
        Params.LAUNCHER_RETRACT_TIME);

    private final FtcDashboard dashboard;
    private final Robot robot;
    private final TrcShooter shooter;
    public final TrcServo launcher;
    private String launchOwner;
    private TrcEvent launchCompletionEvent;
    private int[] trackedAprilTagIds = null;
    private Double goalFieldHeading = null;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param robot specifies the robot object to access the other subsystems.
     */
    public Shooter(Robot robot)
    {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);
        dashboard = FtcDashboard.getInstance();
        this.robot = robot;
        FtcShooter.Params shooterParams = new FtcShooter.Params()
            .setShooterMotor1(
                Params.SHOOTER_MOTOR1_NAME, Params.SHOOTER_MOTOR1_TYPE, Params.SHOOTER_MOTOR1_INVERTED);

        if (Params.HAS_TWO_SHOOTER_MOTORS)
        {
            shooterParams.setShooterMotor2(
                Params.SHOOTER_MOTOR2_NAME, Params.SHOOTER_MOTOR2_TYPE, Params.SHOOTER_MOTOR2_INVERTED);
        }

        if (Params.HAS_PAN_MOTOR)
        {
            shooterParams
                .setPanMotor(
                    Params.PAN_MOTOR_NAME, Params.PAN_MOTOR_TYPE, Params.PAN_MOTOR_INVERTED,
                    Params.PAN_ENCODER_NAME, Params.PAN_ENCODER_INVERTED,
                    new TrcShooter.PanTiltParams(Params.PAN_POWER_LIMIT, Params.PAN_MIN_POS, Params.PAN_MAX_POS))
                .setPanMotorPosPresets(Params.PAN_POS_PRESET_TOLERANCE, Params.PAN_POS_PRESETS);
        }

        if (Params.HAS_TILT_MOTOR)
        {
            shooterParams
                .setTiltMotor(
                    Params.TILT_MOTOR_NAME, Params.TILT_MOTOR_TYPE, Params.TILT_MOTOR_INVERTED,
                    Params.TILT_ENCODER_NAME, Params.TILT_ENCODER_INVERTED,
                    new TrcShooter.PanTiltParams(Params.TILT_POWER_LIMIT, Params.TILT_MIN_POS, Params.TILT_MAX_POS))
                .setTiltMotorPosPresets(Params.TILT_POS_PRESET_TOLERANCE, Params.TILT_POS_PRESETS);
        }

        shooter = new FtcShooter(Params.SUBSYSTEM_NAME, shooterParams).getShooter();

        TrcMotor motor = shooter.getShooterMotor1();
        motor.setPositionSensorScaleAndOffset(Params.SHOOT_MOTOR_REV_PER_COUNT, 0.0);
        motor.setVelocityPidParameters(shootMotor1PidParams, null);

        motor = shooter.getShooterMotor2();
        if (motor != null)
        {
            // Assuming motor2 is the same type of motor as motor1 and has the same gear ratio.
            // If it needs to, this allows different PID coefficients for motor2 in case they are not quite identical.
            motor.setPositionSensorScaleAndOffset(Params.SHOOT_MOTOR_REV_PER_COUNT, 0.0);
            motor.setVelocityPidParameters(shootMotor2PidParams, null);
        }

        motor = shooter.getPanMotor();
        if (motor != null)
        {
            motor.setPositionSensorScaleAndOffset(
                Params.PAN_DEG_PER_COUNT, Params.PAN_POS_OFFSET, Params.PAN_ENCODER_ZERO_OFFSET);
            motor.setPositionPidParameters(panMotorPidParams, this::getPanPosition);
            // There is no lower limit switch, enable stall detection for zero calibration and soft limits for
            // protection.
            motor.setStallProtection(
                Params.PAN_STALL_MIN_POWER, Params.PAN_STALL_TOLERANCE, Params.PAN_STALL_TIMEOUT,
                Params.PAN_STALL_RESET_TIMEOUT);
            motor.setSoftPositionLimits(Params.PAN_MIN_POS, Params.PAN_MAX_POS, false);
        }

        motor = shooter.getTiltMotor();
        if (motor != null)
        {
            motor.setPositionSensorScaleAndOffset(
                Params.TILT_DEG_PER_COUNT, Params.TILT_POS_OFFSET, Params.TILT_ENCODER_ZERO_OFFSET);
            motor.setPositionPidParameters(tiltMotorPidParams, null);
            motor.setSoftPositionLimits(Params.TILT_MIN_POS, Params.TILT_MAX_POS, false);
        }

        if (Params.HAS_LAUNCHER)
        {
            FtcServoActuator.Params launcherParams = new FtcServoActuator.Params()
                .setPrimaryServo(Params.LAUNCHER_SERVO_NAME, Params.LAUNCHER_SERVO_INVERTED)
                .setTuneParams(launcherTuneParams);
            launcher = new FtcServoActuator(launcherParams).getServo();
            launcher.setPosition(Params.LAUNCHER_REST_POS);
        }
        else
        {
            launcher = null;
        }
    }   //Shooter

    /**
     * This method returns the created shooter.
     *
     * @return created shooter.
     */
    public TrcShooter getShooter()
    {
        return shooter;
    }   //getShooter

    /**
     * This method returns the current flywheel velocity in RPM.
     *
     * @return current flywheel velocity in RPM.
     */
    public double getFlywheelRPM()
    {
        return shooter.shooterMotor1.getVelocity()*60.0;
    }   //getFlywheelRPM

    /**
     * This method sets the launcher servo position.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships, null if no ownership required.
     * @param launch specifies true to set servo to launch position, false to set to rest position.
     */
    public void setLaunchPosition(String owner, boolean launch)
    {
        if (launch)
        {
            launcher.setPosition(owner, 0.0, launcherTuneParams.activatePos, null, 0.0);
        }
        else
        {
            launcher.setPosition(owner, 0.0, launcherTuneParams.restPos, null, 0.0);
        }
    }   //setLaunchPosition

    /**
     * This method is called to launch the game piece into the shooter, typically when TrcShooter has reached shooting
     * velocity and Pan/Tilt have aimed at the target and ready to shoot.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships, null if no ownership required.
     * @param completionEvent specifies the event to signal when shooting is done, can be null.
     */
    public void shoot(String owner, TrcEvent completionEvent)
    {
        if (launcher != null)
        {
            TrcDbgTrace.globalTraceInfo(
                instanceName, "shoot(owner=%s, event=%s, pos=%f, duration=%f)",
                owner, completionEvent, launcherTuneParams.activatePos, launcherTuneParams.activateDuration);
            if (robot.spindexerSubsystem != null)
            {
                // Enable Spindexer exit trigger.
                double currFlywheelRPM = getFlywheelRPM();
                robot.spindexerSubsystem.enableExitTrigger(
                    currFlywheelRPM - Params.SHOOT_VEL_TRIGGER_THRESHOLD,
                    currFlywheelRPM + Params.SHOOT_VEL_TRIGGER_THRESHOLD,
                    this::velTriggerCallback);
            }
            launchOwner = owner;
            launchCompletionEvent = completionEvent;
            launcher.setPosition(owner, 0.0, launcherTuneParams.activatePos, null, Params.LAUNCHER_LAUNCH_DURATION);
        }
        else if (completionEvent != null)
        {
            TrcDbgTrace.globalTraceInfo(instanceName, "There is no launcher, signal completion anyway.");
            completionEvent.signal();
        }
    }   //shoot

    /**
     * This method is called when the flywheel velocity trigger occurred.
     *
     * @param context not used.
     * @param canceled specifies true if launch was canceled (not used).
     */
    private void velTriggerCallback(Object context, boolean canceled)
    {
        if (robot.spindexerSubsystem != null)
        {
            robot.spindexerSubsystem.disableExitTrigger();
        }
        // Reset launcher, fire and forget.
        TrcEvent callbackEvent = new TrcEvent("Launcher.retractCallback");
        callbackEvent.setCallback(
            (ctxt, canceld) ->
            {
                if (launchCompletionEvent != null)
                {
                    if (canceled)
                    {
                        launchCompletionEvent.cancel();
                    }
                    else
                    {
                        launchCompletionEvent.signal();
                    }
                    launchCompletionEvent = null;
                }
                launchOwner = null;
            },
            null);
        launcher.setPosition(
            launchOwner, 0.0, launcherTuneParams.restPos, callbackEvent, launcherTuneParams.retractTime);
    }   //velTriggerCallback

    /**
     * This method checks if Goal Tracking is enabled.
     *
     * @return true if Goal Tracking is enabled, false if disabled.
     */
    public boolean isGoalTrackingEnabled()
    {
        return trackedAprilTagIds != null || goalFieldHeading != null;
    }   //isGoalTrackingEnabled

    /**
     * This method enables Goal Tracking with the Turret (Pan motor) using AprilTag Vision.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships, null if no ownership required.
     * @param aprilTagIds specifies the AprilTag IDs to track.
     */
    public void enableGoalTracking(String owner, int[] aprilTagIds)
    {
        if (robot.vision != null)
        {
            if (shooter.acquireExclusiveAccess(owner))
            {
                shooter.tracer.traceInfo(
                    instanceName,
                    "Enabling Goal Tracking using AprilTag (owner=" + owner +
                    ", Ids=" + Arrays.toString(aprilTagIds) + ")");
                robot.vision.setLimelightVisionEnabled(Vision.LimelightPipelineType.APRIL_TAG, true);
                this.trackedAprilTagIds = aprilTagIds;
                this.goalFieldHeading = null;
                shooter.panMotor.setPosition(0.0, true, Params.PAN_POWER_LIMIT);
            }
        }
    }   //enableGoalTracking

    /**
     * This method enables Goal Tracking with the Turret (Pan motor) using Odometry.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships, null if no ownership required.
     * @param goalFieldHeading specifies the Alliance Goal to track.
     */
    public void enableGoalTracking(String owner, double goalFieldHeading)
    {
        if (shooter.acquireExclusiveAccess(owner))
        {
            shooter.tracer.traceInfo(
                instanceName,
                "Enabling Goal Tracking using Odometry (owner=" + owner +
                ", goalFieldHeading=" + goalFieldHeading + ")");
            this.trackedAprilTagIds = null;
            this.goalFieldHeading = goalFieldHeading;
            shooter.panMotor.setPosition(0.0, true, Params.PAN_POWER_LIMIT);
        }
    }   //enableGoalTracking

    /**
     * This method disables Goal Tracking.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships, null if no ownership required.
     */
    public void disableGoalTracking(String owner)
    {
        shooter.releaseExclusiveAccess(owner);
        shooter.panMotor.cancel();
        this.trackedAprilTagIds = null;
        this.goalFieldHeading = null;
        shooter.tracer.traceInfo(
            instanceName, "Disabling Goal Tracking (turretPos=" + shooter.panMotor.getPosition() + ")");
    }   //disableGoalTracking

    /**
     * This method returns the array of tracked AprilTag IDs.
     *
     * @return tracked AprilTag IDs.
     */
    public int[] getTrackedAprilTagIds()
    {
        return trackedAprilTagIds;
    }   //getTrackedArpilTagIds

    /**
     * This method returns the tracked goal field heading.
     *
     * @return tracked alliance.
     */
    public Double getTrackedGoalFieldHeading()
    {
        return goalFieldHeading;
    }   //getTrackedGoalFieldHeading

    /**
     * This method is called by Pan Motor PID Control Task to get the current Pan position. By manipulating this
     * position, we can use the PID controller to track the AprilTag target.
     *
     * @return angle distance between the current position and the AprilTag target if tracking is ON, angle position
     *         of the target relative to robot heading if tracking is OFF.
     */
    private double getPanPosition()
    {
        double panPosition = shooter.panMotor.getPosition();
        Double newPanPosition = null;

        if (trackedAprilTagIds != null)
        {
            TrcVisionTargetInfo<FtcLimelightVision.DetectedObject> aprilTagInfo =
                robot.vision.getLimelightDetectedObject(
                    FtcLimelightVision.ResultType.Fiducial, trackedAprilTagIds, null, null, -1);
            if (aprilTagInfo == null)
            {
                // Not detecting AprilTag or vision is still processing the frame, don't move.
                panPosition = 0.0;
                shooter.tracer.traceDebug(Params.SUBSYSTEM_NAME, "AprilTag not found, don't move.");
            }
            else
            {
                int aprilTagId = (int) aprilTagInfo.detectedObj.objId;
                TrcPose2D targetPose = aprilTagInfo.objPose.addRelativePose(
                     aprilTagId == 20?
                         RobotParams.Game.BLUE_APRILTAG_TO_CORNER: RobotParams.Game.RED_APRILTAG_TO_CORNER);
                newPanPosition = panPosition + targetPose.angle;
                shooter.tracer.traceDebug(
                    Params.SUBSYSTEM_NAME, "AdjustedPoseFromAprilTag{" + aprilTagId + "}=" + targetPose);
            }
        }
        else if (goalFieldHeading != null)
        {
            newPanPosition = goalFieldHeading - robot.robotBase.driveBase.getHeading();
        }
//        else if (trackedAlliance != null)
//        {
//            TrcPose2D goalPose = trackedAlliance == FtcAuto.Alliance.BLUE_ALLIANCE?
//                RobotParams.Game.BLUE_CORNER_POSE: RobotParams.Game.RED_CORNER_POSE;
//            TrcPose2D targetPose = goalPose.relativeTo(robot.robotBase.driveBase.getFieldPosition());
//            newPanPosition = targetPose.angle;
//            shooter.tracer.traceDebug(Params.SUBSYSTEM_NAME, "AdjustedPoseFromOdometry=" + targetPose);
//        }

        if (newPanPosition != null)
        {
            shooter.tracer.traceDebug(Params.SUBSYSTEM_NAME, "Panning: %f->%f", panPosition, newPanPosition);
            // Check if we are crossing over the hard stop.
            if (newPanPosition >= Params.PAN_MIN_POS && newPanPosition <= Params.PAN_MAX_POS)
            {
                // We are moving within valid range.
                panPosition -= newPanPosition;
            }
            else
            {
                // We are crossing over the hard stop, stop it.
                panPosition = 0.0;
                shooter.tracer.traceDebug(Params.SUBSYSTEM_NAME, "Crossing over hard stop. Stop!");
            }
        }

        return panPosition;
    }   //getPanPosition

    /**
     * This method computes the camera pose on the robot given the turret heading.
     *
     * @param turretAngleDeg specifies the turret heading in degrees.
     * @return camera pose relative to the robot center.
     */
    public TrcPose2D getCameraPoseOnRobot(double turretAngleDeg)
    {
        TrcPose2D turretPoseOnRobot = new TrcPose2D(Params.TURRET_X_OFFSET, Params.TURRET_Y_OFFSET, turretAngleDeg);
        return turretPoseOnRobot.addRelativePose(Params.CAMERA_POSE_ON_TURRET);
    }   //getCameraPoseOnRobot

    /**
     * This method returns the Robot Field position adjusted by the camera position on the robot's turret.
     *
     * @param camFieldPose specifies the camera's field position from Vision.
     * @return robot's field position.
     */
    public TrcPose2D adjustRobotFieldPosition(TrcPose2D camFieldPose)
    {
        double turretAngleDeg = shooter.panMotor.getPosition();
        TrcPose2D cameraPoseOnRobot = getCameraPoseOnRobot(turretAngleDeg);
        TrcPose2D adjustedRobotFieldPose = camFieldPose.addRelativePose(cameraPoseOnRobot.invert());
        shooter.tracer.traceInfo(
            Params.SUBSYSTEM_NAME, "turretAngle=%f, camFieldPose=%s, cameraPoseOnRobot=%s, adjustedPose=%s",
            turretAngleDeg, camFieldPose, cameraPoseOnRobot, adjustedRobotFieldPose);
        return adjustedRobotFieldPose;
    }   //adjustRobotFieldPosition

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        shooter.cancel();
        if (launcher != null)
        {
            if (launcher.getPosition() == launcherTuneParams.activatePos)
            {
                velTriggerCallback(null, true);
            }
            launcher.cancel();
        }
    }   //cancel

    /**
     * This method starts zero calibrate of the subsystem.
     *
     * @param owner specifies the owner ID to to claim subsystem ownership, can be null if ownership not required.
     * @param completionEvent specifies an event to signal when zero calibration is done, can be null if not provided.
     */
    @Override
    public void zeroCalibrate(String owner, TrcEvent completionEvent)
    {
        // Shooter does not need zero calibration.
        // Tilter has absolute encoder and therefore no need for zero calibration.
        // Zero calibrate turret (pan).
        TrcEvent callbackEvent = new TrcEvent(Params.PAN_MOTOR_NAME + ".callbackEvent");
        callbackEvent.setCallback(
            (ctxt, canceled) -> {
                TrcEvent event = (TrcEvent) ctxt;
                if (!canceled)
                {
                    if (TrcRobot.getRunMode() != TrcRobot.RunMode.AUTO_MODE ||
                        FtcAuto.autoChoices.startPos != FtcAuto.StartPos.GOAL_ZONE)
                    {
                        shooter.panMotor.setPosition(owner, 0.0, 0.0, true, Params.PAN_POWER_LIMIT, null, 0.0);
                    }
                    else
                    {
                        shooter.panMotor.setPosition(
                            owner, 0.0,
                            FtcAuto.autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE ? -130.0 : -230.0,
                            true, Params.PAN_POWER_LIMIT, null, 0.0);
                    }

                    if (event !=  null)
                    {
                        event.signal();
                    }
                }
                else if (event != null)
                {
                    event.cancel();
                }
            }, completionEvent);
        shooter.panMotor.zeroCalibrate(owner, Params.PAN_ZERO_CAL_POWER, callbackEvent);
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        // Shooter does not support resetState.
        // If you need to tuck away pan and tilt for turtle mode, add code here.
    }   //resetState

    /**
     * This method update the dashboard with the subsystem status.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     * @param slowLoop specifies true if this is a slow loop, false otherwise.
     * @return updated line number for the next subsystem to print.
     */
    @Override
    public int updateStatus(int lineNum, boolean slowLoop)
    {
        if (RobotParams.Preferences.showShooterStatus)
        {
            if (slowLoop)
            {
                TrcMotor motor;

                motor = shooter.getShooterMotor1();
                dashboard.displayPrintf(
                    lineNum++, "%s: power=%.1f, current=%.1f, vel=%.1f, target=%.1f",
                    Params.SUBSYSTEM_NAME + ".Motor", motor.getPower(), motor.getCurrent(),
                    shooter.getShooterMotor1RPM(), shooter.getShooterMotor1TargetRPM());

                motor = shooter.getShooterMotor2();
                if (motor != null)
                {
                    dashboard.displayPrintf(
                        lineNum++, "%s: power=%.1f, current=%.1f, vel=%.1f, target=%.1f",
                        Params.SUBSYSTEM_NAME + ".Motor2", motor.getPower(), motor.getCurrent(),
                        shooter.getShooterMotor2RPM(), shooter.getShooterMotor2TargetRPM());
                }

                motor = shooter.getPanMotor();
                if (motor != null)
                {
                    dashboard.displayPrintf(
                        lineNum++, "%s: power=%.1f, current=%.1f, pos=%.1f/%.1f",
                        Params.SUBSYSTEM_NAME + ".Pan", motor.getPower(), motor.getCurrent(),
                        motor.getPosition(), motor.getPidTarget());
                }

                motor = shooter.getTiltMotor();
                if (motor != null)
                {
                    dashboard.displayPrintf(
                        lineNum++, "%s: power=%.1f, pos=%.1f/%.1f(%f)",
                        Params.SUBSYSTEM_NAME + ".Tilt", motor.getPower(), motor.getPosition(), motor.getPidTarget(),
                        motor.getEncoderRawPosition());
                }

                if (launcher != null)
                {
                    dashboard.displayPrintf(
                        lineNum++, "%s: pos=%.3f",
                        Params.SUBSYSTEM_NAME + ".Launcher", launcher.getPosition());
                }
            }
        }

        if (RobotParams.Preferences.showShooterGraph)
        {
            dashboard.putNumber("Shooter1MotorRPM", shooter.getShooterMotor1RPM());
            dashboard.putNumber("ShooterMotor1TargetRPM", shooter.getShooterMotor1TargetRPM());
            dashboard.putNumber("ShooterMotor1RangeMin", 0.0);
            dashboard.putNumber("ShooterMotor1RangeMax", Params.SHOOT_MOTOR_MAX_VEL);
            dashboard.putNumber("PanTarget", shooter.getPanAngleTarget());
            dashboard.putNumber("PanAngle", shooter.getPanAngle());
            dashboard.putNumber("TiltTarget", shooter.getTiltAngleTarget());
            dashboard.putNumber("TiltAngle", shooter.getTiltAngle());
            dashboard.putNumber("SpindexTarget", robot.spindexer.motor.getPidTarget());
            dashboard.putNumber("SpindexPos", robot.spindexer.motor.getPosition());
        }

        return lineNum;
    }   //updateStatus

    /**
     * This method is called to update subsystem parameter to the Dashboard.
     */
    @Override
    public void updateParamsToDashboard()
    {
    }   //updateParamsToDashboard

    /**
     * This method is called to update subsystem parameters from the Dashboard.
     */
    @Override
    public void updateParamsFromDashboard()
    {
    }   //updateParamsFromDashboard

}   //class Shooter
