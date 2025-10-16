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

import ftclib.driverio.FtcDashboard;
import ftclib.motor.FtcMotorActuator.MotorType;
import ftclib.motor.FtcServoActuator;
import ftclib.subsystem.FtcShooter;
import ftclib.vision.FtcVisionAprilTag;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.vision.Vision;
import trclib.motor.TrcMotor;
import trclib.motor.TrcServo;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
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
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "Shooter";
        public static final boolean NEED_ZERO_CAL               = false;

        public static final boolean HAS_TWO_SHOOTER_MOTORS      = false;
        public static final boolean HAS_PAN_MOTOR               = true;
        public static final boolean HAS_TILT_MOTOR              = true;
        public static final boolean HAS_LAUNCHER                = true;

        // Shooter Motor1
        public static final String SHOOTER_MOTOR1_NAME          = SUBSYSTEM_NAME + ".ShooterMotor1";
        public static final MotorType SHOOTER_MOTOR1_TYPE       = MotorType.DcMotor;
        public static final boolean SHOOTER_MOTOR1_INVERTED     = false;

        // Shooter Motor2
        public static final String SHOOTER_MOTOR2_NAME          = SUBSYSTEM_NAME + ".ShooterMotor2";
        public static final MotorType SHOOTER_MOTOR2_TYPE       = MotorType.DcMotor;
        public static final boolean SHOOTER_MOTOR2_INVERTED     = true;

        // Assume shooter motor1 and motor2 are the same type and have same gear ratio but they could have different
        // PID coefficients due to different motor strengths and frictions.
        public static final double GOBILDA5000_CPR              = 28.0;
        public static final double SHOOT_MOTOR_GEAR_RATIO       = 20.0/28.0;
        public static final double SHOOT_MOTOR_REV_PER_COUNT    = 1.0/(GOBILDA5000_CPR * SHOOT_MOTOR_GEAR_RATIO);
        public static final double SHOOT_MOTOR_MAX_VEL          = 7360.0;

        public static final double SHOOT_MOTOR1_PID_KP          = 1.0;
        public static final double SHOOT_MOTOR1_PID_KI          = 0.0;
        public static final double SHOOT_MOTOR1_PID_KD          = 0.0;
        public static final double SHOOT_MOTOR1_PID_KF          = 0.0125;

        public static final double SHOOT_MOTOR2_PID_KP          = 0.075;
        public static final double SHOOT_MOTOR2_PID_KI          = 0.0;
        public static final double SHOOT_MOTOR2_PID_KD          = 0.0;
        public static final double SHOOT_MOTOR2_PID_KF          = 0.008;

        public static final double SHOOT_PID_TOLERANCE_RPM      = 60.0;
        public static final boolean SHOOT_SOFTWARE_PID_ENABLED  = true;
        public static final double SHOOT_MOTOR_OFF_DELAY        = 0.5;      // in sec

        // Pan Motor
        public static final String PAN_MOTOR_NAME               = SUBSYSTEM_NAME + ".PanMotor";
        public static final MotorType PAN_MOTOR_TYPE            = MotorType.DcMotor;
        public static final boolean PAN_MOTOR_INVERTED          = true;
        public static final String PAN_ENCODER_NAME             = null;
        public static final boolean PAN_ENCODER_INVERTED        = false;

        public static final double PAN_MOTOR_PID_KP             = 0.018;
        public static final double PAN_MOTOR_PID_KI             = 0.0;
        public static final double PAN_MOTOR_PID_KD             = 0.0;
        public static final double PAN_PID_TOLERANCE            = 1.0;
        public static final boolean PAN_SOFTWARE_PID_ENABLED    = true;

        public static final double PAN_GEAR_RATIO               = 75.0/26.0;
        public static final double PAN_DEG_PER_COUNT            =
            360.0/(RobotParams.MotorSpec.REV_COREHEX_ENC_PPR*PAN_GEAR_RATIO);
        public static final double PAN_POS_OFFSET               = 0.0;
        public static final double PAN_ENCODER_ZERO_OFFSET      = 0.0;
        public static final double PAN_POWER_LIMIT              = 1.0;
        public static final double PAN_MIN_POS                  = -135.0;
        public static final double PAN_MAX_POS                  = 135.0;
        public static final double PAN_POS_PRESET_TOLERANCE     = 5.0;
        public static final double[] PAN_POS_PRESETS            =
            {PAN_MIN_POS, -90.0, -45.0, 0.0, 45.0, 90.0, PAN_MAX_POS};

        public static final double PAN_ZERO_CAL_POWER           = -0.2;
        public static final double PAN_STALL_MIN_POWER          = Math.abs(PAN_ZERO_CAL_POWER);
        public static final double PAN_STALL_TOLERANCE          = 0.1;
        public static final double PAN_STALL_TIMEOUT            = 0.1;
        public static final double PAN_STALL_RESET_TIMEOUT      = 0.0;

        // Tilt Motor
        public static final String TILT_MOTOR_NAME              = SUBSYSTEM_NAME + ".TiltMotor";
        public static final MotorType TILT_MOTOR_TYPE           = MotorType.CRServo;
        public static final boolean TILT_MOTOR_INVERTED         = true;
        public static final String TILT_ENCODER_NAME            = SUBSYSTEM_NAME + ".TiltEncoder";
        public static final boolean TILT_ENCODER_INVERTED       = false;

        public static final double TILT_MOTOR_PID_KP            = 0.03;
        public static final double TILT_MOTOR_PID_KI            = 0.0;
        public static final double TILT_MOTOR_PID_KD            = 0.0;
        public static final double TILT_PID_TOLERANCE           = 1.0;
        public static final boolean TILT_SOFTWARE_PID_ENABLED   = true;

        public static final double TILT_GEAR_RATIO              = 543.0/56.0;
        public static final double TILT_DEG_PER_COUNT           = 360.0/TILT_GEAR_RATIO;
        public static final double TILT_POS_OFFSET              = 27.0;
        public static final double TILT_ENCODER_ZERO_OFFSET     = 0.232121;
        public static final double TILT_POWER_LIMIT             = 1.0;
        public static final double TILT_MIN_POS                 = TILT_POS_OFFSET;
        public static final double TILT_MAX_POS                 = 50.0;
        public static final double TILT_POS_PRESET_TOLERANCE    = 2.0;
        public static final double[] TILT_POS_PRESETS           =
            {TILT_MIN_POS, 30.0, 35.0, 40.0, 45.0, TILT_MAX_POS};

        public static final TrcShootParamTable shootParamTable = new TrcShootParamTable()
            .add("test3ft", 36.0, 60.0, 0.0, 60.0)
            .add("test4ft", 48.0, 70.0, 0.0, 60.0)
            .add("test5ft", 60.0, 80.0, 0.0, 60.0)
            .add("test6ft", 72.0, 90.0, 0.0, 60.0);

        // Launcher
        public static final String LAUNCHER_SERVO_NAME          = SUBSYSTEM_NAME + ".Launcher";
        public static final boolean LAUNCHER_SERVO_INVERTED     = true;
        public static double LAUNCHER_REST_POS                  = 0.415;
        public static double LAUNCHER_LAUNCH_POS                = 0.7;
        public static double LAUNCHER_LAUNCH_DURATION           = 0.5;  // in seconds
    }   //class Params

    public static final TrcMotor.TuneParams shootMotor1PidParams = new TrcMotor.TuneParams()
        .setPidCoefficients(
            Params.SHOOT_MOTOR1_PID_KP, Params.SHOOT_MOTOR1_PID_KI, Params.SHOOT_MOTOR1_PID_KD,
            Params.SHOOT_MOTOR1_PID_KF)
        .setPidParams(Params.SHOOT_PID_TOLERANCE_RPM/60.0, Params.SHOOT_SOFTWARE_PID_ENABLED);
    public static final TrcMotor.TuneParams shootMotor2PidParams = new TrcMotor.TuneParams()
        .setPidCoefficients(
            Params.SHOOT_MOTOR2_PID_KP, Params.SHOOT_MOTOR2_PID_KI, Params.SHOOT_MOTOR2_PID_KD,
            Params.SHOOT_MOTOR2_PID_KF)
        .setPidParams(Params.SHOOT_PID_TOLERANCE_RPM/60.0, Params.SHOOT_SOFTWARE_PID_ENABLED);
    public static final TrcMotor.TuneParams panMotorPidParams = new TrcMotor.TuneParams()
        .setPidCoefficients(
            Params.PAN_MOTOR_PID_KP, Params.PAN_MOTOR_PID_KI, Params.PAN_MOTOR_PID_KD)
        .setPidParams(Params.PAN_PID_TOLERANCE, Params.PAN_SOFTWARE_PID_ENABLED);
    public static final TrcMotor.TuneParams tiltMotorPidParams = new TrcMotor.TuneParams()
        .setPidCoefficients(
            Params.TILT_MOTOR_PID_KP, Params.TILT_MOTOR_PID_KI, Params.TILT_MOTOR_PID_KD)
        .setPidParams(Params.TILT_PID_TOLERANCE, Params.TILT_SOFTWARE_PID_ENABLED);
    public static final TrcServo.TuneParams launcherParams = new TrcServo.TuneParams(
        Params.LAUNCHER_SERVO_INVERTED, Params.LAUNCHER_REST_POS, Params.LAUNCHER_LAUNCH_POS,
        Params.LAUNCHER_LAUNCH_DURATION);

    private final FtcDashboard dashboard;
    private final Robot robot;
    private final TrcShooter shooter;
    public final TrcServo launcher;
    private String launchOwner;
    private TrcEvent launchCompletionEvent;
    private TrcEvent launchCallbackEvent = null;
    private Integer trackedAprilTagId = null;

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
        motor.setVelocityPidParameters(
            shootMotor1PidParams.pidCoeffs, shootMotor1PidParams.pidTolerance,
            shootMotor1PidParams.useSoftwarePid, null);

        motor = shooter.getShooterMotor2();
        if (motor != null)
        {
            // Assuming motor2 is the same type of motor as motor1 and has the same gear ratio.
            // If it needs to, this allows different PID coefficients for motor2 in case they are not quite identical.
            motor.setPositionSensorScaleAndOffset(Params.SHOOT_MOTOR_REV_PER_COUNT, 0.0);
            motor.setVelocityPidParameters(
                shootMotor1PidParams.pidCoeffs, shootMotor2PidParams.pidTolerance,
                shootMotor2PidParams.useSoftwarePid, null);
        }

        motor = shooter.getPanMotor();
        if (motor != null)
        {
            motor.setPositionSensorScaleAndOffset(
                Params.PAN_DEG_PER_COUNT, Params.PAN_POS_OFFSET, Params.PAN_ENCODER_ZERO_OFFSET);
            motor.setPositionPidParameters(
                panMotorPidParams.pidCoeffs, panMotorPidParams.pidTolerance, panMotorPidParams.useSoftwarePid,
                this::getPanPosition);
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
            motor.setPositionPidParameters(
                tiltMotorPidParams.pidCoeffs, tiltMotorPidParams.pidTolerance, tiltMotorPidParams.useSoftwarePid,
                null);
            motor.setSoftPositionLimits(Params.TILT_MIN_POS, Params.TILT_MAX_POS, false);
        }

        if (Params.HAS_LAUNCHER)
        {
            FtcServoActuator.Params launcherParams = new FtcServoActuator.Params()
                .setPrimaryServo(Params.LAUNCHER_SERVO_NAME, Params.LAUNCHER_SERVO_INVERTED);
            launcher = new FtcServoActuator(launcherParams).getServo();
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
     * This method sets the launcher servo position.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships, null if no ownership required.
     * @param launch specifies true to set servo to launch position, false to set to rest position.
     */
    public void setLaunchPosition(String owner, boolean launch)
    {
        if (launch)
        {
            launcher.setPosition(owner, 0.0, launcherParams.activatePos, null, 0.0);
        }
        else
        {
            launcher.setPosition(owner, 0.0, launcherParams.restPos, null, 0.0);
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
            TrcDbgTrace.globalTraceInfo(instanceName, "shoot(owner=" + owner + ", event=" + completionEvent + ")");
            if (robot.spindexer != null)
            {
                // Enable Spindexer exit trigger.
                robot.spindexer.setExitTriggerEnabled(true);
            }
            launchOwner = owner;
            launchCompletionEvent = completionEvent;
            launchCallbackEvent = new TrcEvent(Params.SUBSYSTEM_NAME + ".launchCallback");
            launchCallbackEvent.setCallback(this::launchCallback, null);
            launcher.setPosition(
                owner, 0.0, launcherParams.activatePos, launchCallbackEvent, launcherParams.activateDuration);
        }
        else if (completionEvent != null)
        {
            TrcDbgTrace.globalTraceInfo(instanceName, "There is no launcher, signal completion anyway.");
            completionEvent.signal();
        }
    }   //shoot

    /**
     * This method is called when the launch duration has expired.
     *
     * @param context not used.
     * @param canceled specifies true if launch was canceled (not used).
     */
    private void launchCallback(Object context, boolean canceled)
    {
        // Reset launcher, fire and forget.
        launcher.setPosition(launchOwner, 0.0, launcherParams.restPos, null, 0.0);
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
        launchCallbackEvent = null;
    }   //launchCallback

    /**
     * This method enables AprilTag tracking with the Turret (Pan motor).
     *
     * @param aprilTagId specifies the AprilTag ID to track.
     */
    public void enableAprilTagTracking(int aprilTagId)
    {
        if (robot.vision != null && robot.vision.isLimelightVisionEnabled())
        {
            robot.vision.limelightVision.setPipeline(Vision.LimelightPipelineType.APRIL_TAG.ordinal());
            this.trackedAprilTagId = aprilTagId;
            shooter.panMotor.setPosition(0.0, true, Params.PAN_POWER_LIMIT);
        }
    }   //enableAprilTagTracking

    /**
     * This method disables AprilTag tracking.
     */
    public void disableAprilTagTracking()
    {
        shooter.panMotor.cancel();
        this.trackedAprilTagId = null;
    }   //disableAprilTagTracking

    /**
     * This method is called by Pan Motor PID Control Task to get the current Pan position. By manipulating this
     * position, we can use the PID controller to track the AprilTag target.
     *
     * @return angle distance between the current position and the AprilTag target if tracking is ON, angle position
     *         of the target relative to robot heading if tracking is OFF.
     */
    public double getPanPosition()
    {
        double panPosition;

        if (trackedAprilTagId != null)
        {
            // Tracking is enabled.
            TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> object =
                robot.vision.aprilTagVision.getBestDetectedTargetInfo(trackedAprilTagId, null);
            panPosition = object != null ? -object.objPose.angle : 0.0;
        }
        else
        {
            // Tracking is disabled.
            panPosition = shooter.panMotor.getPosition();
        }

        return panPosition;
    }   //getPanPosition

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
            launcher.cancel();
        }
    }   //cancel

    /**
     * This method starts zero calibrate of the subsystem.
     *
     * @param owner specifies the owner ID to to claim subsystem ownership, can be null if ownership not required.
     * @param event specifies an event to signal when zero calibration is done, can be null if not provided.
     */
    @Override
    public void zeroCalibrate(String owner, TrcEvent event)
    {
        // Shooter does not need zero calibration.
        // Tilter has absolute encoder and therefore no need for zero calibration.
        // Zero calibrate turret (pan).
        shooter.panMotor.zeroCalibrate(owner, Params.PAN_ZERO_CAL_POWER, event);
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
        if (slowLoop)
        {
            TrcMotor motor;

            motor = shooter.getShooterMotor1();
            dashboard.displayPrintf(
                lineNum++, "%s: power=%.3f, current=%.3f, vel=%.3f, target=%.3f",
                Params.SHOOTER_MOTOR1_NAME, motor.getPower(), motor.getCurrent(),
                shooter.getShooterMotor1RPM(), shooter.getShooterMotor1TargetRPM());

            motor = shooter.getShooterMotor2();
            if (motor != null)
            {
                dashboard.displayPrintf(
                    lineNum++, "%s: power=%.3f, current=%.3f, vel=%.3f, target=%.3f",
                    Params.SHOOTER_MOTOR2_NAME, motor.getPower(), motor.getCurrent(),
                    shooter.getShooterMotor2RPM(), shooter.getShooterMotor2TargetRPM());
            }

            motor = shooter.getPanMotor();
            if (motor != null)
            {
                dashboard.displayPrintf(
                    lineNum++, "%s: power=%.3f, current=%.3f, pos=%.3f/%.3f",
                    Params.PAN_MOTOR_NAME, motor.getPower(), motor.getCurrent(),
                    motor.getPosition(), motor.getPidTarget());
            }

            motor = shooter.getTiltMotor();
            if (motor != null)
            {
                dashboard.displayPrintf(
                    lineNum++, "%s: power=%.3f, pos=%.3f/%.3f(%f)",
                    Params.TILT_MOTOR_NAME, motor.getPower(), motor.getPosition(), motor.getPidTarget(),
                    motor.getEncoderRawPosition());
            }

            if (launcher != null)
            {
                dashboard.displayPrintf(
                    lineNum++, "%s: pos=%.3f",
                    Params.LAUNCHER_SERVO_NAME, launcher.getPosition());
            }
        }
        else
        {
            dashboard.putObject("Shooter1MotorRPM", shooter.getShooterMotor1RPM());
            dashboard.putObject("ShooterMotor1TargetRPM", shooter.getShooterMotor1TargetRPM());
            dashboard.putObject("ShooterMotor1RangeMin", 0.0);
            dashboard.putObject("ShooterMotor1RangeMax", Params.SHOOT_MOTOR_MAX_VEL);
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
