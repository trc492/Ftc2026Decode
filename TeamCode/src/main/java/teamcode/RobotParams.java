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

package teamcode;

import android.annotation.SuppressLint;

import teamcode.subsystems.BaseDrive;
import teamcode.vision.Vision;
import trclib.dataprocessor.TrcUtil;
import trclib.pathdrive.TrcPose2D;

/**
 * This class contains robot constants and parameters.
 */
public class RobotParams
{
    /**
     * This class contains robot preferences. It enables/disables various robot features. This is especially useful
     * during robot development where some subsystems may not be available or ready yet. By disabling unavailable
     * subsystems, one can test the rest of the robot without the fear of code crashing when some subsystems are not
     * found.
     */
    public static class Preferences
    {
        // Global config
        public static final BaseDrive.RobotType robotType       = BaseDrive.RobotType.IntoTheDeepRobot;
        public static final boolean inCompetition               = false;
        public static final boolean useTraceLog                 = true;
        public static final boolean useBatteryMonitor           = false;
        // Driver feedback
        // Status Update: Dashboard Update may affect robot loop time, don't do it when in competition.
        public static final boolean updateDashboard             = !inCompetition;   // Start up default value.
        public static final boolean useRumble                   = false;
        // Vision
        public static final boolean useVision                   = false;
        public static final boolean showVisionStatus            = true;
        public static final boolean useLimelightVision          = true;
        public static final boolean useWebCam                   = true;
        public static final boolean useWebcamAprilTagVision     = false;
        public static final boolean useArtifactVision           = true;
        public static final boolean useClassifierVision         = true;
        public static final boolean useSolvePnp                 = false;
        public static final boolean streamWebcamToDashboard     = false;
        public static final boolean showVisionView              = false;    // For both HDMI and Dashboard
        public static final boolean showVisionStat              = false;    // For HDMI
        // Master switches for Subsystems
        public static final boolean useSubsystems               = false;
        // Drive Base Subsystem
        public static final boolean useDriveBase                = true;
        public static final boolean showDriveBaseStatus         = false;
        public static final boolean showPidDrive                = false;
        public static final boolean showDriveBaseGraph          = false;
        public static final boolean tuneDriveBase               = false;
        public static final boolean tuneSteerPowerComp          = false;
        // Other Subsystems
        public static final boolean useIntake                   = true;
        public static final boolean showIntakeStatus            = true;
        public static final boolean useSpindexer                = true;
        public static final boolean showSpindexerStatus         = true;
        public static final boolean useShooter                  = true;
        public static final boolean showShooterStatus           = true;
        public static final boolean showShooterGraph            = true;
        // Auto Tasks
        public static final boolean useAutoPickup               = false;
        public static final boolean useAutoShoot                = false;
    }   //class Preferences

    /**
     * This class contains miscellaneous robot info.
     */
    public static class Robot
    {
//        public static final String TEAM_FOLDER_PATH             =
//            Environment.getExternalStorageDirectory().getPath() + "/FIRST/ftc3543";
        @SuppressLint("SdCardPath")
        public static final String TEAM_FOLDER_PATH             = "/sdcard/FIRST/ftc3543";
        public static final String LOG_FOLDER_PATH              = TEAM_FOLDER_PATH + "/tracelogs";
        public static final String STEER_ZERO_CAL_FILE          = TEAM_FOLDER_PATH + "/SteerZeroCalibration.txt";
        //TODO: Needs adjustment with Shooter and side plates mounted.
        public static final double ROBOT_LENGTH                 = 432.0*TrcUtil.INCHES_PER_MM;
        public static final double ROBOT_WIDTH                  = 384.0*TrcUtil.INCHES_PER_MM;
    }   //class Robot

    /**
     * This class contains season specific game element information.
     */
    public static class Game
    {
        public static final boolean fieldIsMirrored             = true;
        // AprilTag locations.
        public static final TrcPose2D[] APRILTAG_POSES          = new TrcPose2D[] {
            new TrcPose2D(-58.3727, -55.6425, -133.62826142189439980808796560491),  // TagId 20: z = 29.5 in
            new TrcPose2D(-72.0, 0.0, -90.0),                                       // TagId 21: z = 18.75 in
            new TrcPose2D(-72.0, 0.0, -90.0),                                       // TagId 22: z = 18.75 in
            new TrcPose2D(-72.0, 0.0, -90.0),                                       // TagId 23: z = 18.75 in
            new TrcPose2D(-58.3727, 55.6425, -46.371738578105600191912034395085)    // TagId 24: z = 29.5 in
        };
        public static final TrcPose2D BLUE_CORNER_POSE          =
            new TrcPose2D(-Field.HALF_FIELD_INCHES, -Field.HALF_FIELD_INCHES, -135.0);
        public static final TrcPose2D RED_CORNER_POSE           =
            new TrcPose2D(-Field.HALF_FIELD_INCHES, Field.HALF_FIELD_INCHES, -45.0);
        public static final TrcPose2D BLUE_APRILTAG_TO_CORNER   = BLUE_CORNER_POSE.relativeTo(APRILTAG_POSES[0]);
        public static final TrcPose2D RED_APRILTAG_TO_CORNER    = RED_CORNER_POSE.relativeTo(APRILTAG_POSES[4]);

        public static final int[] blueGoalAprilTag              = new int[] {20};
        public static final int[] redGoalAprilTag               = new int[] {24};
        public static final int[] anyGoalAprilTags              = new int[] {20, 24};
        public static final int[] obeliskAprilTags              = new int[] {21, 22, 23};
        public static final Vision.ArtifactType[][] motifPatterns =
            new Vision.ArtifactType[][]
            {
                {Vision.ArtifactType.Green, Vision.ArtifactType.Purple, Vision.ArtifactType.Purple},    // AprilTag 21
                {Vision.ArtifactType.Purple, Vision.ArtifactType.Green, Vision.ArtifactType.Purple},    // AprilTag 22
                {Vision.ArtifactType.Purple, Vision.ArtifactType.Purple, Vision.ArtifactType.Green}     // AprilTag 23
            };
        // Robot start locations.
        public static final TrcPose2D STARTPOSE_RED_GOAL_ZONE       =
            new TrcPose2D(-2.0*Field.FULL_TILE_INCHES, 2.0*Field.FULL_TILE_INCHES, 144.5);
        public static final TrcPose2D STARTPOSE_RED_LOAD_CENTER     =
            new TrcPose2D(Field.HALF_FIELD_INCHES - Robot.ROBOT_LENGTH/2.0, 0.5*Field.FULL_TILE_INCHES, -90.0);
        public static final TrcPose2D STARTPOSE_RED_LOAD_CORNER     =
            new TrcPose2D(Field.HALF_FIELD_INCHES - Robot.ROBOT_LENGTH/2.0,
                          Field.FULL_TILE_INCHES + Robot.ROBOT_WIDTH/2.0,
                          -90.0);

        public static final TrcPose2D RED_PRELOAD_LAUNCH_SHOOT_POSE =
            new TrcPose2D(0.0, 0.0, 0.0); // TODO: Determine if we can shoot from starting position
        public static final TrcPose2D RED_PRELOAD_GOAL_SHOOT_POSE   =
            new TrcPose2D(-Field.FULL_TILE_INCHES, Field.FULL_TILE_INCHES, 135.0); //TODO: Can be changed to 1
        // .0 if using motif vision for preload
        public static final TrcPose2D RED_SPIKEMARK_SHOOT_POSE_GOAL      =
            new TrcPose2D(-0.5*Field.FULL_TILE_INCHES, 0.5*Field.FULL_TILE_INCHES, 135.0);
        public static final TrcPose2D RED_SPIKEMARK_SHOOT_POSE_FAR      =
            new TrcPose2D(2.0*Field.FULL_TILE_INCHES, 0.5*Field.FULL_TILE_INCHES, -55.0);

        public static final TrcPose2D RED_SPIKEMARK_PICKUP_POSE_1   =
            new TrcPose2D(-0.5*Field.FULL_TILE_INCHES, Field.FULL_TILE_INCHES, 0.0);
        public static final TrcPose2D RED_SPIKEMARK_PICKUP_POSE_2   =
            new TrcPose2D(0.5*Field.FULL_TILE_INCHES, Field.FULL_TILE_INCHES, 0.0);
        public static final TrcPose2D RED_SPIKEMARK_PICKUP_POSE_3   =
            new TrcPose2D(1.5*Field.FULL_TILE_INCHES, Field.FULL_TILE_INCHES, 0.0);
        public static final TrcPose2D[] RED_SPIKEMARK_POS           =
            {RED_SPIKEMARK_PICKUP_POSE_1, RED_SPIKEMARK_PICKUP_POSE_2, RED_SPIKEMARK_PICKUP_POSE_3};
        public static final TrcPose2D RED_PARK_POSE                 = new TrcPose2D(0.0, 1.5*Field.FULL_TILE_INCHES, 0.0); //TODO: Not sure about this
        // Game elapsed times.
        public static final double AUTO_PERIOD                  = 30.0;     // 30 seconds auto period
        public static final double TELEOP_PERIOD                = 120.0;    // 2 minutes teleop period
        public static final double PARKING_TIME                 = 10.0;
        public static final double ENDGAME_DEADLINE             = TELEOP_PERIOD - PARKING_TIME;
    }   //class Game

    /**
     * This class contains field dimension constants. Generally, these should not be changed.
     */
    public static class Field
    {
        public static final double FULL_FIELD_INCHES            = 141.24;
        public static final double HALF_FIELD_INCHES            = FULL_FIELD_INCHES/2.0;
        public static final double FULL_TILE_INCHES             = FULL_FIELD_INCHES/6.0;
    }   //class Field

    /**
     * This class contains Gobilda motor parameters.
     */
    public static class MotorSpec
    {
        //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-71-2-1-ratio-24mm-length-8mm-rex-shaft-84-rpm-3-3-5v-encoder/
        public static final double GOBILDA_84_ENC_PPR           =
            (((1.0+46.0/17.0)*(1.0+46.0/17.0)*(1.0+46.0/11.0))*28.0);
        public static final double GOBILDA_84_MAX_RPM           = 84.0;
        public static final double GOBILDA_84_MAX_VEL_PPS       = GOBILDA_84_ENC_PPR*GOBILDA_84_MAX_RPM/60.0;
        //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
        public static final double GOBILDA_312_ENC_PPR          = (((1.0+46.0/17.0)*(1.0+46.0/11.0))*28.0);
        public static final double GOBILDA_312_MAX_RPM          = 312.0;
        public static final double GOBILA_312_MAX_VEL_PPS       = GOBILDA_312_ENC_PPR*GOBILDA_312_MAX_RPM/60.0;
        //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/
        public static final double GOBILDA_435_ENC_PPR          = (((1.0+46.0/17.0)*(1.0+46.0/17.0))*28.0);
        public static final double GOBILDA_435_MAX_RPM          = 435.0;
        public static final double GOBILDA_435_MAX_VEL_PPS      = GOBILDA_435_ENC_PPR*GOBILDA_435_MAX_RPM/60.0;
        //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-5-2-1-ratio-24mm-length-8mm-rex-shaft-1150-rpm-3-3-5v-encoder/
        public static final double GOBILDA_1150_ENC_PPR         = ((1.0+(46.0/11.0))*28.0);
        public static final double GOBILDA_1150_MAX_RPM         = 1150.0;
        public static final double GOBILDA_1150_MAX_VEL_PPS     = GOBILDA_1150_ENC_PPR*GOBILDA_1150_MAX_RPM/60.0;
        //https://www.revrobotics.com/rev-41-1300/
        public static final double REV_COREHEX_ENC_PPR          = 288.0;
        public static final double REV_COREHEX_MAX_RPM          = 125.0;
        public static final double REV_COREHEX_MAX_VEL_PPS      = REV_COREHEX_ENC_PPR*REV_COREHEX_MAX_RPM/60.0;
    }   //class MotorSpec

}   //class RobotParams
