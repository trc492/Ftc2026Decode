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

package teamcode.vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import ftclib.drivebase.FtcRobotDrive;
import ftclib.robotcore.FtcOpMode;
import ftclib.vision.FtcEocvColorBlobProcessor;
import ftclib.vision.FtcLimelightVision;
import ftclib.vision.FtcVision;
import ftclib.vision.FtcVisionAprilTag;
import ftclib.vision.FtcVisionEocvColorBlob;
import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.subsystems.LEDIndicator;
import trclib.dataprocessor.TrcUtil;
import trclib.pathdrive.TrcPose2D;
import trclib.pathdrive.TrcPose3D;
import trclib.robotcore.TrcDbgTrace;
import trclib.vision.TrcHomographyMapper;
import trclib.vision.TrcOpenCvColorBlobPipeline;
import trclib.vision.TrcVisionTargetInfo;

/**
 * This class implements AprilTag/Eocv/Limelight Vision for the game season. It creates and initializes all the vision
 * target info as well as providing info for the robot, camera and the field. It also provides methods to get the
 * location of the robot and detected targets.
 */
public class Vision
{
    private final String moduleName = getClass().getSimpleName();
    // Camera lens properties.
    private static final FtcRobotDrive.CameraInfo logitechC920At640x480 = new FtcRobotDrive.CameraInfo()
        .setLensProperties(622.001, 622.001, 319.803, 241.251)
        .setDistortionCoefficents(0.1208, -0.261599, 0, 0, 0.10308, 0, 0, 0);
    private static final FtcRobotDrive.CameraInfo logitechC270At640x480 = new FtcRobotDrive.CameraInfo()
        .setLensProperties(822.317, 822.317, 319.495, 242.502)
        .setDistortionCoefficents(-0.0449369, 1.17277, 0, 0, -3.63244, 0, 0, 0);
    private static final FtcRobotDrive.CameraInfo lifeCamHD3000At640x480 = new FtcRobotDrive.CameraInfo()
        .setLensProperties(678.154, 678.170, 318.135, 228.374)
        .setDistortionCoefficents(0.154576, -1.19143, 0, 0, 2.06105, 0, 0, 0);

    /**
     * This class contains the parameters of the front camera.
     */
    public static class FrontCamParams extends FtcRobotDrive.VisionInfo
    {
        public FrontCamParams()
        {
            camName = "Webcam 1";
            camImageWidth = 320;
            camImageHeight = 240;
            camXOffset = 0.0;                   // Inches to the right from robot center
            camYOffset = 0.0;                   // Inches forward from robot center
            camZOffset = 21.0;                  // Inches up from the floor
            camYaw = 0.0;                       // degrees clockwise from robot forward
            camPitch = -19.0;                   // degrees up from horizontal
            camRoll = 0.0;
            camPose = new TrcPose3D(camXOffset, camYOffset, camZOffset, camYaw, camPitch, camRoll);
            camInfo = lifeCamHD3000At640x480;
            camOrientation = OpenCvCameraRotation.UPRIGHT;
            // Homography: cameraRect in pixels, worldRect in inches
            cameraRect = new TrcHomographyMapper.Rectangle(
                14.0, 28.0,                     // Camera Top Left
                612.0, 33.0,                    // Camera Top Right
                56.0, 448.0,                    // Camera Bottom Left
                581.0, 430.5);                  // Camera Bottom Right
            worldRect = new TrcHomographyMapper.Rectangle(
                -19.0, 37.5,                    // World Top Left
                24.0, 37.5,                     // World Top Right
                -4.75, 9.0,                     // World Bottom Left
                6.25, 9.0);                     // World Bottom Right
        }   //FrontCamParams
    }   //class FrontCamParams

    /**
     * This class contains the parameters of the back camera.
     */
    public static class BackCamParams extends FtcRobotDrive.VisionInfo
    {
        public BackCamParams()
        {
            camName = "Webcam 2";
            camImageWidth = 640;
            camImageHeight = 480;
            camXOffset = 0.0;                   // Inches to the right from robot center
            camYOffset = 2.0;                   // Inches forward from robot center
            camZOffset = 9.75;                  // Inches up from the floor
            camYaw = 0.0;                       // degrees clockwise from robot front
            camPitch = 15.0;                    // degrees down from horizontal
            camRoll = 0.0;
            camPose = new TrcPose3D(camXOffset, camYOffset, camZOffset, camYaw, camPitch, camRoll);
            camOrientation = OpenCvCameraRotation.UPRIGHT;
            // Homography: cameraRect in pixels, worldRect in inches
            cameraRect = new TrcHomographyMapper.Rectangle(
                0.0, 120.0,                                             // Camera Top Left
                camImageWidth -1, 120.0,                                // Camera Top Right
                0.0, camImageHeight - 1,                                // Camera Bottom Left
                camImageWidth - 1, camImageHeight - 1);                 // Camera Bottom Right
            worldRect = new TrcHomographyMapper.Rectangle(
                -12.5626, 48.0 - RobotParams.Robot.ROBOT_LENGTH/2.0 - camYOffset,   // World Top Left
                11.4375, 44.75 - RobotParams.Robot.ROBOT_LENGTH/2.0 - camYOffset,   // World Top Right
                -2.5625, 21.0 - RobotParams.Robot.ROBOT_LENGTH/2.0 - camYOffset,    // World Bottom Left
                2.5626, 21.0 - RobotParams.Robot.ROBOT_LENGTH/2.0 - camYOffset);    // World Bottom Right
        }   //BackCamParams
    }   //class BackCamParams

    /**
     * This class contains the parameters of the Limelight vision processor.
     */
    public static class LimelightParams extends FtcRobotDrive.VisionInfo
    {
        public static final int NUM_PIPELINES = 2;

        public LimelightParams()
        {
            camName = "Limelight3a";
            camImageWidth = 640;
            camImageHeight = 480;
            camHFov = 54.5;                             // in degrees
            camVFov = 42.0;                             // in degrees
            camXOffset = 135.47*TrcUtil.INCHES_PER_MM;  // Inches to the right from robot center
            camYOffset = 2.073;                         // Inches forward from robot center
            camZOffset = 10.758;                        // Inches up from the floor
            camYaw = -3.438;                            // degrees clockwise from robot front
            camPitch = 0.0;                             // degrees down from horizontal
            camRoll = 0.0;
            camPose = new TrcPose3D(camXOffset, camYOffset, camZOffset, camYaw, camPitch, camRoll);
        }   //LimelightParams
    }   //class LimelightParams

    public enum ColorBlobType
    {
        None,
        Purple,
        Green,
        Any
    }   //enum ColorBlobType

    public enum LimelightPipelineType
    {
        APRIL_TAG(0),
        ARTIFACT(1);

        final int value;
        LimelightPipelineType(int value)
        {
            this.value = value;
        }
    }   //enum LimelightPipelineType

    // Warning: EOCV converts camera stream to RGBA whereas Desktop OpenCV converts it to BGRA. Therefore, the correct
    // color conversion must be RGBA (or RGB) to whatever color space you want to convert.
    //
//    // YCrCb Color Space.
//    private static final int colorConversion = Imgproc.COLOR_RGB2YCrCb;
//    private static final double[] purpleThresholdsLow = {80.0, 139.0, 120.0};
//    private static final double[] purpleThresholdsHigh = {180.0, 160.0, 150.0};
//    private static final double[] greenThresholdsLow = {70.0, 40.0, 100.0};
//    private static final double[] greenThresholdsHigh = {220.0, 118.0, 145.0};
    // HSV Color Space.
    private static final TrcOpenCvColorBlobPipeline.ColorConversion colorConversion =
        TrcOpenCvColorBlobPipeline.ColorConversion.RGBToHSV;
    private static final double[] purpleThresholdsLow = {130.0, 20.0, 10.0};
    private static final double[] purpleThresholdsHigh = {165.0, 255.0, 255.0};
    private static final double[] greenThresholdsLow = {48.0, 45.0, 45.0};
    private static final double[] greenThresholdsHigh = {100.0, 250.0, 306.0};
    public static final TrcOpenCvColorBlobPipeline.FilterContourParams artifactFilterContourParams =
        new TrcOpenCvColorBlobPipeline.FilterContourParams()
            .setMinArea(50.0)
            .setMinPerimeter(20.0)
            .setWidthRange(10.0, 500.0)
            .setHeightRange(10.0, 500.0)
            .setSolidityRange(0.0, 100.0)
            .setVerticesRange(0.0, 1000.0)
            .setAspectRatioRange(0.5, 2.0);
    private static final double objectWidth = 5.0;  // inches
    private static final double objectHeight = 5.0; // inches
    private static final Vision.ColorBlobType tuneColorBlobType = Vision.ColorBlobType.Green;

    private static final int CLASSIFIER_HEIGHT_THRESHOLD_LOW = 100;
    private static final int CLASSIFIER_HEIGHT_THRESHOLD_HIGH = 120;
    private static final double ONE_BALL_THRESHOLD = 1.0;
    private static final double TWO_BALL_THRESHOLD = 2.0;
    private static final double THREE_BALL_THRESHOLD = 3.0;
    private static final double FOUR_BALL_THRESHOLD = 4.0;
    private static final double FIVE_BALL_THRESHOLD = 5.0;
    private static final double SIX_BALL_THRESHOLD = 6.0;
    private static final double SEVEN_BALL_THRESHOLD = 7.0;
    private static final double EIGHT_BALL_THRESHOLD = 8.0;
    public static final TrcOpenCvColorBlobPipeline.PipelineParams colorBlobPipelineParams =
        new TrcOpenCvColorBlobPipeline.PipelineParams()
            .setColorConversion(colorConversion)
            .addColorThresholds(LEDIndicator.PURPLE_BLOB, true, purpleThresholdsLow, purpleThresholdsHigh)
            .addColorThresholds(LEDIndicator.GREEN_BLOB, true, greenThresholdsLow, greenThresholdsHigh)
            .setCircleDetection(10.0)
            .setCircleBlur(true, 9)
            .setFilterContourParams(true, artifactFilterContourParams)
            .setObjectSize(objectWidth, objectHeight);

    private final TrcDbgTrace tracer;
    private final Robot robot;
    private final WebcamName webcam1, webcam2;
    public FtcLimelightVision limelightVision;
    public FtcVisionAprilTag aprilTagVision;
    private AprilTagProcessor aprilTagProcessor;
    public FtcVisionEocvColorBlob colorBlobVision;
    private FtcEocvColorBlobProcessor colorBlobProcessor;
    public FtcVision vision;
    private FtcAuto.Alliance alliance = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object.
     */
    public Vision(Robot robot)
    {
        FtcOpMode opMode = FtcOpMode.getInstance();

        if (robot.robotInfo.webCam1 == null && RobotParams.Preferences.useWebCam)
        {
            throw new IllegalArgumentException("Must provide valid WebCam 1 info.");
        }

        this.tracer = new TrcDbgTrace();
        this.robot = robot;

        // Update Dashboard with the initial detection parameters.
//        System.arraycopy(
//            tuneColorBlobType == ColorBlobType.Purple? purpleThresholdsLow: greenThresholdsLow, 0,
//            Dashboard.VisionTuning.colorThresholdsLow, 0, Dashboard.VisionTuning.colorThresholdsLow.length);
//        System.arraycopy(
//            tuneColorBlobType == ColorBlobType.Purple? purpleThresholdsHigh: greenThresholdsHigh, 0,
//            Dashboard.VisionTuning.colorThresholdsHigh, 0, Dashboard.VisionTuning.colorThresholdsHigh.length);
//        Dashboard.VisionTuning.filterContourParams.setAs(artifactFilterContourParams);

        webcam1 = robot.robotInfo.webCam1 != null?
            opMode.hardwareMap.get(WebcamName.class, robot.robotInfo.webCam1.camName): null;
        webcam2 = robot.robotInfo.webCam2 != null?
            opMode.hardwareMap.get(WebcamName.class, robot.robotInfo.webCam2.camName): null;
        // LimelightVision (not a Vision Processor).
        if (RobotParams.Preferences.useLimelightVision && robot.robotInfo.limelight != null)
        {
            limelightVision = new FtcLimelightVision(
                robot.robotInfo.limelight.camName, robot.robotInfo.limelight.camPose,
                this::getLimelightTargetGroundOffset);
            limelightVision.setPipeline(LimelightPipelineType.APRIL_TAG.value);
        }
        // Creating Vision Processors for VisionPortal.
        ArrayList<VisionProcessor> visionProcessorsList = new ArrayList<>();

        if (RobotParams.Preferences.useWebcamAprilTagVision)
        {
            tracer.traceInfo(moduleName, "Starting Webcam AprilTagVision...");
            FtcVisionAprilTag.Parameters aprilTagParams = new FtcVisionAprilTag.Parameters()
                .setDrawTagIdEnabled(true)
                .setDrawTagOutlineEnabled(true)
                .setDrawAxesEnabled(false)
                .setDrawCubeProjectionEnabled(false)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES);
            aprilTagVision = new FtcVisionAprilTag(aprilTagParams, AprilTagProcessor.TagFamily.TAG_36h11);
            aprilTagProcessor = aprilTagVision.getVisionProcessor();
            visionProcessorsList.add(aprilTagProcessor);
        }

        if (RobotParams.Preferences.useColorBlobVision && robot.robotInfo.webCam1 != null)
        {
            FtcRobotDrive.CameraInfo camInfo = null;
            TrcHomographyMapper.Rectangle camRect = null, worldRect = null;

            tracer.traceInfo(moduleName, "Starting Webcam ColorBlobVision...");
            // Create the pipeline for both purple and green artifacts.
            TrcOpenCvColorBlobPipeline.PipelineParams colorBlobPipelineParams =
                new TrcOpenCvColorBlobPipeline.PipelineParams()
                    .setColorConversion(colorConversion)
                    .addColorThresholds(LEDIndicator.PURPLE_BLOB, true, purpleThresholdsLow, purpleThresholdsHigh)
                    .addColorThresholds(LEDIndicator.GREEN_BLOB, true, greenThresholdsLow, greenThresholdsHigh)
                    .setCircleDetection(10.0)
                    .setCircleBlur(true, 9)
                    .setFilterContourParams(true, artifactFilterContourParams)
                    .setObjectSize(objectWidth, objectHeight);

            if (RobotParams.Preferences.useSolvePnp && robot.robotInfo.webCam1 != null)
            {
                camInfo = robot.robotInfo.webCam1.camInfo;
                if (camInfo != null)
                {
                    colorBlobPipelineParams.setSolvePnpParams(
                        camInfo.fx, camInfo.fy, camInfo.cx, camInfo.cy, camInfo.distCoeffs,
                        robot.robotInfo.webCam1.camPose);
                }
                camRect = robot.robotInfo.webCam1.cameraRect;
                worldRect = robot.robotInfo.webCam1.worldRect;
            }

            colorBlobVision = new FtcVisionEocvColorBlob(
                "ColorBlobVision", colorBlobPipelineParams, camRect, worldRect, true, false, false);
            colorBlobProcessor = colorBlobVision.getVisionProcessor();
            if (RobotParams.Preferences.streamToDashboard)
            {
                colorBlobProcessor.enableDashboardStream();
            }
            visionProcessorsList.add(colorBlobProcessor);
        }

        if (!visionProcessorsList.isEmpty())
        {
            VisionProcessor[] visionProcessors = new VisionProcessor[visionProcessorsList.size()];
            visionProcessorsList.toArray(visionProcessors);
            if (RobotParams.Preferences.useWebCam)
            {
                // Use USB webcams.
                vision = new FtcVision(
                    webcam1, webcam2, robot.robotInfo.webCam1.camImageWidth, robot.robotInfo.webCam1.camImageHeight,
                    RobotParams.Preferences.showVisionView, RobotParams.Preferences.showVisionStat,
                    visionProcessors);
            }
            else
            {
                // Use phone camera.
                vision = new FtcVision(
                    RobotParams.Preferences.useBuiltinCamBack?
                        BuiltinCameraDirection.BACK: BuiltinCameraDirection.FRONT,
                    robot.robotInfo.webCam1.camImageWidth, robot.robotInfo.webCam1.camImageHeight,
                    RobotParams.Preferences.showVisionView, RobotParams.Preferences.showVisionStat,
                    visionProcessors);
            }

            // Disable all vision until they are needed.
            for (VisionProcessor processor: visionProcessors)
            {
                vision.setProcessorEnabled(processor, false);
            }

            if (colorBlobProcessor != null)
            {
                updateColorBlobPipelineConfig(colorBlobProcessor.getPipeline());
            }
        }
    }   //Vision

    /**
     * This method closes the vision portal and is normally called at the end of an opmode.
     */
    public void close()
    {
        if (vision != null)
        {
            vision.close();
        }
    }   //close

    /**
     * This method updates the ColorBlob pipeline with the configuration specified in Dashboard.VisionTuning.
     *
     * @param colorBlobPipeline specifies the colorblob pipeline to update its configuration.
     */
    public void updateColorBlobPipelineConfig(TrcOpenCvColorBlobPipeline colorBlobPipeline)
    {
//        if (Dashboard.VisionTuning.annotationEnabled)
//        {
//            colorBlobPipeline.enableAnnotation(
//                Dashboard.VisionTuning.annotateDrawRotatedRect, Dashboard.VisionTuning.annotateDrawCrosshair);
//        }
//        else
//        {
//            colorBlobPipeline.disableAnnotation();
//        }

//        if (Dashboard.VisionTuning.morphologyEnabled)
//        {
//            colorBlobPipeline.enableMorphology(
//                Dashboard.VisionTuning.morphologyClosing? Imgproc.MORPH_CLOSE: Imgproc.MORPH_OPEN,
//                Imgproc.MORPH_ELLIPSE,
//                new Size(Dashboard.VisionTuning.morphologyKernelSize, Dashboard.VisionTuning.morphologyKernelSize));
//        }
//        else
//        {
//            colorBlobPipeline.disableMorphology();
//        }

//        if (Dashboard.VisionTuning.circleDetectionEnabled)
//        {
//            colorBlobPipeline.enableCircleDetection(Dashboard.VisionTuning.circleMinDistance);
//        }
//        else
//        {
//            colorBlobPipeline.disableCircleDetection();
//        }

//        if (Dashboard.VisionTuning.blurEnableGaussian)
//        {
//            colorBlobPipeline.enableCircleBlur(true, Dashboard.VisionTuning.blurKernelSize);
//        }
//        else if (Dashboard.VisionTuning.blurEnableMedian)
//        {
//            colorBlobPipeline.enableCircleBlur(false, Dashboard.VisionTuning.blurKernelSize);
//        }
//        else
//        {
//            colorBlobPipeline.disableCircleBlur();
//        }

//        if (Dashboard.VisionTuning.cannyEdgeEnabled)
//        {
//            colorBlobPipeline.enableCannyEdgeDetection(
//                Dashboard.VisionTuning.cannyEdgeThreshold1, Dashboard.VisionTuning.cannyEdgeThreshold2);
//        }
//        else
//        {
//            colorBlobPipeline.disableCannyEdgeDetection();
//        }
    }   //updateColorBlobPipelineConfig

    /**
     * This method enables/disables FPS meter on the viewport.
     *
     * @param enabled specifies true to enable FPS meter, false to disable.
     */
    public void setFpsMeterEnabled(boolean enabled)
    {
        if (vision != null)
        {
            vision.setFpsMeterEnabled(enabled);
        }
    }   //setFpsMeterEnabled

    /**
     * This method returns the front webcam.
     *
     * @return front webcam.
     */
    public WebcamName getFrontWebcam()
    {
        return webcam1;
    }   //getFrontWebcam

    /**
     * This method returns the rear webcam.
     *
     * @return rear webcam.
     */
    public WebcamName getRearWebcam()
    {
        return webcam2;
    }   //getRearWebcam

    /**
     * This method returns the active camera if we have two webcams.
     *
     * @return active camera.
     */
    public WebcamName getActiveWebcam()
    {
        return vision.getActiveWebcam();
    }   //getActiveWebcam

    /**
     * This method sets the active webcam.
     *
     * @param webcam specifies the webcam to be set as active.
     */
    public void setActiveWebcam(WebcamName webcam)
    {
        vision.setActiveWebcam(webcam);
    }   //setActiveWebcam

    /**
     * This method displays the exposure settings on the dashboard. This helps tuning camera exposure.
     *
     * @param lineNum specifies the dashboard line number to display the info.
     */
    public void displayExposureSettings(int lineNum)
    {
        long[] exposureSetting = vision.getExposureSetting();
        long currExposure = vision.getCurrentExposure();
        int[] gainSetting = vision.getGainSetting();
        int currGain = vision.getCurrentGain();

        if (exposureSetting != null && gainSetting != null)
        {
            robot.dashboard.displayPrintf(
                lineNum, "Exp: %d (%d:%d), Gain: %d (%d:%d)",
                currExposure, exposureSetting[0], exposureSetting[1], currGain, gainSetting[0], gainSetting[1]);
        }
    }   //displayExposureSettings

    /**
     * This method enables/disables Limelight vision for the specified pipeline.
     *
     * @param pipelineIndex specifies the limelight pipeline index to be selected, ignore if disabled.
     * @param enabled specifies true to enable, false to disable.
     */
    public void setLimelightVisionEnabled(int pipelineIndex, boolean enabled)
    {
        if (limelightVision != null)
        {
            if (enabled)
            {
                limelightVision.setPipeline(pipelineIndex);
            }
            limelightVision.setVisionEnabled(enabled);
            tracer.traceInfo(moduleName, "Pipeline %d is %s: running=%s",
                             pipelineIndex, enabled? "enabled": "disabled", limelightVision.limelight.isRunning());
        }
    }   //setLimelightVisionEnabled

    /**
     * This method checks if Limelight vision is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    public boolean isLimelightVisionEnabled()
    {
        return limelightVision != null && limelightVision.isVisionEnabled();
    }   //isLimelightVisionEnabled

    /**
     * This method calls Limelight vision to detect the object.
     *
     * @param resultType specifies the result type to look for.
     * @param label specifies the detected object label, can be null to match any label.
     * @param lineNum specifies the dashboard line number to display the detected object info, -1 to disable printing.
     * @return detected Limelight object info.
     */
    public TrcVisionTargetInfo<FtcLimelightVision.DetectedObject> getLimelightDetectedObject(
        FtcLimelightVision.ResultType resultType, String label, int lineNum)
    {
        TrcVisionTargetInfo<FtcLimelightVision.DetectedObject> limelightInfo = null;

        if (limelightVision != null)
        {
            String objectName = null;
            int pipelineIndex = -1;
            Double robotHeading = robot.robotDrive != null? robot.robotDrive.driveBase.getHeading(): null;

            limelightInfo = limelightVision.getBestDetectedTargetInfo(resultType, label, robotHeading, null);
            if (limelightInfo != null)
            {
                pipelineIndex = limelightVision.getPipeline();
                switch (pipelineIndex)
                {
                    case 0:
                        objectName = LEDIndicator.APRIL_TAG;
                        break;

                    case 1:
                        objectName = LEDIndicator.PURPLE_BLOB;
                        break;

                    case 2:
                        objectName = LEDIndicator.GREEN_BLOB;
                        break;

                    default:
                        break;
                }
            }

            if (objectName != null && robot.ledIndicator != null)
            {
                robot.ledIndicator.setStatusPattern(objectName);
            }

            if (lineNum != -1)
            {
                robot.dashboard.displayPrintf(
                    lineNum, "%s(%d): %s",
                    objectName, pipelineIndex, limelightInfo != null? limelightInfo: "Not found.");
            }
        }

        return limelightInfo;
    }   //getLimelightDetectedObject

    /**
     * This method enables/disables the Vision Processor.
     *
     * @param processor specifies the vision processor to enable/disable.
     * @param enabled specifies true to enable, false to disable.
     */
    public void setVisionProcessorEnabled(VisionProcessor processor, boolean enabled)
    {
        if (processor != null)
        {
            vision.setProcessorEnabled(processor, enabled);
        }
    }   //setVisionProcessorEnabled

    /**
     * This method checks if the Vision Processor is enabled.
     *
     * @param processor specifies the vision processor to enable/disable.
     * @return true if enabled, false if disabled.
     */
    public boolean isVisionProcessorEnabled(VisionProcessor processor)
    {
        return processor != null && vision.isVisionProcessorEnabled(processor);
    }   //isVisionProcessorEnabled

    /**
     * This method enables/disables ColorBlobProcessor to stream to FtcDashboard.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public void setDashboardStreamEnabled(boolean enabled)
    {
        if (vision != null && colorBlobProcessor != null)
        {
            if (enabled)
            {
                colorBlobProcessor.enableDashboardStream();
            }
            else
            {
                colorBlobProcessor.disableDashboardStream();
            }
        }
    }   //setDashboardStreamEnabled

    /**
     * This method checks if the FtcDashboard streaming is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    public boolean isDashboardStreamEnabled()
    {
        return colorBlobProcessor != null && colorBlobProcessor.isDashboardStreamEnabled();
    }   //isDashboardStreamEnabled

    /**
     * This method enables/disables AprilTag vision.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    public void setAprilTagVisionEnabled(boolean enabled)
    {
        setVisionProcessorEnabled(aprilTagProcessor, enabled);
    }   //setAprilTagVisionEnabled

    /**
     * This method checks if AprilTag vision is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    public boolean isAprilTagVisionEnabled()
    {
        return isVisionProcessorEnabled(aprilTagProcessor);
    }   //isAprilTagVisionEnabled

    /**
     * This method calls AprilTag vision to detect the AprilTag object.
     *
     * @param id specifies the AprilTag ID to look for, null if match to any ID.
     * @param lineNum specifies the dashboard line number to display the detected object info, -1 to disable printing.
     * @return detected AprilTag object info.
     */
    public TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> getDetectedAprilTag(Integer id, int lineNum)
    {
        TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> aprilTagInfo =
            aprilTagVision.getBestDetectedTargetInfo(id, null);

        if (aprilTagInfo != null && robot.ledIndicator != null)
        {
            robot.ledIndicator.setStatusPattern(LEDIndicator.APRIL_TAG);
        }

        if (lineNum != -1)
        {
            robot.dashboard.displayPrintf(
                lineNum, "%s: %s", LEDIndicator.APRIL_TAG, aprilTagInfo != null? aprilTagInfo : "Not found.");
        }

        return aprilTagInfo;
    }   //getDetectedAprilTag

    /**
     * This method calculates the robot's absolute field location with the detected AprilTagInfo.
     *
     * @param aprilTagInfo specifies the detected AprilTag info.
     * @return robot field location.
     */
    public TrcPose2D getRobotFieldPose(TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> aprilTagInfo)
    {
        TrcPose2D robotPose = null;

        if (aprilTagInfo != null)
        {
            TrcPose2D aprilTagPose =
                RobotParams.Game.APRILTAG_POSES[aprilTagInfo.detectedObj.aprilTagDetection.id - 1];
            TrcPose2D cameraPose = aprilTagPose.subtractRelativePose(aprilTagInfo.objPose);
            robotPose = cameraPose.subtractRelativePose(
                new TrcPose2D(robot.robotInfo.webCam1.camXOffset, robot.robotInfo.webCam1.camYOffset,
                              robot.robotInfo.webCam1.camYaw));
            tracer.traceInfo(
                moduleName,
                "AprilTagId=" + aprilTagInfo.detectedObj.aprilTagDetection.id +
                ", aprilTagFieldPose=" + aprilTagPose +
                ", aprilTagPoseFromCamera=" + aprilTagInfo.objPose +
                ", cameraPose=" + cameraPose +
                ", robotPose=%s" + robotPose);
        }

        return robotPose;
    }   //getRobotFieldPose

    /**
     * This method uses vision to find an AprilTag and uses the AprilTag's absolute field location and its relative
     * position from the camera to calculate the robot's absolute field location.
     *
     * @return robot field location.
     */
    public TrcPose2D getRobotFieldPose()
    {
        TrcPose2D robotPose = null;

        if (isLimelightVisionEnabled())
        {
            TrcVisionTargetInfo<FtcLimelightVision.DetectedObject> aprilTagInfo =
                getLimelightDetectedObject(FtcLimelightVision.ResultType.Fiducial, null, -1);

            if (aprilTagInfo != null)
            {
                robotPose = aprilTagInfo.detectedObj.robotPose;
            }
        }
        else if (isAprilTagVisionEnabled())
        {
            // Find any AprilTag in view.
            TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> aprilTagInfo = getDetectedAprilTag(null, -1);

            if (aprilTagInfo != null)
            {
                robotPose = getRobotFieldPose(aprilTagInfo);
            }
        }

        return robotPose;
    }   //getRobotFieldPose

    /**
     * This method enables/disables vision for the specified color blob type.
     *
     * @param colorBlobType specifies the color blob type to be detected.
     * @param enabled specifies true to enable, false to disable.
     */
    public void setColorBlobVisionEnabled(ColorBlobType colorBlobType, boolean enabled)
    {
        TrcOpenCvColorBlobPipeline colorBlobPipeline =
            colorBlobProcessor != null? colorBlobProcessor.getPipeline(): null;

        if (colorBlobPipeline != null)
        {
            if (enabled)
            {
                switch (colorBlobType)
                {
                    case Purple:
                        colorBlobPipeline.setColorThresholdsEnabled(LEDIndicator.PURPLE_BLOB, enabled);
                        break;

                    case Green:
                        colorBlobPipeline.setColorThresholdsEnabled(LEDIndicator.GREEN_BLOB, enabled);
                        break;

                    case Any:
                        colorBlobPipeline.setColorThresholdsEnabled(LEDIndicator.PURPLE_BLOB, enabled);
                        colorBlobPipeline.setColorThresholdsEnabled(LEDIndicator.GREEN_BLOB, enabled);
                        break;
                }
                setVisionProcessorEnabled(colorBlobProcessor, true);
                if (RobotParams.Preferences.streamToDashboard)
                {
                    colorBlobProcessor.enableDashboardStream();
                }
            }
            else if (colorBlobType == ColorBlobType.Any)
            {
                // Disabling all color thresholds.
                if (colorBlobProcessor.isDashboardStreamEnabled())
                {
                    colorBlobProcessor.disableDashboardStream();
                }
                setVisionProcessorEnabled(colorBlobProcessor, false);
            }
        }
    }   //setColorBlobVisionEnabled

    /**
     * This method checks if vision is enabled for the specified color blob type.
     *
     * @param colorBlobType specifies the color blob type to be detected.
     * @return true if enabled, false if disabled.
     */
    public boolean isColorBlobVisionEnabled(ColorBlobType colorBlobType)
    {
        boolean enabled = false;
        TrcOpenCvColorBlobPipeline colorBlobPipeline =
            colorBlobProcessor != null? colorBlobProcessor.getPipeline(): null;

        if (colorBlobPipeline != null)
        {
            switch (colorBlobType)
            {
                case Purple:
                    enabled = isVisionProcessorEnabled(colorBlobProcessor) &&
                              colorBlobPipeline.isColorThresholdsEnabled(LEDIndicator.PURPLE_BLOB);
                    break;

                case Green:
                    enabled = isVisionProcessorEnabled(colorBlobProcessor) &&
                              colorBlobPipeline.isColorThresholdsEnabled(LEDIndicator.GREEN_BLOB);
                    break;

                case Any:
                    enabled = isVisionProcessorEnabled(colorBlobProcessor) &&
                              (colorBlobPipeline.isColorThresholdsEnabled(LEDIndicator.PURPLE_BLOB) ||
                               colorBlobPipeline.isColorThresholdsEnabled(LEDIndicator.GREEN_BLOB));
                    break;
            }
        }

        return enabled;
    }   //isColorBlobVisionEnabled

    /**
     * This method calls ColorBlob vision to detect the specified color blob object.
     *
     * @param colorBlobType specifies the color blob type to be detected.
     * @param groundOffset specifies the ground offset of the detected sample.
     * @param lineNum specifies the dashboard line number to display the detected object info, -1 to disable printing.
     * @return detected color blob object info.
     */
    public TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> getDetectedColorBlob(
        ColorBlobType colorBlobType, double groundOffset, int lineNum)
    {
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> colorBlobInfo = null;

        if (isColorBlobVisionEnabled(colorBlobType))
        {
            colorBlobInfo = colorBlobVision == null? null:
                colorBlobVision.getBestDetectedTargetInfo(
                    this::colorBlobFilter, colorBlobType, this::compareDistanceY, groundOffset,
                    robot.robotInfo.webCam1.camZOffset);
        }

        if (colorBlobInfo != null && robot.ledIndicator != null)
        {
            robot.ledIndicator.setStatusPattern(colorBlobInfo.detectedObj.label);
        }

        if (lineNum != -1)
        {
            if (colorBlobInfo != null)
            {
                robot.dashboard.displayPrintf(lineNum, "%s: %s", colorBlobInfo.detectedObj.label, colorBlobInfo);
            }
            else
            {
                robot.dashboard.displayPrintf(lineNum, "No ColorBlob found.");
            }
        }

        return colorBlobInfo;
    }   //getDetectedColorBlob

    /**
     * The method uses vision to detect all Artifacts in the classifier and returns an array of 9 slots specifying the
     * type of artifacts in each slot. It assumes ColorBlob pipeline is enabled to detect Any artifacts and Circle
     * Detection has been turned off before calling this method.
     *
     * @param alliance specifies the alliance color for sorting the array.
     */
    public ColorBlobType[] getClassifierArtifacts(FtcAuto.Alliance alliance)
    {
        ColorBlobType[] colorBlobs = null;

        if (isColorBlobVisionEnabled(ColorBlobType.Any))
        {
            // compareDistanceX is using alliance to determine the sort order of color blobs in the classifier.
            this.alliance = alliance;
            ArrayList<TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject>> objects =
                colorBlobVision.getDetectedTargetsInfo(
                    this::classifierColorBlobFilter, null, this::compareDistanceX, 0.0, 0.0);

            if (objects != null)
            {
                int index = 0;
                colorBlobs = new ColorBlobType[9];

                for (int i = 0; i < objects.size(); i++)
                {
                    TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> obj = objects.get(i);
                    ColorBlobType colorBlob =
                        obj.detectedObj.label.equals(LEDIndicator.PURPLE_BLOB) ?
                            ColorBlobType.Purple : ColorBlobType.Green;
                    int count = getArtifactCount(obj);
                    tracer.traceInfo(moduleName, "[%d] color=%s, count=%d, obj=%s", i, colorBlob, count, obj);

                    for (int j = 0; j < count; j++)
                    {
                        if (index < colorBlobs.length)
                        {
                            colorBlobs[index++] = colorBlob;
                        }
                        else
                        {
                            tracer.traceWarn(
                                moduleName, "Number artifact exceeds capacity (color=%s, count=%d, obj=%s)",
                                colorBlob, count, obj);
                            break;
                        }
                    }
                }

                for (int k = index; k < colorBlobs.length; k++)
                {
                    colorBlobs[k] = ColorBlobType.None;
                }
            }
        }

        return colorBlobs;
    }   //getClassifierArtifacts

    /**
     * This method checks the detected object aspect ratio to determine how many artifacts are in the detected object.
     *
     * @param obj specifies the detected object.
     * @return count of artifact in the detected object.
     */
    private int getArtifactCount(TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> obj)
    {
        double aspectRatio = (double) obj.objRect.width/(double) obj.objRect.height;
        return aspectRatio <= ONE_BALL_THRESHOLD ? 1 :
               aspectRatio <= TWO_BALL_THRESHOLD ? 2 :
               aspectRatio <= THREE_BALL_THRESHOLD ? 3 :
               aspectRatio <= FOUR_BALL_THRESHOLD ? 4 :
               aspectRatio <= FIVE_BALL_THRESHOLD ? 5 :
               aspectRatio <= SIX_BALL_THRESHOLD ? 6 :
               aspectRatio <= SEVEN_BALL_THRESHOLD ? 8 :
               aspectRatio <= EIGHT_BALL_THRESHOLD ? 8 : 9;
    }   //getArtifactCount

    /**
     * This method returns the Limelight target Z offset from ground.
     *
     * @param resultType specifies the detected object result type.
     * @return target ground offset.
     */
    private double getLimelightTargetGroundOffset(FtcLimelightVision.ResultType resultType)
    {
        double offset;

        switch (resultType)
        {
            case Fiducial:
                offset = 29.5;
                break;

            case Python:
            default:
                offset = 0.0;
                break;
        }

        return offset;
    }   //getLimelightTargetGroundOffset

    /**
     * This method is called by Vision to validate if the detected object matches expectation for filtering.
     *
     * @param objInfo specifies the detected object info.
     * @param context specifies the expected color blob type.
     * @return true if it matches expectation, false otherwise.
     */
    public boolean colorBlobFilter(
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> objInfo, Object context)
    {
        ColorBlobType colorBlobType = (ColorBlobType) context;
        boolean match = false;

        switch (colorBlobType)
        {
            case Purple:
                match = objInfo.detectedObj.label.equals(LEDIndicator.PURPLE_BLOB);
                break;

            case Green:
                match = objInfo.detectedObj.label.equals(LEDIndicator.GREEN_BLOB);
                break;

            case Any:
                match = objInfo.detectedObj.label.equals(LEDIndicator.PURPLE_BLOB) ||
                        objInfo.detectedObj.label.equals(LEDIndicator.GREEN_BLOB);
                break;
        }

        return match;
    }   //colorBlobFilter

    /**
     * This method is called by Vision to validate if the detected color blob is in the classifier by checking its
     * height.
     *
     * @param objInfo specifies the detected object info.
     * @param context not used.
     * @return true if it matches expectation, false otherwise.
     */
    public boolean classifierColorBlobFilter(
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> objInfo, Object context)
    {
        return (objInfo.detectedObj.label.equals(LEDIndicator.PURPLE_BLOB) ||
                objInfo.detectedObj.label.equals(LEDIndicator.GREEN_BLOB)) &&
               objInfo.objRect.y >= CLASSIFIER_HEIGHT_THRESHOLD_LOW &&
               objInfo.objRect.y <= CLASSIFIER_HEIGHT_THRESHOLD_HIGH;
    }   //classifierColorBlobFilter

    /**
     * This method is called by the Arrays.sort to sort the target object by increasing distance X. The sort direction
     * will be determined by the alliance color.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has closer distance than b, 0 if a and b have equal distances, positive value
     *         if a has higher distance than b.
     */
    public int compareDistanceX(
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> a,
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> b)
    {
        int diff = (int)((b.objPose.x - a.objPose.x)*100);
        return alliance == FtcAuto.Alliance.RED_ALLIANCE? diff: -diff;
    }   //compareDistanceX

    /**
     * This method is called by the Arrays.sort to sort the target object by increasing distance Y.
     *
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has closer distance than b, 0 if a and b have equal distances, positive value
     *         if a has higher distance than b.
     */
    public int compareDistanceY(
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> a,
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> b)
    {
        return (int)((b.objPose.y - a.objPose.y)*100);
    }   //compareDistanceY

    /**
     * This method update the dashboard with vision status.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     * @param slowLoop specifies true if this is a slow loop, false otherwise.
     * @return updated line number for the next subsystem to print.
     */
    public int updateStatus(int lineNum, boolean slowLoop)
    {
        if (slowLoop)
        {
            if (limelightVision != null)
            {
                lineNum = limelightVision.updateStatus(lineNum);
            }

            if (aprilTagVision != null)
            {
                lineNum = aprilTagVision.updateStatus(lineNum);
            }

            if (colorBlobVision != null)
            {
                lineNum = colorBlobVision.updateStatus(lineNum);
            }
        }

        return lineNum;
    }   //updateStatus

}   //class Vision
