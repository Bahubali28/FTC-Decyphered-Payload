package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Autonomous.AprilTagDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Disabled
@Autonomous(name = "Autonomous_RED_FRONT", group = "Autonomous", preselectTeleOp = "Payload_TeleOp")
public abstract class Autonomous_RED_FRONT extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "RedTeamProp.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
//    private static final String TFO
//    TFOD_MODEL_FILE = "sdcard/FIRST/tflitemodels/Red_Team_Prop.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Red Team Prop",
    };
    public boolean LEFT;
    public boolean RIGHT;
    public boolean CENTER;
    boolean Seen;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    AprilTagDetection tagOfInterest = null;
    boolean dontre;
    boolean desYaw;
    boolean desX;
    boolean checkYaw;
    boolean desZ;
    boolean checkYaw2;

    boolean parkNeeded;
    boolean placedPixel = false;
    int step;
    private Servo serIn4, serIn5;
    private CRServo serIn1, serIn2;

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    private DcMotor fl, fr, bl, br;
    public void moveForward(double power) {
        fl.setPower(-power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(-power);
    }
    public void moveBackward(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }
    public void turnLeft(double power) {
        fl.setPower(power);
        fr.setPower(-power);
        bl.setPower(power);
        br.setPower(-power);
    }
    public void turnRight(double power) {
        fl.setPower(-power);
        fr.setPower(power);
        bl.setPower(-power);
        br.setPower(power);
    }
    public void strafeRight(double power) {
        fl.setPower(power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(power);
    }
    public void strafeLeft(double power) {
        fl.setPower(-power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(-power);
    }
    public void Idle() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException{
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        initTfod();
        Seen = false;
        CENTER = false;
        RIGHT = false;
        LEFT = false;
        boolean placed = false;
        int step2 = 1;
        boolean exe = false;
        boolean exe2 = false;
        telemetry.addLine("Initializing...");
        telemetry.update();
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        serIn4 = hardwareMap.get(Servo.class, "serIn4");
        serIn5 = hardwareMap.get(Servo.class, "serIn5");
        serIn1 = hardwareMap.get(CRServo.class, "serIn1");
        serIn2 = hardwareMap.get(CRServo.class, "serIn2");

        serIn1.setDirection(CRServo.Direction.REVERSE);
        serIn2.setDirection(CRServo.Direction.REVERSE);
        serIn5.setDirection(Servo.Direction.REVERSE);
        serIn4.setDirection(Servo.Direction.FORWARD);
        serIn4.setPosition(0);
        serIn5.setPosition(0);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        dontre = false;
        desYaw = false;
        checkYaw = false;
        desX = false;
        checkYaw2 = false;
        desZ = false;
        parkNeeded = true;
        step = 1;
        telemetry.addLine("Autonomous_RED_BACK Initialized...");
        telemetry.update();
        telemetry.addLine("Everything Works");
        telemetry.addLine("Ready to Start Autonomous");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {


                if (!dontre) {
                    if (step2 == 1) {
                        moveForward(0.5);
                        TimeUnit.MILLISECONDS.sleep(95);
                        Idle();
                        TimeUnit.MILLISECONDS.sleep(100);
                        step2++;
                    }
//                if (!exe2 && !LEFT && !CENTER){
//                    strafeRight(0.5);
//                    TimeUnit.MILLISECONDS.sleep(100);
//                    Idle();
//                    exe2 = true;
//                }
                    // Push telemetry to the Driver Station.
                    telemetryTfod();
                    telemetry.update();
                    //TODO: Insert conditions
                    if (Seen && !placed) {
                        if (CENTER) {
                            telemetry.addLine("Center " + CENTER);
                            telemetry.update();
                            moveForward(0.5);
                            TimeUnit.MILLISECONDS.sleep(1100);
                            Idle();
                            step++;
                            placed = true;
                            //now move back to X
                            moveBackward(0.5);
                            TimeUnit.MILLISECONDS.sleep(1100);
                            Idle();
                            turnRight(0.5);
                            TimeUnit.MILLISECONDS.sleep(500);
                            Idle();
                        } else if (RIGHT) {
                            telemetry.addLine("Right " + RIGHT);
                            telemetry.update();
                            TimeUnit.MILLISECONDS.sleep(550);
                            turnRight(0.5);
                            TimeUnit.MILLISECONDS.sleep(250);
                            Idle();
                            moveForward(0.5);
                            TimeUnit.MILLISECONDS.sleep(850);
                            step++;
                            placed = true;
                            //move back to x
                            moveBackward(0.5);
                            TimeUnit.MILLISECONDS.sleep(850);
                            Idle();
                            turnRight(0.5);
                            TimeUnit.MILLISECONDS.sleep(250);
                            Idle();
                        } else if (LEFT) {
                            telemetry.addLine("Left " + LEFT);
                            telemetry.update();
                            moveForward(0.5);
                            TimeUnit.MILLISECONDS.sleep(250);
                            Idle();
                            TimeUnit.MILLISECONDS.sleep(550);
                            turnLeft(0.5);
                            TimeUnit.MILLISECONDS.sleep(450);
                            Idle();
                            moveForward(0.5);
                            TimeUnit.MILLISECONDS.sleep(240);
                            step++;
                            placed = true;
                            //move back to x
                            moveBackward(0.5);
                            TimeUnit.MILLISECONDS.sleep(240);
                            Idle();
                            turnRight(0.5);
                            TimeUnit.MILLISECONDS.sleep(750);
                        }
                    } else {
                        Idle();
                    }
                }
                if (step == 1) {
                    try{
	    /*
	    1. Move forward to the second bar
	   2. Move sideways through
	   the bar to the backdrop for 5 points
	   */

                        moveForward(0.5);
                        TimeUnit.MILLISECONDS.sleep(200);
                        Idle();
                        telemetry.addLine("Autonomous Setup Complete! :)");
                        telemetry.update();
                    } catch(InterruptedException e){
//                    TODO: handle exception
                    }
                    step++;
                }
                telemetry.addLine("Moving to the April Tag...");
                telemetry.update();
                ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
                //Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
//                .setModelFileName(TFOD_MODEL_FILE)
                .setModelAssetName(TFOD_MODEL_ASSET)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(800, 448));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;
            Seen = true;
            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            if (x <= 300) {
                telemetry.addData("Left Spike Mark", x);
                LEFT = true;
                ID_TAG_OF_INTEREST = 4;
            } else if (x >= 485) {
                telemetry.addData("Right Spike Mark", x);
                RIGHT = true;
                ID_TAG_OF_INTEREST = 6;
            } else if (x > 300 && x < 485) {
                telemetry.addData("Center Spike Mark", x);
                CENTER = true;
                ID_TAG_OF_INTEREST = 5;
            } else
                telemetry.addData("Could not detect the Spike Mark", x);

            Seen = true;
        }   // end for() loop

    }
    void tagToTelemetry(AprilTagDetection detection) {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }
//        telemetryTfod();

}   // end class
@Disabled
abstract class Backdrop extends LinearOpMode{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int  ID_TAG_OF_INTEREST = 0; // Tag ID 18 from the 36h11 family
    AprilTagDetection tagOfInterest = null;
    boolean dontre;
    boolean desYaw;
    boolean desX;
    boolean checkYaw;
    boolean desZ;
    boolean checkYaw2;

    boolean parkNeeded;
    boolean placedPixel = false;
    int step;
    private DcMotor fr, fl, bl, br;
    private Servo serIn4, serIn5;
    private CRServo serIn1, serIn2;

    public void moveForward(double power) {
        fl.setPower(-power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(-power);
    }
    public void moveBackward(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }
    public void turnLeft(double power) {
        fl.setPower(power);
        fr.setPower(-power);
        bl.setPower(power);
        br.setPower(-power);
    }
    public void turnRight(double power) {
        fl.setPower(-power);
        fr.setPower(power);
        bl.setPower(-power);
        br.setPower(power);
    }
    public void strafeRight(double power) {
        fl.setPower(power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(power);
    }
    public void strafeLeft(double power) {
        fl.setPower(-power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(-power);
    }
    public void Idle() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing...");
        telemetry.update();
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        serIn4 = hardwareMap.get(Servo.class, "serIn4");
        serIn5 = hardwareMap.get(Servo.class, "serIn5");
        serIn1 = hardwareMap.get(CRServo.class, "serIn1");
        serIn2 = hardwareMap.get(CRServo.class, "serIn2");

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        serIn1.setDirection(CRServo.Direction.REVERSE);
        serIn2.setDirection(CRServo.Direction.REVERSE);
        serIn5.setDirection(Servo.Direction.REVERSE);
        serIn4.setDirection(Servo.Direction.FORWARD);
        serIn4.setPosition(0);
        serIn5.setPosition(0);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        dontre = false;
        desYaw = false;
        checkYaw = false;
        desX = false;
        checkYaw2 = false;
        desZ = false;
        parkNeeded = true;
        step = 1;
        telemetry.addLine("Autonomous_RED_BACK Initialized...");
        telemetry.update();
        telemetry.addLine("Everything Works");
        telemetry.addLine("Ready to Start Autonomous");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (step == 1) {
                try{
	    /*
	    1. Move forward to the second bar
	   2. Move sideways through
	   the bar to the backdrop for 5 points
	   */
                    moveForward(0.5);
                    TimeUnit.MILLISECONDS.sleep(740);
                    Idle();
                    TimeUnit.MILLISECONDS.sleep(700);
                    telemetry.addLine("Turned right");
                    telemetry.update();
                    turnRight(0.5);
                    TimeUnit.MILLISECONDS.sleep(1050);
                    moveForward(0.5);
                    TimeUnit.MILLISECONDS.sleep(2750);
                    Idle();
                    telemetry.addLine("Autonomous Setup Complete! :)");
                    telemetry.update();
                } catch(InterruptedException e){
//                    TODO: handle exception
                }
                step++;
            }
            telemetry.addLine("Moving to the April Tag...");
            telemetry.update();
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() != 0 && opModeIsActive())
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == ID_TAG_OF_INTEREST)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    double deadzone = 0.1;

                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                    Orientation rot = Orientation.getOrientation(tagOfInterest.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
                    do {
                        if (step == 2 &&!desYaw && opModeIsActive()) {
                            telemetry.addData("step: ", step);
                            telemetry.update();
                            if (rot.firstAngle < -3) {
                                turnLeft(0.2);
                            } else if (rot.firstAngle > 3) {
                                turnRight(0.2);
                            } else {
                                Idle();
                                desYaw = true;
                                telemetry.addData("desYaw: ", desYaw);
                                telemetry.update();
                                step++;
                            }
                        }
                        if (step == 3 && !desX && opModeIsActive()) {
                            telemetry.addData("step: ", step);
                            telemetry.update();
                            if(tagOfInterest.pose.x < -deadzone) {
                                strafeLeft(0.4);
                            }
                            else if(tagOfInterest.pose.x > deadzone) {
                                // do something else
                                strafeRight(0.4);
                            } else {
                                Idle();
                                desX = true;
                                telemetry.addData("desX: ", desX);
                                telemetry.update();
                                step++;
                            }
                        }
                        if (step == 4 && !checkYaw && opModeIsActive()) {
                            telemetry.addData("step: ", step);
                            telemetry.update();
                            if (rot.firstAngle < -5) {
                                turnLeft(0.25);
                            } else if (rot.firstAngle > 5) {
                                turnRight(0.25);
                            } else {
                                Idle();
                                checkYaw = true;
                                telemetry.addData("checkYaw: ", checkYaw);
                                telemetry.update();
                                step++;
                            }
                        }
                        if (step == 5 && !desZ && opModeIsActive()) {
                            telemetry.addData("step: ", step);
                            telemetry.update();
                            if (tagOfInterest.pose.z > 0.90) {
                                moveForward(0.20 );

                            } else {
                                Idle();
                                desZ = true;
                                telemetry.addData("desZ: ", desZ);
                                telemetry.update();
                                step++;
                            }
                        }
                        if (step == 6 && !checkYaw2 && opModeIsActive()) {
                            telemetry.addData("step: ", step);
                            telemetry.update();
                            if (rot.firstAngle < -2) {
                                turnLeft(0.1);
                            } else if (rot.firstAngle > 2) {
                                turnRight(0.1);
                            } else {
                                Idle();
                                checkYaw2 = true;
                                telemetry.addData("checkYaw2: ", checkYaw2);
                                telemetry.update();
                                step++;
                            }
                        }
                        if (desX && desZ) {
                            telemetry.addLine("Reached the april tag.");
                            telemetry.addLine("Placing the pixel");
                            telemetry.update();
                            serIn4.setPosition(0.75);
                            serIn5.setPosition(0.75);
                            TimeUnit.SECONDS.sleep(1);
                            placedPixel = true;
                        }
                        if (placedPixel) {
                            serIn1.setPower(0.5);
                            serIn2.setPower(-0.5);
                            TimeUnit.MILLISECONDS.sleep(500);
                            serIn1.setPower(0);
                            serIn2.setPower(0);
                            TimeUnit.SECONDS.sleep(1);
                            serIn4.setPosition(0);
                            serIn5.setPosition(0);
                            step++;
                            moveBackward(0.3);
                            TimeUnit.MILLISECONDS.sleep(300);
                            Idle();
                            TimeUnit.MILLISECONDS.sleep(300);
                            turnRight(0.5);
                            TimeUnit.MILLISECONDS.sleep(845);
                            Idle();
                            moveForward(0.5);
                            /*Later, this distance needs to be altered
                            because the distance between the april tags
                            makes the distance to park longer or shorter.
                             */
                            TimeUnit.MILLISECONDS.sleep(820);
                            Idle();
                            turnLeft(0.5);
                            TimeUnit.MILLISECONDS.sleep(700);
                            Idle();
                            /*
                            moveForward(0.5);
                            TimeUnit.MILLISECONDS.sleep(500);
                            */
                            Idle();
                            telemetry.addLine("-------------------------------");
                            telemetry.addLine("Autonomous complete.");
                            telemetry.addLine("-------------------------------");
                            telemetry.update();
                        }

                    } while (step == 9 && desX && checkYaw && desZ && checkYaw2 && opModeIsActive() && parkNeeded);
                }
                else
                {
                    Idle();
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

                telemetry.update();
                sleep(20);
            }
            else
            {
                Idle();
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\n But we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }
}