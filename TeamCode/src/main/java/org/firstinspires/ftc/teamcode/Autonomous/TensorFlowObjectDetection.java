package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "TensorFlow Object Detection", group = "Concept")
public class TensorFlowObjectDetection extends LinearOpMode {

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
    boolean LEFT;
    boolean RIGHT;
    boolean CENTER;
    boolean Seen;

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
        int step = 1;
        boolean exe = false;
        boolean exe2 = false;
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryTfod();
                if (step == 1) {
                    moveForward(0.5);
                    TimeUnit.MILLISECONDS.sleep(120);
                    Idle();
                    TimeUnit.MILLISECONDS.sleep(100);
                    step++;
                }
                if (step == 2 && !LEFT && !RIGHT) {
                    moveForward(0.5);
                    TimeUnit.MILLISECONDS.sleep(100);
                    Idle();
                    step++;
                }
//                if (!exe2 && !LEFT && !CENTER){
//                    strafeRight(0.5);
//                    TimeUnit.MILLISECONDS.sleep(100);
//                    Idle();
//                    exe2 = true;
//                }
                // Push telemetry to the Driver Station.
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
                    }
                } else {
                    Idle();
                }
                if (placed && !exe){
                    if (CENTER && !exe) {
                        moveBackward(0.5);
                        TimeUnit.MILLISECONDS.sleep(300);
                        Idle();
                        exe = true;
                    }
                    if (RIGHT && !exe) {
                        moveBackward(0.5);
                        TimeUnit.MILLISECONDS.sleep(500);
                        Idle();
                        turnRight(0.5);
                        TimeUnit.MILLISECONDS.sleep(730);
                        Idle();
                        strafeRight(0.5);
                        TimeUnit.MILLISECONDS.sleep(200);
                        Idle();
                        step++;
                        exe = true;
                    }
                    if (LEFT && !exe){
                        moveBackward(0.5);
                        TimeUnit.MILLISECONDS.sleep(500);
                        Idle();
                        turnRight(0.5);
                        TimeUnit.MILLISECONDS.sleep(1200);
                        Idle();
                        strafeLeft(0.5);
                        TimeUnit.MILLISECONDS.sleep(200);
                        Idle();
                        step++;
                        exe = true;
                    }
                }
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
            } else if (x >= 485) {
                telemetry.addData("Right Spike Mark", x);
                RIGHT = true;
            } else if (x > 300 && x < 485) {
                telemetry.addData("Center Spike Mark", x);
                CENTER = true;
            } else
                telemetry.addData("Could not detect the Spike Mark", x);

            Seen = true;
        }   // end for() loop

    }   // end method telemetryTfod()

}   // end class
