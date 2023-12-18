package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.concurrent.TimeUnit;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous

public class Autonomous_RED_BACK extends LinearOpMode {
    private DcMotor fr, fl, br, bl; // Create the motors' variables
    public void runOpMode() {
        double dis = 0.5;
        fl = hardwareMap.get(DcMotor.class, "fl"); // Define all of the motors
        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");

        telemetry.addLine("Motors are connected for Autonomous mode."); // Telemetry output
        telemetry.addLine("Mode: Red Back");
        telemetry.addLine("Yay! Everything works!");
        telemetry.addLine("-----------------------------");
        telemetry.update(); // Print them all in telemetry
        waitForStart();

        if (opModeIsActive()){ // Actually start the code
            try{
	    /*
	    1. Move forward to the second bar
	   2. Move sideways through
	   the bar to the backdrop for 5 points
	   */

                fl.setPower(dis); // Motors are in reverse, so the value has to be negative to move forward
                fr.setPower(-dis);
                bl.setPower(dis);
                br.setPower(-dis);
                TimeUnit.MILLISECONDS.sleep(785);
                fl.setPower(0); // Motors are in reverse,                                                                       so the value has to be negative to move forward
                fr.setPower(0);
                bl.setPower(0);
                br.setPower(0);
                TimeUnit.MILLISECONDS.sleep(700);
//                TimeUnit.MILLISECONDS.sleep(500);
                telemetry.addLine("Turned right");
                telemetry.update();
                fl.setPower(dis); // Motors are in reverse, so the value has to be positive to move sideways
                fr.setPower(dis);
                bl.setPower(dis);
                br.setPower(dis);
                TimeUnit.MILLISECONDS.sleep(710);
                fl.setPower(dis); // Motors are in reverse, so the value has to be negative to move forward
                fr.setPower(-dis);
                bl.setPower(dis);
                br.setPower(-dis);
                TimeUnit.SECONDS.sleep(2);
            } catch(InterruptedException e){
                //TODO: handle exception
            }
        }
    }
}
