/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "WheelTest")
public class WheelTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LF;
    private DcMotor RF;
    private DcMotor LR;
    private DcMotor RR;

    public WheelTest() {
//        LF = hardwareMap.get(DcMotor.class, "RR");
//        LR = hardwareMap.get(DcMotor.class, "RF");
//        RF = hardwareMap.get(DcMotor.class, "LR");
//        RR = hardwareMap.get(DcMotor.class, "LF");
    }

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // Our motor's web site is https://www.andymark.com/products/neverest-orbital-20-gearmotor
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.6; // Ticks per revolution from
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    private void stopMotion() {
        LR.setPower(0);
        RR.setPower(0);
        LF.setPower(0);
        RF.setPower(0);
    }

    private void resetEncoder() {
        LR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void currentPosition(int target) {
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("%s", "LF=" + LF.getCurrentPosition() +
                " RF=" + RF.getCurrentPosition() +
                " LR=" + LR.getCurrentPosition() +
                " RR=" + RR.getCurrentPosition() +
                " target=" + target);
        telemetry.update();
    }

    private void currentPosition(DcMotor wheel, int delta) {
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("%s", "Motor=" + wheel.getCurrentPosition() +
                " delta=" + delta);
        telemetry.update();
    }


    @Override
    public void runOpMode() {
        LF = hardwareMap.get(DcMotor.class, "LF");
        LR = hardwareMap.get(DcMotor.class, "LR");
        RF = hardwareMap.get(DcMotor.class, "RF");
        RR = hardwareMap.get(DcMotor.class, "RR");
        resetEncoder();
        currentPosition(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Note: Reverse movement is obtained by setting a negative distance (not speed)
//        encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
//        encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        for (int i = 0; i < 10; i++) {
            moveOneWheel(LF, DRIVE_SPEED, 120);
            moveOneWheel(RF, DRIVE_SPEED, 120);
            moveOneWheel(LR, DRIVE_SPEED, 120);
            moveOneWheel(RR, DRIVE_SPEED, 120);
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.

    }

    private void moveOneWheel(DcMotor wheel, double speed, double inches){
        if (opModeIsActive()) {

            //resetEncoder();
            wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Determine new target position, and pass to motor controller
            int distance = (int)(inches * COUNTS_PER_INCH);
            int target = wheel.getCurrentPosition() + distance;
            wheel.setTargetPosition(target);

            // Turn On RUN_TO_POSITION
            wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            wheel.setPower(Math.abs(speed));


            int delta = Math.abs(wheel.getCurrentPosition() - distance);
            currentPosition(delta);
            //currentPosition(wheel, delta);
            telemetry.addData("Running to",  " %7d", target);
            while (opModeIsActive() &&
                    //(runtime.seconds() < 5.0) &&
                    wheel.isBusy()) {

                // Display it for the driver.
                delta = Math.abs(wheel.getCurrentPosition() - distance);
                currentPosition(delta);
                //currentPosition(wheel, delta);
                telemetry.update();
            }

            // Stop all motion;
            stopMotion();

            // Turn off RUN_TO_POSITION
            turnOff();
            telemetry.addData("%s", "We have stopped");
            sleep(3000);   // optional pause after each move.
        }
    }

    private void turnOff() {
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private boolean isBusy(){
        return LF.isBusy() || RF.isBusy() || LR.isBusy() || RR.isBusy();
    }

}

