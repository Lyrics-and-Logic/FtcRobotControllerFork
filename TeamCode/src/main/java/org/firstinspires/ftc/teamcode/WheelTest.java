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




@Autonomous(name = "WheelTest")
public class WheelTest extends LinearOpMode {

    // Declare OpMode members.
    final private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LF;
    private DcMotor RF;
    private DcMotor LR;
    private DcMotor RR;

    public WheelTest() {
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
    static final double     DRIVE_SPEED             = 0.45;
    static final double     TURN_SPEED              = 0.3;
    static final int        NINETY_DEGREES       = 35;
    static final int        PRINT_MESSAGE_DELAY     = 3500; // number of milliseconds to pause after printing a message

    private int degreesToInches(double degrees){
        int magnitude = degrees < 0 ? -1 : 1;
        double factor = Math.abs((int) degrees) / 90.0;
        int inches = (int) Math.ceil(factor * NINETY_DEGREES);
        return magnitude * inches;
    }

    private void currentPosition(DcMotor leftWheel, DcMotor rightWheel) {
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Position ", "%7d :%7d",
                leftWheel.getCurrentPosition(),
                rightWheel.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        LF = hardwareMap.get(DcMotor.class, "LF");
        LR = hardwareMap.get(DcMotor.class, "LR");
        RF = hardwareMap.get(DcMotor.class, "RF");
        RR = hardwareMap.get(DcMotor.class, "RR");
        resetEncoder();
        currentPosition(LR, LF);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Note: Reverse movement is obtained by setting a negative distance (not speed)
//        encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
//        encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        int distance = 30;
        for (int i = 0; i < 10; i++) {
            if (i % 4 == 0)
               moveForward(distance);
            else if (i % 4 == 1)
                moveBackward(distance);
            else if (i % 4 == 2)
                turnRight(90.0);
            else if (i % 4 == 3)
                turnLeft(90.0);
//            sleep(5000);
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.

    }

    private void moveForward(int distance){
        logMessage("move forward");
        moveTwoWheels(RR, RF, DRIVE_SPEED, DRIVE_SPEED, -distance, distance);
    }

    private void moveBackward(int distance){
        logMessage("move backward");
        moveTwoWheels(LR, LF, DRIVE_SPEED, DRIVE_SPEED, distance, -distance);
    }

    private void turnLeft(double degrees){
        int distance = degreesToInches(degrees);
        logMessage("turning left");
        moveTwoWheels(LF, RF, TURN_SPEED, TURN_SPEED, -distance, -distance);
    }

    private void turnRight(double degrees){
        int distance = degreesToInches(degrees);
        logMessage("turning right");
        moveTwoWheels(LR, RR, TURN_SPEED, TURN_SPEED, distance, distance);
    }

    private void logMessage(String msg){
        printMessage(msg, PRINT_MESSAGE_DELAY);
    }
    private void printMessage(String msg, int n){
        telemetry.addData("", "%s", msg);
        telemetry.update();
        sleep(n);

    }

    private void moveTwoWheels(DcMotor leftWheel, DcMotor rightWheel, double leftSpeed, double rightSpeed,
                               int leftInches,
                               int rightInches){
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int rightTarget = rightWheel.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            int leftTarget = leftWheel.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            rightWheel.setTargetPosition(rightTarget);
            leftWheel.setTargetPosition(leftTarget);

            // Turn On RUN_TO_POSITION
            rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            rightWheel.setPower(Math.abs(rightSpeed));
            leftWheel.setPower(Math.abs(leftSpeed));

            while (opModeIsActive() &&
                    (runtime.seconds() < 5.0) &&
                    reachedTarget(rightWheel, leftWheel)){

                // Display it for the driver.
                //telemetry.addData("Position",  " %7d: %7d", leftTarget, rightTarget);
                currentPosition(leftWheel, rightWheel);
                telemetry.update();
            }

            // Stop all motion;
            stopMotion();

            // Turn off RUN_TO_POSITION
            turnOff();
            sleep(250);   // optional pause after each move.
        }
    }

    private void moveOneWheel(DcMotor wheel, double leftSpeed, int rightInches){
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int target = wheel.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            wheel.setTargetPosition(target);

            // Turn On RUN_TO_POSITION
            wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            wheel.setPower(Math.abs(leftSpeed));

            while (opModeIsActive() &&
                    (runtime.seconds() < 5.0) &&
                    wheel.isBusy()){

                // Display it for the driver.
                //telemetry.addData("Position",  " %7d: %7d", leftTarget, rightTarget);
                currentPosition(wheel, wheel);
                telemetry.update();
            }

            // Stop all motion;
            stopMotion();

            // Turn off RUN_TO_POSITION
            turnOff();
            sleep(250);   // optional pause after each move.
        }
    }
    private void turnOff() {
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private boolean reachedTarget(DcMotor frontWheel, DcMotor rearWheel){
        return (frontWheel.isBusy() && rearWheel.isBusy());
    }

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
}

