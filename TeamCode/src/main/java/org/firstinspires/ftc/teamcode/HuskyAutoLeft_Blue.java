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

import static org.firstinspires.ftc.teamcode.HuskyAutoBase.DeliveryLevelPipeline.DeliveryLevel;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Left Blue", group = "Auto", preselectTeleOp = "Husky TeleOpMode")
public class HuskyAutoLeft_Blue extends HuskyAutoBase {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        telemetry.addData("Mode", this.getClass().getSimpleName());
        telemetry.update();

        DeliveryLevel deliveryLevel = DeliveryLevel.LEVEL_0;    // default undefined level
        double forwardDistanceInches = 6;

        waitForStart();

        // get the delivery level with timeout of 5 seconds. if the delivery level isn't detected
        // within timeout, it defaults to level 1 (see switch statement below)
        while (opModeIsActive() && (runtime.seconds() < 5))
        {
            deliveryLevel = pipeline.getDeliveryLevel();
            if (deliveryLevel != DeliveryLevel.LEVEL_0) {
                break;
            }
            sleep(50);
        }

        telemetry.addData("Delivery", "Got delivery level " + deliveryLevel);
        telemetry.update();

        encoderDrive(AUTO_DRIVE_SPEED, 6, 1);

        switch (deliveryLevel) {
            case LEVEL_2:
                huskyBot.arm.setTargetPosition(HuskyBot.ARM_LEVEL_2);
                forwardDistanceInches = 6.5;
                break;
            case LEVEL_3:
                huskyBot.arm.setTargetPosition(HuskyBot.ARM_LEVEL_3);
                forwardDistanceInches = 7;
                break;
            case LEVEL_1:
            default:
                huskyBot.arm.setTargetPosition(HuskyBot.ARM_LEVEL_1);
                forwardDistanceInches = 7.5;
                break;
        }
        huskyBot.arm.setVelocity(300);

        encoderStrafe(AUTO_DRIVE_SPEED, -20, 2);
        encoderDrive(AUTO_DRIVE_SPEED, forwardDistanceInches, 1);

        // deliver cargo
        runtime.reset();
        huskyBot.intaker.setPower(-0.6);
        while (opModeIsActive() && (runtime.seconds() < 1)) {
        }
        huskyBot.intaker.setPower(0);

        // back up before lowering arm
        encoderDrive(AUTO_DRIVE_SPEED, -forwardDistanceInches, 1);

        huskyBot.arm.setTargetPosition(HuskyBot.ARM_LEVEL_0);
        huskyBot.arm.setVelocity(300);
        encoderStrafe(AUTO_DRIVE_SPEED, 20, 2);
        encoderDrive(AUTO_DRIVE_SPEED, -6, 1);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
