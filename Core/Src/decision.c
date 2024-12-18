/*
 * decision.c
 *
 *  Created on: Dec 18, 2024
 *      Author: user
 */
#include "stdint.h"
#include "math.h"

uint8_t decision=0;
//Forward 		- 0
//Strong Left 	- 1
//Left 			- 2
//Backward 		- 3
//Right 		- 4
//Strong Right 	- 5

uint8_t decisionLogic(){
	LL_mDelay(1000);

	decision=decision+1;
	if(decision>5){
		decision=0;
	}

	return decision;
}


uint8_t decisionLogicWIP(float distances[3][4], uint8_t num_sensors) {
    float x[12] = {0};
    float y[12] = {0};

    // Sensor configuration
    float sensor_base_angle = 25.0 / 4.0;
    float sensor_tilt[] = {0.0, -15.0, 15.0};
    float dir[] = {1.0, 1.0, 1.0, 1.0};
    float sensor_pos_offset[][2] = {{0.0, 0.0}, {-0.05, 0.0}, {0.05, 0.0}};

    float sensor_mid_angles[] = {
        -2 * sensor_base_angle,
        -sensor_base_angle,
        sensor_base_angle,
        2 * sensor_base_angle
    };

    float sensor_left_angles[] = {
        (-2 * sensor_base_angle + sensor_tilt[1]),
        (-sensor_base_angle + sensor_tilt[1]),
        (sensor_base_angle + sensor_tilt[1]),
        (2 * sensor_base_angle + sensor_tilt[1])
    };

    float sensor_right_angles[] = {
        (-2 * sensor_base_angle + sensor_tilt[2]),
        (-sensor_base_angle + sensor_tilt[2]),
        (sensor_base_angle + sensor_tilt[2]),
        (2 * sensor_base_angle + sensor_tilt[2])
    };

    // Calculate coordinates
    for (uint8_t i = 0; i < num_sensors && i < 4; ++i) {
        float angle = sensor_mid_angles[i] * M_PI / 180.0;
        y[i] = cos(angle) * distances[0][i];
        x[i] = dir[i] * sin(angle) * distances[0][i] + sensor_pos_offset[0][0];
    }
    for (uint8_t i = 4; i < num_sensors && i < 8; ++i) {
        float angle = sensor_left_angles[i - 4] * M_PI / 180.0;
        y[i] = cos(angle) * distances[1][i - 4];
        x[i] = dir[i - 4] * sin(angle) * distances[1][i - 4] + sensor_pos_offset[1][0];
    }
    for (uint8_t i = 8; i < num_sensors && i < 12; ++i) {
        float angle = sensor_right_angles[i - 8] * M_PI / 180.0;
        y[i] = cos(angle) * distances[2][i - 8];
        x[i] = dir[i - 8] * sin(angle) * distances[2][i - 8] + sensor_pos_offset[2][0];
    }

    // Decision values
    float low_distance = 2.0;
    float command_values[5] = {0, 0, 0, 0, 0};

    for (uint8_t i = 0; i < num_sensors; ++i) {
        float inputF = x[i];
        float foutput = 0.0;

        // Strong Left
        float leftDistance = 2.5;
        float functionAngleRising = 1.0;
        float functionCut = 0.75;
        foutput = ((sqrt(pow(inputF + leftDistance, 2)) / functionAngleRising - inputF / functionAngleRising - leftDistance / functionAngleRising) -
                  (sqrt(pow(inputF + leftDistance + functionCut, 2)) / functionAngleRising - inputF / functionAngleRising - (leftDistance + functionCut) / functionAngleRising));
        command_values[0] += y[i] * foutput;

        // Left
        inputF = x[i];
        foutput = 0.0;
        functionAngleRising = 2.0;
        float functionLevel = 1.0;
        float functionStart = 1.0;
        foutput = sqrt(pow(inputF + functionStart, 2)) / functionAngleRising - (inputF - functionStart) / functionAngleRising - functionLevel -
                  (sqrt(pow(inputF + 2, 2)) / functionAngleRising - (inputF - 2) / functionAngleRising - 2);
        command_values[1] += y[i] * foutput;

        // Middle - AKA backwards
        foutput = (-sqrt(pow(inputF - 0.5, 2)) / 2 - inputF / 2 + 2) + (-sqrt(pow(inputF + 0.5, 2)) / 2 + inputF / 2 + 0.5);
        if (foutput < 0) foutput = 0;
        if ((y[i] - low_distance) < 0) {
            command_values[2] += (y[i] - low_distance) * foutput;
        }

        // Right
        inputF = x[i];
        foutput = 0.0;
        functionAngleRising = 2.0;
        functionLevel = 1.0;
        functionStart = 1.0;
        foutput = sqrt(pow(-inputF + functionStart, 2)) / functionAngleRising - (-inputF - functionStart) / functionAngleRising - functionLevel -
                  (sqrt(pow(-inputF + 2, 2)) / functionAngleRising - (-inputF - 2) / functionAngleRising - 2);
        command_values[3] += y[i] * foutput;

        // Strong Right
        command_values[4] += y[i] * foutput;
    }

    // Determine decision
    float decisionValue = 0;
    int decision = 0;
    for (uint8_t i = 0; i < 5; ++i) {
        if (command_values[i] > decisionValue) {
            decisionValue = command_values[i];
            decision = i + 1;
        }
    }

    return decision;
}



