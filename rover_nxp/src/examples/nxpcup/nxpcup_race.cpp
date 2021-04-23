

#include "nxpcup_race.h"

#include <stdio.h>
#include <string.h>
#include <math.h>


int findVector(Pixy2 &pixy, Vector* first, Vector* second);
void findAverageVector(Vector* result, Vector* first, Vector* second);
double lengthVector(Vector* vector);
double calculateAngle(Vector* vector);
roverControl raceTrack(Pixy2 &pixy);


double max_servo_angle = 45.0;
double inaccuaracy_average_vector = 10.0;
double ref_angle = 75.0;


roverControl raceTrack(Pixy2 &pixy)
{
	roverControl control{};
	/* insert you algorithm here */
	Vector first;
	Vector second;
	control.speed = 0.1f;
	int find = findVector(pixy, &first, &second);

	if (find == 1){
		double angle = (ref_angle - calculateAngle(&first)) * (first.m_x0 > first.m_x1 ? -1 : 1);
		control.steer = fmin(1.0, fmax(-1.0, (angle / max_servo_angle)));
	} else if (find == 2) {
		Vector result;
		findAverageVector(&result, &first, &second);
		double angle = (90.0 - calculateAngle(&result)) * (result.m_x0 > result.m_x1 ? -1 : 1);
		if (abs(angle) > inaccuaracy_average_vector) {
			control.steer = fmin(1.0, fmax(-1.0, angle / max_servo_angle));
		}
	}
	return control;
}

int findVector(Pixy2 &pixy, Vector* first, Vector* second) {
	int max_length_1 = 0;
	int max_length_2 = 0;
	int find = false;
	for (int i = 0; i < pixy.line.numVectors; i++) {
		Vector vector = pixy.line.vectors[i];
		int sqare_length = lengthVector(&vector);
		if (max_length_1 < sqare_length) {
			if (max_length_1 != 0) {
				*second = *first;
				max_length_2 = max_length_1;
				if (find < 2) {
					find = 2;
				}
			}
			*first = vector;
			max_length_1 = sqare_length;
			find = true;
			if (find < 1) {
				find = 1;
			}
		} else if (max_length_2 < sqare_length) {
			*second = vector;
			max_length_2 = sqare_length;
			if (find < 2) {
				find = 2;
			}
		}
	}
	return find;
}

void findAverageVector(Vector* result, Vector* first, Vector* second) {
	result->m_x0 = (first->m_x0 + second->m_x0) / 2;
	result->m_y0 = (first->m_y0 + second->m_y0) / 2;
	result->m_x1 = (first->m_x1 + second->m_x1) / 2;
	result->m_y1 = (first->m_y1 + second->m_y1) / 2;
}

double lengthVector(Vector* vector) {
	return sqrt(pow(vector->m_x1 - vector->m_x0, 2) + pow(vector->m_y1 - vector->m_y0, 2));
}

double calculateAngle(Vector* vector) {
	return abs(asin(((double)vector->m_y1 - vector->m_y0) / lengthVector(vector)) * (180.0 / 3.14159));
}
