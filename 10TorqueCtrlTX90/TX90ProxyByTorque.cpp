#include "TX90ProxyByTorque.h"
#include <iostream>
#include <tuple>
#include <math.h>

typedef struct _Tx90msg
{
	float		speed[6];
	float		pos[6];
	float		torque[6];
	int			stop;
	int			id;
}Tx90msg;

TX90ProxyByTorque::~TX90ProxyByTorque()
{
	Tx90msg sValues;

	for (int i = 0; i < 6; i++)
	{
		sValues.pos[i] = getPosition(i);
		sValues.speed[i] = getSpeed(i);
		sValues.torque[i] = getTorque(i);
	}
	sValues.id = nIteration;
	sValues.stop = 1;

	
	port.send((char*)&sValues, sizeof(sValues));

	port.close();
}

void TX90ProxyByTorque::refresh()
{
	Tx90msg rValues;

	if (!port.recv((char*)&rValues, sizeof(rValues)))
		std::cout << "error al recibir del puerto" << std::endl;
	std::vector<float> p(std::begin(rValues.pos), std::end(rValues.pos));
	positions = p;
	std::vector<float> v(std::begin(rValues.speed), std::end(rValues.speed));
	speeds = v;
	std::vector<float> t(std::begin(rValues.torque), std::end(rValues.torque));
	torques = t;
}

float TX90ProxyByTorque::sigmoid(float x) {
	float result;
	result = 1 / (1 + exp(-x));
	return result;
}

void TX90ProxyByTorque::sendTorqueCommand(std::vector<float> torqueValue)
{
	Tx90msg sValues;

	std::vector<float>  signValue = {0,0,0,0,0,0};
	std::vector<float>  FinalTorque = { 0,0,0,0,0,0 };
	
	for (int i = 0; i < 6; i++)
	{

		/*for (int j = 1; j < 6; j++)
		{

			torqueValue[j] = 0;
		}*/
		//signValue[i] = sigmoid(torqueValue[i]);
		signValue[i] = 1;
		FinalTorque[i] = signValue[i] * torqueValue[i];

		std::cout << FinalTorque[i] << std::endl;
		sValues.torque[i] = FinalTorque[i];
	}
	std::cout << std::endl;

	sValues.stop = 0;
	sValues.id = nIteration;
	nIteration++;

	port.send((char*)&sValues, sizeof(sValues));

}

