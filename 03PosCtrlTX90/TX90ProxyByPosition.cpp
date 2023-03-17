#include "TX90ProxyByPosition.h"
#include <iostream>
#include <tuple>

typedef struct _Tx90msg
{
	float		speed[6];
	float		pos[6];
	int			stop;
	int			id;
}Tx90msg;




TX90ProxyByPosition::~TX90ProxyByPosition()
{
	Tx90msg sValues;

	for (int i = 0; i < 6; i++)
	{
		sValues.pos[i] = getPosition(i);
		sValues.speed[i] = getSpeed(i);
	}
	sValues.id = nIteration;
	sValues.stop = 1;

	
	port.send((char*)&sValues, sizeof(sValues));

	port.close();
}

void TX90ProxyByPosition::refresh()
{
	Tx90msg rValues;

	if (!port.recv((char*)&rValues, sizeof(rValues)))
		std::cout << "error al recibir del puerto" << std::endl;
	std::vector<float> p(std::begin(rValues.pos), std::end(rValues.pos));
	positions = p;
	std::vector<float> v(std::begin(rValues.speed), std::end(rValues.speed));
	speeds = v;
}

void TX90ProxyByPosition::sendCommand(std::vector<float> speed, std::vector<std::tuple <float, float>> robotCommand)
{
	Tx90msg sValues;

	std::vector<float> PIDOUT(6); 
	std::vector<float> errorVel(6);

	for (int i = 0; i < 6; i++)
	{
		PIDOUT[i] = std::get<0>(robotCommand[i]);
		errorVel[i] = std::get<1>(robotCommand[i]);

		commandedPositions[i] += PIDOUT[i]*0.004;
		commandedSpeeds[i] = speed[i] + errorVel[i];

		sValues.pos[i] = commandedPositions[i];
		sValues.speed[i] = commandedSpeeds[i];
	}
	sValues.stop = 0;
	sValues.id = nIteration;
	nIteration++;

	port.send((char*)&sValues, sizeof(sValues));

}