#ifndef __IMOTOR_H__
#define __IMOTOR_H__

class IMotor {
public:
	/**
	 * @brief updage motor's status
	 */
	virtual void onUpdated(float timestamp, int id, int goal,
		int position, int error, int speed, float load,
		float voltage, int temperature, bool moving) = 0;
};

#endif /* __IMOTOR_H__ */
