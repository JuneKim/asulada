#ifndef __IMOTOR_H__
#define __IMOTOR_H__

namespace Asulada
{
typedef enum {
	MOTOR_STATE_INACTIVE = 0,
	MOTOR_STATE_IDLE,
	MOTOR_STATE_MOVING,
} MotorState_t;

class IMotor {
public:
	virtual void onActive(MotorState_t state) = 0;

	virtual void onPosUpdated(double x, double y, double z) = 0;
};

}

#endif /* __IMOTOR_H__ */
