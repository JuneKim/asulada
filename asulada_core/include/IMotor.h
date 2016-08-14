#ifndef __IMOTOR_H__
#define __IMOTOR_H__

namespace asulada {

class IMotor {
public:
	virtual void onCurrentMotorStatus() = 0;
};

} // namespace asulada

#endif /* __IMOTOR_H__ */
