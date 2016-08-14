#ifndef __MOTION_CTRL_H__
#define __MOTION_CTRL_H__

#include "IVision.h"
#include "IMotor.h"

namespace asulada {

class Motor;
class Vision;

class MotionCtrl : public IMotor, public IVision {
public:
	MotionCtrl();
	~MotionCtrl();
	virtual void onFaceDetected(double x, double y, double z); //TODO:
	virtual void onCurrentMotorStatus(); // TODO
	int start();
	void stop();


private:
	int curX_, curY_, curZ_;
	Motor *motor_;
	Vision *vision_;
};

} // namespace asulda

#endif /* __MOTION_CTRL_H__ */
