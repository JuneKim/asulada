#ifndef __MOTION_CONTROL_H__
#define __MOTION_CONTROL_H__

#include "IMotor.h"
#include "ICamera.h"

class Motor;
class Camera;

class MotionControl : public IMotor, ICamera {
public:
	MotionControl();
	~MotionControl();

	int start();
	void stop();
	/*
	 * get
	 */
	bool getValue(double& x, double& y, double& z);

	virtual void onPosUpdated(double x, double y, double z);
	virtual void onFaceMoved(double xdiff, double ydiff, double zdiff);
	
private:
	void setValue(double x, double y, double z);

	double x_, y_, z_;
	bool isStarted_;
	Motion* motor_;
	Camera* camera_;
};

#endif /* __MOTION_CONTROL_H__ */
