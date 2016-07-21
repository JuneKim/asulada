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

	virtual void onUpdated(float timestamp, int id, int goal,
 			int position, int error, int speed, float load,
 			float voltage, int temperature, bool moving);

	virtual void onFaceDetected(std::vector<FaceInfo>& faceInfo);

private:
	int _setGoal(int id, int goal);
	int _setSpeed(int id, int speed);

	MotorInfo curMotor_;
	bool isStarted_;
	Motion* motor_;
	Camera* camera_;
};

#endif /* __MOTION_CONTROL_H__ */
