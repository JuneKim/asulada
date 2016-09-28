#ifndef __MOTION_CTRL_H__
#define __MOTION_CTRL_H__

#include <ros/ros.h>
#include "IVision.h"
#include "IMotor.h"

namespace asulada {

typedef enum {
	FACE_AREA_1 = 1,
	FACE_AREA_2,
	FACE_AREA_3,
	FACE_AREA_4,
	FACE_AREA_5,
} FaceArea_e;

class Motor;
class Vision;

class MotionCtrl : public IMotor, public IVision {
public:
	MotionCtrl(ros::NodeHandle *nh);
	~MotionCtrl();
	virtual void onFaceDetected(double x, double y, double dimension); //TODO:
	virtual void onCurrentMotorStatus(int pos); // TODO
	int start();
	void stop();
	void setMotorGoal(int goal);
	void setLed(int idx);

	static int INIT_ARC;
	static int gTarget_;

private:
	FaceArea_e  _getFaceArea(double x, double y);
	int _getCurrentMotorGoal(FaceArea_e area);
	void _setCurrentArea(FaceArea_e area);
	int _faceArea2Arc(FaceArea_e area);
	
	ros::NodeHandle *pnh_;
	ros::Publisher led_pub_;
	double curX_, curY_, curDimension_;
	FaceArea_e curFaceArea_;
	int curMotorPos_;
	Motor *motor_;
	Vision *vision_;
};

} // namespace asulda

#endif /* __MOTION_CTRL_H__ */
