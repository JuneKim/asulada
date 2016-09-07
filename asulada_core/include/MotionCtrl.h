#ifndef __MOTION_CTRL_H__
#define __MOTION_CTRL_H__

#include <ros/ros.h>
#include "IVision.h"
#include "IMotor.h"

namespace asulada {

typedef enum {
	FACE_AREA_1 = 0,
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


private:
	FaceArea_e  _getFaceArea(double x, double y);
	ros::NodeHandle *pnh_;
	double curX_, curY_, curDimension_;
	FaceArea_e curFaceArea_;
	double curMotorPos_;
	Motor *motor_;
	Vision *vision_;
};

} // namespace asulda

#endif /* __MOTION_CTRL_H__ */
