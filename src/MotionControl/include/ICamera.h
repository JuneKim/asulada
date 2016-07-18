#ifndef __ICAMERA_H__
#define __ICAMERA_H__

class ICamera {
public:
	virtual onFaceMoved(double xdiff, double ydiff, double zdiff) = 0;
};

#endif /* __ICAMERA_H__ */


