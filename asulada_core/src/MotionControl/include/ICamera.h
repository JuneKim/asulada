#ifndef __ICAMERA_H__
#define __ICAMERA_H__

class FaceInfo;

class ICamera {
public:
	virtual onFaceDetected(const std::vector<FaceInfo*>& faceInfo) = 0;
};

#endif /* __ICAMERA_H__ */


