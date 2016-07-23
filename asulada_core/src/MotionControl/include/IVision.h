#ifndef __IVISION_H__
#define __IVISION_H__

class FaceInfo;

class IVision {
public:
	virtual onFaceDetected(const std::vector<FaceInfo*>& faceInfo) = 0;
};

#endif /* __IVISION_H__ */


