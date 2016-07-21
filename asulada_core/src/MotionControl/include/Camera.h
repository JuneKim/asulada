#ifndef __CAMERA_H__
#define __CAMERA_H__

class FaceInfo {
public:
	int centerX;
	int centerY;
	int width;
	int height;
};

typedef enum {
	CAMERA_DISALBED = 0,
	CAMERA_ENABLED,	/* no face */
	CAMERA_FACE_DETECTED,
	CAMERA_MULTI_FACE_DETECTED,
} CameraStatus_t;

class Camera {
public:
	static Camera* getInstance(NodeHandle& nh);
	Camera();
	~Camera();
	int start();
	void stop();
	const CameraStatus_t getStatus();
	void addListener(ICamera* l);
	void removeListener(ICamera* l);

private:
	void _notify();

	static Camera* inst_;
	NodeHandle& nh_;
	/**
	 * @brief get current face. it can be supporting only one face.
	 * otherwise, the closest(largest) face would be selected.
	 */
	CameraStatus_t state_;
	FaceInfo currInfo_;
	std::vector<ICamera*> listeners_;
	
};

#endif /* __CAMERA_H__ */
