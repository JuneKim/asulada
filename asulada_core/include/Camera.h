#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <memory>

namespace Asulada {

class Camera;
typedef std::shared_ptr<Camera> CameraPtr;
typedef std::weak_ptr<Camera> CameraWPtr;

class Camera {
public:
	static CameraWPtr getInstance();
	Camera();
	~Camera();
	int start();
	int stop();

	/**
	 * @brief listen face's pos(x, y, z);
	 */
	void addListener(ICameraPosWPtr l);
	void removeListener(ICameraPosWPtr l);
	/**
	 * @brief listen camera's low image
	 */ 
	void addListener(ICameraImgWPtr l);
	void removeListener(ICameraImgWPtr l);
	
private:
	void _notity();
	
	static CameraPtr inst_;
	bool started_;
};

} // namesapce Asulada

#endif /* __CAMERA_H__ */
