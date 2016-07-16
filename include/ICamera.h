#ifndef __ICAMERA_H__
#define __ICAMERA_H__

namespace Asulada {

class ICamera {
public:
	/**
	 * @brief pos 
	 */
	virtual void onPosUpdated(const Pos[] pos) = 0;
	virtual void onImageUpdated() = 0;
}; // class ICamera

} // namespace Asulada

#endif /* __ICAMERA_H__ */
