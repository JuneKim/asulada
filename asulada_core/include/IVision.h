#ifndef __IVISION_H__
#define __IVISION_H__

namespace asulada {

class IVision {
public:
	virtual void onFaceDetected(double x, double y, double z) = 0;
};

} // namespace asulada

#endif /* __IVISION_H__ */
