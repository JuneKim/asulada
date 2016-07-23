#ifndef __VISION_H__
#define __VISION_H__

class FaceInfo {
public:
	int centerX;
	int centerY;
	int width;
	int height;
};

typedef enum {
	VISION_DISALBED = 0,
	VISION_ENABLED,	/* no face */
	VISION_FACE_DETECTED,
	VISION_MULTI_FACE_DETECTED,
} VisionStatus_t;

class Vision {
public:
	typedef enum ConnectionStatus {
		NOT_INITIALIZED,
		NOT_SUBSCRIBED,
		SUBSCRIBED
	}
	static Vision* getInstance(NodeHandle& nh);
	Vision();
	~Vision();
	int start();
	void stop();
	const VisionStatus_t getStatus();
	void addListener(IVision* l);
	void removeListener(IVision* l);

private:
	void _notify(std::vector<Rect *>&&faceInfo);
	void _notifyStatus(VisionStatus_t status);

	static Vision* inst_;
	NodeHandle* nh_;
	image_transport::ImageTansrport* it_;
//	image_transport::Publisher img_pub_;
	image_transport::Subscriber img_sub_;
	ros::Publisher msg_pub_;

#if 0
	face_detection::FaceDetectionConfig config_;
	dynamic_reconfigure::Server<face_detection::FaceDetectionConfig> srv;
#endif
	bool debug_view_;
	ros::Time prev_stamp_;

	// nodelet
	

	cv::CascadeClassifier face_cascade_;
	cv::CascadeClassifier eyes_cascade_;
	/**
	 * @brief get current face. it can be supporting only one face.
	 * otherwise, the closest(largest) face would be selected.
	 */§
	VisionStatus_t state_;
	FaceInfo currInfo_;
	std::vector<IVision*> listeners_;
	
};

#endif /* __VISION_H__ */
