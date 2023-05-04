# A msg serializing the OpenCV KeyPoint object
# in c++ it is cv::KeyPoint

float32 angle # angle between the keypoint direction and x axis
int32 class_id # object class (if the keypoints need to be clustered by an object they belong to)
int32 octave # octave (pyramid layer) from which the keypoint has been extracted

float32 x # x coordinate of the keypoint
float32 y # y coordinate of the keypoint

float32 response # the response by which the most strong keypoints have been selected. Can be used for the further sorting or subsampling
float32 size # diameter of the meaningful keypoint neighborhood
