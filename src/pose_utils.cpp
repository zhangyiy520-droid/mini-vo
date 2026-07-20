#include "mini_vo/pose_utils.h"

namespace mini_vo{

PoseWC invertPose(const PoseCW& p){
    PoseWC out;
    out.Rwc = p.Rcw.t();
    out.twc = -out.Rwc * p.tcw;
    return out;
}

cv::Point3d cameraCenterWorld(const PoseCW& p){
    const PoseWC pwc = invertPose(p);
    return cv::Point3d(
	pwc.twc.at<double>(0),
	pwc.twc.at<double>(1),
	pwc.twc.at<double>(2)
    );
}

}
