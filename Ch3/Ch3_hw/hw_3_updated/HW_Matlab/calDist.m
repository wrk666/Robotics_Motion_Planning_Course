function dist=calDist(startPose,goalPose)
dist=sqrt(sum((startPose-goalPose).^2));