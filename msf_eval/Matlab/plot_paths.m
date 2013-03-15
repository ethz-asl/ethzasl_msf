

range = 1:991;

hold off;

poseabs_aligned_x = poses_abs(range, 1) - (mean(poses_abs(range, 1) - poses_GT_abs(range, 1)));
poserel_aligned_x = poses_rel(range, 1) - (mean(poses_rel(range, 1) - poses_GT_rel(range, 1)));

poseabs_aligned_y = poses_abs(range, 2) - (mean(poses_abs(range, 2) - poses_GT_abs(range, 2)));
poserel_aligned_y = poses_rel(range, 2) - (mean(poses_rel(range, 2) - poses_GT_rel(range, 2)));

%plot(poseabs_aligned_x, poseabs_aligned_y, 'r');
plot(poserel_aligned_x, poserel_aligned_y, 'b'); hold on;
plot(poses_GT_rel(range, 1), poses_GT_rel(range, 2), 'g');
