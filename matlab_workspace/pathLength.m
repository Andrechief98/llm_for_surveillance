function L = pathLength(poses)
% Length of a nav_msgs/Path: sum of distances between consecutive poses.
% poses: struct array, one element per point of the path.
    L = 0;
    for k = 2:numel(poses)
        dx = poses(k).Pose.Position.X - poses(k-1).Pose.Position.X;
        dy = poses(k).Pose.Position.Y - poses(k-1).Pose.Position.Y;
        L = L + sqrt(dx^2 + dy^2);
    end
end
