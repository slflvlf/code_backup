% body coordinate convert to global coordinate
% 目标点从体坐标转向大地坐标系

function p = BtoG(p0, delta)

    R = rotate_matrix(p0);
    p = R * delta + p0;
end


function R= rotate_matrix(p)
    fai = p(3);
    R = [cos(fai), -sin(fai), 0;
        sin(fai), cos(fai), 0;
        0, 0, 1];
end
