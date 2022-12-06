function tug4_semi_motion_display(p0, p1, p2, p3, p4, fig_id)
    hull_x0 = [-75, 75, 75, -75, -75];
    hull_y0 = [50, 50, -50, -50, 50];
    hull_x1 = [-16, 12, 16, 12, -16, -16];
    hull_y1 = [6, 6, 0, -6, -6, 6];


    global delta10 delta20 delta30 delta40;
    global length_init;
    % global length10 length12;
    deg2rad = pi/180;
    delta10 = [375-200, 350, 0*deg2rad]';%排序按照象限
    delta20 = [-375+200, 350, 0*deg2rad]';
    delta30 = [-375+200, -350, 0*deg2rad]';
    delta40 = [375-200, -350, 0*deg2rad]';


    persistent fig hgtrans0 hgtrans1 hgtrans2 hgtrans3 hgtrans4;
    persistent line1 line2 line3 line4; 

    if isempty(fig)        
        fig = figure(fig_id);
        title('Motion of the ships');
        axis equal;
        hgtrans0 = initial_motion_fig(hull_x0, hull_y0, p0, 0, 'y');
        hgtrans1 = initial_motion_fig(hull_x1, hull_y1, p1, 1,  'm');
        hgtrans2 = initial_motion_fig(hull_x1, hull_y1, p2, 2,  'm');
        hgtrans3 = initial_motion_fig(hull_x1, hull_y1, p3, 3,  'm');
        hgtrans4 = initial_motion_fig(hull_x1, hull_y1, p4, 4,  'm');

        [line1, line2, line3, line4] = intial_moorline_fig(p0, p1, p2, p3, p4, hull_x0, hull_y0, hull_x1, hull_y1);
    end


%         update_motion_fig(hgtrans0, p0);
%         hold on 
%         plot(p0(1), p0(2), '.');
%         update_motion_fig(hgtrans1, p1);
%         update_motion_fig(hgtrans2, p2);
%         update_motion_fig(hgtrans3, p3);
%         update_motion_fig(hgtrans4, p4);
% 
%         update_moorline(line1, line2, line3, line4, p0, p1, p2, p3, p4, hull_x0, hull_y0, hull_x1, hull_y1);
%         pause(0.0001);
    


end



function hgtrans = initial_motion_fig(hull_x, hull_y, p, id, ship_color)
    hgtrans = hgtransform;
    fill(hull_x, hull_y, ship_color, 'parent',hgtrans);
    line(0,0,'marker','o','markeredgecolor','k','markerfacecolor','k','parent',hgtrans); 
%     text(p(1)+id_position(1), p(2)+id_position(2), num2str(id), 'parent', hgtrans);   
end


function [line1, line2, line3, line4] = intial_moorline_fig(p0, p1, p2, p3, p4, hull_x0, hull_y0, hull_x1, hull_y1)
    line1_start = local_to_global([hull_x0(2); hull_y0(2)], p0);    line2_start = local_to_global([hull_x0(1); hull_y0(1)], p0);
    line3_start = local_to_global([hull_x0(4); hull_y0(4)], p0);    line4_start = local_to_global([hull_x0(3); hull_y0(3)], p0);

    line1_end = local_to_global([hull_x1(1); 0], p1);   line2_end = local_to_global([hull_x1(3); 0], p2);
    line3_end = local_to_global([hull_x1(3); 0], p3);   line4_end = local_to_global([hull_x1(1); 0], p4);

    line1 = line([line1_start(1), line1_end(1)], [line1_start(2), line1_end(2)], 'LineStyle','--');
    line2 = line([line2_start(1), line2_end(1)], [line2_start(2), line2_end(2)], 'LineStyle','--');
    line3 = line([line3_start(1), line3_end(1)], [line3_start(2), line3_end(2)], 'LineStyle','--');
    line4 = line([line4_start(1), line4_end(1)], [line4_start(2), line4_end(2)], 'LineStyle','--');

end

function [] = update_motion_fig(hgtrans, p)
    makehg = makehgtform('translate',[p(1) p(2) 0]) * makehgtform('zrotate',p(3));
    set(hgtrans, 'Matrix',makehg); 
end

function update_moorline(line1, line2, line3, line4, p0, p1, p2, p3, p4, hull_x0, hull_y0, hull_x1, hull_y1)
    
    line1_start = local_to_global([hull_x0(2); hull_y0(2)], p0);    line2_start = local_to_global([hull_x0(1); hull_y0(1)], p0);
    line3_start = local_to_global([hull_x0(4); hull_y0(4)], p0);    line4_start = local_to_global([hull_x0(3); hull_y0(3)], p0);

    line1_end = local_to_global([hull_x1(1); 0], p1);   line2_end = local_to_global([hull_x1(3); 0], p2);
    line3_end = local_to_global([hull_x1(3); 0], p3);   line4_end = local_to_global([hull_x1(1); 0], p4);

    set(line1, 'xdata', [line1_start(1), line1_end(1)], 'ydata',  [line1_start(2), line1_end(2)]);
    set(line2, 'xdata', [line2_start(1), line2_end(1)], 'ydata',  [line2_start(2), line2_end(2)]);
    set(line3, 'xdata', [line3_start(1), line3_end(1)], 'ydata',  [line3_start(2), line3_end(2)]);
    set(line4, 'xdata', [line4_start(1), line4_end(1)], 'ydata',  [line4_start(2), line4_end(2)]);


end


%% 寻找锚链连接点(大地坐标系下）
function pc_g = local_to_global(pc_l, p0)
    fai = p0(3);
    R = [cos(fai), -sin(fai);
        sin(fai), cos(fai)];
    pc_g = R * pc_l + p0(1:2);
    
end