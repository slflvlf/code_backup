function [tau, faid_hat, dot_faid_hat] = local_dp_controller(vd, faid, p, v, M, D, time_step, faid_hat, dot_faid_hat)
    % 鐠囷拷瀹革拷
    z1 = p(3) - faid; %妫ｆ牕鎮滅拠锟藉�革拷

    % 濠曞倽锟斤拷
    beta = atan2(v(2), v(1));

    % 瑜版挸澧犻懜锟介崥鎴ｏ拷锟�
    x = fai + beta;

    % 閺堢喐婀滈懜锟介崥鎴ｏ拷锟�
    xd = faid + beta;

    % 娴ｅ酣鈧�姘�鎶ゅ▔銏犵繁閸掞拷 dot_faid
    alpha_filter = 0.15;    beta_filter = 0.005;
    [faid_hat, dot_faid_hat] = alpha_beta_filter(faid, faid_hat, dot_faid_hat, time_step, alpha_filter, beta_filter)
    
    % 缁嬪啿鐣鹃崙鑺ユ殶閿涳拷 alpha
    beta = atan2(v(2), v(1));   
    alpha1 = vd * cos(beta);
    alpha2 = vd * sin(beta);
    alpha3 = dot_faid_hat - (p(3) - faid);
    
    alpha = [alpha1, alpha2, alpha3]';

    % 闁�鐔峰�崇拠锟藉�革拷
    z2 = v - alpha;

    tau = M * 
end

function






function [faid_hat, dot_faid_hat] = alpha_beta_filter(faid, faid_hat, dot_faid_hat, time_step, alpha_filter, beta_filter)
    faid_hat = faid_hat + time_step *  dot_faid_hat;
    rk_hat = faid - faid_hat;

    faid_hat = faid_hat + alpha_filter * rk_hat;
    dot_faid_hat = dot_faid_hat + beta_filter / time_step * rk_hat;
end


function R = rotate_matrix(fai)
    R = [cos(fai), -sin(fai), 0;
        sin(fai), cos(fai), 0;
        0, 0, 1];
end