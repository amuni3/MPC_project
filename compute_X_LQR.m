% BRRIEF:
%   Template for explicit invariant set computation. You MUST NOT change
%   the output.
% OUTPUT:
%   A_x, b_x: Describes polytopic X_LQR = {x| A_x * x <= b_x}
function [A_x, b_x] = compute_X_LQR
    % get basic controller parameters
    param = compute_controller_base_parameters;
    Ucons = param.Ucons;
    Xcons = param.Xcons;
    % MPC Lec 5 slide 43
    Ax_x = [eye(3); -1 * eye(3)];
    Bx_x = [Xcons(:,2); -1 * Xcons(:,1)];
    Ax_u = [eye(2); -1 * eye(2)];
    Bx_u = [Ucons(:,2); -1 * Ucons(:,1)];
    %% Here you need to implement the X_LQR computation and assign the result.
    m = [Ax_x, zeros(6,2)];
    n = [zeros(4,3), Ax_u];
    A_x = [m; n];
    b_x = [Bx_x; Bx_u];
end

