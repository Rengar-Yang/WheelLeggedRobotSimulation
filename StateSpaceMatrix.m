function [A, B] = StateSpaceMatrix(mb, mw, Ib, Iw, rw, l, g)
    % Parameters:
    % mb - mass of the float-based body (kg)
    % mw - mass of the wheel (kg)
    % Ib - moment of inertia of the float-based body (kg·m²)
    % Iw - moment of inertia of the wheel (kg·m²)
    % rw - radius of the wheel (m)
    % l - height of the body (m)
    % g - gravitational acceleration (m/s²)
    
    % Calculations
    m11 = (mb + mw) * rw^2 + Iw;
    m12 = mb * l * rw;
    m22 = mb * l^2 + Ib;
    G = mb * g * l;
    
    % Matrix A
    A = zeros(3, 3);
    A(1, 2) = 1;
    A(2, 1) = G * m11 / (m11 * m22 - m12^2);
    A(3, 1) = -rw * G * m12 / (m11 * m22 - m12^2);
    
    % Matrix B
    B = zeros(3, 1);
    B(2,1) = -(m12+m11) / (m11 * m22 - m12^2);
    B(3,1) = rw*(m22+m12) / (m11 * m22 - m12^2);
    
    % % Display the results
    % disp('Matrix A:');
    % disp(A);
    % disp('Matrix B:');
    % disp(B);
end

% function Iw = compute_Iw(mw, rw)
%     % Iw - moment of inertia of the wheel (kg·m²)
%     % mw - mass of the wheel (kg)
%     % rw - radius of the wheel (m)
% 
%     Iw = 0.5 * mw * rw^2;
% end
% 
% function Ib = compute_Ib(mb, h, w)
%     % Ib - moment of inertia of the float-based body (kg·m²)
%     % mb - mass of the float-based body (kg)
%     % h - height of the body (m)
%     % w - width of the body (m)
% 
%     Ib = (1/12) * mb * (h^2 + w^2);
% end


