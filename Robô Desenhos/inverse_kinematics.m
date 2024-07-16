% Fun��o para calcular a cinem�tica inversa
function [theta1, theta2] = inverse_kinematics(x, y, L1, L2)
    % C�lculo do �ngulo de junta theta1
    theta1 = atan2(y, x);
    
    % Dist�ncia do ponto final ao ponto de ancoragem do primeiro elo
    d = sqrt(x^2 + y^2);
    
    % Lei dos cossenos para calcular o �ngulo de junta theta2
    alpha = acos((L1^2 + L2^2 - d^2) / (2 * L1 * L2));
    beta = acos((d^2 + L1^2 - L2^2) / (2 * d * L1));
    theta2 = pi - alpha - beta;
end