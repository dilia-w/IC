% Parâmetros do robô
L1 = 2; % Comprimento do primeiro elo
L2 = 1.5; % Comprimento do segundo elo

% Definir os vértices do quadrado dentro da área de trabalho do robô
square_vertices = [1 1.5; 1.5 1.5; 1.5 1; 1 1; 1 1.5];

% Número de pontos para a interpolação da trajetória suavizada
num_points_interp = 100;

% Interpolar uma trajetória suavizada usando splines cúbicas
smoothed_square_x = spline(1:size(square_vertices, 1), square_vertices(:, 1), linspace(1, size(square_vertices, 1), num_points_interp));
smoothed_square_y = spline(1:size(square_vertices, 1), square_vertices(:, 2), linspace(1, size(square_vertices, 1), num_points_interp));

% Inicialização das variáveis para posição, velocidade e aceleração
theta_t = zeros(1, num_points_interp);
theta_dot_t = zeros(1, num_points_interp);
theta_dot_dot_t = zeros(1, num_points_interp);

% Plot dos gráficos
figure;

for i = 1:num_points_interp
    % Coordenadas do efetuador para desenhar o quadrado
    x_end_effector = smoothed_square_x(i);
    y_end_effector = smoothed_square_y(i);
    
    % Cinemática inv para encontrar os ângulos dos elos
    D = (x_end_effector^2 + y_end_effector^2 - L1^2 - L2^2) / (2 * L1 * L2);
    theta2 = real(acos(D)); % Garantindo que o ângulo seja real
    theta1 = real(atan2(y_end_effector, x_end_effector) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2)));
    
    x_link1 = L1 * cos(theta1);
    y_link1 = L1 * sin(theta1);
    x_link2 = x_link1 + L2 * cos(theta1 + theta2);
    y_link2 = y_link1 + L2 * sin(theta1 + theta2);
    
    % Desenhar o robô e a trajetória do quadrado
    subplot(2, 2, 1);
    plot(square_vertices(:, 1), square_vertices(:, 2), 'r--', 'LineWidth', 1.5);
    hold on;
    plot(smoothed_square_x, smoothed_square_y, 'k-', 'LineWidth', 1.5);
    plot([0, x_link1], [0, y_link1], 'b-', 'LineWidth', 2);
    plot([x_link1, x_link2], [y_link1, y_link2], 'g-', 'LineWidth', 2);
    plot(x_end_effector, y_end_effector, 'ko', 'MarkerSize', 8);
    grid on;
    axis equal;
    xlim([min(square_vertices(:, 1)) - 3, max(square_vertices(:, 1)) + L1 + L2 + 1]);
    ylim([min(square_vertices(:, 2)) - 3, max(square_vertices(:, 2)) + L1 + L2 + 1]);
    title('Robô e trajetória do quadrado');
    xlabel('X');
    ylabel('Y');
    legend('Vértices do quadrado', 'Trajetória suavizada', 'Elo 1', 'Elo 2', 'Efetuador');
    hold off;
    
    % Cálculo da posição, velocidade e aceleração
    if i > 1
        dt = 1 / num_points_interp;
        d_theta = atan2(smoothed_square_y(i) - smoothed_square_y(i-1), smoothed_square_x(i) - smoothed_square_x(i-1));
        d_theta_dot = d_theta / dt;
        d_theta_dot_dot = d_theta_dot / dt;
        
        theta_t(i) = theta_t(i-1) + d_theta;
        theta_dot_t(i) = d_theta_dot;
        theta_dot_dot_t(i) = d_theta_dot_dot;
    end
    
    % Gráfico de posição
    subplot(2, 2, 2);
    plot(1:i, theta_t(1:i), 'b-', 'LineWidth', 1.5);
    grid on;
    title('Posição vs Tempo');
    xlabel('Tempo (passos)');
    ylabel('Posição (rad)');
    
    % Gráfico de velocidade
    subplot(2, 2, 3);
    plot(1:i, theta_dot_t(1:i), 'r-', 'LineWidth', 1.5);
    grid on;
    title('Velocidade vs Tempo');
    xlabel('Tempo (passos)');
    ylabel('Velocidade (rad/s)');
    
    % Gráfico de aceleração
    subplot(2, 2, 4);
    plot(1:i, theta_dot_dot_t(1:i), 'g-', 'LineWidth', 1.5);
    grid on;
    title('Aceleração vs Tempo');
    xlabel('Tempo (passos)');
    ylabel('Aceleração (rad/s^2)');
    
    pause(0.05); % Pausa para exibição em tempo real
end
