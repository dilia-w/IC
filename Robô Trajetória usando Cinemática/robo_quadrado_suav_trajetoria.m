% Par�metros do rob�
L1 = 2; % Comprimento do primeiro elo
L2 = 1.5; % Comprimento do segundo elo

% Definir os v�rtices do quadrado dentro da �rea de trabalho do rob�
square_vertices = [1 1.5; 1.5 1.5; 1.5 1; 1 1; 1 1.5];

% N�mero de pontos para a interpola��o da trajet�ria suavizada
num_points_interp = 100;

% Interpolar uma trajet�ria suavizada usando splines c�bicas
smoothed_square_x = spline(1:size(square_vertices, 1), square_vertices(:, 1), linspace(1, size(square_vertices, 1), num_points_interp));
smoothed_square_y = spline(1:size(square_vertices, 1), square_vertices(:, 2), linspace(1, size(square_vertices, 1), num_points_interp));

% Inicializa��o das vari�veis para posi��o, velocidade e acelera��o
theta_t = zeros(1, num_points_interp);
theta_dot_t = zeros(1, num_points_interp);
theta_dot_dot_t = zeros(1, num_points_interp);

% Plot dos gr�ficos
figure;

for i = 1:num_points_interp
    % Coordenadas do efetuador para desenhar o quadrado
    x_end_effector = smoothed_square_x(i);
    y_end_effector = smoothed_square_y(i);
    
    % Cinem�tica inv para encontrar os �ngulos dos elos
    D = (x_end_effector^2 + y_end_effector^2 - L1^2 - L2^2) / (2 * L1 * L2);
    theta2 = real(acos(D)); % Garantindo que o �ngulo seja real
    theta1 = real(atan2(y_end_effector, x_end_effector) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2)));
    
    x_link1 = L1 * cos(theta1);
    y_link1 = L1 * sin(theta1);
    x_link2 = x_link1 + L2 * cos(theta1 + theta2);
    y_link2 = y_link1 + L2 * sin(theta1 + theta2);
    
    % Desenhar o rob� e a trajet�ria do quadrado
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
    title('Rob� e trajet�ria do quadrado');
    xlabel('X');
    ylabel('Y');
    legend('V�rtices do quadrado', 'Trajet�ria suavizada', 'Elo 1', 'Elo 2', 'Efetuador');
    hold off;
    
    % C�lculo da posi��o, velocidade e acelera��o
    if i > 1
        dt = 1 / num_points_interp;
        d_theta = atan2(smoothed_square_y(i) - smoothed_square_y(i-1), smoothed_square_x(i) - smoothed_square_x(i-1));
        d_theta_dot = d_theta / dt;
        d_theta_dot_dot = d_theta_dot / dt;
        
        theta_t(i) = theta_t(i-1) + d_theta;
        theta_dot_t(i) = d_theta_dot;
        theta_dot_dot_t(i) = d_theta_dot_dot;
    end
    
    % Gr�fico de posi��o
    subplot(2, 2, 2);
    plot(1:i, theta_t(1:i), 'b-', 'LineWidth', 1.5);
    grid on;
    title('Posi��o vs Tempo');
    xlabel('Tempo (passos)');
    ylabel('Posi��o (rad)');
    
    % Gr�fico de velocidade
    subplot(2, 2, 3);
    plot(1:i, theta_dot_t(1:i), 'r-', 'LineWidth', 1.5);
    grid on;
    title('Velocidade vs Tempo');
    xlabel('Tempo (passos)');
    ylabel('Velocidade (rad/s)');
    
    % Gr�fico de acelera��o
    subplot(2, 2, 4);
    plot(1:i, theta_dot_dot_t(1:i), 'g-', 'LineWidth', 1.5);
    grid on;
    title('Acelera��o vs Tempo');
    xlabel('Tempo (passos)');
    ylabel('Acelera��o (rad/s^2)');
    
    pause(0.05); % Pausa para exibi��o em tempo real
end
