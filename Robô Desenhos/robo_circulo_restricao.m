% Parâmetros do robô
L1 = 2; % Comprimento do primeiro elo
L2 = 1.5; % Comprimento do segundo elo

% Definir centro do círculo
circle_center_x = 2;
circle_center_y = 1;
circle_radius = 1;

% Número de pontos para a trajetória do círculo
num_points = 100;
theta_circle = linspace(0, 2*pi, num_points);

% Inicialização das coordenadas dos elos e do efetuador
x_end_effector = zeros(1, num_points);
y_end_effector = zeros(1, num_points);

% Calcular coordenadas do círculo
x_circle = circle_center_x + circle_radius * cos(theta_circle);
y_circle = circle_center_y + circle_radius * sin(theta_circle);

% Inicializar a figura
h = figure;

for i = 1:num_points
    % Coordenadas do efetuador para desenhar o círculo
    x_end_effector(i) = circle_center_x + circle_radius * cos(theta_circle(i));
    y_end_effector(i) = circle_center_y + circle_radius * sin(theta_circle(i));
    
    % Verificar se o efetuador está dentro da área de trabalho
    inside_workspace = norm([x_end_effector(i), y_end_effector(i)]) <= L1 + L2;
    
    if inside_workspace
        % Cinemática inv para encontrar as coordenadas dos elos
        x_end_effector_temp = x_end_effector(i);
        y_end_effector_temp = y_end_effector(i);
        D = (x_end_effector_temp^2 + y_end_effector_temp^2 - L1^2 - L2^2) / (2 * L1 * L2);
        theta2 = real(acos(D)); % Garantindo que o ângulo seja real
        theta1 = real(atan2(y_end_effector_temp, x_end_effector_temp) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2)));

        x_link1 = L1 * cos(theta1);
        y_link1 = L1 * sin(theta1);

        x_link2 = x_link1 + L2 * cos(theta1 + theta2);
        y_link2 = y_link1 + L2 * sin(theta1 + theta2);

        % Desenhar o robô e a trajetória do círculo
        plot(x_circle, y_circle, 'r--', 'LineWidth', 1.5);
        hold on;
        plot([0, x_link1], [0, y_link1], 'b-', 'LineWidth', 2);
        plot([x_link1, x_link2], [y_link1, y_link2], 'g-', 'LineWidth', 2);
        plot(x_end_effector(1:i), y_end_effector(1:i), 'k.', 'MarkerSize', 10);
        grid on;
        axis equal;
        xlim([circle_center_x - circle_radius - 1, circle_center_x + circle_radius + L1 + L2 - 1]);
        ylim([circle_center_y - circle_radius - 3, circle_center_y + circle_radius + L1 + L2 - 1]);
        title('Robô planar com dois elos desenhando um círculo');
        xlabel('X');
        ylabel('Y');
        legend('Trajetória do círculo', 'Elo 1', 'Elo 2', 'Efetuador');
        hold off;
        pause(0.05); % Pausa para exibição em tempo real
    else
        % Fechar a figura se o efetuador sair da área de trabalho
        close(h);
        disp('O efetuador saiu da área de trabalho!');
        break;
    end
end
