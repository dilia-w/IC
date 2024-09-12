% Parâmetros do robô
L1 = 2; % Comprimento do primeiro elo
L2 = 1.75; % Comprimento do segundo elo

% Definir centro do círculo
circle_center_x = 0.5;
circle_center_y = 1;

% Parâmetros da limaçon
a = 0.5; % Coeficiente 'a' da limaçon
b = 1.75; % Coeficiente 'b' da limaçon

% Número de pontos para a trajetória do círculo
num_points = 100;

% Tempo total da trajetória
tf = 10; % por exemplo, 10 segundos
t = linspace(0, tf, num_points);

% Coeficientes do polinômio de trajetória
theta0 = 0; % ângulo inicial
thetaf = 2*pi; % 360 graus
a0 = theta0;
a1 = 0;
a2 = (3/tf^2) * (thetaf - theta0);
a3 = -(2/tf^3) * (thetaf - theta0);

% Calcular theta(t), theta_dot(t) e theta_dot_dot(t)
theta_t = a0 + a1 * t + a2 * t.^2 + a3 * t.^3;
theta_dot_t = a1 + 2 * a2 * t + 3 * a3 * t.^2;
theta_dot_dot_t = 2 * a2 + 6 * a3 * t;

theta_limacon = linspace(0, 2*pi, num_points);
r_limacon = a + b * cos(theta_limacon);
x_limacon = r_limacon .* cos(theta_limacon) + circle_center_x;
y_limacon = r_limacon .* sin(theta_limacon) + circle_center_y;

% Inicialização das coordenadas dos elos e do efetuador
x_end_effector = zeros(1, num_points);
y_end_effector = zeros(1, num_points);

% Inicializar a figura
figure;

for i = 1:num_points
    % Coordenadas do efetuador para desenhar o círculo
    x_end_effector(i) = x_limacon(i);
    y_end_effector(i) = y_limacon(i);
    
    % Verificar se o efetuador está dentro da área de trabalho
    inside_workspace = norm([x_end_effector(i), y_end_effector(i)]) <= L1 + L2;
    
    if inside_workspace
        % Cinemática inv para encontrar as coordenadas dos elos
        x_end_effector_temp = x_end_effector(i);
        y_end_effector_temp = y_end_effector(i);
        D = (x_end_effector_temp^2 + y_end_effector_temp^2 - L1^2 - L2^2) / (2 * L1 * L2);
        theta2 = real(acos(D)); % Garantindo que o ângulo seja real
        %theta2 = 0;
        theta1 = real(atan2(y_end_effector_temp, x_end_effector_temp) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2)));
        %theta1 = 0;

        x_link1 = L1 * cos(theta1);
        y_link1 = L1 * sin(theta1);

        x_link2 = x_link1 + L2 * cos(theta1 + theta2);
        y_link2 = y_link1 + L2 * sin(theta1 + theta2);

        % Desenhar o robô e a trajetória do círculo
        subplot(2,2,1);
        plot(x_limacon, y_limacon, 'r--', 'LineWidth', 1.5);
        hold on;
        plot([0, x_link1], [0, y_link1], 'b-', 'LineWidth', 2);
        plot([x_link1, x_link2], [y_link1, y_link2], 'g-', 'LineWidth', 2);
        plot(x_end_effector(1:i), y_end_effector(1:i), 'k.', 'MarkerSize', 10);
        grid on;
        axis equal;
        xlim([circle_center_x - circle_radius - 1, circle_center_x + circle_radius + L1 + L2 + 2]);
        ylim([circle_center_y - circle_radius - 2, circle_center_y + circle_radius + L1 + L2 - 1]);
        title('Robô e trajetória do limaçon');
        xlabel('X');
        ylabel('Y');
        legend('Trajetória do limaçon', 'Elo 1', 'Elo 2', 'Efetuador');
        hold off;

        % Subplot 2: Posição
        subplot(2,2,2);
        plot(t(1:i), theta_t(1:i), 'b-', 'LineWidth', 1.5);
        grid on;
        title('Posição vs Tempo');
        xlabel('Tempo (s)');
        ylabel('Posição (rad)');

        % Subplot 3: Velocidade
        subplot(2,2,3);
        plot(t(1:i), theta_dot_t(1:i), 'r-', 'LineWidth', 1.5);
        grid on;
        title('Velocidade vs Tempo');
        xlabel('Tempo (s)');
        ylabel('Velocidade (rad/s)');

        % Subplot 4: Aceleração
        subplot(2,2,4);
        plot(t(1:i), theta_dot_dot_t(1:i), 'g-', 'LineWidth', 1.5);
        grid on;
        title('Aceleração vs Tempo');
        xlabel('Tempo (s)');
        ylabel('Aceleração (rad/s^2)');

        pause(0.05); % Pausa para exibição em tempo real
    else
        % Fechar a figura se o efetuador sair da área de trabalho
        close(gcf);
        disp('O efetuador saiu da área de trabalho!');
        break;
    end
end
