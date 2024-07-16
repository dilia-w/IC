% Parâmetros do robô
L1 = 3; % Comprimento do primeiro elo
L2 = 2; % Comprimento do segundo elo

% Definir centro do círculo
circle_center_x = 2;
circle_center_y = 1;
circle_radius = 2;

% Parâmetros do limaçon
a = 0.5; % Parâmetro de posição
b = 1; % Parâmetro de tamanho

% Número de pontos para a trajetória do limaçon
num_points = 100;
t = linspace(0, 10, num_points); % Tempo de 0 a 10 segundos

% Inicialização das coordenadas dos elos e do efetuador
x_end_effector = zeros(1, num_points);
y_end_effector = zeros(1, num_points);

% Calcular coordenadas do limaçon
theta_limacon = linspace(0, 2*pi, num_points);
r_limacon = a + b * cos(theta_limacon);
x_limacon = r_limacon .* cos(theta_limacon) + circle_center_x;
y_limacon = r_limacon .* sin(theta_limacon) + circle_center_y;

% Inicializar a figura
h = figure;

% Inicialização das variáveis para os gráficos de posição, velocidade e aceleração
pos = zeros(1, num_points);
vel = zeros(1, num_points);
acc = zeros(1, num_points);

for i = 1:num_points
    % Coordenadas do efetuador para desenhar o limaçon
    x_end_effector(i) = x_limacon(i);
    y_end_effector(i) = y_limacon(i);

    % Verificar se o efetuador está dentro da área de trabalho
    inside_workspace = norm([x_end_effector(i), y_end_effector(i)]) <= L1 + L2;
    
    if inside_workspace
        % Cinemática inversa para encontrar as coordenadas dos elos
        x_end_effector_temp = x_end_effector(i);
        y_end_effector_temp = y_end_effector(i);
        D = (x_end_effector_temp^2 + y_end_effector_temp^2 - L1^2 - L2^2) / (2 * L1 * L2);
        theta2 = real(acos(D)); % Garantindo que o ângulo seja real
        theta1 = real(atan2(y_end_effector_temp, x_end_effector_temp) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2)));

        x_link1 = L1 * cos(theta1);
        y_link1 = L1 * sin(theta1);

        x_link2 = x_link1 + L2 * cos(theta1 + theta2);
        y_link2 = y_link1 + L2 * sin(theta1 + theta2);

        % Cálculo das posições, velocidades e acelerações
        pos(i) = theta1 + theta2;
        if i > 1
            vel(i) = (pos(i) - pos(i-1)) / (t(i) - t(i-1));
            acc(i) = (vel(i) - vel(i-1)) / (t(i) - t(i-1));
        end
        
        % Desenhar o robô e a trajetória do limaçon
        subplot(2,2,1);
        plot(x_limacon, y_limacon, 'r--', 'LineWidth', 1.5);
        hold on;
        plot([0, x_link1], [0, y_link1], 'b-', 'LineWidth', 2);
        plot([x_link1, x_link2], [y_link1, y_link2], 'g-', 'LineWidth', 2);
        plot(x_end_effector(1:i), y_end_effector(1:i), 'k.', 'MarkerSize', 10);
        grid on;
        axis equal;
        xlim([circle_center_x - circle_radius - 2, circle_center_x + circle_radius + L1 + L2 + 2]);
        ylim([circle_center_y - circle_radius - 3, circle_center_y + circle_radius + L1 + L2 - 1]);
        title('Robô planar com dois elos desenhando um limaçon');
        xlabel('X');
        ylabel('Y');
        legend('Trajetória do limaçon', 'Elo 1', 'Elo 2', 'Efetuador');
        hold off;

        % Plotar gráfico de posição
        subplot(2, 2, 2);
        plot(t(1:i), pos(1:i), 'b-', 'LineWidth', 1.5);
        grid on;
        title('Posição em função do tempo');
        xlabel('Tempo (s)');
        ylabel('Posição (rad)');

        % Plotar gráfico de velocidade
        subplot(2, 2, 3);
        plot(t(2:i), vel(2:i), 'g-', 'LineWidth', 1.5);
        hold on;
        grid on;
        title('Velocidade em função do tempo');
        xlabel('Tempo (s)');
        ylabel('Velocidade (rad/s)');

        % Plotar gráfico de aceleração
        subplot(2, 2, 4);
        plot(t(3:i), acc(3:i), 'r-', 'LineWidth', 1.5);
        hold on;
        grid on;
        title('Aceleração em função do tempo');
        xlabel('Tempo (s)');
        ylabel('Aceleração (rad/s^2)');

        pause(0.05); % Pausa para exibição em tempo real
    else
        % Fechar a figura se o efetuador sair da área de trabalho
        close(h);
        disp('O efetuador saiu da área de trabalho!');
        break;
    end
end
