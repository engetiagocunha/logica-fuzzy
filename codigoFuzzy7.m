% Simulação de Lógica Fuzzy em GNU Octave
% Robô que navega em direção à luz evitando obstáculos

clc;
clear all;
close all;

% =============================================
% 1. DEFINIÇÃO DAS FUNÇÕES DE PERTINÊNCIA
% =============================================

function y = trimf(x, params)
    a = params(1); b = params(2); c = params(3);
    y = zeros(size(x));
    idx1 = (x <= a); y(idx1) = 0;
    idx2 = (x > a & x <= b); y(idx2) = (x(idx2) - a) / (b - a);
    idx3 = (x > b & x < c); y(idx3) = (c - x(idx3)) / (c - b);
    idx4 = (x >= c); y(idx4) = 0;
end

function y = trapmf(x, params)
    a = params(1); b = params(2); c = params(3); d = params(4);
    y = zeros(size(x));
    idx1 = (x <= a); y(idx1) = 0;
    idx2 = (x > a & x < b); y(idx2) = (x(idx2) - a) / (b - a);
    idx3 = (x >= b & x <= c); y(idx3) = 1;
    idx4 = (x > c & x < d); y(idx4) = (d - x(idx4)) / (d - c);
    idx5 = (x >= d); y(idx5) = 0;
end

function angle = wrapToPi(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end

% =============================================
% 2. CONFIGURAÇÃO DO SISTEMA FUZZY
% =============================================

% Variáveis de entrada e saída
x_dist = 0:0.1:100; % Domínio da distância
x_lum = 0:0.1:100;  % Domínio da luminosidade
x_dir = -1:0.01:1;  % Domínio da direção
x_vel = 0:0.01:1;   % Domínio da velocidade

% Conjuntos fuzzy para distância
perto_dist = trimf(x_dist, [0 0 40]);
medio_dist = trimf(x_dist, [30 50 70]);
longe_dist = trimf(x_dist, [60 100 100]);

% Conjuntos fuzzy para luminosidade
fraca_lum = trimf(x_lum, [0 0 40]);
moderada_lum = trimf(x_lum, [30 50 70]);
forte_lum = trimf(x_lum, [60 100 100]);

% Conjuntos fuzzy para direção
esquerda_dir = trapmf(x_dir, [-1 -1 -0.5 0]);
centro_dir = trimf(x_dir, [-0.5 0 0.5]);
direita_dir = trapmf(x_dir, [0 0.5 1 1]);

% Conjuntos fuzzy para velocidade
baixa_vel = trimf(x_vel, [0 0 0.5]);
media_vel = trimf(x_vel, [0.3 0.5 0.7]);
alta_vel = trimf(x_vel, [0.5 1 1]);

% Plotar funções de pertinência
figure;
subplot(2, 2, 1);
plot(x_dist, perto_dist, 'b-', x_dist, medio_dist, 'g-', x_dist, longe_dist, 'r-');
title('Distância');
legend('Perto', 'Médio', 'Longe');
xlabel('Distância (unidades)');
ylabel('Pertinência');
grid on;

subplot(2, 2, 2);
plot(x_lum, fraca_lum, 'b-', x_lum, moderada_lum, 'g-', x_lum, forte_lum, 'r-');
title('Luminosidade');
legend('Fraca', 'Moderada', 'Forte');
xlabel('Luminosidade (%)');
ylabel('Pertinência');
grid on;

subplot(2, 2, 3);
plot(x_dir, esquerda_dir, 'b-', x_dir, centro_dir, 'g-', x_dir, direita_dir, 'r-');
title('Direção');
legend('Esquerda', 'Centro', 'Direita');
xlabel('Direção (-1 a 1)');
ylabel('Pertinência');
grid on;

subplot(2, 2, 4);
plot(x_vel, baixa_vel, 'b-', x_vel, media_vel, 'g-', x_vel, alta_vel, 'r-');
title('Velocidade');
legend('Baixa', 'Média', 'Alta');
xlabel('Velocidade (0 a 1)');
ylabel('Pertinência');
grid on;

% =============================================
% 3. SIMULAÇÃO DINÂMICA
% =============================================

% Definição do ambiente
obstaculos = [1 1; 2 3; 4 2]; % Posições dos centros dos obstáculos
fonte_luz = [3 4]; % Posição da fonte de luz
pos_atual = [0 0]; % Posição inicial do robô
orientacao = 0; % Orientação inicial do robô (em radianos)
t = 0:0.1:50; % Tempo de simulação aumentado
trajetoria_x = zeros(size(t));
trajetoria_y = zeros(size(t));
trajetoria_x(1) = pos_atual(1);
trajetoria_y(1) = pos_atual(2);

% Variáveis para armazenar saídas ao longo do tempo
direcoes = zeros(size(t));
velocidades = zeros(size(t));
buzinas = zeros(size(t));
direcoes(1) = 0;
velocidades(1) = 0;
buzinas(1) = 0;

for i = 2:length(t)
    % Calcular distâncias e luminosidades dinamicamente
    dist_obstaculo = min(sqrt((obstaculos(:,1) - pos_atual(1)).^2 + (obstaculos(:,2) - pos_atual(2)).^2));
    dist_frente = dist_obstaculo; % Considera o obstáculo mais próximo como frontal

    % Calcular a direção relativa à fonte de luz
    dx = fonte_luz(1) - pos_atual(1);
    dy = fonte_luz(2) - pos_atual(2);
    angulo_fonte = atan2(dy, dx); % Ângulo em relação à fonte de luz

    % Calcular luminosidades em diferentes direções
    dist_fonte = sqrt(dx^2 + dy^2);
    lum_base = 100 * (1 - dist_fonte / 5); % Intensidade decresce com a distância
    lum_base = max(0, min(100, lum_base));

    % Calcular ângulos relativos para cada sensor
    ang_frente = wrapToPi(angulo_fonte - orientacao);
    ang_esquerda = wrapToPi(ang_frente + pi/2);
    ang_direita = wrapToPi(ang_frente - pi/2);
    ang_tras = wrapToPi(ang_frente + pi);

    % Ajustar luminosidade com base no ângulo
    lum_frente = lum_base * max(0, cos(ang_frente));
    lum_esquerda = lum_base * max(0, cos(ang_esquerda));
    lum_direita = lum_base * max(0, cos(ang_direita));
    lum_tras = lum_base * max(0, cos(ang_tras));

    % Fuzzificação
    mf_perto = trimf(dist_frente, [0 0 40]);
    mf_longe = trimf(dist_frente, [60 100 100]);
    mf_fraca_f = trimf(lum_frente, [0 0 40]);
    mf_forte_f = trimf(lum_frente, [60 100 100]);
    mf_fraca_e = trimf(lum_esquerda, [0 0 40]);
    mf_forte_e = trimf(lum_esquerda, [60 100 100]);
    mf_fraca_d = trimf(lum_direita, [0 0 40]);
    mf_forte_d = trimf(lum_direita, [60 100 100]);
    mf_fraca_t = trimf(lum_tras, [0 0 40]);
    mf_forte_t = trimf(lum_tras, [60 100 100]);

    % Regras fuzzy
    rule_dir = zeros(1, 6);
    rule_vel = zeros(1, 6);
    rule_buz = zeros(1, 6);
    dir_sugerida = zeros(1, 6);
    vel_sugerida = zeros(1, 6);

    % Regra 1: Se obstáculo muito perto -> desviar imediatamente
    rule_dir(1) = (dist_frente < 1);
    rule_vel(1) = (dist_frente < 1);
    rule_buz(1) = 0;
    dir_sugerida(1) = -sign(dx) * 0.9;
    vel_sugerida(1) = 0.1;

    % Regra 2: Se luz forte à esquerda -> virar esquerda
    rule_dir(2) = mf_forte_e;
    rule_vel(2) = mf_forte_e;
    rule_buz(2) = 0;
    dir_sugerida(2) = -0.8;
    vel_sugerida(2) = 0.5;

    % Regra 3: Se luz forte à direita -> virar direita
    rule_dir(3) = mf_forte_d;
    rule_vel(3) = mf_forte_d;
    rule_buz(3) = 0;
    dir_sugerida(3) = 0.8;
    vel_sugerida(3) = 0.5;

    % Regra 4: Se luz forte frontal -> seguir reto e buzinar
    rule_dir(4) = min(mf_longe, mf_forte_f);
    rule_vel(4) = min(mf_longe, mf_forte_f);
    rule_buz(4) = min(mf_longe, mf_forte_f);
    dir_sugerida(4) = 0;
    vel_sugerida(4) = 0.2;

    % Regra 5: Se luz fraca frontal -> seguir reto rápido
    rule_dir(5) = min(mf_longe, mf_fraca_f);
    rule_vel(5) = min(mf_longe, mf_fraca_f);
    rule_buz(5) = 0;
    dir_sugerida(5) = 0;
    vel_sugerida(5) = 0.8;

    % Regra 6: Padrão
    rule_dir(6) = 1 - max([mf_perto mf_forte_e mf_forte_d mf_forte_f]);
    rule_vel(6) = 1 - max([mf_perto mf_forte_e mf_forte_d mf_forte_f]);
    rule_buz(6) = 0;
    dir_sugerida(6) = (lum_direita - lum_esquerda) / 100;
    vel_sugerida(6) = 0.4;

    % Agregação e Defuzzificação
    dir_agregada = zeros(size(x_dir));
    for j = 1:6
        if dir_sugerida(j) < 0
            temp = min(rule_dir(j), esquerda_dir);
        elseif dir_sugerida(j) > 0
            temp = min(rule_dir(j), direita_dir);
        else
            temp = min(rule_dir(j), centro_dir);
        end
        dir_agregada = max(dir_agregada, temp);
    end
    denominador = trapz(x_dir, dir_agregada);
    if denominador ~= 0
        direcao = trapz(x_dir, x_dir .* dir_agregada) / denominador;
    else
        direcao = 0;
    end

    vel_agregada = zeros(size(x_vel));
    for j = 1:6
        if vel_sugerida(j) < 0.4
            temp = min(rule_vel(j), baixa_vel);
        elseif vel_sugerida(j) > 0.6
            temp = min(rule_vel(j), alta_vel);
        else
            temp = min(rule_vel(j), media_vel);
        end
        vel_agregada = max(vel_agregada, temp);
    end
    denominador = trapz(x_vel, vel_agregada);
    if denominador ~= 0
        velocidade = trapz(x_vel, x_vel .* vel_agregada) / denominador;
    else
        velocidade = 0.4;
    end

    buzina = max(rule_buz) > 0.5;

    % Armazenar saídas
    direcoes(i) = direcao;
    velocidades(i) = velocidade;
    buzinas(i) = buzina;

    % Verificação de colisão antes de atualizar a posição
    novo_x = pos_atual(1) + cos(orientacao) * velocidade * 0.1;
    novo_y = pos_atual(2) + sin(orientacao) * velocidade * 0.1;
    colisao = false;
    for k = 1:size(obstaculos, 1)
        if (novo_x >= obstaculos(k,1) - 0.5 && novo_x <= obstaculos(k,1) + 0.5 && ...
            novo_y >= obstaculos(k,2) - 0.5 && novo_y <= obstaculos(k,2) + 0.5)
            colisao = true;
            break;
        end
    end

    if colisao
        direcao = -sign(dx) * 0.9;
        velocidade = 0.1;
    end

    % Atualizar orientação e posição
    orientacao = orientacao + direcao * 0.1;
    pos_atual = pos_atual + [cos(orientacao) * velocidade * 0.1, sin(orientacao) * velocidade * 0.1];
    trajetoria_x(i) = pos_atual(1);
    trajetoria_y(i) = pos_atual(2);

    % Condição de parada
    if dist_fonte < 0.5
        fprintf('\nRobô alcançou a fonte de luz!\n');
        trajetoria_x = trajetoria_x(1:i);
        trajetoria_y = trajetoria_y(1:i);
        direcoes = direcoes(1:i);
        velocidades = velocidades(1:i);
        buzinas = buzinas(1:i);
        t = t(1:i);
        break;
    end
end

% =============================================
% 4. VISUALIZAÇÃO DOS RESULTADOS
% =============================================

figure;
hold on;
rectangle('Position', [1 1 1 1], 'FaceColor', [0.8 0.6 0.6]); % Obstáculo 1
rectangle('Position', [2 3 1 1], 'FaceColor', [0.8 0.6 0.6]); % Obstáculo 2
rectangle('Position', [4 2 1 1], 'FaceColor', [0.8 0.6 0.6]); % Obstáculo 3
plot(3, 4, 'yo', 'MarkerSize', 20, 'LineWidth', 3); % Fonte de luz
plot(0, 0, 'rs', 'MarkerSize', 10, 'LineWidth', 2); % Posição inicial
plot(trajetoria_x, trajetoria_y, 'b-', 'LineWidth', 2); % Trajetória
xlabel('Posição X'); ylabel('Posição Y'); title('Trajetória do Robô');
legend('Obstáculos', '', '', 'Fonte de Luz', 'Início', 'Trajetória');
axis([0 5 0 5]);
grid on;
hold off;

% Plotar saídas ao longo do tempo
figure;
subplot(3,1,1);
plot(t, direcoes, 'b-');
xlabel('Tempo (s)');
ylabel('Direção (-1 a 1)');
title('Direção ao Longo do Tempo');
grid on;

subplot(3,1,2);
plot(t, velocidades, 'g-');
xlabel('Tempo (s)');
ylabel('Velocidade (0 a 1)');
title('Velocidade ao Longo do Tempo');
grid on;

subplot(3,1,3);
plot(t, buzinas, 'r-');
xlabel('Tempo (s)');
ylabel('Buzina (0 ou 1)');
title('Buzina ao Longo do Tempo');
grid on;

fprintf('\n=== RESULTADOS FINAIS ===\n');
fprintf('Direção final: %.2f (-1=esq, 0=centro, 1=dir)\n', direcao);
fprintf('Velocidade final: %.2f (0=parado, 1=máxima)\n', velocidade);
fprintf('Buzina: %d (0=desligada, 1=ligada)\n', buzina);
