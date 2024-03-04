function sol = inverse_kinematics(x, y, z, euler_alfa, euler_beta, euler_gama)
    %rotation matrix a partir dos angulos de euler (ZYZ)
    r = eul2rotm([euler_alfa, euler_beta, euler_gama], 'ZYZ')
    
    %transformacao da base para o end-effector
    T = [r [x;y;z]; 0 0 0 1];
    
    %distancia do J2 ao J3
    a = 0.210;
    %distancia J3 ao J5 (hipotenusa de um tringulo retangulo)
    b = sqrt(0.030^2 + (0.180+0.0415)^2);
    
    teta1 = zeros(8, 1);
    teta2 = zeros(8, 1);
    teta3 = zeros(8, 1);
    teta4 = zeros(8, 1);
    teta5 = zeros(8, 1);
    teta6 = zeros(8, 1);
    
    sol = zeros(0, 6);
%     solScore = [];
    
    for teta6_tentativa=-pi:0.001:pi
        %teta6_tentativa = -pi/3
%          if teta6_tentativa > -1.05
%             disp("aaa") 
%          end
        %% obter coordenadas do joint 5 a partir de uma suposição do angulo teta6
%         T6=[1 0 0 0
%             0 1 0 -0.0237
%             0 0 1 -0.0055
%             0 0 0 1];
        T6=[0; -0.0237; -0.0055];
        R56=[cos(teta6_tentativa) 0 sin(teta6_tentativa)       %duvida no sentido do angulo
            0 1 0
            -sin(teta6_tentativa) 0 cos(teta6_tentativa)];
%         T56=T6*R56;
%         
%         % transformacao da base para o joint5
%         T5 = T*inv(T56);
%         
%         x5 = T5(1, 4);
%         y5 = T5(2, 4);
%         z5 = T5(3, 4);

        ponto5 = [x;y;z] - r*transpose(R56) * T6;
        %ponto5 = [0.2759    0.1593    0.4648]
        x5 = ponto5(1);
        y5 = ponto5(2);
        z5 = ponto5(3);

        %% calcular teta1
        % assumnindo que a posição do end effector 6 (x,y,z) é aproximavel à posicao 5
        teta1(1:4) = wrapToPi(atan2(y5, x5)-3*pi/2);
        teta1(5:8) = wrapToPi(atan2(y5, x5)-3*pi/2 + pi);

        %% calcular teta2

        %distancia do Joint 2 até ao ponto final em Z
        h = z5 - (0.080 + 0.103);
        %distancia entre J2 e J5 (assumindo (x,y,z) = pos J5)
        c = sqrt(h^2 + x5^2 + y5^2);
        alfa = real(acos((a^2 + c^2 - b^2)/(2*a*c)));
        gama = real(atan2(sqrt(x5^2 + y5^2), h));

        teta2(1:2) = wrapToPi(gama - alfa);
        teta2(3:4) = wrapToPi(gama + alfa);
        teta2(5:6) = wrapToPi(-(gama - alfa));
        teta2(7:8) = wrapToPi(-(gama + alfa));

        %% calcular teta3
        lambda = atan2(0.030, 0.180+0.0415); %constante
        phi = real(acos(min(max((a^2 + b^2 - c^2)/(2*a*b), -1), 1)));

        teta3(1:2) = wrapToPi(-(phi - lambda - pi/2));
        teta3(3:4) = wrapToPi(-(3*pi/2 - phi - lambda));
        teta3(5:6) = wrapToPi(-(3*pi/2 - phi - lambda));
        teta3(7:8) = wrapToPi(-(phi - lambda - pi/2));


        %% calcular restantes tetas pelo metodo algebrico
        for i=1:2:8
            %% calcular teta5
            c5 = cos(teta2(i)) * (-r(1, 2)*cos(teta3(i))*teta1(i) + r(3, 2)*sin(teta3(i))) + sin(teta2(i))*(r(3,2)*cos(teta3(i)) + r(1,2)*sin(teta1(i))*sin(teta3(i))) +  cos(teta1(i))*(r(2,2)*cos(teta2(i))*cos(teta3(i)) - r(2,2)*sin(teta2(i))*sin(teta3(i)));
            s5 = (0.2215 + 0.0237*c5 + (-0.183*cos(teta3(i))*sin(teta2(i)) + z*cos(teta3(i))*sin(teta2(i)) - 0.21*sin(teta3(i)) + x*sin(teta1(i))*sin(teta2(i))*sin(teta3(i)) + cos(teta2(i))*(-x*cos(teta3(i))*sin(teta1(i)) + (-0.183 + z)*sin(teta3(i))) + cos(teta1(i))*(y*cos(teta2(i))*cos(teta3(i)) - y*sin(teta2(i))*sin(teta3(i))))) / 0.0055;

            s5 = min(max(s5, -1), 1);
            c5 = min(max(c5, -1), 1);
            
            teta5(i) = wrapToPi(real(atan2(s5, c5)));
            teta5(i+1) = wrapToPi(-teta5(i) - 2*atan2(0.0055, 0.0237));
            
            s4 = 0;
            c4 = 0;
            s6 = 0;
            c6 = 0;
            
            if abs(s5) > 0.0001
                %% calcular teta4
                s4 = (r(1, 2) * cos(teta1(i)) + r(2, 2) * sin(teta1(i))) / s5;
                c4 = ((sin(teta2(i))*(-r(2, 2)*cos(teta1(i))*cos(teta3(i)) + r(1, 2)*cos(teta3(i))*sin(teta1(i)) - r(3, 2)*sin(teta3(i))) + cos(teta2(i))*(r(3, 2)*cos(teta3(i)) + (-r(2, 2)*cos(teta1(i)) + r(1, 2)*sin(teta1(i)))*sin(teta3(i)))))/s5;
                
                %% calcular teta6
                %c6 = (-cos(teta2(i))*(-r(1, 3)*cos(teta3(i))*sin(teta1(i)) + r(3, 3)*sin(teta3(i))) + sin(teta2(i))*(r(3, 3)*cos(teta3(i)) + r(1, 3)*sin(teta1(i))*sin(teta3(i))) + cos(teta1(i))*(r(2,3)*cos(teta2(i))*cos(teta3(i)) - r(2, 3)*sin(teta2(i))*sin(teta3(i)))) / s5;
                c6  = (cos(teta2(i))* (- r(1,3) *cos(teta3(i))* sin(teta1(i)) + r(3,3)* sin(teta3(i))) + sin(teta2(i))* ( r(3,3)* cos(teta3(i)) +  r(1,3) *sin(teta1(i))* sin(teta3(i))) + cos(teta1(i))* (r(2,3)* cos(teta2(i)) *cos(teta3(i)) -  r(2,3)* sin(teta2(i)) *sin(teta3(i))))/(-s5);
                s6 = (cos(teta2(i))*(-r(1,1)*cos(teta3(i))*sin(teta1(i)) + r(3, 1)*sin(teta3(i))) + sin(teta2(i))*(r(3,1)*cos(teta3(i)) + r(1, 1)*sin(teta1(i))*sin(teta3(i))) + cos(teta1(i))*(r(2,1)*cos(teta2(i))*cos(teta3(i)) - r(2,1)*sin(teta2(i))*sin(teta3(i)))) / s5;
            else
                % Seno de theta5 é 0. Logo é necessário recorrer a outras
                % expressões que não dividam por s5
                
                %% calcular teta4
                s4 = (x*cos(teta1(i)) + y*sin(teta1(i))) / (-0.0055*c5);
                c4 = ((cos(teta3(i))*(-0.21 + (-0.183 + z)*cos(teta2(i)) -  y*cos(teta1(i))*sin(teta2(i)) + x*sin(teta1(i))*sin(teta2(i))) + (-y*cos(teta1(i))*cos(teta2(i)) + x*cos(teta2(i))*sin(teta1(i)) + (0.183 - z)*sin(teta2(i)))*sin(teta3(i))) - 0.03) / (-0.0055*c5);
                
                %% calcular teta6
                A = r(1,1)*cos(teta1(i)) + r(2,1)*sin(teta1(i));
                B = r(1,3)*cos(teta1(i)) + r(2,3)*sin(teta1(i));
                
                s6 = (B - A/c4*c5*s4) / ((c5*s4)/c4*c5*s4 + c4);
                c6 = (A + c5*s4*s6) / c4;
            end
            
            s4 = min(max(s4, -1), 1);
            c4 = min(max(c4, -1), 1);

            teta4(i) = wrapToPi(real(atan2(s4, c4)));
            teta4(i+1) = wrapToPi(teta4(i) + pi);
            
            s6 = min(max(s6, -1), 1);
            c6 = min(max(c6, -1), 1);

            teta6(i) = wrapToPi(real(atan2(s6, c6)));
            teta6(i+1) = wrapToPi(teta6(i) + pi);
        end
        
        
        result = zeros(8, 6);
        result(:, 1) = teta1;
        result(:, 2) = teta2;
        result(:, 3) = teta3;
        result(:, 4) = teta4;
        result(:, 5) = teta5;
        result(:, 6) = teta6;
        
        
        
        for i=1:8
            % Verifica se algum dos teta6 calculado das várias soluções
            % corresponde ao teta6_tentativa
            if abs(teta6(i)-teta6_tentativa) < 0.005
                % Verifica se a solução é válida
                % (verifica se aplicando as transformações vai bater certo com o direct_kinematics)
                direct = direct_kinematics(teta1(i), teta2(i), teta3(i), teta4(i), teta5(i), teta6(i));
%                 disp(O)
%                 disp([x, y, z, euler_alfa, euler_beta, euler_gama])
                
                
                rotm_dir_kinematics = eul2rotm([direct(4), direct(5), direct(6)], 'ZYZ');
                %verifica se todas as coordenadas são proximas, e se todos
                %os elementos das matrizes de rotação são proximos
                if abs(x-direct(1)) < 0.0005 && abs(y-direct(2)) < 0.0005 && abs(z-direct(3)) < 0.0005 && sum(sum(abs(r - rotm_dir_kinematics) < 0.01)) == 9
                    disp("solucao encontrada!")
                    
                    % SSE da posições e angulo de euler. É um score do quão
                    % proxima a solução é (menor é melhor)
                    score = sqrt(sum(([x y z euler_alfa euler_beta euler_gama] - direct) .^ 2))
                    
                    % Indica em que indice da matriz das solucoes deve
                    % substituir. 
%                     substitui = -1;
                    insereSolucao = 1;
                    
                    %% Verifica se a solução já existe no vetor de soluções
                    for s=1:size(sol, 1)
                        thresh = 0.01;
                        if angdiff(sol(s,1), teta1(i)) < thresh && angdiff(sol(s,2), teta2(i)) < thresh && angdiff(sol(s,3), teta3(i)) < thresh && angdiff(sol(s,4), teta4(i)) < thresh && angdiff(sol(s,5), teta5(i)) < thresh && angdiff(sol(s,6), teta6(i)) < thresh
                            % Esta solução é semelhante a uma já guardada no indice s
                            
                            insereSolucao = 0;
                            
%                             if score < solScore(s)
%                                 % Esta solução é melhor. substitui-a
%                                 disp("Solução melhor!!")
%                                 substitui = s;
%                             end
                        end
                    end
                    
                    if insereSolucao
                        sol = [sol; [teta1(i), teta2(i), teta3(i), teta4(i), teta5(i), teta6(i)]];
                        % faz plot (a flag do_plot=1)
                        direct_kinematics(teta1(i), teta2(i), teta3(i), teta4(i), teta5(i), teta6(i),1);
                    end
%                     %%
%                     if substitui == -1
%                         sol = [sol; [teta1(i), teta2(i), teta3(i), teta4(i), teta5(i), teta6(i)]];
%                         solScore = [solScore; score]
%                     else
%                         sol(substitui, :) = [teta1(i), teta2(i), teta3(i), teta4(i), teta5(i), teta6(i)];
%                         solScore(substitui) = score;
%                     end
                    
                    % faz plot (a flag do_plot=1)
                    % direct_kinematics(teta1(i), teta2(i), teta3(i), teta4(i), teta5(i), teta6(i), 1);
                    
                    
                end
            end
        end
    end
    
    
        
    
    if size(sol) == 0   %caso nenhuma solução seja encontrada
       disp('No solutions were found!'); 
    else                %caso existam soluções
        %Calculo de qual a solução que tem menor gasto de energia considerando 
        %que todos os joints gastam a mesma energia para rodarem um radiano e
        %que gastam 1 Joule por cada radiano
        energy_cost=10000;
        best_sol_energy=0;
        for i=1:size(sol,1)
            if sum(abs(sol(i,:)))< energy_cost
                energy_cost=sum(sol(i,:));
                best_sol_energy=i;
            end
        end
        disp('The best solution energy wise is:');
        disp(sol(best_sol_energy,:));
        disp('All the solutions found:');
        disp(sol);
    end
    
end