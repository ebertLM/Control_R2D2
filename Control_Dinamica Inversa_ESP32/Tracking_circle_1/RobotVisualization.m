classdef RobotVisualization < handle
  % Gráficos y transformaciones homogéneas

  properties (Access = private)
    ROBOT_DATA  % struct con parámetros del robot
    h1
    h2
    h3
    h_link1
    h_link2
    h_link3
  end

  methods
    function this = RobotVisualization(data)
      this.ROBOT_DATA = data; 
    end

    %---------------- FUNCIONES DE PLOTEO ----------------------%
    function [T01,T02,T03] = transformacion_homogenea(this,q,punto_final)
        % Matriz de transformación homogénea

        %--------------------------------------------------------%
        % 1. LONGITUDES DEL BRAZO
        %--------------------------------------------------------%
        d1 = this.ROBOT_DATA.d1;
        a2 = this.ROBOT_DATA.a2;
        a3 = this.ROBOT_DATA.a3; 
        a4 = this.ROBOT_DATA.a4;

        %--------------------------------------------------------%
        % 1. LEEMOS PARAMETROS DE ENTRADA
        %--------------------------------------------------------%
        q1 = q(1);
        q2 = q(2);
        q3 = q(3);

        %--------------------------------------------------------%
        % 2. PUNTO FINAL DESEADO
        %--------------------------------------------------------%            
        if strcmp(punto_final,"ef")
            l = a3;
        elseif strcmp(punto_final,"pt")
            l = a3 + a4;
        end
        
        %--------------------------------------------------------%
        % 3. SISTEMAS COORDENADOS
        %--------------------------------------------------------%
        %  3.1. SISTEMA COORDENADO {1}
        theta1 = q1;
        a1     = 0;
        alpha1 = pi/2;
        A1 = matriz_homogenea_DH(theta1,d1,a1,alpha1);
        T01 = A1;

        %  3.2. SISTEMA COORDENADO {2}
        theta2 = q2;
        d2     = 0;
        alpha2 = 0;
        A2 = matriz_homogenea_DH(theta2,d2,a2,alpha2);
        T02 = A1*A2;

        %  3.3. SISTEMA COORDENADO {3}
        theta3 = q3;
        d3     = 0;
        alpha3 = 0;
        A3 = matriz_homogenea_DH(theta3,d3,l,alpha3);
        T03 = T02*A3;
    end

    function ax = drawInitialAxes(this)
        % CINEMATICA EN LA POSICION HOME
        q1 = 0;
        q2 =  pi/2;
        q3 = -pi/2;
        [T01,T02,T03] = this.transformacion_homogenea([q1,q2,q3],"pt");
        % CREAMOS FIGURA
        figure('Name', 'ROBOT', 'Position', [100 100 800 700])
        ax = axes;
        axis(ax,[-0.5  1.0  -0.5  1.0  0  0.6])
        hold(ax,"on")
        grid(ax,"on")
        axis(ax,"square")
        set(gca, 'box', 'on')
        xlabel('x(m)')
        ylabel('y(m)')
        % SISTEMAS COORDENADOS
        T0 = eye(4);
        plot_frame(ax,T0,'frame','0','length',0.1,'view',3);
        this.h1 = plot_frame(ax,T01,'frame','1','length',0.1);
        this.h2 = plot_frame(ax,T02,'frame','2','length',0.1);
        this.h3 = plot_frame(ax,T03,'frame','3','length',0.1);
        this.h_link1 = plot_link_DH(ax,T01,T0,'k');
        this.h_link2 = plot_link_DH(ax,T02,T01,'k');
        this.h_link3 = plot_link_DH(ax,T03,T02,'k');
    end

    function ax = drawAxesOfSimpleBox(this)
        % % CREAMOS FIGURA            
        % Devuelve el handle de los ejes para graficar posteriormente
        
        % Crea la figura y el axes
        fig = figure('Name','Trayectoria','Position',[100 100 800 700]);
        ax = axes('Parent',fig);
        hold(ax, 'on');
        grid(ax, 'on');
        box(ax, 'on');
        
        % Define límites de los 3 ejes y aspecto igual
        axis(ax, [-0.5 1.0   -0.5 1.0   0 0.6]);
        axis(ax, 'manual');
        % Etiquetas
        xlabel(ax, 'x (m)', 'FontSize', 10, 'FontWeight','bold');
        ylabel(ax, 'y (m)', 'FontSize', 10, 'FontWeight','bold');
        zlabel(ax, 'z (m)', 'FontSize', 10, 'FontWeight','bold');
        
        % Vista 3D por defecto
        view(ax, 3);         
    end

    function drawRobotLinks(this,q)
        [T01,T02,T03] = this.transformacion_homogenea(q, "pt");
        plot_frame(this.h1, T01);
        plot_frame(this.h2, T02);
        plot_frame(this.h3, T03);
        T0 = eye(4);
        plot_link_DH(this.h_link1,T01,T0);
        plot_link_DH(this.h_link2,T02,T01);
        plot_link_DH(this.h_link3,T03,T02);
    end

    function drawTrajectory(this,axes,XYZ)
        plot3(axes,XYZ(1),XYZ(2),XYZ(3),'.b')
    end

    function drawSingularity(this,Q2,Q3)
        % Cálculo de los puntos singulares
        % Se grafica en 2D la solución de Det(Jacobiano_analitico)=0
        % Esta solución SOLO esta resuelta para el rango de q2:[0º,150º] y q3:[-180º,0]
        % La solución de Det(Jacobiano_analitico) = 0, es:
        % sin(q3)*[a2*cos(q2)+(a3+a4)*cos(q2+q3)] = 0, esto nos lleva a la unión de dos
        % soluciones particulares:
        % 1) Primera solución:
        %    a2*cos(q2)+(a3+a4)*cos(q2+q3) = 0    .......(A)
        %    La solución de (A) es:
        %                 (a3+a4)*cos(q3) + a2
        %      q2 = atan (--------------------) 
        %                      a2*sin(q3)
        % 2) Segunda solución:
        %    sin(q3) = 0                      ......(B)
        %    La solución de (B) es:
        %      q3 = +/-2*pi*K,    K:0,1,2,..
        %
        % ENTRADA: 
        % Q2, Q3: Coordenadas articulares del robot [rad]

        a2 = this.ROBOT_DATA.a2;  %Cargamos parámetros del Robot
        a3 = this.ROBOT_DATA.a3;
        a4 = this.ROBOT_DATA.a4;
        l = a3 + a4;         

        % 1) DOMINIOS DE q2 y q3:
        n = 300;                   % resolución de la malla
        q2 = linspace(0, 150, n);  % Grados sexagesimales
        q3 = linspace(-180, 0, n); % Grados sexagesimales
        [Q2_grad, Q3_grad] = meshgrid(q2, q3);
        Q2_mesh = Q2_grad*pi/180;
        Q3_mesh = Q3_grad*pi/180;
        
        % 2) PRIMERA SOLUCIÓN:
        %    F(q2,q3) = a2*cos(q2)+(a3+a4)*cos(q2+q3), 
        %    F(q2,q3) = 0, se resuelve con 'contour' en GRÁFICAS
        %    Calculo de F(q2,q3) para cada punto de Q2_mesh y Q3_mesh
        F = a2*cos(Q2_mesh) + l*cos(Q2_mesh + Q3_mesh);
        
        % 3) SEGUNDA SOLUCIÓN:
        %    sin(q3) = 0, entonces q3:..,-180,0,180,..
        q3_line1 = zeros(size(q2));     % q3 = 0º
        q3_line2 = -ones(size(q2))*180; % q3 = -180º
        
        
        % 4) GRÁFICAS Y CONFIGURACIÓN
        figure('Position', [20 50 700 600])
        hold on
        %    Primera solucion
        contour(Q2_grad, Q3_grad, F, [0 0], 'LineWidth', 2, 'LineColor', 'k')
        
        %    Segunda solución
        scatter(q2, q3_line1, 5, [0.1 0.1 0.1], 'filled', 'r') 
        scatter(q2, q3_line2, 5, [0.1 0.1 0.1], 'filled', 'r')
        
        %    Coordenadas articulares del robot
        scatter(Q2*180/pi, Q3*180/pi, 10,[0.1 0.1 0.1], 'filled','MarkerFaceColor','b')
        hold off
        
        xlabel('q2(º)','FontSize',10,'FontWeight','bold')
        ylabel('q3(º)','FontSize',10,'FontWeight','bold')
        title('Singularidades y coordenadas (q2,q3) del robot','FontSize',12)
        grid on
        legend( ...
          'F(q2,q3)=0', ...
          'q3=0º', ...
          'q3=-180º', ...
          'q2, q3 del robot', ...
          'Location','northeast' ...
        )
    end    

  end
end
