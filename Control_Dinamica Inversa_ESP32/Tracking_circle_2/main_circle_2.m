% Este script recibe datos por el puerto serial y los grafica según las
% variables de interés

% Cuando se recibe datos tipo String, se hace de la siguiente manera:
% 1. Leer la longitud del mensaje (1 byte)
%    messageLength = read(s1, 1, "uint8");      
% 2. Leer el mensaje completo según la longitud recibida
%    messageReceived = read(s1, messageLength, "char"); 
%    messageReceived: es el mensaje tipo 'string' recibido 
%                  por el puerto serial

clc; clearvars; close all

load_parameters

freeports = serialportlist("available"); % Visualizamos los puertos disponibles
% disp(freeports)
s1 = serialport('COM3', 115200); % Crear objeto de puerto serial 's1'
fprintf('%s Port Serial Conected...\n',freeports)
pause(2); % Esperar a que la conexión se establezca

% Limpiamos el puerto
flush(s1);

% Enviamos por puerto serial, el mensaje de inicio para el movimiento del Robot
message2send = 'start';
write(s1,message2send,'string'); % Enviamos por puerto serial

tic;
tocLast = 0;
timeEllapsed = 0;
time2wait = 8; % Tiempo de espera(seg) de para recibir datos por el puerto serial

% Esperamos hasta obtener datos en el puerto serial
while true
    % Leemos la data disponible del puerto serial
    if s1.NumBytesAvailable > 0
        % Se recibe el mensaje para iniciar la lectura de datos 
        % del puerto serial
        messageLength = read(s1, 1, "uint8");      
        messageReceived = read(s1, messageLength, "char");      
        fprintf('%s\n', messageReceived)  % Starting...
        
        % Verificar si el mensaje recibido es identico al pre-establecido
        % para empezar a obtener los valores de los vectores
        if strcmp(messageReceived, 'Starting...')
            % Numero de vectores que se espera obtener por puerto serial
            nVectors = read(s1, 1, "int32");

            % Iteración por cada uno de los vectores
            for i=1:nVectors
                % Obtención del nombre del vector
                messageLength = read(s1, 1, "uint8");
                nameVector = read(s1, messageLength, "char");   
    
                % Leer los valores si es el 'vectorQ'
                if strcmp(nameVector, 'vectorQ')
                    lengthColumn = read(s1, 1, "int32"); % Numeros de colmunas del vector
                    Q = zeros(3,lengthColumn);           % Para guardar "q"
                    dataReceived = read(s1, lengthColumn*3, "single"); % Lee los valores del vector
                    disp('vectorQ')
                    Q(1,:) = dataReceived(1:lengthColumn);                  %Q1
                    Q(2,:) = dataReceived(lengthColumn+1:2*lengthColumn);   %Q2
                    Q(3,:) = dataReceived(2*lengthColumn+1:3*lengthColumn); %Q3
                % Leer los valores si es el 'vectorVc'   
                elseif strcmp(nameVector, 'vectorVc')
                    lengthColumn = read(s1, 1, "int32"); % Numeros de colmunas del vector
                    VC = zeros(3,lengthColumn);           % Para guardar "q"
                    dataReceived = read(s1, lengthColumn*3, "single"); % Lee los valores del vector
                    disp('vectorVc')
                    VC(1,:) = dataReceived(1:lengthColumn);                  %Vc1
                    VC(2,:) = dataReceived(lengthColumn+1:2*lengthColumn);   %Vc2
                    VC(3,:) = dataReceived(2*lengthColumn+1:3*lengthColumn); %Vc3

                    % lengthColumn = read(s1, 1, "int32"); % Lee la longitud del vector
                    % dataC = read(s1, lengthColumn, "single"); % Lee los valores del vector
                    % disp('vectorC')
                    % for j=1:lengthColumn
                    %     fprintf('q%d: %0.3f\n',j, dataC(j));
                    % end            
                end
            end
            
            % Se recibe el mensaje para finalizar la lectura de datos 
            % del puerto serial            
            messageLength = read(s1, 1, "uint8");  
            message2finished = read(s1, messageLength, "char");

            % Verificar si el mensaje recibido es identico al pre-establecido
            % para terminar de leer los valores de los vectores
            if strcmp(message2finished, 'Finished...')
                disp('Todos los valores han sido obtenidos correctamente...')
                break; %Sale del 'while' al terminar de leer los vectores
            else
                delete(s1)
                error('Error al obtener los datos del puerto serial...')
            end
        else
            delete(s1)
            error('Error al obtener los datos del puerto serial...')
        end
    end

    % Calculamos el tiempo de espera transcurrido hasta recibir 
    % datos por el puerto serial
    tocNow = toc;
    timeEllapsed = timeEllapsed + (tocNow - tocLast);
    tocLast = tocNow;

    % Mensaje de error por no recibir datos en el puerto serial 
    % durante el tiempo de espera
    if timeEllapsed > time2wait
        delete(s1)
        error(['No se encontraron datos disponibles en el puerto serial\n'...
               'Tiempo de espera de %d segundos agotado...'],time2wait)
    end
end

% Eliminar el puerto serial al terminar de leer la data correctamente
delete(s1) 

% ___________ Gráficos _____________
% Objeto para gráficos
plotter = RobotVisualization(ROBOT_DATA);

% Grafica de las singularidades
plotter.drawSingularity(Q(2,:),Q(3,:))

% ___________ Cálculos de datos reales de ejecución _____________
processRealData

% Script con diversos gráficos
Graphics
