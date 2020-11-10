%% Importar archivos de .txt a .mat
close all
clc
clear variables
tic
%%
% Se registran los ensayos grabados
Origen = pwd;
cd('bags')
archivos = dir(fullfile(pwd,'*-Ensayo*'));
archivos_reg = strings(length(archivos),1);
for k=1:length(archivos)
    archivos_reg(k,1) = cellstr(archivos(k).name);
    aux = split(archivos_reg(k,1),'.');
    archivos_reg(k,1) = aux(1);
end
cd ..
cd('matlab_data')
archivos = dir(fullfile(pwd,'*-Ensayo*'));
archivos_matlab = strings(length(archivos),1);
for k=1:length(archivos)
    archivos_matlab(k,1) = cellstr(archivos(k).name);
end
cd ..
%%
cd('Simulation')
% Se determinan las carpetas en el nivel de clasificación por finalidad
nivel0 = dir(fullfile(pwd,'/*-Ensayo*'));
N1 = length(nivel0(1:end));
carpeta_subnivel = strings(N1,1);
for i=1:N1
    carpeta_subnivel(i,1) = cellstr(nivel0(i).name);
end
for i = 1:N1
    cd(carpeta_subnivel{i,1})
    % Las carpetas con topics que se puedan leer en Matlab tienen el
    % strig característico '*Ensayo*'
    fprintf('\nProcesando %s.\n',pwd);
    Ensayos = dir(fullfile(pwd,'*Ensayo*'));
    [w0,w1]=size(Ensayos);
    if w0~=0
        Datos = strings(w0,1);
        for j=1:w0
            Datos(j,1) = cellstr(Ensayos(j).name);
        end
        save SREG.mat
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        Destino = pwd;
        x = strfind(Destino,'\')'; % Usar '\' en windows y '/' en Mac
        titulo = [Destino(x(end)+1:end),'.mat'];
        % Se comprueba que el ensayo no se haya importado
        % anteriormente
        % Si no se ha registrado el ensayo con anterioridad, se
        % realiza el if
        aux = strfind(archivos_matlab,titulo);
        if isempty(aux)
            %%%%%%%%%%%%%%%%%%%
            %%%%% Topics %%%%%%
            %%%%%%%%%%%%%%%%%%%
            fprintf('\tProcesando %s...\n',titulo);
            %+----------------------+
            %|    Cf2_cmd_mspeed    |
            %+----------------------+
            aux = strcat(Destino(x(end)+1:end),'_Cf2_cmd_mspeed.txt');
            if exist(aux,'file')
                fprintf('\t\t1. Cf2_cmd_mspeed ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    Cf2_cmd_mspeed = A.data;
                end
            end
            %+------------------------+
            %|   Cf2_gaz_cmd_mspeed   |
            %+------------------------+
            aux = strcat(Destino(x(end)+1:end),'_Cf2_gaz_cmd_mspeed.txt');
            if exist(aux,'file')
                fprintf('\t\t2. Cf2_gaz_cmd_mspeed ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    Cf2_gaz_cmd_mspeed = A.data;
                end
            end
            %+----------------+
            %|   Cf2_gt_imu   |
            %+----------------+
            aux = strcat(Destino(x(end)+1:end),'_Cf2_gt_imu.txt');
            if exist(aux,'file')
                fprintf('\t\t3. Cf2_gt_imu ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    Cf2_gt_imu = A.data;
                end
            end
            %+-----------------+
            %|   Cf2_gt_odom   |
            %+-----------------+
            aux = strcat(Destino(x(end)+1:end),'_Cf2_gt_odom.txt');
            if exist(aux,'file')
                fprintf('\t\t4. Cf2_gt_odom ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    Cf2_gt_odom = A.data;
                end
            end
            %+----------------+
            %|   Cf2_gt_pos   |
            %+----------------+
            aux = strcat(Destino(x(end)+1:end),'_Cf2_gt_pos.txt');
            if exist(aux,'file')
                fprintf('\t\t5. Cf2_gaz_cmd_mspeed ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    Cf2_gt_pos = A.data;
                end
            end
            %+-----------------+
            %|   Cf2_gt_pose   |
            %+-----------------+
            aux = strcat(Destino(x(end)+1:end),'_Cf2_gt_pose.txt');
            if exist(aux,'file')
                fprintf('\t\t6. Cf2_gt_pose ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    Cf2_gt_pose = A.data;
                end
            end
            %+--------------------+
            %|   Cf2_gt_posecov   |
            %+--------------------+
            aux = strcat(Destino(x(end)+1:end),'_Cf2_gt_posecov.txt');
            if exist(aux,'file')
                fprintf('\t\t7. Cf2_gt_posecov ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    Cf2_gt_posecov = A.data;
                end
            end
            %+-------------------+
            %|   Cf2_gt_transf   |
            %+-------------------+
            aux = strcat(Destino(x(end)+1:end),'_Cf2_gt_transf.txt');
            if exist(aux,'file')
                fprintf('\t\t8. Cf2_gt_transf ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    Cf2_gt_transf = A.data;
                end
            end
            %+------------+
            %|   Cf2_js   |
            %+------------+
%             aux = strcat(Destino(x(end)+1:end),'_Cf2_js.txt');
%             if exist(aux,'file')
%                 fprintf('\t\t9. Cf2_js ...\n');
%                 A = importdata(aux);
%                 if ~isempty(A)
%                     Cf2_js = A.data;
%                 end
%             end
            %+----------------+
            %|   Cf2_mspeed   |
            %+----------------+
            aux = strcat(Destino(x(end)+1:end),'_Cf2_mspeed.txt');
            if exist(aux,'file')
                fprintf('\t\t10. Cf2_mspeed ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    Cf2_mspeed = A.data;
                end
            end
            %+--------------+
            %|   Cf2_odom   |
            %+--------------+
            aux = strcat(Destino(x(end)+1:end),'_Cf2_odom.txt');
            if exist(aux,'file')
                fprintf('\t\t11. Cf2_odom ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    Cf2_odom = A.data;
                end
            end
            %+---------------------+
            %|   Cf2_odoms1_odom   |
            %+---------------------+
            aux = strcat(Destino(x(end)+1:end),'_Cf2_odoms1_odom.txt');
            if exist(aux,'file')
                fprintf('\t\t12. Cf2_odoms1_odom ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    Cf2_odoms1_odom = A.data;
                end
            end
            %+--------------------+
            %|   Cf2_odoms1_pos   |
            %+--------------------+
            aux = strcat(Destino(x(end)+1:end),'_Cf2_odoms1_pos.txt');
            if exist(aux,'file')
                fprintf('\t\t13. Cf2_odoms1_pos ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    Cf2_odoms1_pos = A.data;
                end
            end
            %+---------------------+
            %|   Cf2_odoms1_pose   |
            %+---------------------+
            aux = strcat(Destino(x(end)+1:end),'_Cf2_odoms1_pose.txt');
            if exist(aux,'file')
                fprintf('\t\t14. Cf2_odoms1_pose ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    Cf2_odoms1_pose = A.data;
                end
            end
            %+------------------------+
            %|   Cf2_odoms1_posecov   |
            %+------------------------+
            aux = strcat(Destino(x(end)+1:end),'_Cf2_odoms1_posecov.txt');
            if exist(aux,'file')
                fprintf('\t\t15. Cf2_odoms1_posecov ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    Cf2_odoms1_posecov = A.data;
                end
            end
            %+-----------------------+
            %|   Cf2_odoms1_transf   |
            %+-----------------------+
            aux = strcat(Destino(x(end)+1:end),'_Cf2_odoms1_transf.txt');
            if exist(aux,'file')
                fprintf('\t\t16. Cf2_odoms1_transf ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    Cf2_odoms1_transf = A.data;
                end
            end
            %+-------------+
            %|   Cf2_pos   |
            %+-------------+
            aux = strcat(Destino(x(end)+1:end),'_Cf2_pos.txt');
            if exist(aux,'file')
                fprintf('\t\t17. Cf2_pos ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    Cf2_pos = A.data;
                end
            end
            %+--------------+
            %|   Cf2_pose   |
            %+--------------+
            aux = strcat(Destino(x(end)+1:end),'_Cf2_pose.txt');
            if exist(aux,'file')
                fprintf('\t\t18. Cf2_pose ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    Cf2_pose = A.data;
                end
            end
            %+-----------------+
            %|   Cf2_posecov   |
            %+-----------------+
            aux = strcat(Destino(x(end)+1:end),'_Cf2_posecov.txt');
            if exist(aux,'file')
                fprintf('\t\t19. Cf2_posecov ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    Cf2_posecov = A.data;
                end
            end
            %+-------------+
            %|   Cf2_rpy   |
            %+-------------+
            aux = strcat(Destino(x(end)+1:end),'_Cf2_rpy.txt');
            if exist(aux,'file')
                fprintf('\t\t20. Cf2_rpy ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    Cf2_rpy = A.data;
                end
            end
            %+----------------+
            %|   Cf2_transf   |
            %+----------------+
            aux = strcat(Destino(x(end)+1:end),'_Cf2_transf.txt');
            if exist(aux,'file')
                fprintf('\t\t21. Cf2_transf ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    Cf2_transf = A.data;
                end
            end
            %+-----------+
            %|   clock   |
            %+-----------+
            aux = strcat(Destino(x(end)+1:end),'_clock.txt');
            if exist(aux,'file')
                fprintf('\t\t22. Clock ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    clock = A.data;
                end
            end
            %+-----------------+
            %|   gaz_lstates   |
            %+-----------------+
            aux = strcat(Destino(x(end)+1:end),'_gaz_lstates.txt');
            if exist(aux,'file')
                fprintf('\t\t23. gaz_lstates ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    gaz_lstates = A.data;
                end
            end
            %+-----------------+
            %|   gaz_mstates   |
            %+-----------------+
            aux = strcat(Destino(x(end)+1:end),'_gaz_mstates.txt');
            if exist(aux,'file')
                fprintf('\t\t24. gaz_mstates ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    gaz_mstates = A.data;
                end
            end
            %+---------------+
            %|   gaz_param   |
            %+---------------+
            aux = strcat(Destino(x(end)+1:end),'_gaz_param.txt');
            if exist(aux,'file')
                fprintf('\t\t25. gaz_param ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    gaz_param = A.data;
                end
            end
            %+---------------------+
            %|   gaz_paramupdate   |
            %+---------------------+
            aux = strcat(Destino(x(end)+1:end),'_gaz_paramupdate.txt');
            if exist(aux,'file')
                fprintf('\t\t26. gaz_paramupdate ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    gaz_paramupdate = A.data;
                end
            end
            %+--------+
            %|   tf   |
            %+--------+
            aux = strcat(Destino(x(end)+1:end),'_tf.txt');
            if exist(aux,'file')
                fprintf('\t\t27. tf ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    tf = A.data;
                end
            end
            %+---------------+
            %|   tf_static   |
            %+---------------+
            aux = strcat(Destino(x(end)+1:end),'_tf_static.txt');
            if exist(aux,'file')
                fprintf('\t\t28. tf_static ...\n');
                A = importdata(aux);
                if ~isempty(A)
                    tf_static = A.data;
                end
            end

            cd(Origen);
            clearvars -except -regexp ^Cf2_ ^clock ^gaz_ ^tf ^tf_ ...
                ^Destino ^titulo

            cd('matlab_data')
            save(titulo)
            cd(Destino);
            fprintf('\tRegistrado \n');
        else
            fprintf('\t%s ya está registrado\n',titulo);
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        clear variables
        load SREG.mat
        delete SREG.mat
    else
        fprintf('\tNo contiene datos de ensayos para matlab\n');
    end
    cd ..
end
cd ..
    
clear variables