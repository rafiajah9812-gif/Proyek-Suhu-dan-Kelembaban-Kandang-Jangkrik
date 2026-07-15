% === SETUP SERIAL ===
port = 'COM6';  % Ganti sesuai port Arduino
baud = 115200;
s = serial(port,'BaudRate',baud,'Terminator','LF');
fopen(s);

disp('=== Auto Tuning Ziegler-Nichols (Open Loop) ===');
disp('Data akan muncul di Command Window...');

% --- Variabel ---
time_data = [];
temp_data = [];
power_data = [];
powerLevel = 70;       % Daya awal
stepDone = false;      % Flag supaya step hanya sekali
stepTime = 0;          % Waktu ketika step dilakukan

% --- Plot awal ---
figure; hold on;
hTemp = plot(NaN,NaN,'b','LineWidth',1.5);
hPower = plot(NaN,NaN,'r--','LineWidth',1.5);
legend('Suhu','Daya Heater','Location','best');
xlabel('Waktu (s)');
ylabel('Suhu (C) / Daya (%)');
title('Proses Auto Tuning Ziegler-Nichols Open Loop');
grid on;

% Kirim daya awal ke Arduino
fprintf(s, '%.1f\n', powerLevel);

startTime = tic;
while toc(startTime) < 1800  % Maks 10 menit
    if s.BytesAvailable > 0
        dataLine = fgets(s);  % Baca baris data
        dataVals = str2num(dataLine); %#ok<ST2NM>
        if length(dataVals) == 3
            t = dataVals(1);
            suhu = dataVals(2);
            daya = dataVals(3);
            
            % Simpan data
            time_data = [time_data; t];
            temp_data = [temp_data; suhu];
            power_data = [power_data; daya];
            
            % Smoothing
            if length(temp_data) > 5
                suhu = mean(temp_data(end-4:end));
            end
            
            % Tampilkan di command window
            fprintf('Waktu: %.1f s | Suhu: %.2f C | Daya: %.1f %%\n',t,suhu,daya);
            
            % Update grafik
            set(hTemp,'XData',time_data,'YData',temp_data);
            set(hPower,'XData',time_data,'YData',power_data);
            drawnow;
            
            % Hanya sekali step: dari 20 ? 70%
            if ~stepDone && t > 60  % Step setelah 30 detik
                powerLevel = 70;
                fprintf(s,'%.1f\n',powerLevel);
                stepTime = t;
                stepDone = true;
            end
        end
    end
end

% Tutup serial
fclose(s);
delete(s);
clear s;

% === Analisis Open Loop Ziegler-Nichols ===
if stepDone && length(temp_data) > 10
    % Cari L dan T
    y0 = temp_data(find(time_data>=stepTime,1));  % Suhu awal
    y_end = temp_data(end);                       % Suhu akhir
    deltaY = y_end - y0;
    
    % Titik 63.2% dari respon
    y63 = y0 + 0.632*deltaY;
    t63_idx = find(temp_data>=y63,1);
    if ~isempty(t63_idx)
        T = time_data(t63_idx) - stepTime;  % Time constant
    else
        T = NaN;
    end
    
    % Dead time: titik awal respon
    L_idx = find(temp_data>=y0+0.05*deltaY,1); % 5% kenaikan awal
    if ~isempty(L_idx)
        L = time_data(L_idx) - stepTime;
    else
        L = NaN;
    end
    
    fprintf('\n=== HASIL OPEN LOOP ZIEGLER-NICHOLS ===\n');
    fprintf('L = %.3f s | T = %.3f s\n',L,T);
    
    if ~isnan(L) && ~isnan(T) && L>0 && T>0
        % Hitung Kp, Ki, Kd
        Kp = 1.2 * (T/L);
        Ti = 2*L;
        Td = 0.5*L;
        Ki = Kp/Ti;
        Kd = Kp*Td;
        fprintf('Kp = %.3f | Ki = %.3f | Kd = %.3f\n',Kp,Ki,Kd);
    else
        disp('Data tidak cukup untuk hitung parameter PID.');
    end
end