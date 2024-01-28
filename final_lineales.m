clc;
clear all;
close all;

%% Señal Original x[n]
name_a="counting_stars_cut.wav";
%Informacion del audio original

info_x = audioinfo(name_a)


[audio, fs] = audioread(name_a);

%%
audio = 0.5*(audio(:, 1) + audio(:, 2)).';  %%Audio estereo a mono

duracion_audio = length(audio) / fs;   %%duración

n_samples = length(audio);  %%num_muestras


%%
vec_xt = 0:seconds(1/fs): seconds(duracion_audio);
vec_xt = vec_xt(1:end-1);

figure(1)
subplot(3,1,1);
plot(vec_xt,transpose(audio));
title("Audio x(t)");
xlabel("Tiempo");
ylabel("Amplitud");
grid on;
grid minor;

figure(1)
subplot(3,1,2);
vec_nx = 1:n_samples;
stem(vec_nx, audio);
title("Audio x[n]");
xlabel("Tiempo (Solo un Intervalo de muestras)");
%xlim([500000 500100])
ylabel("Amplitud");
grid on;
grid minor;


%% Transformada de Fourier del audio original
X = fft(audio, n_samples)/n_samples;  
Xi = fftshift(X);

frec_X = linspace(-fs/2, fs/2, length(X));

figure(1);
subplot(3,1,3);
plot(frec_X, 2*abs(Xi));
title("Espectro de frecuencias de x[n]: $X(e^{j\omega})$", 'Interpreter', 'latex');
xlabel("frecuencia [Hz]");
ylabel("Amplitud");

grid on;
grid minor;

%% Decimación

M = 10;
% Filtro Anti-aliasing
Hdec_LPF = abs(frec_X)<= (fs/(2*M));

% Filtrado para decimación
X_filtered = (Xi .* Hdec_LPF);

x_filt = ifft(X_filtered*n_samples); 

%x_filt = real(x_filt);

% Operacion de decimación
f_dec = fs / M;
muestras_dec = n_samples / M;

y_dec = zeros;
for n = 1:muestras_dec
    y_dec(n) = (x_filt(n*M));
end

%%%FFT de la señal diezmada
Y_dec = fft(y_dec)/round(muestras_dec);
Yi_dec = fftshift(Y_dec);

frecY_dec = linspace(-fs/2, fs/2, muestras_dec);


%%% Graficas de decimación

figure(2);
subplot(4,1,1);
plot(frec_X, abs(X_filtered)/max(abs(X_filtered)));
hold on;
plot(frec_X, Hdec_LPF,'r');
title("Espectro (normalizado) de X por el filtro anti aliasing $X(e^{j\omega}) . H(e^{j\omega})$", 'Interpreter', 'latex');
xlabel("Frecuencia [Hz]");
ylabel("Amplitud");
ylim([-0.1 1.1])
grid on; grid minor;
ax = gca;

tdec = 0:seconds(1/f_dec): seconds(duracion_audio);
tdec = tdec(1:end-1);
figure(2)
subplot(4,1,2);
plot(tdec, real(y_dec*(-1)));
title("Señal x(t) diezmada");
xlabel("Tiempo");
ylabel("Amplitud");
grid on;
grid minor;

figure(2)
subplot(4,1,3);
nx_dec = 1:muestras_dec;
stem(nx_dec, real(y_dec*(-1)));
title("Señal x[n] diezmada");
xlabel("Tiempo (Solo un Intervalo de muestras)");
%xlim([500000 500100]);
ylabel("Amplitud");
grid on;
grid minor;

figure(2);
subplot(4,1,4);
plot(frecY_dec, 2*abs(Yi_dec));
title("Espectro de frecuencias de la señal diezmada");
xlabel("frecuencia [Hz]");
ylabel("Amplitud");

%%%%%%

%% Expansión / Interpolación 

L = 3;  %%factor de interpolación 

f_interp = fs*L;
muestras_interp = n_samples*L;

%%% Operación de interpolación 

y_expand = zeros;

for n = 1:muestras_interp   
    if mod(n,L) == 1
        y_expand(n) = 0;
    else
        y_expand(n) = audio(round(n/L));
    end
end



%%% FFT de la señal expandida y filtrado
Y_expand = fft(y_expand, muestras_interp)/muestras_interp;  

Yi_expand = fftshift(Y_expand);

frecY_expand = linspace(-fs/2, fs/2, muestras_interp);

%%% Filtro de interpolación
Hexp_LPF = abs(frecY_expand) <= (fs/L)*(0.5);

Yexp_filt = Yi_expand .* Hexp_LPF;



figure(3);
subplot(4,1,1);
plot(frecY_expand, 2*abs(Yi_expand));
title("Espectro de frecuencias la señal expandida");
xlabel("frecuencia [Hz]");
ylabel("Amplitud");

figure(3);
subplot(4,1,2);
plot(frecY_expand, Hexp_LPF, 'r');
hold on;
plot(frecY_expand, abs(Yexp_filt)/max(abs(Yexp_filt)), 'b');
title("Espectro de la señal expandida y filtrada");
xlabel("frecuencia [Hz]");
ylabel("Amplitud");


%% extrapolacion en el tiempo;
yexp_filtered = ifft(fftshift(Yexp_filt*muestras_interp));

texp = 0:seconds(1/f_interp): seconds(duracion_audio);
texp = texp(1:length(yexp_filtered));

figure(3)
subplot(4,1,3);
plot(texp, real(yexp_filtered));
title("Señal x(t) Expandida");
xlabel("Tiempo");
ylabel("Amplitud");
grid on;
grid minor;

figure(3)
subplot(4,1,4);
ny_exp = 1:muestras_interp;
stem(ny_exp, real(yexp_filtered));
title("Señal x[n] expandida");
xlabel("Tiempo (Solo un Intervalo de muestras)");
%xlim([500000 500100]);
ylabel("Amplitud");
grid on;
grid minor;

%% Ecualizadores 

%% Se escoge que datos salen decimacion o expansion para ecualizar
%%Salida de las FFT de la señales expandidas o decimadas
% Y_in = Yexp_filt;  %%interpolacion
Y_in = Yi_dec;     %%decimación

%%Frecuencias de sampleo de la decimación o interpolacion
% f_in = f_interp; %%interpolacion
f_in = f_dec;      %%decimación

%%Rango de frecuencias de la decimación o interpoladas
% frec_Y = frecY_expand; %%interpolacion
frec_Y = frecY_dec;     %%decimación


%%% abAjo va la ecualización
%% Ganancias  que salen de la app
G1 =2;  %%
G2 =0;
G3 =0;
G4 =10;
G5 =0;
G6 =10;

%%Sub-Bass
Sub = ((abs(frec_Y)>=16).*(abs(frec_Y)<=60));
Y_sub = G1*(Y_in.* Sub);
%%Bass
Bass = ((abs(frec_Y)>=60).*(abs(frec_Y)<=250));
Y_bass = G2*(Y_in.* Bass);
%%Low Mid
L_Mid = ((abs(frec_Y)>=250).*(abs(frec_Y)<=2e3));
Y_Lmid = G3*(Y_in.* L_Mid);
%%High Mid
H_Mid = ((abs(frec_Y)>=2e3).*(abs(frec_Y)<=4e3));
Y_Hmid = G4*(Y_in.* H_Mid);
%%Presence
Pres = ((abs(frec_Y)>=4e3).*(abs(frec_Y)<=6e3));
Y_pres = G5*(Y_in.* Pres);
%Brrilliance 
Brillo = ((abs(frec_Y)>=6e3).*(abs(frec_Y)<=16e3));
Y_brillo = G6*(Y_in.* Brillo);

Z_EQ = Y_sub + Y_bass + Y_Lmid + Y_Hmid + Y_pres + Y_brillo;


figure(4);
subplot(4,1,2);
plot(frec_Y, 2*abs(Z_EQ) , 'b');
title("Espectro de la señal expandida y filtrada");
xlabel("frecuencia [Hz]");
ylabel("Amplitud");
%ylim([-0.5 1.5])

 sound(real(yexp_filtered),f_interp); % Audio filtrado (LPF)
 pause( duracion_audio + 1 );
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


