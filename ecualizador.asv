function varargout = ecualizador(varargin)
gui_Singleton = 1;
gui_State = struct('gui_Name',mfilename,'gui_Singleton',gui_Singleton,'gui_OpeningFcn',@ecualizador_OpeningFcn,'gui_OutputFcn',@ecualizador_OutputFcn,'gui_LayoutFcn',[] ,'gui_Callback',[]);
if nargin && ischar(varargin{1})
gui_State.gui_Callback =str2func(varargin{1});
end
if nargout
[varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
gui_mainfcn(gui_State, varargin{:});
end
function ecualizador_OpeningFcn(hObject,eventdata, handles, varargin)
handles.output = hObject;
guidata(hObject, handles);

function varargout = ecualizador_OutputFcn(hObject,eventdata, handles)
varargout{1} = handles.output;
function slider1_Callback(hObject,eventdata, handles)
valor=get(hObject,'Value');
set(handles.text2,'String',valor);
function slider1_CreateFcn(hObject,eventdata, handles)
if isequal(get(hObject,'BackgroundColor'),get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function slider7_Callback(hObject,eventdata,handles)
valor=get(hObject,'Value');
set(handles.text3,'String',valor);
function slider7_CreateFcn(hObject,eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function slider8_Callback(hObject,eventdata, handles)
valor=get(hObject,'Value');
set(handles.text4,'String',valor);
function slider8_CreateFcn(hObject,eventdata, handles)
if isequal(get(hObject,'BackgroundColor'),get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function slider9_Callback(hObject,eventdata, handles)
valor=get(hObject,'Value');
set(handles.text5,'String',valor);
function slider9_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject, 'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function slider10_Callback(hObject,eventdata, handles)
valor=get(hObject,'Value');
set(handles.text6,'String',valor);
function slider10_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function slider11_Callback(hObject, eventdata, handles)
valor=get(hObject,'Value');
set(handles.text7,'String',valor);
function slider11_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'),get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function edit1_Callback(hObject, eventdata, handles)
global factor
factor=get(hObject,'String');
function edit1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end
function edit2_Callback(hObject, eventdata, handles)
global caso
caso=get(hObject,'String');
function edit2_CreateFcn(hObject,eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
set(hObject,'BackgroundColor','white');
end
function Ecualizar_Callback(hObject,eventdata, handles)
global Y FS At
name_a="do.wav";
%%Codigo exterior
caso = get(handles.edit2,'String');
factor = get(handles.edit1,'String');
caso = str2double(caso);
factor = str2double(factor);
%% Se˜nal Original x[n]
%Informacion del audio original
info_x = audioinfo(name_a);
nsamples_x = info_x.TotalSamples;
srate_x = info_x.SampleRate;
x = audioread(name_a);
%x1 = x(:,1);
%x2 = x(:,2);
%x=x1;
NFFT_x=2^nextpow2(nsamples_x);
tx = 0:seconds(1/srate_x):seconds(info_x.Duration);
tx = tx(1:end-1);
figure(1)
subplot(3,1,1);
plot(tx,x);
title("Se˜nal de audio x(t)");
xlabel("Tiempo");
ylabel("Amplitud de onda");
grid on;
grid minor;
%%
figure(1)
subplot(3,1,2);
x_d=1:nsamples_x;
stem(x_d,transpose(x));
title("Se˜nal de audio x[n]");
xlabel("Tiempo");
ylabel("Amplitud de onda");
grid on;
grid minor;
%%
X = fft(x, NFFT_x)/nsamples_x;
frec_X = srate_x/2*linspace(0, 1,NFFT_x/2+1);
figure(1);
subplot(3,1,3); 
plot(frec_X,2*abs(X(1:NFFT_x/2+1)));
title("Espectro de frecuencias de x[n]");
xlabel("f [Hz] (Sin cambio de dominio)");
ylabel("Amplitud");
ax = gca;
%ax.XAxis.Exponent = 3;
grid on;
grid minor;
if caso == 0
%% Decimacion
factor_d=factor;
nsamples_y = nsamples_x/factor_d;
srate_y = info_x.SampleRate/factor_d;
y=[];
for i = 1:round(nsamples_y)
y(i)=x(i*factor_d);
end
y = transpose(y);
NFFT_y=2^nextpow2(nsamples_y);
ty = 0:seconds(1/srate_y): ...
seconds(info_x.Duration);
ty = ty(1:length(y));
audiowrite("decimada.wav", y, srate_y)
audiowrite("ayuda.wav", y, srate_y)
figure(2)
subplot(3,1,1);
plot(ty,y);
title("Se˜nal de audio y(t) - Decimada");
xlabel("Tiempo");
ylabel("Amplitud de onda");
grid on;
grid minor;
%%
figure(2)
subplot(3,1,2);
x_y=1:nsamples_y;
stem(x_y,transpose(y));
title("Se˜nal de audio y[n] - Decimada");
xlabel("Tiempo");
ylabel("Amplitud de onda");
grid on;
grid minor;
Y = fft(y, NFFT_y)/nsamples_y;
frec_Y = srate_x/2*linspace(0, 1,...
NFFT_x/2+1);
figure(2);
subplot(3,1,3);
plot(frec_Y(1:(NFFT_x/factor_d)),...
2*abs(Y(1:NFFT_x/factor_d)));
title("Espectro de frecuencias de y[n]");
xlabel("f [Hz] (sin cambio de dominio)");
ylabel("Amplitud");
ax = gca;
%ax.XAxis.Exponent = 3;
grid on;
grid minor;
elseif caso == 1
%%Expansi´on
factor_e=factor;
nsamples_y = nsamples_x*factor_e;
srate_y = info_x.SampleRate*factor_e;
y=[];
for i = 1:round(nsamples_y)
if mod(i,factor_e) == 0
y(i)=x(i/factor_e);
else
y(i)=0;
end
end
y = transpose(y);
NFFT_y=2^nextpow2(nsamples_y);
ty = 0:seconds(1/srate_y): seconds(info_x.Duration);
ty = ty(1:length(y));
audiowrite("expandida.wav", y, srate_y)
figure(2)
subplot(3,1,1);
plot(ty,y);
title("Se˜nal de audio y_e(t)");
xlabel("Tiempo");
ylabel("Amplitud de onda");
grid on;
grid minor;
%%
figure(2)
subplot(3,1,2);
x_y=1:nsamples_y;
stem(x_y,transpose(y));
title("Se˜nal de audio y_e[n]");
xlabel("Tiempo");
ylabel("Amplitud de onda");
grid on;
grid minor;
Y = fft(y)/nsamples_y;
%Y = fft(y, NFFT_y)/nsamples_y;
%frec_Y = srate_x/2* ...
linspace(0, 1, nsamples_y);
frec_Y = srate_x/2*linspace(0, 1, NFFT_x/2+1);
figure(2);
subplot(3,1,3);

plot(frec_Y,2*abs(Y(1:NFFT_x/2+1)));
title("Espectro de frecuencias de y_e[n]");
xlabel("f [Hz] (Sin cambio de dominio)");
ylabel("Amplitud");
ax = gca;
%ax.XAxis.Exponent = 3;
grid on;
grid minor;
% Filto Pasa Bajo Ideal
fpb = 1.*(abs(frec_X)<=(srate_x)/(2*factor_e));
% Grafica filtro pasa bajo
figure(3);
subplot(3,1,1);
plot(frec_X, fpb, 'r');
title("Filtro Pasa Bajos");
xlabel("Frecuencia [Hz]");
ylabel("Amplitud");
grid on;
grid minor;
ax = gca;
ax.XAxis.Exponent = 3;
% Filtering
for i=length(fpb)+1:length(Y)
fpb(i)=0;
end
Y0 = Y.* fpb';
figure(3);
subplot(3,1,2);
plot(frec_Y, 2*abs(Y0(1:NFFT_x/2+1)));
title("Se˜nal filtrada: Filtro Pasa Bajos * Espectro de la original");
xlabel("Frecuencia [Hz]");
ylabel("Amplitud");
grid on;
grid minor;
ax = gca;
ax.XAxis.Exponent = 3;
%%
x_re=ifft(X, nsamples_x);
y0=ifft(Y0, nsamples_y);
ty = 0:seconds(1/srate_y): seconds(info_x.Duration);
ty = ty(1:length(y));
audiowrite("interpolada.wav",real(y0/max(abs(y0))), srate_y)
audiowrite("ayuda.wav", real(y0/max(abs(y0))), srate_y)
figure(3)
subplot(3,1,3);

x_y=1:nsamples_y;
stem(x_y,transpose(y0));
title("Se˜nal de audio y[n] interpolada");
xlabel("Tiempo");
ylabel("Amplitud de onda");
grid on;
grid minor;
end
Y = audioread("ayuda.wav");
info_y = audioinfo("ayuda.wav");
FS = info_y.SampleRate;
%%
B1= get(handles.slider1,'value');
B2= get(handles.slider7,'value');
B3= get(handles.slider8,'value');
B4= get(handles.slider9,'value');
B5= get(handles.slider10,'value');
B6= get(handles.slider11,'value');
%conversion de db a ganacia
B1=10^(B1/20);
B2=10^(B2/20);
B3=10^(B3/20);
B4=10^(B4/20);
B5=10^(B5/20);
B6=10^(B6/20);
%%
%Transformada
T=length(Y);
FN=2^nextpow2(T);
AudioF=fft(Y,FN)/T;
%Vector tiempo para la se˜nal de audio y para dominio en frecuencia
t=1:T;
n=FS*linspace(0,1,FN/2+1);
%%Filtros
sub=subgrave;
Asub=B1*filter(sub,Y);
AsubF=fft(Asub,FN)/T;
Grave=FGrave;
AuGra=B2*filter(Grave,Y);
AuGraF=fft(AuGra,FN)/T;
Fbajas=FMediasBajas;
Abajas=B3*filter(Fbajas,Y);
AbajasF=fft(Abajas,FN)/T;
MediasAltas=FMediasAltas;
Aalta=B4*filter(MediasAltas,Y);
AaltaF=fft(Aalta,FN)/T;


pre=presencia;
Apre=B5*filter(pre,Y);
ApreF=fft(Apre,FN)/T;
Brillo=FBrillo;
AuBrillo=B6*filter(Brillo,Y);
AuBrilloF=fft(AuBrillo,FN)/T;
%%
axes(handles.axes3);
plot(t,Asub,'blue');
axes(handles.axes4);
plot(n,2*abs(AsubF(1:FN/2+1)),'red');
axes(handles.axes5);
plot(t,AuGra,'blue');
axes(handles.axes6);
plot(n,2*abs(AuGraF(1:FN/2+1)),'red');
axes(handles.axes7);
plot(t,Abajas,'blue');
axes(handles.axes8);
plot(n,2*abs(AbajasF(1:FN/2+1)),'red');
axes(handles.axes9);
plot(t,Aalta,'blue');
axes(handles.axes10);
plot(n,2*abs(AaltaF(1:FN/2+1)),'red');
axes(handles.axes11);
plot(t,Apre,'blue');
axes(handles.axes12);
plot(n,2*abs(ApreF(1:FN/2+1)),'red');
axes(handles.axes13);
plot(t,AuBrillo,'blue');
axes(handles.axes14);
plot(n,2*abs(AuBrilloF(1:FN/2+1)),'red');
%%
At=Asub+AuGra+Abajas+Aalta+Apre+AuBrillo;
AtF=AsubF+AuGraF+AbajasF+AaltaF+ApreF+AuBrilloF;
AUXF=audioplayer(At,FS);
ty = 0:seconds(1/FS):seconds(info_y.Duration);
ty = ty(1:end-1);
audiowrite("ecualizada.wav", At, FS)
figure(3+caso)
subplot(2,1,1)
plot(ty, real(At))
title("z(t)");
xlabel("Tiempo");
ylabel("Amplitud");
ax = gca;
%ax.XAxis.Exponent = 3;
grid on;
grid minor;
X = fft(x)/nsamples_y;
frec_Y = srate_x/2*linspace(0, 1, NFFT_x/2+1);
figure(3+caso)
subplot(2,1,2)
plot(frec_Y,2*abs(X(1:NFFT_x/2+1)));
title("Espectro de frecuencias de z(t)");
xlabel("f [Hz] (Sin cambio de dominio)");
ylabel("Amplitud");
ax = gca;
%ax.XAxis.Exponent = 3;
grid on;
grid minor;