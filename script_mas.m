%cargamos los datos experimentales
load oscilador.data

%declaramos las variables necesarias para el analisis de los datos
pos=oscilador(:,1);
t=oscilador(:,2);
n=1;
m=1;
ciclos=[];
indices_ciclo=[];
ciclo_1=[];

%identifiacamos los indices de los vectores en los cuales 
%se completa un ciclo de la oscilaci�n, iniciando en el primer
%ciclo donde la funci�n posici�n crece

for i= 2:length(pos)-1
  if n==1
    if pos(i)==0.0 && pos(i)> pos(i-1)
      indices_ciclo(n)=i;
      n=n+1;
    endif
  elseif pos(i)==0.0
    indices_ciclo(n)=i;
    n=n+1;
  elseif pos(i)==0.01 && pos(i-1)~=0.0000 && pos(i+1)~=0.0000
    indices_ciclo(n)=i;
    n=n+1;
  endif  
endfor

%separamos los datos en vectores con los datos para cada ciclo de las oscilciones
cic_1=pos(indices_ciclo(1):indices_ciclo(3));
cic_2=pos(indices_ciclo(3):indices_ciclo(5));
cic_3=pos(indices_ciclo(5):indices_ciclo(7));
cic_4=pos(indices_ciclo(7):indices_ciclo(9));

%Hallamos los valores maximios de la amplitud en cada ciclo, segun los datos experimentales
maximos=[max(cic_1),max(cic_2),max(cic_3),max(cic_4)];

%Hallamos los valores promedio de amplitud, para la funcion ajustada
amplitud=mean(maximos)

%Hallamos los pareiodos de cada oscilacion.
periodo_1=t(indices_ciclo(3))-t(indices_ciclo(1));
periodo_2=t(indices_ciclo(5))-t(indices_ciclo(3));
periodo_3=t(indices_ciclo(7))-t(indices_ciclo(5));
periodo_4=t(indices_ciclo(9))-t(indices_ciclo(7));
periodos=[periodo_1,periodo_2,periodo_3,periodo_4];

%Hllamos el valor promedio de los ciclos, segun los datos experimentales
periodo=mean(periodos)
t_inicio_grafica=t(indices_ciclo(1));

%hallamos la frecuencia del sistema para la funci�n ajustada
frec=1/periodo

%Hallamos el angulo de desfasaje para la funci�n ajustada, evaluando la posicion en t=0
%Y(0)=0.1 m
fi=acos(0.1/amplitud)

%Calculamos el valor de W para la funci�n ajustada
w=2*pi*frec

%datos de la funcion ajustada, el angulo fi se resta dado que la funci�n
%se encuentra adelantada respecto de x=0
%definimos un vector para graf�car la funci�n ajstada
x=[0:0.01:6];

%definimos la funci�n que se ajusta a los datos experimentales, en base a referencias te�ricas
y=amplitud*cos(w*x-fi);

%definimos la funci�n para graficar la V(t), en base a referencias te�ricas 
vel_t=-w*amplitud*sin(w*x-fi);

%definimos la funci�n para graficar la A(t), en base a referencias te�ricas 
ac_t=-w^2*amplitud*cos(w*x-fi);

%velocidad y aceleracion maximas
V_max=w*amplitud
A_max=w^2*amplitud

%grafica datos experimentales
hold on
datos_exp=plot(t,pos,".");
title('Sistema oscilador armonico simple')

%grafica funci�n ajustada
func=plot(x,y);

%grafica velocidad en funcion del tiempo
plot(x,vel_t);

%grafica de adceleracion en funcion del teimpo 
plot(x,ac_t);

%Estetica de los graficos
%eje ox
x_eje=[0 6];
eje=plot(x_eje,0*x_eje);
legend('Datos experimentales',' Funci�n ajustada-> Y(t)','Vel(t)','A(t)')
ylabel('Y(t)=A.cos(wt+fi),         V(t)=-w.A.sen(wt+fi),        A(t)-w^2.A.cos(w*x-fi)')
xlabel('Tiempo en segundos')



set(datos_exp,'color','blue','LineWidth',1)
set(func,'color','red','LineWidth',1)
set(eje,'color','black')

