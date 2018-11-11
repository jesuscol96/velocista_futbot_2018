%{  
  FUTBOT 2018
  Proyecto: Velocista.

  Prueba del algoritmo de deteccion de linea
 
    Este algoritmo permite leer un conjunto de puntos en el plano, 
  los cuales con calificados entre 0 y un valor máximo indicando la
  probabilidad de pertenecer a la funcion que los describe (una recta).
  El objetivo es filtrar el ruido, permitiendo enfocar la lectura en los
  valores relevantes.
%}

clear;
max=255;  %valor maximo
max_noise=30; %ruido maximo
n=3;  %orden de la diferenciacion por probabilidad
noise=fix(max_noise*rand(7,7));

M =[
   0   0   0   0   0   0   0
   0   0   0   0   0   0   0
   max   0   0   0   0   0   0
   0   max   0   0   0   0   0  
   0   0   max   0   0   0   0
   0   0   0   max   0   0   0
   0   0   0   0   max   0   0
   ];
   
 M=M+noise;
 M(find(M>255))=255;
 
 %Se asume el origen en la esquina inferior izquierda
 %El primer valor es (1,1)
 
  M_vector=fliplr(reshape(fliplr(M)',1,49))';  
  
 %implementacion del algoritmo a la matrix M  

  p=(M_vector ./ 255);  %probabilidad de ser un punto para las muestras 
 
 X= [ones(49,1)  repmat([1:7]',7,1)] .* (p.^(0.5*n));
 d= reshape(ones(7,1) .* [1:7],49,1) .* (p.^(0.5*n));

 W=((X'*X)^-1)*X'*d;  %proyecciones
 
 %calculo del error:
 error=d-X*W;
 error=error .* error;
 error=sum(0.5*error .* p);
 
 %resultados
 error
 b=W(1)
 m=W(2)
 angulo=(180/pi)*atan(m)
 promedio=mean(mean(M))
   