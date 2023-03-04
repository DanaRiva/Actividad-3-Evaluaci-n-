# Actividad-3-Evaluacion

## Robot cilíndrico 

Antes de empezar con la programación de este robot, teníamos que hacer las rotaciones pertinentes en el plano, consideramos un plano original que se debe transformar dejando ambos planos en Z perpendiculares haciendo las rotaciones necesarias, en este caso los planos ya eran perpendiculares por lo que solo se tomó en cuenta el movimiento de la articulación base, que sería representado como una rotación de -90 grados en el plano de Z por el sentido del movimiento, por lo que la matriz de movimiento en Z se sustituye con -90 regresando la matriz que usaremos para modelar el movimiento correspondiente.

MovZ=[1 0 0; 0 0 1; 0 -1 0]

Creamos las variables simbólicas, el vector de configuración del robot donde se representan las articulaciones y el tipo de movimiento que realizan, finalmente se realizan las primeras derivadas con respecto al tiempo para obtener las velocidades generales

``` matlab
syms th1(t) l1(t) l2(t) l3(t) t 
RP=[0 1 1];
Q= [th1, l2, l3];
Qp= diff(Q, t);
```
Generamos los vectores de posición y rotación de cada articulación, en la primera al ser un movimiento angular se utilizan las funciones de la matriz del movimiento en el plano que le corresponde, en la segunda articulación se utiliza la matriz que obtuvimos antes de empezar ya que considera la rotación del plano para que coincidan los planos y el movimiento que se genera en la primera articulación, en la tercera se utiliza una matriz de identidad ya que solo varía el largo que tiene, en todas se considera solo el largo en un plano para representar la posición y las variaciones que existen en este.

``` matlab
P(:,:,1)= [0; 0;l1];
R(:,:,1)= [cos(th1) -sin(th1)  0;
           sin(th1)  cos(th1)  0;
           0         0         1];

P(:,:,2)= [0; 0;l2];
R(:,:,2)= [1 0 0;
           0 0 1;
           0 -1 0];
           
P(:,:,3)= [0; l3;0];
R(:,:,3)= [1 0 0;
           0 1 0;
           0 0 1];
```

Creamos las matrices de transformación e iniciamos las posiciones

``` matlab
A(:,:,GDL)=simplify([R(:,:,GDL) P(:,:,GDL); Vector_Zeros 1]);
T(:,:,GDL)=simplify([R(:,:,GDL) P(:,:,GDL); Vector_Zeros 1]);
PO(:,:,GDL)= P(:,:,GDL); 
RO(:,:,GDL)= R(:,:,GDL); 
```

Creamos el vector jacobiano usando derivadas parciales en cada plano para cada articulación con respecto al largo o al ángulo según sea el tipo de movimiento que realiza cada una

``` matlab
Jv11= functionalDerivative(PO(1,1,GDL), th1);
Jv12= functionalDerivative(PO(1,1,GDL), l2);
Jv13= functionalDerivative(PO(1,1,GDL), l3);

Jv21= functionalDerivative(PO(2,1,GDL), th1);
Jv22= functionalDerivative(PO(2,1,GDL), l2);
Jv23= functionalDerivative(PO(2,1,GDL), l3);

Jv31= functionalDerivative(PO(3,1,GDL), th1);
Jv32= functionalDerivative(PO(3,1,GDL), l2);
Jv33= functionalDerivative(PO(3,1,GDL), l3);

jv_d=simplify([Jv11 Jv12 Jv13;
              Jv21 Jv22 Jv23;
              Jv31 Jv32 Jv33]);
``` 

Finalmente se realiza el algoritmo para obtener cada una de las velocidades angulares y lineales y el código nos regresa los vectores con las velocidades 

### Resultados:
![image](https://user-images.githubusercontent.com/100874942/222858417-b4cb230c-e6cc-4f09-a2cc-192fc0c76733.png)


## Robot angular 

Se realiza el análisis de los planos considerando el tipo de movimiento y la posición de los planos "objetivo", en este caso tuvimos que realizar una rotación en el plano Y de -90 grados para dejar paralelos los planos Z y también se consideró el movimiento angular del robot por lo que la matriz que modela el movimiento en las tres articulaciones, ya que no existe una variación en los planos en las otras articulaciones, además de tener el mismo tipo de movimiento y dirección del mismo, esta matriz se obtiene como resultado de la multiplicación de la matriz de movimiento en Y evaluada en -90 grados para dejar los planos paralelos y la matriz de movimiento en Z evaluada en -90 por la dirección del movimiento en ese plano.
Mov=[0 0 -1; -1 0 0; 0 1 0]

Creamos las variables simbólicas, el vector de configuración del robot donde se representan las articulaciones y el tipo de movimiento que realizan, finalmente se realizan las primeras derivadas con respecto al tiempo para obtener las velocidades generales
En este caso se consideran los ángulos para las tres articulaciones y las distancias quedan de manera simbólica ya que el cambio que se pueda generar respecto a estas es mínimo

``` matlab
syms th1(t) th2(t) th3(t) t l1 l2 l3
RP=[0 0 0];
Q= [th1, th2, th3];
Qp= diff(Q, t);
```
Generamos los vectores de posición y rotación de cada articulación, en la primera al ser un movimiento angular se utilizan las funciones de la matriz del movimiento en el plano que le corresponde, en la segunda articulación se utiliza la matriz que obtuvimos antes de empezar ya que considera la rotación del plano para que coincidan los planos y el movimiento que se genera en la primera articulación, en la tercera se utiliza una matriz de identidad ya que solo varía el largo que tiene, en todas se considera solo el largo en un plano para representar la posición y las variaciones que existen en este.

``` matlab
P(:,:,1)= [0; 0;l1];
R(:,:,1)= [0 0 -1;
          -1 0 0;
           0 1 0];

P(:,:,2)= [0; 0;l2];
R(:,:,2)= [0 0 -1;
          -1 0 0;
           0 1 0];

P(:,:,3)= [0; 0;l3];
R(:,:,3)= [0 0 -1;
          -1 0 0;
           0 1 0];

```

Creamos las matrices de transformación e iniciamos las posiciones

``` matlab
A(:,:,GDL)=simplify([R(:,:,GDL) P(:,:,GDL); Vector_Zeros 1]);
T(:,:,GDL)=simplify([R(:,:,GDL) P(:,:,GDL); Vector_Zeros 1]);
PO(:,:,GDL)= P(:,:,GDL); 
RO(:,:,GDL)= R(:,:,GDL); 
```

Creamos el vector jacobiano usando derivadas parciales en cada plano para cada articulación con respecto al largo o al ángulo según sea el tipo de movimiento que realiza cada una

``` matlab
Jv11= functionalDerivative(PO(1,1,GDL), th1);
Jv12= functionalDerivative(PO(1,1,GDL), th2);
Jv13= functionalDerivative(PO(1,1,GDL), th3);

Jv21= functionalDerivative(PO(2,1,GDL), th1);
Jv22= functionalDerivative(PO(2,1,GDL), th2);
Jv23= functionalDerivative(PO(2,1,GDL), th3);

Jv31= functionalDerivative(PO(3,1,GDL), th1);
Jv32= functionalDerivative(PO(3,1,GDL), th2);
Jv33= functionalDerivative(PO(3,1,GDL), th3);

jv_d=simplify([Jv11 Jv12 Jv13;
              Jv21 Jv22 Jv23;
              Jv31 Jv32 Jv33]);
``` 

Finalmente se realiza el algoritmo para obtener cada una de las velocidades angulares y lineales y el código nos regresa los vectores con las velocidades 

### Resultados:
![image](https://user-images.githubusercontent.com/100874942/222860164-5fe604db-5da2-4f39-9efc-94c11c1d9bd5.png)
