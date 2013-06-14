%////////////////////
% sessio3.m
%////////////////////

function sessio3(serPort)

		%bug1(serPort,[3.5,-1]);
		bug1(serPort,[-1,-3]);
	
		function bug1(serPort,objectiu)
			obstacle=false;
			%[x, y, anguloRads]=OverheadLocalizationCreate(serPort);
			%linia = (objectiu - [x, y]);
			[BumpRight,BumpLeft,WheDropRight,WheDropLeft,WheDropCaster,BumpFront] = ...     
                 BumpsWheelDropsSensorsRoomba(serPort);
            [x, y, anguloRads]=OverheadLocalizationCreate(serPort);
			DecisionAnguloGiro(x, y, anguloRads,objectiu);
			
			function trobat=hayObstaculo()
				 trobat=false;
				 distDerecha= ReadSonarMultiple(serPort,1);
           		 distFrontal = ReadSonarMultiple(serPort,2)
           		 distIzquierda = ReadSonarMultiple(serPort,3);
				if distDerecha < 0.5 && distIzquierda < 0.5  && distFrontal < 0.5 
					trobat=true; 
				else
					trobat =false;
				end

			end
			
			while ~hayObstaculo() && ~hemArribat([x, y], objectiu)
				fprintf('bucle principal');
				[x, y]=OverheadLocalizationCreate(serPort);
				%[BumpRight,BumpLeft,WheDropRight,WheDropLeft,WheDropCaster,BumpFront] = ...     
                 %BumpsWheelDropsSensorsRoomba(serPort);
                 %[x, y, anguloRads]=OverheadLocalizationCreate(serPort);
				 SetDriveWheelsCreate(serPort,.5,.5);
				 pause(0.000001);
			end 

		end
		
		function followBoundary(serPort,objectiu)
			
		
		   	
		   		
            
		end

		function distancia=hemArribat(posicioActual,objectiu)

			 distancia=sqrt( ((objectiu(1)-posicioActual(1)).^2)...
						    + ((objectiu(2)-posicioActual(2)).^2)...
						   )
			 % distancia;
			 if distancia < 0.2
			 	distancia=true;
			 else 
			 	distancia=false;
			 end
		end
		
		
		function valor=Entre(sensor,Valorinferior,Valorsuperior)
			if sensor > Valorinferior && sensor < Valorsuperior
				valor=1;% se cumple
			else
				valor=0;
			end

		end

		function calcularFrenada(serPort)
			for i=0:200
				%fprintf('frenoooooo\n');
           		 		pause(0.0001);
           		 		SetDriveWheelsCreate(serPort, .2,.2);
           		 		
           	end

		end
		function recalcularAngulo(serPort)
			
			[x, y, anguloRads]=OverheadLocalizationCreate(serPort);
           	anguloGiro=pasarAGrados(anguloRads);
			if anguloGiro > 1 && anguloGiro < 46
				turnAngle(serPort, .2,-anguloGiro);
			elseif anguloGiro > 46 && anguloGiro < 89
				anguloCorregido=abs(anguloGiro-90)
				turnAngle(serPort, .2,anguloCorregido);
			elseif anguloGiro > 89 && anguloGiro < 135
				anguloCorregido=abs(anguloGiro-90)
				turnAngle(serPort, .2,-anguloCorregido);
			elseif anguloGiro > 135 && anguloGiro < 180
				anguloCorregido=abs(anguloGiro-180)
				turnAngle(serPort, .2,anguloCorregido);
			elseif anguloGiro >-180 && anguloGiro <- 135
					anguloCorregido=anguloGiro+180
					turnAngle(serPort, .2,-anguloCorregido);
			elseif anguloGiro >-135 && anguloGiro <-90
					anguloCorregido=abs(abs(anguloGiro)-90);
					turnAngle(serPort, .2,anguloCorregido);
			elseif anguloGiro >-90 && anguloGiro <-45
					anguloCorregido=anguloGiro+90;
					turnAngle(serPort, .2,-anguloCorregido);
			elseif anguloGiro >-45 && anguloGiro < 0
				turnAngle(serPort, .2,abs(anguloGiro));
			end
			
				
			pause(0.00001);


		end
		function StopCreate(serPort) 
		        % Stop the robot 
		        % serPort is the serial port number (for controlling the actual robot). 
		        SetDriveWheelsCreate(serPort, 0,0)  
		end
		function grados=pasarAGrados(angulo)
			angulo=double(angulo);
			grados=double(angulo*(180/pi));
		end
		function DecisionAnguloGiro(x, y, anguloRads,objectiu)
			% x , y->>>> posicion actual
			%esta funcion decide hace donde girar el robot para que haga 
			%el minimo giro posible y ademas se mueva hacia el angulo que 
			%forma la linia mas corta hacia el objetivo atan(angulo)
			% donde anguloRads es el angulo actual del robot en sentio antihorario
			% donde objectiu es el punto x e y del punto final al que llegar
			%donde angulo giro sera el angulo al que girara el robot hacia el objetivo
			anguloGiro=0
			catetoContiguo=0%para calcular el angulo
			catetoOpuesto=0%para calcular el angulo
			
			anguloActual=pasarAGrados(anguloRads);
			if objectiu(1) >= 0 && objectiu(2) >= 0%primer cuadrante	
				catetoContiguo=objectiu(1)-x;
				catetoOpuesto=objectiu(2)-y;
				anguloGiro=atan(catetoOpuesto/catetoContiguo);
				anguloGiro=pasarAGrados(anguloGiro);
				anguloGiro=abs(anguloGiro)
				turnAngle(serPort, .2,anguloGiro);
			elseif objectiu(1) < 0 && objectiu(2) >= 0%segundo cuadrante
				catetoContiguo=abs(objectiu(1)-x);
				catetoOpuesto=abs(objectiu(2)-y);
				anguloGiro=atan(catetoOpuesto/catetoContiguo);
				anguloGiro=pasarAGrados(anguloGiro);
				anguloGiro=abs(180-anguloGiro)
				turnAngle(serPort, .2,anguloGiro);
			elseif objectiu(1)<0 && objectiu(2) < 0 %tercer cuadrande 
				catetoContiguo=abs(objectiu(1)-x);
				catetoOpuesto=abs(objectiu(2)-y);
				anguloGiro=atan(catetoOpuesto/catetoContiguo);
				anguloGiro=pasarAGrados(anguloGiro)
				anguloGiro=abs(180+anguloGiro)
				turnAngle(serPort, .2,anguloGiro);
			else                                    %cuarto cuadrande
				catetoContiguo=abs(objectiu(1)-x);
				catetoOpuesto=abs(objectiu(2)-y);
				anguloGiro=atan(catetoOpuesto/catetoContiguo);
				anguloGiro=pasarAGrados(anguloGiro)
				anguloGiro=abs(anguloGiro)
				turnAngle(serPort, .2,-anguloGiro);
			end
			
			
		end

			%%%%%%%%%%%%%%%%%%%%%%%%
		function Signal()
			% Make signal sound (4 beeps)
			n= 4;
			for k=1:4
				beep
				pause(1)
			end
		end
end

