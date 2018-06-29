(define (problem ej1)

  ;NOMBRE DOMINIO
  (:domain ej1-domain)
    
  ;OBJETOS DEL MUNDO
		(:OBJECTS 
		 Z1 Z2 Z3 Z4 Z5 Z6 Z7 Z8 Z9 Z10 Z11 Z12 Z13 Z14 Z15 Z16 Z17 Z18 Z19 Z20 Z21 Z22 Z23 Z24 Z25 - zona 
		 PLAYER - jugador
		 OSCAR MANZANA ROSA ALGORITMO ORO - objeto
		 PRINCESA PRINCIPE BRUJA PROFESOR LEONARDO - personaje
		 norte sur este oeste - orient
		)

(:INIT
	;connected ?x ?y este: De la zona x a la y se va por el este
		
		
		;FILAS
		 (connected Z1 Z2 este)
		 (connected Z2 Z1 oeste)
		 (connected Z3 Z4 este)
		 (connected Z4 Z3 oeste)
		 (connected Z4 Z5 este)
		 (connected Z5 Z4 oeste)
		 
		 (connected Z9 Z10 este)
		 (connected Z10 Z9 oeste)
		 
		 (connected Z12 Z13 este)
		 (connected Z13 Z12 oeste)
		 (connected Z13 Z14 este)
		 (connected Z14 Z13 oeste)
		 
		 (connected Z17 Z18 este)
		 (connected Z18 Z17 oeste)
		 

		 ;COLUMNAS
		 
		(connected Z1 Z6 sur)
		(connected Z6 Z1 norte)
		(connected Z6 Z11 sur)
		(connected Z11 Z6 norte)
		(connected Z11 Z16 sur)
		(connected Z16 Z11 norte)
		(connected Z16 Z21 sur)
		(connected Z21 Z16 norte)
		
		(connected Z2 Z7 sur)
		(connected Z7 Z2 norte)
		(connected Z7 Z12 sur)
		(connected Z12 Z7 norte)
		
		(connected Z3 Z8 sur)
		(connected Z8 Z3 norte)
		(connected Z8 Z13 sur)
		(connected Z13 Z8 norte)
		(connected Z13 Z18 sur)
		(connected Z18 Z13 norte)
		(connected Z18 Z23 sur)
		(connected Z23 Z18 norte)
		
		(connected Z9 Z14 sur)
		(connected Z14 Z9 norte)
		(connected Z14 Z19 sur)
		(connected Z19 Z14 norte)
		(connected Z19 Z24 sur)
		(connected Z24 Z19 norte)
		
		(connected Z5 Z10 sur)
		(connected Z10 Z5 norte)
		(connected Z10 Z15 sur)
		(connected Z15 Z10 norte)
		(connected Z15 Z20 sur)
		(connected Z20 Z15 norte)
		(connected Z20 Z25 sur)
		(connected Z25 Z20 norte)

    ; OBJETOS MAPA, PERSONAJES MAPA
		(at_objeto ORO Z11)
		(at_objeto OSCAR Z10)
		(at_objeto MANZANA Z18)
		(at_objeto ROSA Z15)
		(at_objeto ALGORITMO Z2)

		(at_personaje PRINCIPE Z25)
		(at_personaje LEONARDO Z6)
		(at_personaje BRUJA Z5)
		(at_personaje PRINCESA Z21)
		(at_personaje PROFESOR Z23)
    
		 ;PLAYER
		 (at_jugador PLAYER Z13)
		 (orientation_player PLAYER este)
		 (empty_hand PLAYER)
)

; Objetivo: entregar TODOS los objetos a TODOS los personajes
(:goal 
    (and (has_personaje LEONARDO OSCAR) (has_personaje PRINCESA ROSA) (has_personaje BRUJA MANZANA) (has_personaje PROFESOR ALGORITMO) (has_personaje PRINCIPE ORO)))
)