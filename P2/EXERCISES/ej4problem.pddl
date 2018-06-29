(define (problem ej4)

  ;NOMBRE DOMINIO
  (:domain ej4-domain)
    
  ;OBJETOS DEL MUNDO
		(:OBJECTS 
		 Z1 Z2 Z3 Z4 Z5 Z6 Z7 Z8 Z9 Z10 Z11 Z12 Z13 Z14 Z15 Z16 Z17 Z18 Z19 Z20 Z21 Z22 Z23 Z24 Z25 - zona 
		 PLAYER - jugador
		 OSCAR MANZANA ROSA ALGORITMO ORO ZAPATILLAS BIKINI - objeto
		 PRINCESA PRINCIPE BRUJA PROFESOR LEONARDO - personaje
		 BOSQUE AGUA PRECIPICIO ARENA PIEDRA - tipo
		 norte sur este oeste - orient
		)

	(:INIT
		;connected ?x ?y este: De la zona x a la y se va por el este
		
		
		;FILAS
		 (connected Z1 Z2 este)
		 (connected Z2 Z1 oeste)
		 (connected Z2 Z3 este)
		 (connected Z3 Z2 oeste)
		 (connected Z3 Z4 este)
		 (connected Z4 Z3 oeste)
		 (connected Z4 Z5 este)
		 (connected Z5 Z4 oeste)
		 
		 (connected Z6 Z7 este)
		 (connected Z7 Z6 oeste)
		 (connected Z7 Z8 este)
		 (connected Z8 Z7 oeste)
		 (connected Z8 Z9 este)
		 (connected Z9 Z8 oeste)
		 (connected Z9 Z10 este)
		 (connected Z10 Z9 oeste)
		 
		 (connected Z11 Z12 este)
		 (connected Z12 Z11 oeste)
		 (connected Z12 Z13 este)
		 (connected Z13 Z12 oeste)
		 (connected Z13 Z14 este)
		 (connected Z14 Z13 oeste)
		 (connected Z14 Z15 este)
		 (connected Z15 Z14 oeste)
		 
		 (connected Z16 Z17 este)
		 (connected Z17 Z16 oeste)
		 (connected Z17 Z18 este)
		 (connected Z18 Z17 oeste)
		 (connected Z18 Z19 este)
		 (connected Z19 Z18 oeste)
		 (connected Z19 Z20 este)
		 (connected Z20 Z19 oeste)
		 
		 (connected Z21 Z22 este)
		 (connected Z22 Z21 oeste)
		 (connected Z22 Z23 este)
		 (connected Z23 Z22 oeste)
		 (connected Z23 Z24 este)
		 (connected Z24 Z23 oeste)
		 (connected Z24 Z25 este)
		 (connected Z25 Z24 oeste)

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
		(connected Z12 Z17 sur)
		(connected Z17 Z12 norte)
		(connected Z17 Z22 sur)
		(connected Z22 Z17 norte)
		
		(connected Z3 Z8 sur)
		(connected Z8 Z3 norte)
		(connected Z8 Z13 sur)
		(connected Z13 Z8 norte)
		(connected Z13 Z18 sur)
		(connected Z18 Z13 norte)
		(connected Z18 Z23 sur)
		(connected Z23 Z18 norte)
		
		(connected Z4 Z9 sur)
		(connected Z9 Z4 norte)
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

		;Distancias
		;FILAS
		 (= (distance Z1 Z2 ) 1)
		 (= (distance Z2 Z1 ) 1)
		 (= (distance Z2 Z3 ) 1)
		 (= (distance Z3 Z2 ) 1)
		 (= (distance Z3 Z4 ) 1)
		 (= (distance Z4 Z3 ) 1)
		 (= (distance Z4 Z5 ) 1)
		 (= (distance Z5 Z4 ) 1)
		 
		 (= (distance Z6 Z7 ) 1)
		 (= (distance Z7 Z6 ) 1)
		 (= (distance Z7 Z8 ) 1)
		 (= (distance Z8 Z7 ) 1)
		 (= (distance Z8 Z9 ) 1)
		 (= (distance Z9 Z8 ) 1)
		 (= (distance Z9 Z10 ) 1)
		 (= (distance Z10 Z9 ) 1)
		 
		 (= (distance Z11 Z12 ) 1)
		 (= (distance Z12 Z11 ) 1)
		 (= (distance Z12 Z13 ) 1)
		 (= (distance Z13 Z12 ) 1)
		 (= (distance Z13 Z14 ) 1)
		 (= (distance Z14 Z13 ) 1)
		 (= (distance Z14 Z15 ) 1)
		 (= (distance Z15 Z14 ) 1)
		 
		 (= (distance Z16 Z17 ) 1)
		 (= (distance Z17 Z16 ) 1)
		 (= (distance Z17 Z18 ) 1)
		 (= (distance Z18 Z17 ) 1)
		 (= (distance Z18 Z19 ) 1)
		 (= (distance Z19 Z18 ) 1)
		 (= (distance Z19 Z20 ) 1)
		 (= (distance Z20 Z19 ) 1)
		 
		 (= (distance Z21 Z22 ) 1)
		 (= (distance Z22 Z21 ) 1)
		 (= (distance Z22 Z23 ) 1)
		 (= (distance Z23 Z22 ) 1)
		 (= (distance Z23 Z24 ) 1)
		 (= (distance Z24 Z23 ) 1)
		 (= (distance Z24 Z25 ) 1)
		 (= (distance Z25 Z24 ) 1)

		 ;COLUMNAS
		 
		(= (distance Z1 Z6 ) 1)
		(= (distance Z6 Z1 ) 1)
		(= (distance Z6 Z11 ) 1)
		(= (distance Z11 Z6 ) 1)
		(= (distance Z11 Z16 ) 1)
		(= (distance Z16 Z11 ) 1)
		(= (distance Z16 Z21 ) 1)
		(= (distance Z21 Z16 ) 1)
		
		(= (distance Z2 Z7 ) 1)
		(= (distance Z7 Z2 ) 1)
		(= (distance Z7 Z12 ) 1)
		(= (distance Z12 Z7 ) 1)
		(= (distance Z12 Z17 ) 1)
		(= (distance Z17 Z12 ) 1)
		(= (distance Z17 Z22 ) 1)
		(= (distance Z22 Z17 ) 1)
		
		(= (distance Z3 Z8 ) 1)
		(= (distance Z8 Z3 ) 1)
		(= (distance Z8 Z13 ) 1)
		(= (distance Z13 Z8 ) 1)
		(= (distance Z13 Z18 ) 1)
		(= (distance Z18 Z13 ) 1)
		(= (distance Z18 Z23 ) 1)
		(= (distance Z23 Z18 ) 1)
		
		(= (distance Z4 Z9 ) 1)
		(= (distance Z9 Z4 ) 1)
		(= (distance Z9 Z14 ) 1)
		(= (distance Z14 Z9 ) 1)
		(= (distance Z14 Z19 ) 1)
		(= (distance Z19 Z14 ) 1)
		(= (distance Z19 Z24 ) 1)
		(= (distance Z24 Z19 ) 1)
		
		(= (distance Z5 Z10 ) 1)
		(= (distance Z10 Z5 ) 1)
		(= (distance Z10 Z15 ) 1)
		(= (distance Z15 Z10 ) 1)
		(= (distance Z15 Z20 ) 1)
		(= (distance Z20 Z15 ) 1)
		(= (distance Z20 Z25 ) 1)
		(= (distance Z25 Z20 ) 1)
		
		; Se indica el tipo de zona que es cada una
       (is_z Z1 AGUA)
		 (is_z Z2 ARENA)
		 (is_z Z3 ARENA)
		 (is_z Z4 ARENA)
		 (is_z Z5 ARENA)
		 (is_z Z6 AGUA)
		 (is_z Z7 PRECIPICIO)
		 (is_z Z8 PIEDRA)
		 (is_z Z9 PIEDRA)
		 (is_z Z10 PIEDRA)
		 (is_z Z11 AGUA)
		 (is_z Z12 PRECIPICIO)
		 (is_z Z13 PIEDRA)
		 (is_z Z14 BOSQUE)
		 (is_z Z15 BOSQUE)
		 (is_z Z16 AGUA)
		 (is_z Z17 PRECIPICIO)
		 (is_z Z18 PIEDRA)
		 (is_z Z19 BOSQUE)
		 (is_z Z20 BOSQUE)
		 (is_z Z21 AGUA)
		 (is_z Z22 PRECIPICIO)
		 (is_z Z23 PIEDRA)
		 (is_z Z24 BOSQUE)
		 (is_z Z25 BOSQUE)
		 
		 ;PUNTOS (entrega-objeto)
		(= (points OSCAR LEONARDO) 10)
		(= (points ROSA LEONARDO) 1)
		(= (points MANZANA LEONARDO) 3)
		(= (points ALGORITMO LEONARDO) 4)
		(= (points ORO LEONARDO) 5)

		(= (points OSCAR PRINCESA) 5)
		(= (points ROSA PRINCESA) 10)
		(= (points MANZANA PRINCESA) 1)
		(= (points ALGORITMO PRINCESA) 3)
		(= (points ORO PRINCESA) 4)

		(= (points OSCAR BRUJA) 4)
		(= (points ROSA BRUJA) 5)
		(= (points MANZANA BRUJA) 10)
		(= (points ALGORITMO BRUJA) 1)
		(= (points ORO BRUJA) 3)

		(= (points OSCAR PROFESOR) 3)
		(= (points ROSA PROFESOR) 4)
		(= (points MANZANA PROFESOR) 5)
		(= (points ALGORITMO PROFESOR) 10)
		(= (points ORO PROFESOR) 1)

		(= (points OSCAR PRINCIPE) 1)
		(= (points ROSA PRINCIPE) 3)
		(= (points MANZANA PRINCIPE) 4)
		(= (points ALGORITMO PRINCIPE) 5)
		(= (points ORO PRINCIPE) 10)
		  
		 ; OBJETOS MAPA, PERSONAJES MAPA
		 ;p3
		 ;(at_objeto ORO Z11)
		 ;(at_objeto OSCAR Z10)
		 ;(at_objeto MANZANA Z15)
		 ;(at_objeto ROSA Z15)
		 ;(at_objeto ALGORITMO Z2)
		 
		 ;P4
		 (at_objeto OSCAR Z1)
		 (at_objeto OSCAR Z2)
		 (at_objeto OSCAR Z3)
		 
		 (at_objeto MANZANA Z4)
		 (at_objeto MANZANA Z5)
		 (at_objeto MANZANA Z6)
		 (at_objeto MANZANA Z8)
		 (at_objeto MANZANA Z14)
		 (at_objeto MANZANA Z20)
		 
		 (at_objeto ALGORITMO Z18)
		 
		  ;ZAPATILLAS y bikini
        (at_objeto ZAPATILLAS Z4)
        (at_objeto ZAPATILLAS Z16)
        (at_objeto BIKINI Z19)
		  (at_objeto BIKINI Z20)

		 (at_personaje PRINCIPE Z25)
		 (at_personaje LEONARDO Z6)
		 (at_personaje BRUJA Z5)
		 (at_personaje PRINCESA Z21)
		 (at_personaje PROFESOR Z23)
		 
		 ;PLAYER
		 (at_jugador PLAYER Z13)
		 (orientation_player PLAYER este)
		 (= (total_cost) 0)
		 (= (total_points) 0)
		 (empty_hand PLAYER)
		 (empty_back PLAYER)
	)
	
	; Objetivo: entregar TODOS los objetos a TODOS los personajes
	(:goal 
		 ;(and  (has_personaje LEONARDO OSCAR) (has_personaje PRINCIPE ORO) (has_personaje PROFESOR ALGORITMO) (has_personaje PRINCESA ROSA) (has_personaje BRUJA MANZANA))
		 ;(and (at_jugador Z25) (en_mano OSCAR))
		 ;(and (has_personaje PRINCESA ROSA))
		 
		 ;P4
		 (and (>= (total_points) 100))
	)
	 
	 ; Optimización: minimizar total_cost
    (:metric minimize (total_cost))
)