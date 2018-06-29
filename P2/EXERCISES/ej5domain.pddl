(define (domain ej5-domain)	  
  (:requirements :strips :equality :typing :fluents)
  (:types jugador personaje objeto zona orient tipo)
 
	(:functions
		(total_cost)
		(distance ?x ?y - object)
		
		(total_points)
		(points ?x - objeto ?y - personaje)
		
		(max_capacity ?per - personaje)
		(how_many ?per - personaje)
	)
 
 ;Predicados
  (:predicates 
		(has_personaje ?p - personaje ?obj - objeto)
		(en_mano ?j - jugador ?obj - objeto)
		(empty_hand ?j - jugador)
		(at_personaje ?p - personaje ?z - zona)
		(at_jugador ?j - jugador ?z - zona)
		(at_objeto ?o - objeto ?z - zona)
		(orientation_player ?j - jugador ?o - orient)
		(connected ?zona1 ?zona2 - zona ?o - orient)
		
		(empty_back ?j - jugador)
		(backpack ?j - jugador ?o - objeto)
		(is_z ?z - zona ?t - tipo)
		
  )
  
  ;Acciones:
  
  ;moverse a otra zona
  (:action moveTo
		:parameters (?j - jugador ?or_fin - orient ?zona_ini ?zona_fin - zona)
		:precondition 
		(
			and 
			(at_jugador ?j ?zona_ini)
			(orientation_player ?j ?or_fin)
			(connected ?zona_ini ?zona_fin ?or_fin)
			(not (is_z ?zona_fin PRECIPICIO))
			(or
				(and
				  (not (is_z ?zona_fin BOSQUE))
				  (not (is_z ?zona_fin AGUA))
				)
				(and
				  (is_z ?zona_fin BOSQUE)
				  (or
						(en_mano ?j ZAPATILLAS)
						(backpack ?j ZAPATILLAS)
				  )
				)
				(and
				  (is_z ?zona_fin AGUA)
				  (or
						(en_mano ?j BIKINI)
						(backpack ?j BIKINI)
				  )
				)			
			)
		)
		:effect 
		(
		  and
		  (at_jugador ?j ?zona_fin)
		  (not(at_jugador ?j ?zona_ini))
		  (increase (total_cost) (distance ?zona_ini ?zona_fin))
		)

  )
  
  ;coger objeto
  (:action takeObject
	     :parameters (?j - jugador ?obj - objeto ?z - zona)
	     :precondition 
	     (
	       and
			(empty_hand ?j)
			(at_jugador ?j ?z)
			(at_objeto ?obj ?z)
			(not (en_mano ?j ?obj))
			(not (backpack ?j ?obj))
	     )
	     :effect 
	     (
				and
				(en_mano ?j ?obj)
				(not(at_objeto ?obj ?z))
				(not (empty_hand ?j))
	     )
  )
  
  ;soltar objeto
  (:action dropObject
	     :parameters (?j - jugador ?obj - objeto ?z - zona)
	     :precondition 
	     (
				and
				(at_jugador ?j ?z)
				(en_mano ?j ?obj)
				(not (empty_hand ?j))
				;(not (backpack ?j ?obj))
				(or
					(and
					  (not (is_z ?z BOSQUE))
					  (not (is_z ?z AGUA))
					)
					(and
						(is_z ?z BOSQUE)
						(not (en_mano ?j ZAPATILLAS))
					)
					(and
						(is_z ?z AGUA)
						(not (en_mano ?j BIKINI))
					)
				)
	     )
	     :effect 
	     (
				and
				(not (en_mano ?j ?obj))
				(at_objeto ?obj ?z)
				(empty_hand ?j)
	     )
  )
  
	; Meter un objeto en la mochila
	(:action putBackpack
		:parameters (?j - jugador ?o - objeto)

		;Se debe tener el objeto en la mano 
		:precondition (and
			(not (empty_hand ?j))
			(en_mano ?j ?o)
			(empty_back ?j)
		)

		;Mano vacía y objeto en la mochila
		:effect (and
			(not (en_mano ?j ?o))
			(empty_hand ?j)
			(backpack ?j ?o)
			(not (empty_back ?j))
		)
	)

	; Sacar un objeto de la mochila
	(:action popBackpack
	  :parameters (?J - jugador ?o - objeto)
	  
	  ;Mano vacía y objeto en la mochila
	  :precondition (and
			(not (empty_back ?j))
			(empty_hand ?j)
			(backpack ?j ?o)
	  )

	  ; Objeto en la mano y no en la mochila
	  :effect (and
			(not (empty_hand ?j))
			(not (backpack ?j ?o))
			(empty_back ?j)
			(en_mano ?j ?o)
	  )
	)
  
  ;entregar objeto
  (:action giveObject
	  :parameters (?j - jugador ?obj - objeto ?z - zona ?p - personaje)
	  :precondition 
	  (
			and
			(not (empty_hand ?j))
			(en_mano ?j ?obj)
			(at_jugador ?j ?z)
			(at_personaje ?p ?z)
			(< (how_many ?p) (max_capacity ?p) )
			(not (en_mano ?j ZAPATILLAS))
			(not (en_mano ?j BIKINI))
	  )
	  :effect 
	  (
		  and
		  (not(en_mano ?j ?obj))
		  (has_personaje ?p ?obj)
		  (empty_hand ?j)
		  (increase (total_points) (points ?obj ?p))
		  (increase (how_many ?p) 1)
	  )
  )
  
  ;giro a izquierda
  (:action turnLeft
	     :parameters (?j - jugador ?o - orient)
	     :precondition (orientation_player ?j ?o)
	     :effect 
	     (
	        and
	        (when(and (orientation_player ?j norte))
	            (and (orientation_player ?j oeste))
	        )
	        
	        (when(and (orientation_player ?j sur))
	            (and (orientation_player ?j este))
	        )
	        
	        (when(and (orientation_player ?j este))
	            (and (orientation_player ?j norte))
	        )
	        
	        (when(and (orientation_player ?j oeste))
	            (and (orientation_player ?j sur))
	        )
	        
	        ;eliminamos el predicado que había anteriormente
	        (not (orientation_player ?j ?o))
	     )
  )
  
  
  ;giro derecha
  (:action turnRight
	     :parameters (?j - jugador ?o - orient)
	     :precondition (orientation_player ?j ?o)
	     :effect 
	     (
	        and(when(and (orientation_player ?j norte))
	            (and (orientation_player ?j este))
	        )
	        
	        (when(and (orientation_player ?j sur))
	            (and (orientation_player ?j oeste))
	        )
	        
	        (when(and (orientation_player ?j este))
	            (and (orientation_player ?j sur))
	        )
	        
	        (when(and (orientation_player ?j oeste))
	            (and (orientation_player ?j norte))
	        )
	        
	        (not (orientation_player ?j ?o))
	     )
  )
  
)

