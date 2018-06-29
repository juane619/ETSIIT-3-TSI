(define (domain ej2-domain)	  
  (:requirements :strips :equality :typing :fluents)
  (:types jugador personaje objeto zona orient)
 
	(:functions
		(total_cost)
		(distance ?x ?y - object)
	)
 
 ;Predicados
  (:predicates 
		(has_personaje ?p - personaje ?obj - objeto)
		(has_player ?j - jugador ?obj - objeto)
		(empty_hand ?j - jugador)
		(at_personaje ?p - personaje ?z - zona)
		(at_jugador ?j - jugador ?z - zona)
		(at_objeto ?o - objeto ?z - zona)
		(orientation_player ?j - jugador ?o - orient)
		(connected ?zona1 ?zona2 - zona ?o - orient)
  )
  
  ;Acciones:
  
  ;moverse a otra zona
  (:action moveTo
		:parameters (?j - jugador ?or_fin - orient ?zona_ini ?zona_fin - zona)
		:precondition 
		(
			and 
			(orientation_player ?j ?or_fin)
			(at_jugador ?j ?zona_ini)
			(connected ?zona_ini ?zona_fin ?or_fin)
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
	     )
	     :effect 
	     (
	        and
	        (has_player ?j ?obj)
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
	        (has_player ?j ?obj)
	     )
	     :effect 
	     (
	        and
	        (not(has_player ?j ?obj))
	        (at_objeto ?obj ?z)
			  (empty_hand ?j)
	     )
  )
  
  ;entregar objeto
  (:action giveObject
	     :parameters (?j - jugador ?obj - objeto ?z - zona ?p - personaje)
	     :precondition 
	     (
	        and
	        (at_jugador ?j ?z)
	        (at_personaje ?p ?z)
	        (has_player ?j ?obj)
	     )
	     :effect 
	     (
	        and
	        (not(has_player ?j ?obj))
	        (has_personaje ?p ?obj)
			  (empty_hand ?j)
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
	        (not(orientation_player ?j ?o))
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
	        
	        (not(orientation_player ?j ?o))
	     )
  )
  
)

