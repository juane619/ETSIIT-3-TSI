(define (domain zeno-travel)

(:requirements
  :typing
  :fluents
  :derived-predicates
  :negative-preconditions
  :universal-preconditions
  :disjuntive-preconditions
  :conditional-effects
  :htn-expansion

  ; Requisitos adicionales para el manejo del tiempo
  :durative-actions
  :metatags
 )

(:types aircraft person city - object)

(:constants slow fast - object)

(:predicates (at ?x - (either person aircraft) ?c - city)
             (in ?p - person ?a - aircraft)
             
             (hay-fuel-lento ?a ?c1 ?c2)
				 (hay-fuel-rapido ?a ?c1 ?c2)
				 
				 ;ej4
				 (destino ?p - person ?c - city)
				(disponible-duracion-fast ?a - aircraft ?c1 - city ?c2 - city)
				(disponible-duracion-slow ?a - aircraft ?c1 - city ?c2 - city)
)

(:functions (fuel ?a - aircraft)
            (distance ?c1 - city ?c2 - city)
            (slow-speed ?a - aircraft)
            (fast-speed ?a - aircraft)
            (slow-burn ?a - aircraft)
            (fast-burn ?a - aircraft)
            (capacity ?a - aircraft)
            (refuel-rate ?a - aircraft)
            (total-fuel-used ?a - aircraft)
            (boarding-time)
            (debarking-time)
				(fuel-limit ?a - aircraft)
				
				(actual-pass ?a - aircraft)
				(max-pass ?a - aircraft)
				
				(act-duracion ?a - aircraft)
				(max-duracion ?a - aircraft)
)

;; el consecuente "vacio" se representa como "()" y significa "siempre verdad"


;; este literal derivado se utiliza para deducir, a partir de la información en el estado actual, 
;; si hay fuel suficiente para que el avión ?a vuele de la ciudad ?c1 a la ?c2
;; el antecedente de este literal derivado comprueba si el fuel actual de ?a es mayor que 1. 
;; En este caso es una forma de describir que no hay restricciones de fuel. Pueden introducirse una
;; restricción más copleja  si en lugar de 1 se representa una expresión más elaborada (esto es objeto de
;; los siguientes ejercicios).

  (:derived   
	  (hay-fuel-lento ?a - aircraft ?c1 - city ?c2 - city)
	  (>= (fuel ?a) (* (distance ?c1 ?c2) (slow-burn ?a)))
	)

	(:derived   
	  (hay-fuel-rapido ?a - aircraft ?c1 - city ?c2 - city)
	  (>= (fuel ?a) (* (distance ?c1 ?c2) (fast-burn ?a)))
	)
	
	;ej4
	;;duracion no sobrepasada
	(:derived
	 (disponible-duracion-fast ?a - aircraft ?c1 - city ?c2 - city)
	 (<= (+ (act-duracion ?a) (/ (distance ?c1 ?c2) (fast-speed ?a))) (max-duracion ?a))
	)

	(:derived
	 (disponible-duracion-slow ?a - aircraft ?c1 - city ?c2 - city)
	 (<= (+ (act-duracion ?a) (/ (distance ?c1 ?c2) (slow-speed ?a))) (max-duracion ?a))
	)

(:task transport-person
	:parameters (?p - person ?c - city)
	
   (:method Case1 ; si la persona esta en la ciudad no se hace nada
	 :precondition (and 	(at ?p ?c)
						)
	 :tasks ()
   )
	 
   (:method Case2 ;si no esta en la ciudad destino, pero avion y persona estan en la misma ciudad
	  :precondition (and (at ?p - person ?c1 - city)
			               (at ?a - aircraft ?c1 - city)
						)
				     
	  :tasks (
	  	      (board)
		      (mover-avion ?a ?c1 ?c)
		      (debark)
		)
	)
	
	
	(:method Case3 ;avion y persona en distintas ciudades
		:precondition 	(and (at ?p - person ?c1 - city) 
							(at ?a - aircraft ?c2 - city)
							(not (= ?c1 ?c2)))
		
		:tasks (	(mover-avion ?a ?c2 ?c1)
					(board)
					(mover-avion ?a ?c1 ?c)
					(debark)
		)
	)
	(:method Case4 ;avion en distinta ciudad de destino pero con la persona dentro
		:precondition 	(and  
							(at ?a - aircraft ?c2 - city)
							(destino ?p ?c)
							(in ?p ?a))
		
		:tasks (	(mover-avion ?a ?c2 ?c)
					(debark)
		)
	)
	(:method Case5 ;avion atrancado, debe bajar al pasajero y "llamar" a otro avion
        :precondition (and
            (in ?p - person ?a1 - aircraft)
        )

        :tasks (
            (debark)
            (transport-person ?p ?c)
        )
	)
)

;; Ejercicio 4


(:task mover-avion
 :parameters (?a - aircraft ?c1 - city ?c2 -city)
 
	;; este método se escogerá para usar la acción fly siempre que el avión tenga fuel para
	;; volar desde ?c1 a ?c2
	;; si no hay fuel suficiente el método no se aplicará y la descomposición de esta tarea
	;; se intentará hacer con otro método. Cuando se agotan todos los métodos posibles, la
	;; descomponsición de la tarea mover-avión "fallará". 
	;; En consecuencia HTNP hará backtracking y escogerá otra posible vía para descomponer
	;; la tarea mover-avion (por ejemplo, escogiendo otra instanciación para la variable ?a)
	(:method fuel-suficiente-rapido
		:precondition (and 
								(hay-fuel-rapido ?a ?c1 ?c2)
								(disponible-duracion-fast ?a ?c1 ?c2)
								(> (fuel-limit ?a) (total-fuel-used ?a))
							)
		
		:tasks ((zoom ?a ?c1 ?c2))
	)
	
	(:method fuel-insuficiente-rapido
		:precondition (and 	(not (hay-fuel-rapido ?a ?c1 ?c2)) 
									(> (fuel-limit ?a) (total-fuel-used ?a))
							)
		
		:tasks ((refuel ?a ?c1) 
					(mover-avion ?a ?c1 ?c2))
	)
	
	(:method fuel-suficiente-lento
		:precondition (and 
								(hay-fuel-lento ?a ?c1 ?c2) 
								(disponible-duracion-slow ?a ?c1 ?c2)
								(> (fuel-limit ?a) (total-fuel-used ?a)))
		
		:tasks ((fly ?a ?c1 ?c2))
	)
	
	(:method fuel-insuficiente-lento
		:precondition (and 
									(not (hay-fuel-lento ?a ?c1 ?c2)) 
									(> (fuel-limit ?a) (total-fuel-used ?a)))
		
		:tasks ((refuel ?a ?c1) 
					(mover-avion ?a ?c1 ?c2))
	)
  )
 
(:import "Primitivas-ZenoTravel4.pddl") 


)
