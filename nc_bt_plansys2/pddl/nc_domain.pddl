(define (domain factory)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
location
item
door
grandma
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

; (battery_full ?r - robot)

; (robot_available ?r - robot)

(robot_at ?r - robot ?l - location)
(item_at ?i - item ?l - location)
(connected ?l1 ?l2 - location ?d - door)

(open ?d - door)
(close ?d - door)
(front_door_at ?d - door ?l - location)
(tidy_house ?r - robot)
(items_location ?l - location)
(object1 ?i - item)
(object2 ?i - item)
(object3 ?i - item)


(grandma_at ?g - grandma ?l - location)
(grandma_assisted ?g - grandma)
(open_front_door ?d - door)

(give_grandma ?r - robot ?i - item ?g - grandma)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?l1 ?l2 - location ?d - door)
    :duration (= ?duration 1)
    :condition 
        (and 
        (at start (robot_at ?r ?l1))
        (at start (connected ?l1 ?l2 ?d))
        (at start (open ?d))
        )
    :effect 
        (and 
        (at end (robot_at ?r ?l2))
        (at start (not (robot_at ?r ?l1)))
    )
)

(:durative-action open-door
    :parameters (?r - robot ?l1 ?l2 - location ?d - door)
    :duration (= ?duration 1)
    :condition 
        (and 
        (at start (robot_at ?r ?l1))
        (at start (close ?d))
        (at start (connected ?l1 ?l2 ?d))
    )
    :effect 
        (and 
        (at end (open ?d))
        (at start (not (close ?d)))
    )
)

(:durative-action close-door
    :parameters (?r - robot ?l1 ?l2 - location ?d - door)
    :duration (= ?duration 1)
    :condition 
        (and 
        (at start (robot_at ?r ?l1))
        (at start (open ?d))
        (at start (connected ?l1 ?l2 ?d))
    )
    :effect 
        (and 
        (at end (close ?d))
        (at start (not (open ?d)))
    )
)

(:durative-action transport
    :parameters (?r - robot ?i - item ?l1 ?l2 - location ?d - door)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?l1))
        (at start(item_at ?i ?l1))
        (at start(connected ?l1 ?l2 ?d))
        (at start(open ?d))
    )
    :effect (and
        (at start(not(robot_at ?r ?l1)))
        (at end(robot_at ?r ?l2))
        (at start(not(item_at ?i ?l1)))
        (at end(item_at ?i ?l2))
    )
)

(:durative-action assisted
    :parameters (?r - robot ?l - location ?i - item ?g - grandma)
    :duration ( = ?duration 5)
    :condition (and
        (at start(grandma_at ?g ?l))
        (at start(robot_at ?r ?l))
        (at start(item_at ?i ?l))
    )
    :effect (and
        (at end(give_grandma ?r ?i ?g))
        (at end(grandma_assisted ?g))
    )
)

(:durative-action tidy
    :parameters (?l1 ?l2 ?l3 - location ?i1 ?i2 ?i3 - item ?g - grandma ?r - robot)
    :duration (= ?duration 1)
    :condition 
        (and 
          (at start (item_at ?i1 ?l1))
          (at start (item_at ?i2 ?l1))
          (at start (item_at ?i3 ?l1))
          (at start (object1 ?i1))
          (at start (object2 ?i2))
          (at start (object3 ?i3))
          (at start (items_location ?l1))
          (over all (grandma_assisted ?g))
    )
    :effect 
        (and 
          (at end (tidy_house ?r))
    )
)

(:durative-action opened
    :parameters (?l - location ?r - robot ?d - door)
    :duration (= ?duration 1)
    :condition 
        (and 
          (at start (robot_at ?r ?l))
          (at start (front_door_at ?d ?l))
          (at start (close ?d))
    )
    :effect 
        (and 
          (at start (not(close ?d)))
          (at end (open ?d))
          (at end (open_front_door ?d))
    )
)


);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
