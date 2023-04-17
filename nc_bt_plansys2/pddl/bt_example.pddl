(define (domain factory)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
location
item
door
car
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

(piece_is_wheel ?i - item)
(piece_is_body_car ?i - item)
(piece_is_steering_wheel ?i - item)

(not_item_in_place ?i - item)

(is_assembly_zone ?l - location)

(car_assembled ?c - car)

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
        ; (at start(robot_available ?r))
    )
    :effect (and
        (at start(not(robot_at ?r ?l1)))
        (at end(robot_at ?r ?l2))
        (at start(not(item_at ?i ?l1)))
        (at end(item_at ?i ?l2))
        ; (at start(not(robot_available ?r)))
        ; (at end(robot_available ?r))
    )
)

; (:durative-action recharge
;     :parameters (?r - robot ?l - location)
;     :duration ( = ?duration 5)
;     :condition (and
;         (at start(is_recharge_zone ?l))
;         (over all(robot_at ?r ?l))
;         (at start(robot_available ?r))
;       )
;     :effect (and
;         (at end(battery_full ?r))
;                 (at start(not(robot_available ?r)))
;         (at end(robot_available ?r))
;     )
; )

(:durative-action assemble
    :parameters (?r - robot ?l - location ?i1 ?i2 ?i3 - item ?c - car)
    :duration ( = ?duration 5)
    :condition (and
        ; (over all(battery_full ?r))

        (at start(is_assembly_zone ?l))
        (at start(robot_at ?r ?l))

        (at start(item_at ?i1 ?l))
        (at start(item_at ?i2 ?l))
        (at start(item_at ?i3 ?l))

        (at start(not_item_in_place ?i1))
        (at start(not_item_in_place ?i2))
        (at start(not_item_in_place ?i3))

        ; (at start(robot_available ?r))
    )
    :effect (and
        (at start(not(not_item_in_place ?i1)))
        (at start(not(not_item_in_place ?i2)))
        (at start(not(not_item_in_place ?i3)))
        (at end(car_assembled ?c))
        ; (at start(not(robot_available ?r)))
        ; (at end(robot_available ?r))

    )
)


);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
