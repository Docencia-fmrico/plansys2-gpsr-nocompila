(define  (domain grandma-nav)
(:requirements :strips :equality :typing :negative-preconditions :durative-actions)

(:types
item
location
door
robot
gripper
grandma
)

(:predicates 
    (robot_at ?r - robot ?l - location)
    (grandma_at ?g - grandma ?l - location)
    (item_at ?i - item ?l - location)
    (placed ?i - item ?l - location)
    (gripper_free ?gr - gripper) 
    (gripper_at ?gr - gripper ?r - robot)
    (robot_carry ?r - robot ?gr - gripper ?i - item)
    (connected ?l1 ?l2 - location ?d - door)
    (open ?d - door)
    (close ?d - door)
    (grandma_assisted ?g - grandma)
    (give_me_item ?i - item ?g - grandma)
    (close_me_door ?d - door ?g - grandma)
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

(:durative-action move
    :parameters (?r - robot ?from ?to - location ?d - door)
    :duration (= ?duration 1)
    :condition 
        (and 
        (at start (robot_at ?r ?from))
        (at start (connected ?from ?to ?d))
        (at start (open ?d))
        )
    :effect 
        (and 
        (at end (robot_at ?r ?to))
        (at start (not (robot_at ?r ?from)))
        )
    )

(:durative-action pick
    :parameters (?i - item ?l - location ?r - robot ?gr - gripper ?g - grandma)
    :duration (= ?duration 1)
    :condition 
        (and
        (at start (gripper_at ?gr ?r))
        (at start (item_at ?i ?l))
        (at start (robot_at ?r ?l))
        (at start (gripper_free ?gr))
        )
    :effect 
        (and 
        (at end (robot_carry ?r ?gr ?i))
        (at start (not (item_at ?i ?l)))
        (at start (not (gripper_free ?gr)))
        )
    )

(:durative-action drop
    :parameters (?i - item ?l - location ?r - robot ?gr - gripper ?g - grandma)
    :duration (= ?duration 1)
    :condition 
        (and 
        (at start (gripper_at ?gr ?r))
        (at start (robot_at ?r ?l))
        (at start (robot_carry ?r ?gr ?i))
        )
    :effect 
        (and 
        (at end (item_at ?i ?l))
        (at end (gripper_free ?gr))
        (at start (not (robot_carry ?r ?gr ?i)))
        )
    )

(:durative-action object_placed
    :parameters (?l - location ?i - item ?g - grandma)
    :duration (= ?duration 1)
    :condition 
        (and 
            (at start (item_at ?i ?l))
            (over all (grandma_assisted ?g))
        )
    :effect 
        (and 
            (at start (placed ?i ?l))
        )
    )

(:durative-action Give_item_asked
    :parameters (?l - location ?g - grandma ?i - item)
    :duration (= ?duration 1)
    :condition 
        (and 
        (at start (grandma_at ?g ?l))
        (at start (item_at ?i ?l))
        (at start (give_me_item ?i ?g))
        )
    :effect 
        (and 
        (at start(not (give_me_item ?i ?g)))
        (at end(grandma_assisted ?g))
        )
    )  

(:durative-action Close_door_asked
    :parameters (?l - location ?g - grandma ?r - robot ?d - door)
    :duration (= ?duration 1)
    :condition 
        (and
        (at start (robot_at ?r ?l))
        (at start (grandma_at ?g ?l))
        (at start (close ?d ))
        )
    :effect 
        (and 
        (at start(not (close_me_door ?d ?g)))
        (at end(grandma_assisted ?g))
        )
    )  
)