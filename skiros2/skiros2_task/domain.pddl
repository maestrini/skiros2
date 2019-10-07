(define (domain untitled)
(:requirements :typing :fluents :universal-preconditions)
(:types 
	skiros:Product rparts:GripperEffector rparts:ArmDevice skiros:Location sumo:Agent skiros:Camera - thing
	rparts:GripperEffector skiros:Location - skiros:containx
)
(:predicates 
	(skiros:at ?x - sumo:Agent  ?y - skiros:Location )
	(skiros:contain ?x - skiros:containx  ?y - skiros:Product )
	(Empty ?x - rparts:GripperEffector )
	(skiros:partReference ?x - skiros:Location  ?y - skiros:Product )
	(Idle ?x - skiros:Camera )
	(skiros:Position ?x - skiros:Product )
)
(:functions 
)
(:durative-action pick_fake
	:parameters (?Object - skiros:Product ?Container - skiros:Location ?Arm - rparts:ArmDevice ?Gripper - rparts:GripperEffector ?Robot - sumo:Agent )
	:duration (= ?duration 1)
	:condition (and
		(at start (Empty ?Gripper))
		(at start (skiros:at ?Robot ?Container))
		(at start (skiros:contain ?Container ?Object))
	)
	:effect (and
		(at start (not (skiros:at ?Robot ?Container)))
		(at end (skiros:at ?Robot ?Container))
		(at end (skiros:contain ?Gripper ?Object))
		(at start (not (Empty ?Gripper)))
	)
)

(:durative-action locate_fake
	:parameters (?Object - skiros:Product ?Camera - skiros:Camera ?Container - skiros:Location ?Robot - sumo:Agent )
	:duration (= ?duration 1)
	:condition (and
		(at start (skiros:partReference ?Container ?Object))
		(at start (Idle ?Camera))
		(at start (skiros:at ?Robot ?Container))
	)
	:effect (and
		(at end (skiros:contain ?Container ?Object))
		(at end (skiros:Position ?Object))
		(at start (not (Idle ?Camera)))
		(at end (Idle ?Camera))
	)
)

(:durative-action place_fake
	:parameters (?Object - skiros:Product ?Gripper - rparts:GripperEffector ?Arm - rparts:ArmDevice ?PlacingLocation - skiros:Location ?Robot - sumo:Agent )
	:duration (= ?duration 1)
	:condition (and
		(at start (skiros:at ?Robot ?PlacingLocation))
		(at start (skiros:contain ?Gripper ?Object))
	)
	:effect (and
		(at start (not (skiros:contain ?Gripper ?Object)))
		(at end (Empty ?Gripper))
		(at end (skiros:contain ?PlacingLocation ?Object))
	)
)

(:durative-action drive_fake
	:parameters (?TargetLocation - skiros:Location ?StartLocation - skiros:Location ?Robot - sumo:Agent )
	:duration (= ?duration 1)
	:condition (and
		(at start (skiros:at ?Robot ?StartLocation))
	)
	:effect (and
		(at end (skiros:at ?Robot ?TargetLocation))
		(at start (not (skiros:at ?Robot ?StartLocation)))
	)
)

)
