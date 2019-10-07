(define (problem 1) (:domain untitled)
(:objects 
	rparts:ArmDevice-142 rparts:ArmDevice-2 rparts:ArmDevice-7 - rparts:ArmDevice
	cora:Robot-1 cora:Robot-6 - sumo:Agent
	skiros:Starter-145 skiros:Starter-52 - skiros:Product
	rparts:GripperEffector-3 rparts:GripperEffector-143 rparts:GripperEffector-8 - rparts:GripperEffector
	skiros:Location-10 skiros:Location-5 skiros:LargeBox-51 skiros:LargeBox-80 - skiros:Location
)
(:init 
	(skiros:at cora:Robot-1 skiros:Location-5)
	(skiros:at cora:Robot-6 skiros:Location-10)
	(skiros:at skiros:test_robot skiros:unknown_location)
	(skiros:at skiros:test_robot skiros:Location-5)
	(skiros:at skiros:test_robot skiros:Location-10)
	(skiros:at cora:Robot-6 skiros:unknown_location)
	(skiros:at cora:Robot-1 skiros:unknown_location)
	(skiros:contain skiros:LargeBox-51 skiros:Starter-145)
	(skiros:contain skiros:LargeBox-51 skiros:Starter-52)
	(skiros:contain skiros:large_box_test_starter skiros:starter)
	(skiros:contain skiros:large_box_test_starter skiros:Starter-145)
	(skiros:contain skiros:large_box_test_starter skiros:Starter-52)
	(skiros:contain skiros:LargeBox-51 skiros:starter)
	(Empty rparts:GripperEffector-3)
	(Empty rparts:GripperEffector-143)
	(Empty rparts:GripperEffector-8)
)
(:goal (and 
	(skiros:contain skiros:LargeBox-80 skiros:Starter-145)
))
)
