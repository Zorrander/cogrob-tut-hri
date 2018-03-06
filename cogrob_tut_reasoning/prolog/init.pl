/**

  @author Alexandre Angleraud

*/

:- register_ros_package(cob_reasoning).

:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_objects).
:- register_ros_package(knowrob_srdl).
:- register_ros_package(knowrob_mongo).

:- owl_parser:owl_parse('package://cob_reasoning/owl/cob4-2.owl').
:- owl_parser:owl_parse('package://cob_reasoning/owl/humans.owl').
:- owl_parser:owl_parse('package://cob_reasoning/owl/targets.owl').
:- owl_parser:owl_parse('package://cob_reasoning/owl/actions.owl').
:- owl_parser:owl_parse('package://cob_reasoning/owl/ipa-apartment.owl').


:- rdf_db:rdf_register_ns(cob, 'http://aalto.intelligent.robotics/kb/cob4-2.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(rdfs, 'http://www.w3.org/2000/01/rdf-schema#', [keep(true)]).
:- rdf_db:rdf_register_ns(aalto_humans, 'http://aalto.intelligent.robotics/kb/humans.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(ipa-apartment, 'http://cob.gazebo.world/kb/ipa-apartment.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(actions, 'http://aalto.intelligent.robotics/kb/actions.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(targets, 'http://aalto.intelligent.robotics/kb/targets.owl#', [keep(true)]).

:- use_module(library('reasoner')).


:- rdf_instance_from_class(knowrob:'SemanticEnvironmentMap', Inst),
	rdf_assert(Inst, knowrob:'linkToMapFile', 'http://cob.gazebo.world/kb/ipa-apartment.owl#ipa-apartment_robot1').
