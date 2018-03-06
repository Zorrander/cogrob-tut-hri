/** <module> Methods for interacting with the knowledge base

  @author Alexandre Angleraud

*/

:- module(reasoner,
    [
	match_literal/2,
	    get_robot/1,
	    get_map/1,
      retrieve_rotation_matrix/2,
      compute_X_coordinate/2,
      compute_Y_coordinate/2,
      compute_Z_coordinate/2,
	    create_and_set_location_request/3,
      create_new_demand/2,
      check_existence_target/2,
	    check_existence_request/2,
	    set_objectif_request/2,
      set_location_request/2,
      set_author_request/2,
      create_request_perception/5,
      is_missing_something/3,
      link_restriction/3
    ]).

%% match_literal(+literal(type(D,Lex)))
%

% @param literal
match_literal(literal(type(_,Lex)), Num):-
	atom_number(Lex,Num).

%% get_robot(-Inst)
% Retrieve the robot from the knowledge base

% @param Inst	Instance of the robot

get_robot(Inst) :-
	owl_individual_of(Inst, knowrob:'WheeledRobot').


%% get_map(-Inst)
% Retrieve the map from the knowledge base

% @param Inst	Instance of the map

get_map(Inst) :-
	owl_individual_of(Env, knowrob:'SemanticEnvironmentMap'),
	owl_has(Env, knowrob:'linkToMapFile', Inst).


%% retrieve_rotation_matrix(+Place, -Mat)

% Retrieve the rotation matrix of a component of the environment

% @param Target	Name of the place to go
% @param Mat    rotation matrix associated with the proprioception

retrieve_rotation_matrix(Target, Mat) :-
    owl_individual_of(Ind, Target),
    owl_has(Joint, 'http://knowrob.org/kb/srdl2-comp.owl#succeedingLink', Ind),
    owl_has(Perception, knowrob:'objectActedOn', Joint),
    owl_has(Perception,knowrob:'eventOccursAt',Mat).

%% compute_X_coordinate(Mat, X) :-

% Retrieve the X coordinate of a place given the rotation matrix associated with proprioception of this place

% @param Mat    rotation matrix instance associated with the proprioception
% @param X      X coordinate of the place

compute_X_coordinate(Mat, X) :-
    owl_has(Mat, knowrob:m03, Literal_X),
    match_literal(Literal_X, X).

%% compute_Y_coordinate(Mat, Y) :-

% Retrieve the Y coordinate of a place given the rotation matrix associated with proprioception of this place

% @param Mat    rotation matrix instance associated with the proprioception
% @param Y      Y coordinate of the place

compute_Y_coordinate(Mat, Y) :-
    owl_has(Mat, knowrob:m13, Literal_Y),
    match_literal(Literal_Y, Y).

%% compute_Z_coordinate(Mat, Z) :-

% Retrieve the Z coordinate of a place given the rotation matrix associated with proprioception of this place

% @param Mat    rotation matrix instance associated with the proprioception
% @param Z      Z coordinate of the place

compute_Z_coordinate(Mat, Z) :-
    owl_has(Mat, knowrob:m23, Literal_Z),
    match_literal(Literal_Z, Z).


create_and_set_location_request(TargetName, InstAction, TargetInstance) :-
	atom_concat('http://aalto.intelligent.robotics/kb/targets.owl#', TargetName, SubClass),
	rdf_instance_from_class(SubClass, TargetInstance),
    rdf_assert(InstAction, knowrob:'toLocation', TargetInstance).

%% create_new_demand(+Class, -Inst, -Perception)


% Creates an instance of a request, an instance of a perception
% and sets the objectif and requirements of the request

% @param Class			Type of the request received (No distinction for now)
% @param Inst    		Instance of the request created by this predicate
% @param Perception		Perception instance that has been created by this predicate

create_new_demand(Class, Inst) :-
    rdf_instance_from_class(Class, Inst).
    %create_perception_instance(['OralPerception'], Perception).

%% set_objectif_request(+Request, -Class)

% Check if the action requested is in the ontology

% @param Action		Instance of a request received (No distinction for now)
% @param InstAction	Action dicted by the request

check_existence_request(Action, InstAction) :-
	atom_concat('http://aalto.intelligent.robotics/kb/actions.owl#', Action, SubClass),
	rdfs_subclass_of(SubClass, actions:'Independent_entity'),
	rdf_instance_from_class(SubClass, InstAction).

check_existence_target(Target, InstTarget) :-
	atom_concat('http://aalto.intelligent.robotics/kb/targets.owl#', Target, SubClass),
	rdfs_subclass_of(SubClass, targets:'Independent_entity'),
	rdf_instance_from_class(SubClass, InstTarget).

%% set_objectif_request(+Request, -Target)

% Creates an instance of MovementEvent : TranslationToPointOfInterest. Sets the bodyParts required
% and connects it to an instance of a Request via "has_purpose"

% @param Request		Instance of a request received (No distinction for now)
% @param Target			Action dicted by the request

set_objectif_request(InstRequest, InstAction) :-
	rdf_assert(InstRequest, aalto_humans:'has_purpose', InstAction).


%% set_location_request(+Deplacement, +Place)

% Associates a place to go to an instance of Deplacement

% @param Deplacement	Instance of a deplacement
% @param Place		    Name of the place to go

set_location_request(InstAction, Place) :-
    rdf_assert(InstAction, knowrob:'toLocation', Place).

%% set_author_request(+Request, +InstHuman)

% Associates a request with the person who asked

% @param Request	Instance of a request
% @param InstHuman	Instance of a person

set_author_request(Request, InstHuman) :-
    rdf_assert(Request, aalto_humans:'spoken_by', InstHuman).


%% create_request_perception(+Class, +Place, +InstHuman, -Request, -Perception)

% Reacts to a request

% @param Class		Type of the request received (No distinction for now)
% @param Action		Name of the action to perform
% @param Place		Name of the thing acted on or acted to
% @param Perception	Perception instance that has been created by this predicate

create_request_perception(Class, Action, Place, InstHuman, Request) :-
    create_new_demand(Class, Request),
    check_existence_request(Action, InstAction),
    set_objectif_request(Request, InstAction),
    check_existence_target(Place, InstPlace),
    set_location_request(InstAction, InstPlace).
    %set_author_request(Request, InstHuman).


%% is_missing_something(+Request, -Cap, -Comp)

% Checks if the action requested is realizable

% @param Request    Instance of a request

is_missing_something(Request, Cap, Comp) :-
    get_robot(Robot),
    owl_has(Request, aalto_humans:'has_purpose', Action),
    owl_has(Action, rdf:type, Type),
    missing_for_action(Type, Robot, Cap, Comp).


%% link_restriction(+Restriction,+Class, -Prolog)

link_restriction(Restriction, Class, Prolog) :-
    owl_description(Restriction, Prolog),
    rdf_assert_prolog(Class, 'http://www.w3.org/2000/01/rdf-schema#subClassOf', Prolog).
