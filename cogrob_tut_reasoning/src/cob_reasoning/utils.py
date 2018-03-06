#!/usr/bin/env python


name_spaces={
    'owl' : 'http://www.w3.org/2002/07/owl#',
    'rdfs' : 'http://www.w3.org/2000/01/rdf-schema#',
    'knowrob' : 'http://knowrob.org/kb/knowrob.owl#',
    'aalto_humans' : 'http://aalto.intelligent.robotics/kb/humans.owl#',
    'ipa_apartment' : 'http://cob.gazebo.world/kb/ipa-apartment.owl#',
    'cob4-2' : 'http://aalto.intelligent.robotics/kb/cob4-2.owl',
    'targets' : 'http://aalto.intelligent.robotics/kb/targets.owl#',
    'actions' : 'http://aalto.intelligent.robotics/kb/actions.owl#'
    }


def make_uri(base_uri, uri_fragment):
    uri =  "'" + name_spaces[base_uri] + uri_fragment + "'"
    return uri

def get_uri_fragment(uri):
    uri_broke_down =  uri.split("#")
    uri2 = ''.join(e for e in uri_broke_down[1] if e.isalnum())
    return uri2
