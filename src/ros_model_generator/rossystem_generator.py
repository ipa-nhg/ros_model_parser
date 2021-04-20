#!/usr/bin/env python
#
# Copyright 2020 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pprint
from pyparsing import *
import ros_metamodels.rossystem_metamodel_core as system_model
import ros_metamodels.ros_metamodel_core as model
import rosgraph
import rosparam
import rosservice


BLACK_LIST_PARAM = ['/rosdistro', '/rosversion', '/run_id']
BLACK_LIST_TOPIC = ["/tf", "/tf_static", "/rosout", "/clock"]
BLACK_LIST_SERV = ["/set_logger_level", "/get_loggers"]
BLACK_LIST_NODE = ["/rosout"]

ACTION_FILTER = ['goal', 'cancel']
ACTION_FILTER2 = ['status', 'result', 'feedback']

def check_black_list(name, black_list):
    for bl_ in black_list:
        if bl_ in name:
            return False
    return True

def init_node_dict(nodes, name):
    nodes[name] = {'publishers' : dict(),
                   'subscribers' : dict(),
                   'service_servers' : dict(),
                   'service_clients' :dict(),
                   'action_servers' :dict(),
                   'action_clients' : dict() }

def check_actions(publishers, subscribers, action_clients, action_servers):
        pubs_ = [pub for pub in publishers.keys()]
        subs_ = [sub for sub in subscribers.keys()]

        remove_pubs = list()
        remove_subs = list()

        # Check Action client
        for topic_name, topic_type in publishers.items():
            if topic_name.endswith(ACTION_FILTER[0]):
                _action_name = topic_name[:-len(ACTION_FILTER[0]) - 1]
                cancel_topic = _action_name + '/' + ACTION_FILTER[1]
                if not (cancel_topic in pubs_):
                    continue
                remove_pubs.append(topic_name)
                remove_pubs.append(cancel_topic)
                for name in ACTION_FILTER2:
                    topic = _action_name + '/' + name
                    if not (topic in subs_):
                        continue
                    remove_subs.append(topic)
                _action_type = topic_type[:-10]  # Hardcoded ActionGoal
                action_clients.add((_action_name, _action_type))

        # Check Action Server
        for topic_name, topic_type in subscribers.items():
            if topic_name.endswith(ACTION_FILTER[0]):
                _action_name = topic_name[:-len(ACTION_FILTER[0]) - 1]
                cancel_topic = _action_name + '/' + ACTION_FILTER[1]
                if not (cancel_topic in subs_):
                    continue
                remove_subs.append(topic_name)
                remove_subs.append(cancel_topic)
                for name in ACTION_FILTER2:
                    topic = _action_name + '/' + name
                    if not (topic in pubs_):
                        continue
                    remove_pubs.append(topic)
                _action_type = topic_type[:-10]  # Hardcode ActionGoal
                action_servers[_action_name] = _action_type

        for topic in remove_pubs:
            publishers.pop(topic)
        for topic in remove_subs:
            subscribers.pop(topic)

def create_ros_graph_snapshot():
    master = rosgraph.Master('snapshot')
    params = list()
    topics_dict = dict()

    if not(master.is_online()):
        print("Error: ROSMaster not found")
        return list()

    # Get parameters
    for param_name in master.getParamNames():
        if param_name not in BLACK_LIST_PARAM and not(param_name.startswith('/roslaunch')):
            params.append(param_name)
    state = master.getSystemState() #get the system state
    pubs, subs, services = state

    #get all topics type
    topic_list = master.getTopicTypes()
    for topic, topic_type in topic_list:
        topics_dict[topic] = topic_type

    components = dict()
    for pub, nodes in pubs:
        if not check_black_list(pub, BLACK_LIST_TOPIC):
            continue
        for node in nodes:
            if not check_black_list(node, BLACK_LIST_NODE):
                continue
            if node not in components:
                init_node_dict(components, node)
            components[node]['publishers'][pub] = topics_dict[pub]

    for sub, nodes in subs:
        if not check_black_list(sub, BLACK_LIST_TOPIC):
            continue
        for node in nodes:
            if not check_black_list(node, BLACK_LIST_NODE):
                continue
            if node not in components:
                init_node_dict(components, node)
            components[node]['subscribers'][sub] = topics_dict[sub]

    for serv, nodes in services:
        if not check_black_list(serv, BLACK_LIST_SERV):
            continue
        for node in nodes:
            if not check_black_list(node, BLACK_LIST_NODE):
                continue
            if node not in components:
                init_node_dict(components, node)
            components[node]['service_servers'][serv] = rosservice.get_service_type(serv)

    for name in components:
        publishers = components[name]['publishers']
        subscribers = components[name]['subscribers']
        action_clients = components[name]['action_clients']
        action_servers = components[name]['action_servers']
        check_actions(publishers, subscribers, action_clients, action_servers)

    return components


class RosSystemModelGenerator(object):
  def __init__(self,name=""):
    self.system = system_model.RosSystem(name);

  def setSystemName(self, name):
    self.system.name = name;

  def addParameter(self, name, value):
    self.system.params.add(model.Parameter(name, value, type(value)))

  def addComponent(self, name):
    self.system.components.add(system_model.Component(name))

  def addComponent(self, component):
    self.system.components.add(component)

  def dump_ros_system_model(self, rosystem_model_file):
    sucess, ros_system_model_str = self.create_ros_system_model()
    with open(rosystem_model_file, 'w') as outfile:
      outfile.write(ros_system_model_str)

  def dump_ros_system_model_list(self, components, rosystem_model_file):
    sucess, ros_system_model_str = self.create_ros_system_model_list(components)
    with open(rosystem_model_file, 'w') as outfile:
      outfile.write(ros_system_model_str)

  def create_ros_system_model(self):
    ros_system_model_str = self.system.dump_xtext_model()
    return True, ros_system_model_str

  def create_ros_system_model_list(self, components):
    for name in components:
      component = system_model.Component(name)

      publishers = components[name]['publishers']
      for pub, pub_type in publishers.items():
        component.publishers.add(system_model.RosInterface(pub, pub_type))

      subscribers = components[name]['publishers']
      for sub, sub_type in subscribers.items():
        component.subscribers.add(system_model.RosInterface(sub, sub_type))

      service_servers = components[name]['service_servers']
      for serv, serv_type in service_servers.items():
        component.service_servers.add(system_model.RosInterface(serv, serv_type))

      service_clients = components[name]['service_clients']
      for serv, serv_type in service_clients.items():
        component.service_clients.add(system_model.RosInterface(serv, serv_type))

      action_clients = components[name]['action_clients']
      for action, action_type in action_clients.items():
        component.action_clients.add(system_model.RosInterface(action, action_type))

      action_servers = components[name]['action_servers']
      for action, action_type in action_servers.items():
        component.action_servers.add(system_model.RosInterface(action, action_type))

      self.addComponent(component)

    return self.create_ros_system_model()

  def generate_ros_system_model_from_graph(self):
    components = create_ros_graph_snapshot()
    system_model_str = self.create_ros_system_model_list(components)[1]
    return system_model_str


if __name__ == "__main__":
  generator = RosSystemModelGenerator()
  try:
    print(generator.dump_java_ros_system_model("/tmp/test").dump())
  except Exception as e:
    print(e.args)

