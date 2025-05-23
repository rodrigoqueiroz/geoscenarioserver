#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
# ---------------------------------------------
# Custom Behavior Tree Render in String form for Dashboard and graph image using PyDot
# --------------------------------------------

import logging
log = logging.getLogger(__name__)
import pydot
from SimConfig import *
from py_trees import *
from sv.btree.BTreeLeaves import BCondition, ManeuverAction


def generate_string_tree(tree, vid, current_mconfig, show_status = True, hide_unvisited = False):
    ''''
        Generates a plain String Version of the Behavior Tree.
        show_status: wheter it shows the runtime status (SUCCESS, FAILURE, RUNNING)
        hide_unvisited: hide unvisited nodes (use this option when the tree is too large)
    '''
    
    root = tree.root
    INDENT = '  '   #space per indentation
    indent=0    #starting indentation  
    tip_id = root.tip().id if root.tip() else None
    
    def get_node_label(node,show_status):
        node_status = {
            common.Status.SUCCESS: ' o ',
            common.Status.FAILURE: ' x ',
            common.Status.INVALID: ' - ',
            common.Status.RUNNING: ' ... '
        }
        if isinstance(node, composites.Selector):
           node_label = "[ ? ]" 
        elif isinstance(node, composites.Sequence):
            node_label = "[ -> ]"
        elif isinstance(node, composites.Parallel):
            node_label = "[ => ]"
        elif isinstance(node,BCondition):
            node_label = node.name + "(" + node.condition +  ")" #+ node.tree_label  #str(node.kwargs)
        elif isinstance(node,ManeuverAction):
            node_label = node.name  #+ "(" + node.tree_label + ")"
        node_label += (" " + node_status[node.status]) if show_status else ""

        return node_label

    #recursive
    def generate_lines(root, internal_indent):
        def single_line(node):
            s = ""
            if node.id == tip_id:
                s = "**"
            s += INDENT * internal_indent
            s += get_node_label(node,show_status)
            return s
        lines = []

        #root node
        if internal_indent == indent:
            lines.append(single_line(root)) 
            internal_indent += 1

        #child nodes
        for child in root.children:
            lines.append(single_line(child))
            if child.children != []:
                for line in generate_lines(child, internal_indent + 1):
                    if not hide_unvisited or not child.status == common.Status.INVALID:
                        lines.append(line)
        return lines
    
    tree_string = ""
    for line in generate_lines(root, indent):
        if line:
            tree_string += "%s\n" % line
    
    if SHOW_MCONFIG:
        tree_string += " \n MANEUVER: \n" + str(current_mconfig)

    return "Behavior Tree. Vehicle " + str(vid) + "\n" + tree_string

def generate_graph_tree(tree, vid, current_mconfig = None, show_status = False, hide_unvisited = False):
    
    root = tree.root
    
    def get_node_attributes(node):
        node_status = {
            common.Status.SUCCESS: '[o]',
            common.Status.FAILURE: '[x]',
            common.Status.INVALID: '[-]',
            common.Status.RUNNING: '[...]'
        }
        if isinstance(node, composites.Selector):
            attributes = ('box', 'lightgrey', 'black', 12, "filled", "?")
        
        elif isinstance(node, composites.Sequence):
            attributes = ('box', 'lightgrey', 'black', 16,"filled", "→")
        
        elif isinstance(node, composites.Parallel):
            attributes = ('box', 'lightgrey', 'black', 12, "filled", "=>")
        
        elif isinstance(node,BCondition):
            node_label = node.name + "\n\n" + node.condition +  "\n" + node.tree_label #str(node.kwargs)
            attributes = ('ellipse', 'white', 'black', 10, "filled", node_label)
            #node_label = node.condition
            #attributes = ('elipse', 'white', 'black', 10, "filled", node_label)

        elif isinstance(node,ManeuverAction):
            node_label = node.name  +  "\n" +  node.tree_label
            attributes = ('box', 'white', 'black',10, "filled", node_label) #

        return attributes

    graph = pydot.Dot(graph_type='digraph', ordering="out")
    graph.set_name("sdv_model_behaviortree")
    graph.set_graph_defaults(fontname='times-roman')
    graph.set_node_defaults(fontname='times-roman')
    graph.set_edge_defaults(fontname='times-roman')

    #Root
    (node_shape, node_colour, node_font_colour,fontsize, style, label) = get_node_attributes(root)
    node_root = pydot.Node(
        name=root.name,
        label= label,
        shape=node_shape,
        style=style,
        fillcolor=node_colour,
        fontsize=fontsize,
        fontcolor=node_font_colour,
    )
    graph.add_node(node_root)
    behaviour_id_name_map = {root.id: root.name}

    #Child nodes (recursive)
    def add_children_and_edges(root, root_node, root_dot_name):
        node_names = []
        for c in root.children:
            (node_shape, node_colour, node_font_colour, fontsize, style, label) = get_node_attributes(c)
            node_name = c.name
            while node_name in behaviour_id_name_map.values():
                node_name += "*"
            behaviour_id_name_map[c.id] = node_name
            node = pydot.Node(
                name=node_name,
                label = label,
                shape=node_shape,
                style= style,
                fillcolor=node_colour,
                fontsize=fontsize,
                fontcolor=node_font_colour,
            )
            node_names.append(node_name)
            graph.add_node(node)
            edge = pydot.Edge(root_dot_name, node_name)
            graph.add_edge(edge)
            if c.children != []:
                add_children_and_edges(c, node, node_name)
    add_children_and_edges(root, node_root, root.name)

    #Output File
    filename = "btreegraph_vid"+ str(vid) + ".png"
    location = os.getenv("GSS_OUTPUTS", os.path.join(os.getcwd(), "outputs"))
    pathname = os.path.join(location, filename)
    graph.write_png(pathname)
    log.info("Writing graph tree in {}".format(pathname))
