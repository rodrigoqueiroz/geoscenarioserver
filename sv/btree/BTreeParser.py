#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
#dinizr@chalmers.se

from py_trees import *
from sv.ManeuverConfig import MVelKeepConfig
from sv.ManeuverUtils import *
from sv.btree.BTreeLeaves import *

from antlr4 import *
from sv.btree.parser.BTreeDSLLexer import BTreeDSLLexer
from sv.btree.parser.BTreeDSLParser import BTreeDSLParser
from sv.btree.parser.BTreeDSLListener import BTreeDSLListener

import random
import string

import re
import os

'''For readability:
    st_ = subtree
    seq_    = sequence  (progressively tick each node)
    sel_    = selector  (decision makers, ticks each node until one succeed)
    par_    = parallel
    m_      = maneuver (action)
    c_      = condition
    - Cross cutting maneuver configuration (that will apply to the whole tree)
    must be passed from the root tree (scenario tree)
    - Alternative configurations from the same maneuver
    must be handled with a specialized tree (create and modify)
'''

class Subtree(object):
    def __init__(self, name="", args="", parent="", pos=0):
        self.name = name 
        self.args = args
        self.parent = parent 
        self.pos = pos

# might still need naming refactorings
class BTreeParser(object):
    def __init__(self, vid, bmodel):
        self.vid = vid
        self.bmodel = bmodel

    def gen_func(self, tree_name, _input):
        ''' 
            Transformation from input syntax to function
            ready to be instantiated.
        '''
        lexer = BTreeDSLLexer(InputStream(_input))
        parser = BTreeDSLParser(CommonTokenStream(lexer))
        listener = BTreeListener(self.vid, tree_name)
        ast = parser.behaviorTree()
        ParseTreeWalker().walk(listener, ast)
        function = listener.getTreeStr()
        subtrees = listener.getSubtrees()

        return function, subtrees

    def load_tree(self, path, tree_name):
        try:
            f = open(path + tree_name + ".btree",'r')
        except:
            raise Exception("Tree \'"+ tree_name +"\' was not found in" + path)
        finally:
            loaded_tree = f.read()
            f.close()

        return loaded_tree

    def create_instance(self, func):
        '''
            Creates an instance of the behavior tree by
            _exec_cuting the passed function.
        '''
        try:
            exec(func, globals())
            tree, nodes = getTreeInstance(self)
        except:
            raise RuntimeError("Could not instantiate tree.")

        return tree, nodes

    def instantiate(self, tree_name):
        ''' Instantiates a tree from the name.'''
        _input = self.load_tree(path="scenarios/trees/", tree_name=tree_name)
        func, subtrees = self.gen_func(tree_name, _input)
        root, nodes = self.create_instance(func)
        return root, nodes, subtrees

    def parse_tree(self, tree_name, args=""):

        root, nodes, subtrees = self.instantiate(tree_name)
        
        if(args != ""):
            args = args.split(";")
            if args[0] != '' :
                for arg in args: 
                    m_id = arg.split("=",1)[0]
                    m_config = arg.split("=",1)[1]
                    reconfigured=False
                    for node in nodes:
                        if node.name == m_id:
                            node.reconfigure(eval(m_config))
                            reconfigured=True
                            break
                    if not reconfigured: raise RuntimeError(m_id + " node could not be found in " + tree_name)

        tree = trees.BehaviourTree(root=root)
        for subtree in subtrees:
            parent = None
            for node in nodes:
                if node.name == subtree.parent: 
                    parent = node
                    break
            if parent == None: raise RuntimeError("Parent "+ subtree.parent +" not found.")
            stree_to_append = self.parse_tree(subtree.name, subtree.args).root
            tree.insert_subtree(stree_to_append, parent.id, int(subtree.pos))

        return tree

class BTreeListener(BTreeDSLListener):

    def __init__(self, vid, name):
        self.exec_stack=[]
        self.vid = vid
        self.name = name
        self.tree = ""
        self.subtrees = []
        self.nodes = "["
    
    def genstr(self, i):
        # gen 10 word random string
        #j = ''.join(random.choice(string.ascii_letters) for x in range(10))
        # interleave input with randword
        #res = "".join(m + n for m, n in zip(i, j))
        # to_lower
        s = i.lower()
        # remove non-alphabetical chars
        s=re.sub(r'\W+', '', s)
        #reduce the string size, 3-fold
        aux = ""
        for i in range(0,len(s),int(len(s)/3)):
            aux += s[i]
        s = aux[::-1]
        return s

    def map(self, symbol):
        if symbol == "->" : return "Sequence" 
        if symbol == "?"  : return "Selector"
        if symbol == "||" : return "Parallel"
        
        raise RuntimeError("No symbol matched (" + symbol + ")")
    
    def postProcessExecStack(self):
        self.fixSubtreeParentPosition()

    def fixSubtreeParentPosition(self):
        aux = []
        for item in self.exec_stack: # for every auxiliary line starting with ~
            if re.search("~", item): 
                aux = item[1:].split(",")
                self.exec_stack.remove(item) #remove auxiliary line from stack
                for subtree in self.subtrees:
                    if subtree.name == aux[0]:
                        subtree.parent = aux[1]
                        subtree.pos = aux[2]
    
    def getSubtrees(self):
        return self.subtrees
        
    def getTreeStr(self):
        return self.tree
    
    def buildTreeFunc(self):
        self.tree += "def getTreeInstance(parser):\n"
        for cmd in self.exec_stack: self.tree +=  "    " + cmd + "\n"
        self.tree += "    nodes = " 
        if len(self.nodes) > 1:  self.tree += self.nodes[:-1] + "]\n"
        else : self.tree += "[]\n"
        self.tree += "    return root, nodes"

    def exitBehaviorTree(self, ctx):
        self.postProcessExecStack()
        self.buildTreeFunc()

    def exitRootNode(self,ctx):
        node = ctx.node()
        if node.leafNode() != None:
            raise RuntimeError("The root node must be an operator (e.g. '?', '->', '||').")
        elif node.nodeComposition() != None:
            var = self.map(node.nodeComposition().OPERATOR().getText()).lower()[:3] + "_" + self.genstr(node.nodeComposition().getText())
        s = "root = " + var
        self.exec_stack.append(s)

    def exitNodeComposition(self, ctx): 
        nm = self.genstr(ctx.getText())
        op = self.map(ctx.OPERATOR().getText())
        name = op.lower()[:3] + "_" + nm
        
        s =  name + " = composites." + op + "(\"" + name + "\")"
        self.exec_stack.append(s)
        
        aux = []
        if hasattr(ctx.node(), '__iter__'):
            pos=0
            for node in ctx.node():

                if node.leafNode() != None:
                    if node.leafNode().maneuver() != None:
                        aux.append(node.leafNode().maneuver().name().getText())
                    elif node.leafNode().condition() != None:
                        aux.append(node.leafNode().condition().name().getText())
                    elif node.leafNode().subtree() != None:     
                        self.exec_stack.append("~" + node.leafNode().subtree().name().getText()+","+name+","+str(pos))
                    
                elif node.nodeComposition() != None:
                    aux.append(self.map(node.nodeComposition().OPERATOR().getText()).lower()[:3] + "_" + self.genstr(node.nodeComposition().getText()))
                pos += 1
        else:    
            aux = "["+self.genstr(ctx.node().getText()) + "]"

        s = name + ".add_children(" + str(aux).replace("\'","") + ")"
        self.exec_stack.append(s)
        self.nodes += name + ","
    
    # e.g. stop = ManeuverAction(self.bmodel, "stop", MStopConfig(time=2, dist=10, decel=3))
    def exitManeuver(self, ctx):
        name=ctx.name().getText() 
        mconfig = ctx.mconfig().getText()
        s = name + " = " + "ManeuverAction(parser.bmodel," + "\""+ name + "\""+"," + mconfig + ")"
        self.exec_stack.append(s)
        self.nodes += name + ","

    # e.g. endpoint = BCondition(self.bmodel,"endpoint", "lane_occupied", args)
    def exitCondition(self, ctx): 
        name = ctx.name().getText() 
        cconfig_name = ctx.cconfig().name().getText()
        s = name + " = " + "BCondition(parser.bmodel," + "\"" + name + "\"" + "," + "\"" + cconfig_name + "\""
        cconfig_params = ""
        if hasattr(ctx.cconfig().params(), '__iter__'):
            for param in ctx.cconfig().params(): cconfig_params += "," + param.getText()
        else:    
            cconfig_params = ctx.cconfig().params().getText()
        if cconfig_params != "": s += cconfig_params
        s += ")"
        self.exec_stack.append(s)
        self.nodes += name + ","
    
    # e.g. tree.insert_subtree(parser.load_subtree(drive_tree,  MVelKeepConfig(), MStopConfig()), [parent].id, [pos]))
    def exitSubtree(self, ctx): 
            name = ctx.name().getText()
            midconfs = ""
            if hasattr(ctx.midconf(), '__iter__'):
                for idconf in ctx.midconf(): midconfs += idconf.getText() + ";"
                midconfs = midconfs[:-1] #remove the last ;
            else:    
                midconfs = ctx.midconf().getText()
            
            stree = Subtree(name=name, args=midconfs)
            self.subtrees.append(stree)


'''DEPRECATED-keeping for reference

#REUSABLE TREES 
def drive_tree(self, bmodel, mvk_config=MVelKeepConfig(), mstop_config=MStopAtConfig()):
    #Drive SubTree
    #maneuvers
    m_vk = ManeuverAction(bmodel,"m vel keep", mvk_config)
    m_follow = ManeuverAction(bmodel,"m follow", MFollowConfig())
    m_stop = ManeuverAction(bmodel,"m stop", mstop_config)
    #conditions
    c_lane_occupied = BCondition(bmodel,"c lane_occupied?", "lane_occupied")
    c_lv_stop = BCondition(bmodel,"c lv_stopped?", "lv_stopped")
    c_reached_goal = BCondition(bmodel, "c reached_goal", "reached_goal")

    #Coordinate Maneuvers and Conditions
    # stopping when leading vehicle is stopped shouldn't have to be a separate behaviour,
    # follow maneuver should just do that. Also this will never be triggered because the
    # condition is broken (not keeping leading_vehicle in behaviour models anymore)
    # seq_lv_stop = composites.Sequence("seq lv stop")
    # seq_lv_stop.add_children([c_lv_stop, ManeuverAction(bmodel, "stop when lv stopped", mstop_config)])
    # if reached goal, do stop maneuver
    seq_reached_goal = composites.Sequence("seq reached goal")
    seq_reached_goal.add_children([c_reached_goal, m_stop])

    # sel_lv = composites.Selector("sel lv")
    # sel_lv.add_children([seq_lv_stop, m_follow])

    seq_busy_lane = composites.Sequence("seq busy Lane")
    seq_busy_lane.add_children([c_lane_occupied, m_follow])

    st_sel_drive = composites.Selector("ST Drive")
    st_sel_drive.add_children([seq_reached_goal, seq_busy_lane, m_vk])

    return st_sel_drive

def lane_change_tree(self, bmodel, target=1):
    m_lane_swerve = ManeuverAction(bmodel, "m lane swerve", MLaneSwerveConfig(target_lid=target))
    m_cutin = ManeuverAction(bmodel, "m cutin", MCutInConfig(target_lid=target))
    c_should_cutin = BCondition(bmodel, "c should do cutin?", "should_cutin", target_lane_id=target)

    # determine whether to do cutin
    seq_cutin = composites.Sequence("seq cutin", children=[c_should_cutin, m_cutin])

    sel_cutin_or_lane_swerve = composites.Selector("sel cutin or lane swerve", children=[seq_cutin, m_lane_swerve])

    return sel_cutin_or_lane_swerve

def reverse_tree(self, bmodel):
    m_reverse = ManeuverAction(bmodel, "m reverse", MReverseConfig())
    m_stop = ManeuverAction(bmodel,"m stop", MStopConfig())
    c_reached_goal = BCondition(bmodel, "c reached_goal", "reached_goal", reverse=True)

    # if reached goal, do stop maneuver
    seq_reached_goal = composites.Sequence("seq reached goal")
    seq_reached_goal.add_children([c_reached_goal, m_stop])

    st_sel_reverse = composites.Selector("ST Reverse")
    st_sel_reverse.add_children([seq_reached_goal, m_reverse])

    return st_sel_reverse
'''