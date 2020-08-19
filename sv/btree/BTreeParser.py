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

#class FindNode(VisitorBase):
#    def __init__(self, nodename):
#        super(Visitor, self).__init__(full=True)
#    def run(self, behaviour):
#        if behaviour.name:
#            behaviour.logger.debug("%s.run() [%s][%s]" % (self.__class__.__name__, behaviour.feedback_message, behaviour.status))
#        else:
#            behaviour.logger.debug("%s.run() [%s]" % (self.__class__.__name__, behaviour.status))

class Subtree(object):
    def __init__(self, name="", args="", parent="", pos=0):
        self.name = name 
        self.args = args
        self.parent = parent 
        self.pos = pos

class BTreeParser(object):
    def __init__(self, vid):
        self.vid = vid
    def load_subtree(self, sbtree_name, bmodel, args):
        args = args.split(";")

        sbtree_path = "./scenarios/trees/"
        try:
            s_ff = open(sbtree_path + sbtree_name + ".btree",'r')
        except:
            raise RuntimeError("Subtree \'"+ sbtree_name +"\' was not found in the list of available subtrees.")
        s_to_parse = s_ff.read()
        s_ff.close()
        s_lexer = BTreeDSLLexer(InputStream(s_to_parse))
        s_parser = BTreeDSLParser(CommonTokenStream(s_lexer))
        s_listener = self.BTreeListener(self.vid, sbtree_name)
        s_ast = s_parser.behaviorTree()
        ParseTreeWalker().walk(s_listener, s_ast)
        stree_str = s_listener.getTreeStr()
        
        try:
            exec(stree_str, globals())
            subtree, nodes = getTreeInstance(self, bmodel)
        except:
            raise RuntimeError("Failed to set " + sbtree_name + " subtree up.")
        
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
                if not reconfigured: raise RuntimeError(m_id + " node could not be found in " + sbtree_name)

        return subtree

    def parse_tree(self, bmodel, btree_name, textual_model):
        lexer = BTreeDSLLexer(InputStream(textual_model))
        parser = BTreeDSLParser(CommonTokenStream(lexer))
        ast = parser.behaviorTree()
        listener = self.BTreeListener(self.vid, btree_name)
        ParseTreeWalker().walk(listener, ast)
        tree_str = listener.getTreeStr()
        subtrees = listener.getSubtrees()

        try:
            exec(tree_str, globals())
            root,nodes = getTreeInstance(self, bmodel)
        except:
            raise RuntimeError("Could not set behavior tree up.")
        
        tree = trees.BehaviourTree(root=root)
        for subtree in subtrees:
            parent = None
            for node in nodes:
                if node.name == subtree.parent: 
                    parent = node
                    break
            if parent == None: raise RuntimeError("Parent "+ subtree.parent +" not found.")
            stree_to_append = self.load_subtree(subtree.name, bmodel, subtree.args)
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

        def sortExecStack(self):
            aux = []
            for item in self.exec_stack:
                if re.search("tree.insert_subtree",item):
                    aux.append(item)
            
            for aux_item in aux:
                self.exec_stack.remove(aux_item)
                self.exec_stack.append(aux_item)
        
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
        
        def build_tree(self):
            self.tree += "def getTreeInstance(parser, bmodel):\n"
            for cmd in self.exec_stack: self.tree +=  "    " + cmd + "\n"
            self.tree += "    nodes = " 
            if len(self.nodes) > 1:  self.tree += self.nodes[:-1] + "]\n"
            else : self.tree += "[]\n"
            self.tree += "    return root, nodes"
            #print(self.tree)

        def exitBehaviorTree(self, ctx):
            #s = "tree = trees.BehaviourTree(root=root)"
            #self.exec_stack.append(s)

            self.postProcessExecStack()
            self.build_tree()

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
                        elif node.leafNode().subtree() != None:     self.exec_stack.append("~" + node.leafNode().subtree().name().getText()+","+name+","+str(pos))
                        
                    elif node.nodeComposition() != None:
                        aux.append(self.map(node.nodeComposition().OPERATOR().getText()).lower()[:3] + "_" + self.genstr(node.nodeComposition().getText()))
                    pos += 1
            else:    
                aux = "["+self.genstr(ctx.node().getText()) + "]"

            s = name + ".add_children(" + str(aux).replace("\'","") + ")"
            self.exec_stack.append(s)
            self.nodes += name + ","
        
        # e.g. stop = ManeuverAction(bmodel, "stop", MStopConfig(time=2, dist=10, decel=3))
        def exitManeuver(self, ctx):
            name=ctx.name().getText() 
            mconfig = ctx.mconfig().getText()
            s = name + " = " + "ManeuverAction(bmodel," + "\""+ name + "\""+"," + mconfig + ")"
            self.exec_stack.append(s)
            self.nodes += name + ","

        # e.g. endpoint = BCondition(bmodel,"endpoint", "lane_occupied", args)
        def exitCondition(self, ctx): 
            name = ctx.name().getText() 
            cconfig_name = ctx.cconfig().name().getText()
            s = name + " = " + "BCondition(bmodel," + "\"" + name + "\"" + "," + "\"" + cconfig_name + "\""
            cconfig_params = ""
            if hasattr(ctx.cconfig().params(), '__iter__'):
                for param in ctx.cconfig().params(): cconfig_params += "," + param.getText()
            else:    
                cconfig_params = ctx.cconfig().params().getText()
            if cconfig_params != "": s += cconfig_params
            s += ")"
            self.exec_stack.append(s)
            self.nodes += name + ","
        
        # e.g. tree.insert_subtree(parser.load_subtree(drive_tree, bmodel,  MVelKeepConfig(), MStopConfig()), [parent].id, [pos]))
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